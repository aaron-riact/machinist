"""Stdlib HTTP + Server-Sent-Events server for the Machinist web UI.

Why no framework? Machinist's whole pitch is "pure-Python, single binary,
no hard deps". A websocket library would betray that, so the live feed
rides on **Server-Sent Events** instead — a one-way, text/event-stream push
that maps perfectly onto the :class:`~machinist.core.events.EventBus` and is
natively consumed by the browser's ``EventSource``. Everything else is a
couple of JSON endpoints and a static-file handler.

Routes
------
``GET  /``               → ``static/index.html``
``GET  /<asset>``        → ``static/<asset>`` (js/css)
``GET  /api/state``      → full :func:`~machinist.web.api.snapshot_world`
``GET  /api/events``     → SSE stream of live :class:`Event` objects
``POST /api/command``    → ``{"command": "..."}`` → dispatch result

The server runs each request on its own thread (``ThreadingHTTPServer``);
the SSE handler simply blocks on a per-client queue fed by an ``EventBus``
subscription, so slow browsers can never stall a device's publisher thread.
"""

from __future__ import annotations

import contextlib
import json
import queue
import threading
from dataclasses import asdict, is_dataclass
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

from ..core.events import Event
from ..core.world import World
from .api import CommandError, dispatch_command, snapshot_world

STATIC_DIR = Path(__file__).parent / "static"

_CONTENT_TYPES = {
    ".html": "text/html; charset=utf-8",
    ".js": "text/javascript; charset=utf-8",
    ".css": "text/css; charset=utf-8",
    ".svg": "image/svg+xml",
    ".json": "application/json; charset=utf-8",
}

#: Heartbeat cadence (seconds) so SSE clients and proxies keep the socket
#: alive even when a fleet is momentarily idle.
_SSE_HEARTBEAT = 15.0


def event_to_dict(event: Event) -> dict[str, Any]:
    """Serialize an :class:`Event` to a JSON-able dict for the SSE feed."""
    return {
        "device": event.device,
        "kind": event.kind,
        "payload": event.payload,
        "timestamp": event.timestamp,
    }


class WebServer:
    """Owns the HTTP server thread and the world it exposes."""

    def __init__(self, world: World, *, host: str = "127.0.0.1", port: int = 8080) -> None:
        self.world = world
        self._httpd = ThreadingHTTPServer((host, port), _make_handler(world))
        self._httpd.daemon_threads = True
        self._thread: threading.Thread | None = None

    @property
    def address(self) -> tuple[str, int]:
        """The bound ``(host, port)`` — useful when port 0 was requested."""
        host, port = self._httpd.server_address[:2]
        return str(host), int(port)

    @property
    def url(self) -> str:
        host, port = self.address
        return f"http://{host}:{port}"

    def start(self) -> None:
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="machinist-web", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._httpd.shutdown()
        self._httpd.server_close()
        if self._thread is not None:
            self._thread.join(timeout=5.0)

    def serve_forever(self) -> None:
        """Block in the calling thread until interrupted."""
        self._httpd.serve_forever()


def serve(world: World, *, host: str = "127.0.0.1", port: int = 8080) -> WebServer:
    """Build and start a :class:`WebServer`; returns it for later ``stop()``."""
    server = WebServer(world, host=host, port=port)
    server.start()
    return server


def _make_handler(world: World) -> type[BaseHTTPRequestHandler]:
    """Bind a request-handler class to a specific world."""

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        # Quiet by default; the framework has its own event log.
        def log_message(self, *_args: Any) -> None:
            pass

        # ----- GET ----------------------------------------------------
        def do_GET(self) -> None:
            path = self.path.split("?", 1)[0]
            if path == "/api/state":
                self._send_json(snapshot_world(world))
            elif path == "/api/events":
                self._stream_events()
            else:
                self._send_static(path)

        # ----- POST ---------------------------------------------------
        def do_POST(self) -> None:
            if self.path.split("?", 1)[0] != "/api/command":
                self._send_json({"ok": False, "message": "not found"}, HTTPStatus.NOT_FOUND)
                return
            body = self._read_json()
            command = str(body.get("command", "")) if isinstance(body, dict) else ""
            try:
                result = dispatch_command(world, command)
            except CommandError as exc:
                self._send_json({"ok": False, "message": str(exc)}, HTTPStatus.BAD_REQUEST)
                return
            self._send_json(result)

        # ----- helpers ------------------------------------------------
        def _read_json(self) -> Any:
            length = int(self.headers.get("Content-Length", 0) or 0)
            raw = self.rfile.read(length) if length else b""
            try:
                return json.loads(raw or b"{}")
            except json.JSONDecodeError:
                return {}

        def _send_json(self, payload: Any, status: HTTPStatus = HTTPStatus.OK) -> None:
            data = json.dumps(payload, default=_json_default).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(data)

        def _send_static(self, path: str) -> None:
            rel = "index.html" if path in ("/", "") else path.lstrip("/")
            target = (STATIC_DIR / rel).resolve()
            if STATIC_DIR.resolve() not in target.parents or not target.is_file():
                self._send_json({"ok": False, "message": "not found"}, HTTPStatus.NOT_FOUND)
                return
            data = target.read_bytes()
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", _CONTENT_TYPES.get(target.suffix, "text/plain"))
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def _stream_events(self) -> None:
            inbox: queue.Queue[Event] = queue.Queue(maxsize=4096)

            def push(event: Event) -> None:
                with contextlib.suppress(queue.Full):
                    inbox.put_nowait(event)

            unsubscribe = world.bus.subscribe(push)
            self.send_response(HTTPStatus.OK)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "keep-alive")
            self.end_headers()
            try:
                while True:
                    try:
                        event = inbox.get(timeout=_SSE_HEARTBEAT)
                    except queue.Empty:
                        self.wfile.write(b": keep-alive\n\n")
                        self.wfile.flush()
                        continue
                    frame = json.dumps(event_to_dict(event), default=_json_default)
                    self.wfile.write(f"data: {frame}\n\n".encode())
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ValueError):
                pass  # client went away
            finally:
                unsubscribe()

    return Handler


def _json_default(obj: Any) -> Any:
    if is_dataclass(obj) and not isinstance(obj, type):
        return asdict(obj)
    return str(obj)
