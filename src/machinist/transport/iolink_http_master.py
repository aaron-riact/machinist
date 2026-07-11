"""Tiny HTTP server emulating an IFM AL1350 IO-Link master.

The real AL1350 exposes a JSON REST API ("IoT Core") for reading and
writing process data on each of its IO-Link ports. We model that with
two endpoints:

* ``GET  /iolink/port/{port}/pd``  -> returns the current process data
* ``POST /iolink/port/{port}/pd``  -> accepts a JSON body to write data

Devices implement :class:`IOLinkPort` and plug into the master.

We use a stdlib HTTP server so the framework has no extra dependency.
"""

from __future__ import annotations

import json
import threading
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Protocol


@dataclass(frozen=True, slots=True)
class ProcessData:
    """Typed schema for IO-Link process data."""

    diameter_mm: float = 0.0
    target_mm: float = 0.0
    grip_force_n: int = 0
    moving: bool = False


class IOLinkPort(Protocol):
    def read_process_data(self) -> ProcessData: ...
    def write_process_data(self, data: ProcessData) -> None: ...


class IOLinkHttpMaster:
    """Single-port (port 1) IO-Link master HTTP gateway."""

    def __init__(self, *, host: str, port: int, port_device: IOLinkPort) -> None:
        self._host = host
        self._port = port
        self._device = port_device
        self._server: ThreadingHTTPServer | None = None

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        device = self._device

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_: Any) -> None:  # silence default logging
                return

            def do_GET(self) -> None:  # noqa: N802 (stdlib API)
                if self.path.endswith("/pd"):
                    pd = device.read_process_data()
                    self._reply(200, {
                        "diameter_mm": pd.diameter_mm,
                        "target_mm": pd.target_mm,
                        "grip_force_n": pd.grip_force_n,
                        "moving": pd.moving,
                    })
                else:
                    self._reply(404, {"error": "not found"})

            def do_POST(self) -> None:  # noqa: N802 (stdlib API)
                length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(length).decode("utf-8") if length else "{}"
                try:
                    data = json.loads(body)
                except json.JSONDecodeError:
                    self._reply(400, {"error": "invalid json"})
                    return
                if self.path.endswith("/pd"):
                    device.write_process_data(ProcessData(**data))
                    self._reply(200, {"status": "ok"})
                else:
                    self._reply(404, {"error": "not found"})

            def _reply(self, status: int, payload: dict[str, object]) -> None:
                body = json.dumps(payload).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

        self._server = ThreadingHTTPServer((self._host, self._port), Handler)
        if ready is not None:
            ready.set()
        self._server.serve_forever(poll_interval=0.05)

    def shutdown(self) -> None:
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
