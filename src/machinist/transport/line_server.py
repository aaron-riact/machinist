"""A minimal threaded TCP line-protocol server.

Responsibilities are split cleanly:

* :class:`~machinist.transport.framing.Framer` owns byte ↔ message
  framing (newlines, CRLF, Dobot's parens, …).
* :class:`SessionHandler` owns **per-connection state**: anything that
  has to survive across messages on one socket (e.g. Motoman's
  ``CONNECT`` handshake gate).
* :class:`LineServer` is a dumb accept loop: one thread per client.

This separation kills the two worst foot-guns of the old design: a
single terminator that had to serve both rx and tx, and a stateless
handler that couldn't express multi-message handshakes.
"""

from __future__ import annotations

import socket
import threading
from collections.abc import Callable, Iterable
from typing import Protocol

from .framing import Framer, TerminatorFramer

Reply = Iterable[str] | str | None


class SessionHandler(Protocol):
    """Per-connection message handler."""

    def handle(self, message: str) -> Reply: ...


#: A factory that produces a fresh :class:`SessionHandler` per client.
SessionFactory = Callable[[], SessionHandler]


class _Stateless:
    """Adapter: wrap a plain ``str -> Reply`` callable as a SessionHandler."""

    __slots__ = ("_fn",)

    def __init__(self, fn: Callable[[str], Reply]) -> None:
        self._fn = fn

    def handle(self, message: str) -> Reply:
        return self._fn(message)


def stateless(fn: Callable[[str], Reply]) -> SessionFactory:
    """Wrap a stateless ``str -> Reply`` callable as a session factory."""
    return lambda: _Stateless(fn)


class LineServer:
    """Threaded line-protocol TCP server."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        session_factory: SessionFactory,
        framer: Framer | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._framer: Framer = framer or TerminatorFramer()
        self._make_session = session_factory
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._client_threads: list[threading.Thread] = []

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        """Run the accept loop until :meth:`shutdown` is called."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.listen(8)
        self._sock.settimeout(0.25)
        if ready is not None:
            ready.set()
        try:
            while not self._stop.is_set():
                try:
                    client, _ = self._sock.accept()
                except TimeoutError:
                    continue
                except OSError:
                    return
                t = threading.Thread(target=self._serve_client, args=(client,), daemon=True)
                t.start()
                self._client_threads.append(t)
        finally:
            if self._sock is not None:
                self._sock.close()

    def shutdown(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    # -----------------------------------------------------------------

    def _serve_client(self, client: socket.socket) -> None:
        client.settimeout(0.25)
        session = self._make_session()
        buf = bytearray()
        try:
            while not self._stop.is_set():
                try:
                    chunk = client.recv(4096)
                except TimeoutError:
                    continue
                if not chunk:
                    return
                buf.extend(chunk)
                for message in self._framer.decode(buf):
                    self._dispatch(client, session, message)
        finally:
            client.close()

    def _dispatch(self, client: socket.socket, session: SessionHandler, msg: str) -> None:
        reply = session.handle(msg)
        if reply is None:
            return
        lines = (reply,) if isinstance(reply, str) else tuple(reply)
        out = b"".join(self._framer.encode(line) for line in lines)
        client.sendall(out)
