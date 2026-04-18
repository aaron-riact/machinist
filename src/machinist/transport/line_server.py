"""A minimal threaded TCP line-protocol server.

Industrial protocols that we need to emulate (UR Dashboard, Yaskawa
NX100 telnet, HAAS DPRINT, Dobot TCP) all boil down to "newline
terminated text in, newline terminated text out". Centralising that
pattern keeps device code trivial and easy to test.

Concurrency model: one accept thread + one thread per client. Industrial
servers rarely see >1-2 simultaneous clients so this is plenty.
"""

from __future__ import annotations

import socket
import threading
from collections.abc import Callable, Iterable

#: A handler returns the lines to send back. Returning ``None`` for any
#: line keeps the connection alive without a reply.
LineHandler = Callable[[str], "Iterable[str] | str | None"]


class LineServer:
    """Threaded line-protocol TCP server.

    Parameters
    ----------
    host, port:
        Endpoint to bind.
    handler:
        Callable invoked with each received line (already stripped of
        the terminator). Its return value is sent back to the client.
    terminator:
        Line terminator on the wire. Defaults to ``"\\n"``; some
        protocols want ``"\\r\\n"``.
    encoding:
        Text encoding (defaults to ASCII for industrial robustness).
    """

    def __init__(
        self,
        host: str,
        port: int,
        handler: LineHandler,
        *,
        terminator: str = "\n",
        encoding: str = "ascii",
    ) -> None:
        self._host = host
        self._port = port
        self._handler = handler
        self._terminator = terminator
        self._encoding = encoding
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
        buf = bytearray()
        term = self._terminator.encode(self._encoding)
        try:
            while not self._stop.is_set():
                try:
                    chunk = client.recv(4096)
                except TimeoutError:
                    continue
                if not chunk:
                    return
                buf.extend(chunk)
                while term in buf:
                    line, _, rest = buf.partition(term)
                    buf[:] = rest
                    self._dispatch(client, line.decode(self._encoding, errors="replace"))
        finally:
            client.close()

    def _dispatch(self, client: socket.socket, line: str) -> None:
        reply = self._handler(line)
        if reply is None:
            return
        lines = (reply,) if isinstance(reply, str) else tuple(reply)
        out = "".join(f"{line}{self._terminator}" for line in lines)
        client.sendall(out.encode(self._encoding))
