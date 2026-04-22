"""Tiny line-based broadcast TCP server.

One server, many clients: whatever :meth:`broadcast` is called with is
sent as a line to every connected client. This is the shape of
industrial side-channels like HAAS DPRINT where the controller pushes
log lines and a monitoring client tails them.

Thread-safe. Dead clients are pruned lazily on the next broadcast.
"""

from __future__ import annotations

import socket
import threading
from collections.abc import Iterable


class BroadcastServer:
    """Listen on TCP; broadcast any line to all connected clients."""

    def __init__(self, host: str, port: int, *, terminator: str = "\n") -> None:
        self._host = host
        self._port = port
        self._term = terminator.encode("ascii")
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._clients: list[socket.socket] = []
        self._clients_lock = threading.Lock()

    # ----- lifecycle -------------------------------------------------

    def serve_forever(self, ready: threading.Event | None = None) -> None:
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
                with self._clients_lock:
                    self._clients.append(client)
        finally:
            if self._sock is not None:
                self._sock.close()
            with self._clients_lock:
                for c in self._clients:
                    c.close()
                self._clients.clear()

    def shutdown(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    # ----- broadcast -------------------------------------------------

    def broadcast(self, line: str) -> None:
        """Send ``line`` plus the terminator to every live client."""
        payload = line.encode("ascii", errors="replace") + self._term
        with self._clients_lock:
            alive: list[socket.socket] = []
            for c in self._clients:
                try:
                    c.sendall(payload)
                    alive.append(c)
                except OSError:
                    c.close()
            self._clients = alive

    def broadcast_all(self, lines: Iterable[str]) -> None:
        for line in lines:
            self.broadcast(line)
