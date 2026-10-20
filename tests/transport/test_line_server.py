from __future__ import annotations

import socket
import threading
from collections.abc import Iterator

import pytest

from machinist.transport.line_server import Hangup, LineServer, Reply, stateless

from ..conftest import free_port


def _reply(message: str) -> Reply:
    if message == "pair":
        return ["first", "second"]
    return Hangup(["bye"]) if message == "quit" else message


@pytest.fixture
def server() -> Iterator[LineServer]:
    srv = LineServer("127.0.0.1", free_port(), session_factory=stateless(_reply))
    ready = threading.Event()
    thread = threading.Thread(target=srv.serve_forever, args=(ready,), daemon=True)
    thread.start()
    ready.wait(2)
    yield srv
    srv.shutdown()
    thread.join(2)


def _connect(server: LineServer) -> socket.socket:
    return socket.create_connection(("127.0.0.1", server._port), timeout=2)


def test_a_plain_reply_keeps_the_connection_open(server: LineServer) -> None:
    with _connect(server) as s:
        s.sendall(b"one\ntwo\n")
        buf = b""
        while buf.count(b"\n") < 2:
            buf += s.recv(4096)
        assert buf == b"one\ntwo\n"


def test_a_hangup_sends_its_lines_then_closes(server: LineServer) -> None:
    with _connect(server) as s:
        s.sendall(b"quit\nnever answered\n")
        buf = b""
        while chunk := s.recv(4096):
            buf += chunk
        assert buf == b"bye\n"


def test_a_line_gap_sends_each_line_apart() -> None:
    """A client that reads each line with its own recv gets them one at a time."""
    srv = LineServer("127.0.0.1", free_port(), session_factory=stateless(_reply),
                     line_gap_seconds=0.2)
    ready = threading.Event()
    thread = threading.Thread(target=srv.serve_forever, args=(ready,), daemon=True)
    thread.start()
    ready.wait(2)
    try:
        with _connect(srv) as s:
            s.sendall(b"pair\n")
            assert s.recv(4096) == b"first\n"
            assert s.recv(4096) == b"second\n"
    finally:
        srv.shutdown()
        thread.join(2)
