"""Transport-agnostic message channels: TCP and UDP frame round-trips."""

from __future__ import annotations

import socket
import threading

import pytest

from machinist.transport.message import (
    UdpDatagramServer,
    open_server,
    open_transport,
    transports,
)

from ..conftest import free_port


def _echo(frame: bytes) -> bytes:
    return b"ack:" + frame


@pytest.mark.parametrize("name", ["tcp", "udp"])
def test_request_response_round_trip(name: str) -> None:
    port = free_port()
    server = open_server(name, "127.0.0.1", port, _echo)
    ready = threading.Event()
    thread = threading.Thread(target=server.serve_forever, args=(ready,), daemon=True)
    thread.start()
    assert ready.wait(timeout=2.0)
    client = open_transport(name, "127.0.0.1", port)
    try:
        assert client.request(b"hello") == b"ack:hello"
        assert client.request(b"\x00\x01\x02") == b"ack:\x00\x01\x02"
    finally:
        client.close()
        server.shutdown()
        thread.join(timeout=2.0)


def test_unknown_transport_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown transport"):
        open_transport("carrier-pigeon", "127.0.0.1", 1)
    with pytest.raises(ValueError, match="unknown transport"):
        open_server("carrier-pigeon", "127.0.0.1", 1, _echo)


def test_transports_lists_available() -> None:
    assert set(transports()) == {"tcp", "udp"}


def test_a_datagram_server_knows_the_sender_and_may_stay_silent() -> None:
    senders: list[tuple[str, int]] = []

    def handle(payload: bytes, addr: tuple[str, int]) -> bytes | None:
        senders.append(addr)
        return None if payload == b"quiet" else payload.upper()

    port = free_port()
    server = UdpDatagramServer("127.0.0.1", port, handle)
    ready = threading.Event()
    thread = threading.Thread(target=server.serve_forever, args=(ready,), daemon=True)
    thread.start()
    ready.wait(2)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(0.5)
            s.sendto(b"quiet", ("127.0.0.1", port))
            s.sendto(b"hello", ("127.0.0.1", port))
            assert s.recv(64) == b"HELLO"
            assert [port for _, port in senders] == [s.getsockname()[1]] * 2
    finally:
        server.shutdown()
        thread.join(2)
