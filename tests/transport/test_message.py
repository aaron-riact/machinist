"""Transport-agnostic message channels: TCP and UDP frame round-trips."""

from __future__ import annotations

import threading

import pytest

from machinist.transport.message import (
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
    server = open_server(name, "127.0.0.1", port)
    ready = threading.Event()
    thread = threading.Thread(
        target=server.serve_forever, args=(_echo, ready), daemon=True
    )
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
        open_server("carrier-pigeon", "127.0.0.1", 1)


def test_transports_lists_available() -> None:
    assert set(transports()) == {"tcp", "udp"}
