from __future__ import annotations

import socket

import pytest

from machinist.core.events import Event, EventBus
from machinist.core.line_device import LineServerDevice
from machinist.core.types import Endpoint


class EchoDevice(LineServerDevice):
    kind = "echo"

    def handle_line(self, line: str) -> str:
        return f"echo:{line}"


class QuietEchoDevice(LineServerDevice):
    kind = "quiet_echo"
    _quiet_commands = frozenset({"ping"})

    def handle_line(self, line: str) -> str:
        verb = line.split("(")[0].lower().strip() if "(" in line else ""
        if verb == "ping":
            return "pong"
        return f"echo:{line}"


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.mark.timeout(5)
def test_line_device_round_trip() -> None:
    bus = EventBus()
    received: list[Event] = []
    bus.subscribe(received.append)

    port = _free_port()
    device = EchoDevice("echo1", Endpoint("127.0.0.1", port), bus)
    device.start()
    try:
        assert device.wait_ready(timeout=2.0)
        with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
            s.sendall(b"hello\n")
            data = s.recv(64)
        assert data == b"echo:hello\n"
    finally:
        device.stop()

    kinds = {e.kind for e in received}
    assert {"state", "rx", "tx"}.issubset(kinds)


@pytest.mark.timeout(5)
def test_quiet_commands_suppress_rx_tx() -> None:
    bus = EventBus()
    received: list[Event] = []
    bus.subscribe(received.append)

    port = _free_port()
    device = QuietEchoDevice("quiet1", Endpoint("127.0.0.1", port), bus)
    device.start()
    try:
        assert device.wait_ready(timeout=2.0)
        with socket.create_connection(("127.0.0.1", port), timeout=2) as s:
            s.sendall(b"ping()\n")
            data = s.recv(64)
        assert data == b"pong\n"
    finally:
        device.stop()

    rx_events = [e for e in received if e.kind == "rx"]
    tx_events = [e for e in received if e.kind == "tx"]
    assert len(rx_events) == 0
    assert len(tx_events) == 0
