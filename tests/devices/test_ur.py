from __future__ import annotations

import socket

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions
from machinist.devices.robots.ur import URDashboardServer

from ..conftest import free_port, wait_running


def _send(host: str, port: int, msg: str) -> str:
    with socket.create_connection((host, port), timeout=2) as s:
        s.sendall(msg.encode() + b"\n")
        return s.recv(256).decode().strip()


def test_dashboard_basic_commands() -> None:
    port = free_port()
    bus = EventBus()
    device = URDashboardServer("ur1", Endpoint("127.0.0.1", port), bus, ArmOptions())
    device.start()
    try:
        wait_running(device)
        assert _send("127.0.0.1", port, "polyscope") == "Connected: Universal Robots Dashboard Server"
        assert "RUNNING" in _send("127.0.0.1", port, "robotmode")
        assert "Loading program" in _send("127.0.0.1", port, "load /programs/x")
        # Trigger e-stop via the underlying arm and observe through robotmode.
        device.arm.estop()
        assert "PROTECTIVE_STOP" in _send("127.0.0.1", port, "robotmode")
    finally:
        device.stop()
