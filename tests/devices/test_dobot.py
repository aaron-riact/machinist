from __future__ import annotations

import socket

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions
from machinist.devices.robots.dobot import DobotDashboard, DobotFeedbackPacket

from ..conftest import free_port, wait_running


@pytest.fixture
def dobot() -> DobotDashboard:
    bus = EventBus()
    d = DobotDashboard("dobot1", Endpoint("127.0.0.1", free_port()), bus, ArmOptions())
    d.start()
    try:
        wait_running(d)
        yield d
    finally:
        d.stop()


def _send(dobot: DobotDashboard, message: str, *, expect: int = 1) -> str:
    """Send paren-delimited command(s), read ``expect`` semicolon-terminated replies."""
    with socket.create_connection((dobot.endpoint.host, dobot.endpoint.port), timeout=2) as s:
        s.sendall(message.encode())
        buf = b""
        while buf.count(b";") < expect:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        return buf.decode().rstrip(";")


def test_dobot_uses_paren_framing_not_semicolon(dobot: DobotDashboard) -> None:
    """Regression: the earlier impl treated ';' as the *incoming* terminator.

    Per the Dobot V4.6.2 interface guide, commands end at the closing
    paren and only *replies* carry ';'. Sending ``EnableRobot()`` should
    therefore produce a reply even without a trailing semicolon.
    """
    reply = _send(dobot, "EnableRobot()")
    assert reply == "0,{},EnableRobot()"


def test_dobot_movj_ack(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "EnableRobot()MovJ(0,0,0,0,0,0)", expect=2)
    # Two replies concatenated.
    assert "0,{},EnableRobot()" in reply
    assert "0,{},MovJ(0,0,0,0,0,0)" in reply


def test_dobot_unknown_command_returns_error_code(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "Nonsense()")
    assert reply.startswith("-10000,")


def test_feedback_packet_layout() -> None:
    import ctypes
    assert ctypes.sizeof(DobotFeedbackPacket) == 1440

    pkt = DobotFeedbackPacket()
    assert all(b == 0 for b in bytes(pkt))

    pkt.len = 1440
    pkt.TestValue = 0x123456789abcdef
    pkt.RobotMode = 5
    pkt.QActual[:] = (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    pkt.ToolVectorActual[:] = (100.0, 200.0, 300.0, 0.0, 0.0, 0.0)
    pkt.EnableStatus = 1
    pkt.BrakeStatus = 1
    pkt.SpeedScaling = 1.0

    buf = bytes(pkt)
    assert len(buf) == 1440

    assert int.from_bytes(buf[0:2], "little") == 1440
    assert int.from_bytes(buf[48:56], "little") == 0x123456789abcdef
