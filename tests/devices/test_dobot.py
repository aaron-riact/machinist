from __future__ import annotations

import socket
import time

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions
from machinist.devices.robots.arm import ArmMode, ArmStateView
from machinist.devices.robots.dobot import (
    DOBOT_FEEDBACK_FAST_PORT,
    DobotDashboard,
    DobotFeedbackPacket,
    _ARM_MODE_TO_ROBOT_MODE,
    _update_feedback_packet,
)

from ..conftest import free_port, wait_running


@pytest.fixture
def dobot() -> DobotDashboard:
    bus = EventBus()
    d = DobotDashboard("dobot1", Endpoint("127.0.0.1", free_port()), bus, ArmOptions(),
                       feedback_enabled=False)
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


def test_dobot_robotmode_returns_enable_idle(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RobotMode()")
    assert reply == "0,{5},RobotMode()"


def test_dobot_movj_ack(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "EnableRobot()MovJ(0,0,0,0,0,0)", expect=2)
    # Two replies concatenated. MovJ returns a command ID in the value field.
    assert "0,{},EnableRobot()" in reply
    assert "MovJ(0,0,0,0,0,0)" in reply
    assert "{1}" in reply  # command ID 1


def test_dobot_unknown_command_returns_error_code(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "Nonsense()")
    assert reply.startswith("-10000,")


def test_dobot_geterrorid_returns_empty_when_no_errors(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetErrorID()")
    assert reply == "0,{[]},GetErrorID()"


def test_dobot_geterrorid_returns_errors_after_emergency_stop(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "EmergencyStop()GetErrorID()", expect=2)
    assert "0,{},EmergencyStop()" in reply
    assert "0,{[1]},GetErrorID()" in reply


def test_dobot_geterrorid_clears_after_clearerror(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "EmergencyStop()ClearError()GetErrorID()", expect=3)
    assert "0,{},EmergencyStop()" in reply
    assert "0,{},ClearError()" in reply
    assert "0,{[]},GetErrorID()"


def test_dobot_stop_on_idle_robot_is_noop(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "Stop()RobotMode()", expect=2)
    assert "0,{},Stop()" in reply
    assert "0,{5},RobotMode()" in reply


def test_dobot_stop_during_motion_returns_to_idle(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "EnableRobot()MovJ(10,20,30,40,50,60)Stop()RobotMode()", expect=4)
    assert "0,{},EnableRobot()" in reply
    assert "MovJ(10,20,30,40,50,60)" in reply
    assert "0,{},Stop()" in reply
    assert "0,{5},RobotMode()" in reply


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


def test_robot_mode_mapping_covers_all_arm_modes() -> None:
    for mode in ArmMode:
        assert mode in _ARM_MODE_TO_ROBOT_MODE, f"missing mapping for {mode}"


def test_feedback_server_streams_packets() -> None:
    """Connect to the fast feedback port and verify we receive 1440-byte packets."""
    bus = EventBus()
    d = DobotDashboard(
        "dobot_fb", Endpoint("127.0.0.1", free_port()), bus, ArmOptions(),
    )
    d.start()
    try:
        wait_running(d)
        s = socket.create_connection(("127.0.0.1", DOBOT_FEEDBACK_FAST_PORT), timeout=2)
        try:
            data = s.recv(1440, socket.MSG_WAITALL)
            assert len(data) == 1440
            assert int.from_bytes(data[0:2], "little") == 1440
            assert int.from_bytes(data[48:56], "little") == 0x123456789abcdef
            assert int.from_bytes(data[24:32], "little") == 5  # RobotMode=5 (IDLE)
        finally:
            s.close()
    finally:
        d.stop()


def test_update_feedback_packet_populates_fields() -> None:
    pkt = DobotFeedbackPacket()
    state = ArmStateView(
        joints=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        pose=(100.0, 200.0, 300.0, 0.1, 0.2, 0.3),
        mode=ArmMode.MOVING,
        servo_on=True,
        program_running=False,
        speed_fraction=0.8,
    )
    _update_feedback_packet(pkt, state, now_us=5000, command_id=42)

    assert pkt.len == 1440
    assert pkt.TestValue == 0x123456789abcdef
    assert pkt.RobotMode == 7  # MOVING → ROBOT_MODE_RUNNING
    assert pkt.TimeStamp == 5000
    assert pkt.SpeedScaling == 0.8
    assert list(pkt.QActual) == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert list(pkt.ToolVectorActual) == [100.0, 200.0, 300.0, 0.1, 0.2, 0.3]
    assert pkt.EnableStatus == 1
    assert pkt.BrakeStatus == 0   # MOVING → brakes off
    assert pkt.ErrorStatus == 0
    assert pkt.RunningStatus == 1
    assert pkt.CurrentCommandId == 42

    # Modes that produce different outputs
    idle = ArmStateView(joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0), mode=ArmMode.IDLE, servo_on=False, program_running=False, speed_fraction=1.0)
    _update_feedback_packet(pkt, idle, command_id=99)
    assert pkt.RobotMode == 5       # IDLE → ROBOT_MODE_ENABLE
    assert pkt.EnableStatus == 0    # servo_off
    assert pkt.BrakeStatus == 1     # IDLE → brakes on
    assert pkt.ErrorStatus == 0
    assert pkt.RunningStatus == 0
    assert pkt.CurrentCommandId == 99

    # ESTOPPED and FAULTED both map to RobotMode 9 (ERROR)
    estop = ArmStateView(
        joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        mode=ArmMode.ESTOPPED, servo_on=False, program_running=False, speed_fraction=1.0,
    )
    _update_feedback_packet(pkt, estop, command_id=100)
    assert pkt.RobotMode == 9
    assert pkt.BrakeStatus == 1
    assert pkt.ErrorStatus == 1

    faulted = ArmStateView(
        joints=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        mode=ArmMode.FAULTED, servo_on=False, program_running=False, speed_fraction=1.0,
    )
    _update_feedback_packet(pkt, faulted, command_id=101)
    assert pkt.RobotMode == 9
    assert pkt.BrakeStatus == 0  # FAULTED → brakes off
    assert pkt.ErrorStatus == 1
