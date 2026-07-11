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


def test_dobot_tooldi_returns_zero_for_valid_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolDI(1)")
    assert reply == "0,{0},ToolDI(1)"


def test_dobot_tooldi_rejects_missing_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolDI()")
    assert reply.startswith("-20000,")


def test_dobot_tooldi_rejects_non_numeric_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolDI(a)")
    assert reply.startswith("-30001,")


def test_dobot_tooldi_rejects_out_of_range_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolDI(99)")
    assert reply.startswith("-40001,")


def test_dobot_gettooldo_returns_zero_for_valid_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetToolDO(1)")
    assert reply == "0,{0},GetToolDO(1)"


def test_dobot_gettooldo_rejects_missing_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetToolDO()")
    assert reply.startswith("-20000,")


def test_dobot_gettooldo_rejects_non_numeric_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetToolDO(a)")
    assert reply.startswith("-30001,")


def test_dobot_gettooldo_rejects_out_of_range_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetToolDO(99)")
    assert reply.startswith("-40001,")


def test_dobot_ai_returns_zero_for_valid_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "AI(1)")
    assert reply == "0,{0.0},AI(1)"


def test_dobot_ai_rejects_missing_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "AI()")
    assert reply.startswith("-20000,")


def test_dobot_ai_rejects_non_numeric_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "AI(a)")
    assert reply.startswith("-30001,")


def test_dobot_ai_rejects_out_of_range_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "AI(3)")
    assert reply.startswith("-40001,")


def test_dobot_getao_returns_zero_for_valid_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetAO(1)")
    assert reply == "0,{0.0},GetAO(1)"


def test_dobot_getao_rejects_missing_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetAO()")
    assert reply.startswith("-20000,")


def test_dobot_getao_rejects_non_numeric_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetAO(a)")
    assert reply.startswith("-30001,")


def test_dobot_getao_rejects_out_of_range_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetAO(3)")
    assert reply.startswith("-40001,")


def test_dobot_toolai_returns_zero_for_valid_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolAI(1)")
    assert reply == "0,{0.0},ToolAI(1)"


def test_dobot_toolai_rejects_missing_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolAI()")
    assert reply.startswith("-20000,")


def test_dobot_toolai_rejects_non_numeric_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolAI(a)")
    assert reply.startswith("-30001,")


def test_dobot_toolai_rejects_out_of_range_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "ToolAI(3)")
    assert reply.startswith("-40001,")


def test_dobot_speedfactor_sets_global_speed_ratio(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SpeedFactor(50)")
    assert reply == "0,{},SpeedFactor(50)"
    assert dobot.arm.state.speed_fraction == 0.5


def test_dobot_speedfactor_appears_in_build_detail(dobot: DobotDashboard) -> None:
    _send(dobot, "SpeedFactor(75)")
    detail = dobot.build_detail()
    fields = detail["derived_fields"]
    sf = next(f for f in fields if f["signal"] == "speedfactor")
    assert sf["value"] == "75%"


def test_dobot_robot_type_defaults_to_cr5() -> None:
    bus = EventBus()
    d = DobotDashboard("d", Endpoint("127.0.0.1", free_port()), bus, ArmOptions(), feedback_enabled=False)
    assert d._robot_type_code == 5


def test_dobot_robot_type_configured_via_factory() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr10", "feedback_ports": False})
    assert d._robot_type_code == 10
    assert d._tool_di_count == 2
    assert d._tool_do_count == 2
    d.stop()


def test_dobot_robot_type_cr5_via_factory_uses_dh_kinematics() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr5", "feedback_ports": False})
    s = d.arm.state.snapshot()
    assert any(abs(v) > 1e-9 for v in s.pose), "expected non-zero pose from CR5 DH kinematics"
    assert d._tool_di_count == 2
    assert d._tool_do_count == 2
    d.stop()


def test_dobot_robot_type_cr20_uses_max_io() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr20", "feedback_ports": False})
    assert d._tool_di_count == 4
    assert d._tool_do_count == 4
    d.stop()


def test_dobot_unknown_robot_type_defaults_to_max_io() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "nonexistent", "feedback_ports": False})
    assert d._tool_di_count == 4
    assert d._tool_do_count == 4
    d.stop()


def test_dobot_cr5_rejects_tool_di_outside_bounds() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr5", "feedback_ports": False})
    d.start()
    try:
        wait_running(d)
        reply = _send(d, "ToolDI(3)")
        assert reply.startswith("-40001,")
    finally:
        d.stop()


def test_dobot_cr20_accepts_tool_di_inside_bounds() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr20", "feedback_ports": False})
    d.start()
    try:
        wait_running(d)
        reply = _send(d, "ToolDI(3)")
        assert reply == "0,{0},ToolDI(3)"
    finally:
        d.stop()


def test_dobot_speedfactor_rejects_missing_ratio(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SpeedFactor()")
    assert reply.startswith("-20000,")


def test_dobot_speedfactor_rejects_non_numeric_ratio(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SpeedFactor(a)")
    assert reply.startswith("-30001,")


def test_dobot_speedfactor_rejects_out_of_range_ratio(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SpeedFactor(0)")
    assert reply.startswith("-40001,")


def test_dobot_speedfactor_rejects_above_100(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SpeedFactor(101)")
    assert reply.startswith("-40001,")


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
    assert pkt.CRRobotType == 5  # default CR5

    # Custom robot type code
    _update_feedback_packet(pkt, state, now_us=5001, command_id=43, robot_type_code=10)
    assert pkt.CRRobotType == 10

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


def test_dobot_robot_type_cr10a_via_factory_uses_dh_kinematics() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr10a", "feedback_ports": False})
    s = d.arm.state.snapshot()
    assert any(abs(v) > 1e-9 for v in s.pose), "expected non-zero pose from CR10A DH kinematics"
    d.stop()


def test_dobot_robot_type_cr20a_via_factory_uses_dh_kinematics() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr20a", "feedback_ports": False})
    s = d.arm.state.snapshot()
    assert any(abs(v) > 1e-9 for v in s.pose), "expected non-zero pose from CR20A DH kinematics"
    d.stop()


def test_dobot_robot_type_cr10a_io_bounds() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr10a", "feedback_ports": False})
    assert d._tool_di_count == 2
    assert d._tool_do_count == 2
    d.stop()


def test_dobot_robot_type_cr20a_io_bounds() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr20a", "feedback_ports": False})
    assert d._tool_di_count == 4
    assert d._tool_do_count == 4
    d.stop()


def test_dobot_robot_type_cr10_via_factory_uses_dh_kinematics() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr10", "feedback_ports": False})
    s = d.arm.state.snapshot()
    assert any(abs(v) > 1e-9 for v in s.pose), "expected non-zero pose from CR10 (CR10A) DH kinematics"
    d.stop()


def test_dobot_robot_type_cr20_via_factory_uses_dh_kinematics() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr20", "feedback_ports": False})
    s = d.arm.state.snapshot()
    assert any(abs(v) > 1e-9 for v in s.pose), "expected non-zero pose from CR20 (CR20A) DH kinematics"
    d.stop()


def test_dobot_robot_type_cr10_io_bounds() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr10", "feedback_ports": False})
    assert d._tool_di_count == 2
    assert d._tool_do_count == 2
    d.stop()


def test_dobot_robot_type_cr20_io_bounds() -> None:
    from machinist.devices.robots.dobot import _factory
    bus = EventBus()
    d = _factory("d", Endpoint("127.0.0.1", free_port()), bus, {"robot_type": "cr20", "feedback_ports": False})
    assert d._tool_di_count == 4
    assert d._tool_do_count == 4
    d.stop()


def test_dobot_quiet_commands_suppress_rx_tx_events() -> None:
    bus = EventBus()
    received: list[Event] = []
    bus.subscribe(received.append)
    d = DobotDashboard("quiet1", Endpoint("127.0.0.1", free_port()), bus, ArmOptions(),
                       feedback_enabled=False)
    d.start()
    try:
        wait_running(d)
        _send(d, "ToolDI(1)")
        _send(d, "AI(1)")
        _send(d, "GetToolDO(1)")
        _send(d, "GetAO(1)")
        _send(d, "ToolAI(1)")
    finally:
        d.stop()

    rx_events = [e for e in received if e.kind == "rx"]
    tx_events = [e for e in received if e.kind == "tx"]
    assert len(rx_events) == 0
    assert len(tx_events) == 0


def test_dobot_settool_defines_new_frame(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SetTool(1,{10,20,30,0,0,0})")
    assert reply == "0,{},SetTool(1,{10,20,30,0,0,0})"
    assert dobot._tool_frames[1] == (10.0, 20.0, 30.0, 0.0, 0.0, 0.0)


def test_dobot_settool_with_type_arg(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SetTool(1,{10,20,30,0,0,0},0)")
    assert reply == "0,{},SetTool(1,{10,20,30,0,0,0},0)"
    assert dobot._tool_frames[1] == (10.0, 20.0, 30.0, 0.0, 0.0, 0.0)


def test_dobot_settool_rejects_out_of_range_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SetTool(99,{10,20,30,0,0,0})")
    assert reply.startswith("-40001,")


def test_dobot_settool_rejects_missing_args(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SetTool(1)")
    assert reply.startswith("-30001,")


def test_dobot_tool_zero_always_succeeds(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "Tool(0)")
    assert reply.startswith("0,{1},")  # command ID 1


def test_dobot_tool_with_frame_selects_active(dobot: DobotDashboard) -> None:
    _send(dobot, "SetTool(1,{0,0,0,0,0,0})Tool(1)", expect=2)
    assert dobot._active_tool == 1


def test_dobot_tool_fails_for_undefined_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "Tool(42)")
    assert reply.startswith("-1,")


def test_dobot_tool_rejects_out_of_range(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "Tool(99)")
    assert reply.startswith("-40001,")


def test_dobot_reljointmovj_moves_by_delta(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelJointMovJ(10,0,0,0,0,0)")
    assert reply.startswith("0,") and "RelJointMovJ" in reply
    assert "{1}" in reply  # command ID 1


def test_dobot_reljointmovj_with_kwargs(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelJointMovJ(5,0,0,0,0,0,tool=1,user=0)")
    assert reply.startswith("0,") and "RelJointMovJ" in reply


def test_dobot_reljointmovj_fails_on_bad_args(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelJointMovJ(10,20)")
    assert reply.startswith("-30001,")


def test_dobot_relmovltool_moves(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelMovLTool(10,0,0,0,0,0)")
    assert reply.startswith("0,") and "RelMovLTool" in reply
    assert "{1}" in reply  # command ID 1


def test_dobot_relmovltool_moves_in_tool_frame(dobot: DobotDashboard) -> None:
    _send(dobot, "SetTool(1,{0,0,0,0.785,0,0})Tool(1)RelMovLTool(10,0,0,0,0,0)", expect=3)
    assert dobot._current_command_id[0] == 2  # Tool + RelMovLTool (SetTool is non-queued)


def test_dobot_relmovltool_fails_on_bad_args(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelMovLTool(10,20,30)")
    assert reply.startswith("-30001,")


def test_dobot_relmovltool_with_kwargs(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelMovLTool(0,0,0,0,0,0,tool=1,user=0)")
    assert reply.startswith("0,") and "RelMovLTool" in reply


def test_dobot_relmovltool_with_speed_kwarg(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "RelMovLTool(5,0,0,0,0,0,speed=20)")
    assert reply.startswith("0,") and "RelMovLTool" in reply


def test_dobot_movl_with_pose_braces(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovL(pose={-500,100,200,150,0,90})")
    assert reply.startswith("0,{1},") and "MovL" in reply


def test_dobot_movl_with_pose_braces_and_kwargs(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovL(pose={-500,100,200,150,0,90},v=60)")
    assert reply.startswith("0,{1},") and "MovL" in reply


def test_dobot_movj_with_joint_braces(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovJ(joint={10,20,30,40,50,60})")
    assert reply.startswith("0,{1},") and "MovJ" in reply


def test_dobot_movl_bare_floats_with_kwargs(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovL(0,0,0,0,0,0,v=60)")
    assert reply.startswith("0,{1},") and "MovL" in reply


def test_dobot_movl_pose_braces_rejects_wrong_count(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovL(pose={1,2,3,4,5})")
    assert reply.startswith("-30001,")


def test_dobot_movl_rejects_bad_args(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovL(not_valid)")
    assert reply.startswith("-30001,")
