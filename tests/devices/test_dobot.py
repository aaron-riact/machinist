from __future__ import annotations

import math
import socket
import time

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions
from machinist.devices.robots.arm import ArmMode, ArmStateView
from machinist.devices.robots.dobot import (
    DOBOT_FEEDBACK_FAST_PORT,
    DOBOT_ROBOT_MODELS,
    ERR_COMMAND_FAILED,
    ERR_ROBOT_IN_ERROR_STATE,
    MAX_MODBUS_MASTERS,
    PROTECTIVE_STOP_MODES,
    ROBOT_MODE_COLLISION,
    ROBOT_MODE_DISABLED,
    ROBOT_MODE_ERROR,
    DobotDashboard,
    DobotFeedbackPacket,
    EnableFailure,
    _ARM_MODE_TO_ROBOT_MODE,
    _CR20A_DH,
    _FaultState,
    _update_feedback_packet,
)
from machinist.kinematics.api import KinematicsOptions

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


@pytest.fixture
def dobot_cr20a() -> DobotDashboard:
    """A CR20a with real DH kinematics so ``MovJ`` pose targets exercise IK."""
    bus = EventBus()
    opts = ArmOptions(kinematics=KinematicsOptions(backend="dh", dh=_CR20A_DH))
    d = DobotDashboard(
        "cr20a", Endpoint("127.0.0.1", free_port()), bus, opts,
        feedback_enabled=False, model_info=DOBOT_ROBOT_MODELS["cr20a"],
    )
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
    assert list(pkt.QActual) == pytest.approx([math.degrees(1.0), math.degrees(2.0), math.degrees(3.0), math.degrees(4.0), math.degrees(5.0), math.degrees(6.0)])
    assert pkt.ToolVectorActual[:3] == pytest.approx((100000.0, 200000.0, 300000.0))
    assert pkt.ToolVectorActual[3:] == pytest.approx((math.degrees(0.1), math.degrees(0.2), math.degrees(0.3)))
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
    assert dobot._tool_frames[1] == (0.01, 0.02, 0.03, 0.0, 0.0, 0.0)


def test_dobot_settool_with_type_arg(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "SetTool(1,{10,20,30,0,0,0},0)")
    assert reply == "0,{},SetTool(1,{10,20,30,0,0,0},0)"
    assert dobot._tool_frames[1] == (0.01, 0.02, 0.03, 0.0, 0.0, 0.0)


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
    assert dobot._active_tool[0] == 1


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
    assert dobot._current_command_id[0] == 3  # SetTool + Tool + RelMovLTool


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


def _joint_angles(dobot: DobotDashboard) -> list[float]:
    """Read the current joint angles (degrees) via ``GetAngle()``."""
    reply = _send(dobot, "GetAngle()")
    inner = reply[reply.index("{") + 1:reply.index("}")]
    return [float(x) for x in inner.split(",")]


def test_movj_bare_floats_is_cartesian_not_joint_degrees(dobot_cr20a: DobotDashboard) -> None:
    """Regression: ``MovJ(x,y,z,rx,ry,rz)`` is a Cartesian target, not joint angles.

    The target is a *point*; per the interface guide MovJ moves there via joint
    motion. Feeding workspace coordinates (hundreds/thousands of mm) straight in
    as joint degrees produced impossible joint values (>2000°). The command must
    solve IK, so every joint stays within a sane range.
    """
    _send(dobot_cr20a, "EnableRobot()MovJ(-1300,-500,-100,85,-44,12)", expect=2)
    time.sleep(1.3)
    angles = _joint_angles(dobot_cr20a)
    assert all(abs(a) < 360.0 for a in angles), angles


def test_movj_pose_form_is_cartesian(dobot_cr20a: DobotDashboard) -> None:
    """``MovJ(pose={…})`` solves IK for the pose rather than using it as joints."""
    _send(dobot_cr20a, "EnableRobot()MovJ(pose={-1300,-500,-100,85,-44,12})", expect=2)
    time.sleep(1.3)
    angles = _joint_angles(dobot_cr20a)
    assert all(abs(a) < 360.0 for a in angles), angles


def test_movj_joint_form_sets_joint_angles(dobot_cr20a: DobotDashboard) -> None:
    """``MovJ(joint={…})`` still commands joint coordinates directly (degrees)."""
    _send(dobot_cr20a, "EnableRobot()MovJ(joint={10,20,30,40,50,60})", expect=2)
    time.sleep(1.3)
    angles = _joint_angles(dobot_cr20a)
    expected = [10.0, 20.0, 30.0, 40.0, 50.0, 60.0]
    assert all(abs(a - e) < 0.5 for a, e in zip(expected, angles)), angles


def test_dobot_movl_rejects_bad_args(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "MovL(not_valid)")
    assert reply.startswith("-30001,")


def test_dobot_getpose_returns_mm_and_degrees(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetPose()")
    assert reply.startswith("0,")
    assert reply.endswith("GetPose()")
    body = reply.split(",{", 1)[1].rsplit("},", 1)[0]
    vals = [float(v) for v in body.split(",")]
    assert len(vals) == 6
    # At home (0,0,0,0,0,0) with NoOpKinematics pose = (0,0,0,0,0,0).


def test_dobot_getpose_with_tool_frame(dobot: DobotDashboard) -> None:
    _send(dobot, "SetTool(1,{100,0,0,0,0,0})Tool(1)", expect=2)
    reply = _send(dobot, "GetPose(tool=1)")
    assert reply.startswith("0,{")
    body = reply.split(",{", 1)[1].rsplit("},", 1)[0]
    vals = [float(v) for v in body.split(",")]
    # Tool frame {100,0,0,0,0,0} mm/deg composed onto identity flange → {100,0,0,0,0,0}
    assert vals == pytest.approx([100.0, 0.0, 0.0, 0.0, 0.0, 0.0], abs=1e-4)


def test_dobot_getpose_rejects_bad_tool_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetPose(tool=99)")
    assert reply.startswith("-40001,")


def test_dobot_getpose_rejects_undefined_tool(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetPose(tool=5)")
    assert reply.startswith("-1,")  # frame not defined


def test_dobot_getpose_rejects_bad_user_index(dobot: DobotDashboard) -> None:
    reply = _send(dobot, "GetPose(user=99)")
    assert reply.startswith("-40001,")


# --- injected protective stop ----------------------------------------


def test_robotmode_reports_the_injected_collision(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop()

    assert _send(dobot, "RobotMode()") == "0,{11},RobotMode()"


def test_robotmode_reports_an_injected_disabled_stop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(robot_mode=ROBOT_MODE_DISABLED)

    assert _send(dobot, "RobotMode()") == "0,{4},RobotMode()"


def test_injected_stop_does_not_engage_the_estop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop()

    s = dobot.arm.state.snapshot()
    assert s.faulted
    assert not s.estopped


def test_injected_stop_halts_a_move_in_progress(dobot: DobotDashboard) -> None:
    _send(dobot, "MovJ(10,20,30,40,50,60)")
    assert dobot.arm.state.snapshot().moving

    dobot.inject_protective_stop()
    assert not dobot.arm.state.snapshot().moving


def test_geterrorid_reports_the_injected_alarm_ids(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17, 116])

    assert _send(dobot, "GetErrorID()") == "0,{[17,116]},GetErrorID()"


def test_alarm_ids_can_change_while_the_stop_stays_engaged(dobot: DobotDashboard) -> None:
    """The driver re-derives its alarm text every tick, not only on transition."""
    dobot.inject_protective_stop(controller_ids=[17])
    assert _send(dobot, "GetErrorID()") == "0,{[17]},GetErrorID()"

    dobot.inject_protective_stop(controller_ids=[26])
    assert _send(dobot, "GetErrorID()") == "0,{[26]},GetErrorID()"
    assert _send(dobot, "RobotMode()") == "0,{11},RobotMode()"


def test_clear_protective_stop_returns_the_robot_to_idle(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17])
    dobot.clear_protective_stop()

    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"
    assert _send(dobot, "GetErrorID()") == "0,{[]},GetErrorID()"


def test_a_real_estop_outranks_an_injected_stop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(robot_mode=ROBOT_MODE_DISABLED)
    _send(dobot, "EmergencyStop()")

    assert _send(dobot, "RobotMode()") == "0,{9},RobotMode()"


def test_injecting_a_non_protective_stop_mode_is_rejected(dobot: DobotDashboard) -> None:
    with pytest.raises(ValueError, match="not a protective stop"):
        dobot.inject_protective_stop(robot_mode=5)

    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"


def test_feedback_packet_reports_the_injected_mode() -> None:
    pkt = DobotFeedbackPacket()
    state = ArmStateView(
        joints=(0.0,) * 6, pose=(0.0,) * 6, mode=ArmMode.FAULTED,
        servo_on=True, program_running=False, speed_fraction=1.0,
    )
    fault = _FaultState(active=True, robot_mode=ROBOT_MODE_COLLISION)

    _update_feedback_packet(pkt, state, fault=fault)

    assert pkt.RobotMode == ROBOT_MODE_COLLISION
    assert pkt.ErrorStatus == 1


def test_feedback_packet_ignores_an_inactive_fault() -> None:
    pkt = DobotFeedbackPacket()
    state = ArmStateView(
        joints=(0.0,) * 6, pose=(0.0,) * 6, mode=ArmMode.IDLE,
        servo_on=True, program_running=False, speed_fraction=1.0,
    )

    _update_feedback_packet(pkt, state, fault=_FaultState())

    assert pkt.RobotMode == 5


def test_every_protective_stop_mode_is_injectable(dobot: DobotDashboard) -> None:
    for mode in PROTECTIVE_STOP_MODES:
        dobot.inject_protective_stop(robot_mode=mode)
        assert _send(dobot, "RobotMode()") == f"0,{{{mode}}},RobotMode()"


# --- a protective stop refuses motion, and a sticky one survives ClearError ---


def test_movj_is_refused_while_the_stop_is_engaged(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop()

    reply = _send(dobot, "MovJ(10,20,30,40,50,60)")
    assert reply == f"{ERR_ROBOT_IN_ERROR_STATE},{{}},MovJ(10,20,30,40,50,60)"
    assert not dobot.arm.state.snapshot().moving


def test_movl_is_refused_while_the_stop_is_engaged(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop()

    reply = _send(dobot, "MovL(10,20,30,40,50,60)")
    assert reply.startswith(f"{ERR_ROBOT_IN_ERROR_STATE},")


def test_motion_is_accepted_again_once_the_stop_clears(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop()
    dobot.clear_protective_stop()

    reply = _send(dobot, "MovJ(10,20,30,40,50,60)")
    assert reply.startswith("0,")


def test_reading_commands_still_work_while_stopped(dobot: DobotDashboard) -> None:
    """Only motion is refused -- the driver must still be able to poll."""
    dobot.inject_protective_stop(controller_ids=[17])

    assert _send(dobot, "GetErrorID()") == "0,{[17]},GetErrorID()"
    assert _send(dobot, "GetAngle()").startswith("0,{")


def test_clearerror_releases_a_non_sticky_stop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17])

    _send(dobot, "ClearError()")

    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"
    assert _send(dobot, "GetErrorID()") == "0,{[]},GetErrorID()"


def test_clearerror_does_not_release_a_sticky_stop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17], sticky=True)

    assert _send(dobot, "ClearError()") == "0,{},ClearError()"
    assert _send(dobot, "RobotMode()") == "0,{11},RobotMode()"
    assert _send(dobot, "GetErrorID()") == "0,{[17]},GetErrorID()"


def test_a_sticky_stop_is_still_released_by_clear_protective_stop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17], sticky=True)
    dobot.clear_protective_stop()

    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"


def test_clearerror_still_releases_an_emergency_stop(dobot: DobotDashboard) -> None:
    """Regression: the sticky path must not change plain e-stop recovery."""
    _send(dobot, "EmergencyStop()")

    _send(dobot, "ClearError()")

    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"
    assert _send(dobot, "GetErrorID()") == "0,{[]},GetErrorID()"


# --- an EnableRobot that will not succeed ----------------------------


def test_enablerobot_is_rejected_under_an_error_failure(dobot: DobotDashboard) -> None:
    dobot.set_enable_failure(EnableFailure.ERROR)

    reply = _send(dobot, "EnableRobot()")
    assert reply == f"{ERR_ROBOT_IN_ERROR_STATE},{{}},EnableRobot()"


def test_enablerobot_is_accepted_but_stays_disabled_under_a_stuck_failure(
    dobot: DobotDashboard,
) -> None:
    dobot.set_enable_failure(EnableFailure.STUCK)

    assert _send(dobot, "EnableRobot()") == "0,{},EnableRobot()"
    assert _send(dobot, "RobotMode()") == "0,{4},RobotMode()"


def test_a_stuck_failure_survives_clearerror(dobot: DobotDashboard) -> None:
    """A driver's unlock is ClearError then EnableRobot; both must not help."""
    dobot.set_enable_failure(EnableFailure.STUCK)

    _send(dobot, "ClearError()")
    _send(dobot, "EnableRobot()")

    assert _send(dobot, "RobotMode()") == "0,{4},RobotMode()"


def test_an_engaged_stop_outranks_a_stuck_failure(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop()
    dobot.set_enable_failure(EnableFailure.STUCK)

    assert _send(dobot, "RobotMode()") == "0,{11},RobotMode()"


def test_clearing_the_enable_failure_lets_the_robot_enable_again(
    dobot: DobotDashboard,
) -> None:
    dobot.set_enable_failure(EnableFailure.STUCK)
    dobot.set_enable_failure(None)

    assert _send(dobot, "EnableRobot()") == "0,{},EnableRobot()"
    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"


def test_clear_faults_releases_the_stop_and_the_enable_failure(
    dobot: DobotDashboard,
) -> None:
    dobot.inject_protective_stop(controller_ids=[17], sticky=True)
    dobot.set_enable_failure(EnableFailure.STUCK)

    dobot.clear_faults()

    assert _send(dobot, "RobotMode()") == "0,{5},RobotMode()"
    assert _send(dobot, "GetErrorID()") == "0,{[]},GetErrorID()"


def test_a_sticky_error_stop_keeps_a_driver_enable_loop_failing(
    dobot: DobotDashboard,
) -> None:
    """The combination a failing unlock needs: a mode outside 'enabled'.

    A driver treats COLLISION as already-enabled, so a stop that must defeat
    an enable loop has to report ERROR or DISABLED.
    """
    dobot.inject_protective_stop(robot_mode=ROBOT_MODE_ERROR, controller_ids=[17], sticky=True)
    dobot.set_enable_failure(EnableFailure.STUCK)

    _send(dobot, "ClearError()")
    _send(dobot, "EnableRobot()")

    assert _send(dobot, "RobotMode()") == "0,{9},RobotMode()"


# --- the injected fault is visible in the detail panel ----------------


def _derived(dobot: DobotDashboard) -> dict[str, str]:
    return {f["signal"]: f["value"] for f in dobot.build_detail()["derived_fields"]}


def test_detail_panel_reads_clear_when_nothing_is_injected(dobot: DobotDashboard) -> None:
    fields = _derived(dobot)

    assert fields["pstop"] == "clear"
    assert fields["alarmids"] == "-"
    assert fields["enablefail"] == "-"


def test_detail_panel_names_the_engaged_stop_mode(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17, 116])

    fields = _derived(dobot)
    assert fields["pstop"] == "ENGAGED: COLLISION (11)"
    assert fields["alarmids"] == "17,116"


def test_detail_panel_marks_a_sticky_stop(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(robot_mode=ROBOT_MODE_ERROR, sticky=True)

    assert _derived(dobot)["pstop"] == "ENGAGED: ERROR (9), sticky"


def test_detail_panel_shows_the_enable_failure(dobot: DobotDashboard) -> None:
    dobot.set_enable_failure(EnableFailure.STUCK)
    assert _derived(dobot)["enablefail"] == "stuck (stays DISABLED)"

    dobot.set_enable_failure(EnableFailure.ERROR)
    assert _derived(dobot)["enablefail"] == "error (EnableRobot rejected)"


def test_detail_panel_goes_back_to_clear_after_clear_faults(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17], sticky=True)
    dobot.set_enable_failure(EnableFailure.STUCK)

    dobot.clear_faults()

    assert _derived(dobot)["pstop"] == "clear"
    assert _derived(dobot)["enablefail"] == "-"


# --- injecting a fault announces itself, so views repaint -------------


def _fault_events(dobot: DobotDashboard) -> list[dict]:
    seen: list[dict] = []
    dobot._bus.subscribe(
        lambda event: seen.append(event.payload) if event.kind == "fault" else None
    )
    return seen


def test_injecting_a_stop_emits_a_fault_event(dobot: DobotDashboard) -> None:
    events = _fault_events(dobot)

    dobot.inject_protective_stop(controller_ids=[17, 116])

    assert events == [
        {"state": "engaged", "mode": "COLLISION", "alarms": "17,116", "sticky": False}
    ]


def test_clearing_a_stop_emits_a_fault_event(dobot: DobotDashboard) -> None:
    dobot.inject_protective_stop(controller_ids=[17])
    events = _fault_events(dobot)

    dobot.clear_protective_stop()

    assert events == [{"state": "clear"}]


def test_setting_an_enable_failure_emits_a_fault_event(dobot: DobotDashboard) -> None:
    events = _fault_events(dobot)

    dobot.set_enable_failure(EnableFailure.STUCK)
    dobot.set_enable_failure(None)

    assert events == [{"enable_failure": "stuck"}, {"enable_failure": "none"}]


# --- Modbus masters on the tool flange --------------------------------


def test_modbusrtucreate_returns_the_first_master_index(dobot: DobotDashboard) -> None:
    assert _send(dobot, "ModbusRTUCreate(65,115200,E,1)") == "0,{0},ModbusRTUCreate(65,115200,E,1)"


def test_modbusrtucreate_records_the_slave_id_and_baud(dobot: DobotDashboard) -> None:
    _send(dobot, "ModbusRTUCreate(65,115200,E,1)")

    master = dobot._masters[0]
    assert (master.slave_id, master.baud) == (65, 115200)


def test_modbusrtucreate_keeps_the_ambiguous_serial_args_verbatim(dobot: DobotDashboard) -> None:
    """E,1 is parity then stop bit -- the client drops data_bit when it is 8."""
    _send(dobot, "ModbusRTUCreate(65,115200,E,1)")

    assert dobot._masters[0].serial == "E,1"


def test_modbusrtucreate_works_without_the_optional_serial_args(dobot: DobotDashboard) -> None:
    assert _send(dobot, "ModbusRTUCreate(65,115200)") == "0,{0},ModbusRTUCreate(65,115200)"


def test_each_master_gets_its_own_index(dobot: DobotDashboard) -> None:
    """A Dual Quick Changer's two grippers are two sessions, two indices."""
    first = _send(dobot, "ModbusRTUCreate(65,115200,E,1)")
    second = _send(dobot, "ModbusRTUCreate(66,115200,E,1)")

    assert first.startswith("0,{0}")
    assert second.startswith("0,{1}")
    assert dobot._masters[1].slave_id == 66


def test_a_sixth_master_is_refused(dobot: DobotDashboard) -> None:
    for slave in range(MAX_MODBUS_MASTERS):
        _send(dobot, f"ModbusRTUCreate({65 + slave},115200)")

    reply = _send(dobot, "ModbusRTUCreate(70,115200)")
    assert reply == f"{ERR_COMMAND_FAILED},{{}},ModbusRTUCreate(70,115200)"


def test_modbusrtucreate_rejects_non_numeric_arguments(dobot: DobotDashboard) -> None:
    assert _send(dobot, "ModbusRTUCreate(x,115200)").startswith("-30001,")


def test_modbusrtucreate_rejects_a_missing_baud(dobot: DobotDashboard) -> None:
    assert _send(dobot, "ModbusRTUCreate(65)").startswith("-30001,")


def test_modbusclose_releases_the_master(dobot: DobotDashboard) -> None:
    _send(dobot, "ModbusRTUCreate(65,115200)")

    assert _send(dobot, "ModbusClose(0)") == "0,{},ModbusClose(0)"
    assert dobot._masters == {}


def test_a_released_index_is_handed_out_again(dobot: DobotDashboard) -> None:
    _send(dobot, "ModbusRTUCreate(65,115200)")
    _send(dobot, "ModbusClose(0)")

    assert _send(dobot, "ModbusRTUCreate(66,115200)").startswith("0,{0}")


def test_closing_a_master_that_was_never_created_fails(dobot: DobotDashboard) -> None:
    assert _send(dobot, "ModbusClose(0)") == f"{ERR_COMMAND_FAILED},{{}},ModbusClose(0)"


def test_closing_an_out_of_range_index_fails(dobot: DobotDashboard) -> None:
    assert _send(dobot, "ModbusClose(9)").startswith("-40001,")


def test_a_dobot_starts_with_an_empty_flange(dobot: DobotDashboard) -> None:
    assert dobot.flange.slave_ids == ()
