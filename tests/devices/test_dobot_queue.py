"""The Dobot runs moves and settings from a queue, in the order they were sent."""
from __future__ import annotations

import math
from unittest.mock import patch

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots import arm as arm_module
from machinist.devices.robots import dobot as dobot_module
from machinist.devices.robots.arm import ArmMode, ArmOptions
from machinist.devices.robots.dobot import DOBOT_ROBOT_MODELS, DobotDashboard, DobotFeedbackPacket, DobotOptions
from machinist.transport.flange_bus import FlangeBus

FIRST = "MovJ(joint={10,0,0,0,0,0})"
SECOND = "MovJ(joint={20,0,0,0,0,0})"


@pytest.fixture
def dobot() -> DobotDashboard:
    """A Dobot that is never started: no server, no feedback stream, no arm ticker.

    Commands go straight to ``handle_line``, and a test finishes a move with
    :func:`_finish_move` instead of waiting for one.
    """
    d = DobotDashboard("dobot1", Endpoint("127.0.0.1", 0), EventBus(), ArmOptions(),
                       flange=FlangeBus(), feedback_enabled=False)
    d.handle_line("EnableRobot()")
    return d


def _finish_move(dobot: DobotDashboard) -> None:
    """One arm tick, late enough for the move under way to reach its target."""
    move = dobot.arm._move
    assert move is not None, "no move under way"
    done_at = move.started_at + move.duration + 1
    with patch.object(arm_module.time, "monotonic", return_value=done_at):
        dobot.arm._tick()


def _reported_id(dobot: DobotDashboard) -> int:
    pkt = DobotFeedbackPacket()
    dobot.fill_feedback(pkt)
    return pkt.CurrentCommandId


def _value(reply: str) -> int:
    return int(reply.split("{", 1)[1].split("}", 1)[0])


def test_each_command_gets_the_next_queue_id(dobot: DobotDashboard) -> None:
    assert [_value(dobot.handle_line(FIRST)), _value(dobot.handle_line(SECOND))] == [1, 2]


def test_a_move_waits_for_the_one_ahead_of_it(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    _finish_move(dobot)
    assert dobot.arm.state.view.joints[0] == pytest.approx(0.1745, abs=1e-3)  # 10 degrees
    assert dobot.arm.state.view.mode is ArmMode.MOVING  # and the second one has started


def test_the_feedback_reports_the_command_running(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    assert _reported_id(dobot) == 1


def test_the_feedback_moves_on_when_the_next_command_starts(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    _finish_move(dobot)
    assert _reported_id(dobot) == 2


def test_the_robot_is_idle_once_the_queue_is_done(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    _finish_move(dobot)
    _finish_move(dobot)
    assert dobot.handle_line("RobotMode()") == "0,{5},RobotMode()"


def test_a_setting_behind_a_move_takes_effect_after_it(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line("SetPayload(2.5)")
    assert dobot._payload[0] == 0.0
    _finish_move(dobot)
    assert dobot._payload[0] == 2.5


def test_a_setting_on_an_idle_robot_takes_effect_at_once(dobot: DobotDashboard) -> None:
    reply = dobot.handle_line("SetPayload(2.5)")
    assert dobot._payload[0] == 2.5
    assert _reported_id(dobot) == _value(reply)


def test_a_relative_joint_move_starts_from_where_the_move_ahead_ends(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line("RelJointMovJ(5,0,0,0,0,0)")
    _finish_move(dobot)
    _finish_move(dobot)
    assert dobot.arm.state.view.joints[0] == pytest.approx(0.2618, abs=1e-3)  # 10 + 5 degrees


def test_stop_drops_the_commands_still_queued(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    dobot.handle_line("Stop()")
    dobot.arm._tick()
    assert dobot.arm.state.view.mode is ArmMode.IDLE  # the second move never started


def test_a_move_sent_after_stop_runs(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line("Stop()")
    dobot.handle_line(SECOND)
    assert dobot.arm.state.view.mode is ArmMode.MOVING


def test_after_stop_the_feedback_reports_command_one(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    _finish_move(dobot)
    assert _reported_id(dobot) == 2  # the second move is running
    dobot.handle_line("Stop()")
    assert _reported_id(dobot) == 1


def test_after_stop_the_next_command_gets_id_two(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line(SECOND)
    dobot.handle_line("SetPayload(2.5)")
    dobot.handle_line("Stop()")
    assert _value(dobot.handle_line(FIRST)) == 2


def _slow_stopping_dobot() -> DobotDashboard:
    d = DobotDashboard("dobot1", Endpoint("127.0.0.1", 0), EventBus(), ArmOptions(),
                       flange=FlangeBus(), feedback_enabled=False, stop_seconds=0.6)
    d.handle_line("EnableRobot()")
    return d


def test_stop_replies_once_a_moving_arm_has_stopped() -> None:
    dobot = _slow_stopping_dobot()
    dobot.handle_line(FIRST)
    with patch.object(dobot_module.time, "sleep") as sleep:
        dobot.handle_line("Stop()")
    sleep.assert_called_once_with(0.6)


def test_stop_on_a_still_arm_replies_at_once() -> None:
    dobot = _slow_stopping_dobot()
    with patch.object(dobot_module.time, "sleep") as sleep:
        dobot.handle_line("Stop()")
    sleep.assert_not_called()


def test_a_scene_dobot_takes_as_long_to_stop_as_a_cr20a() -> None:
    assert DobotOptions().stop_seconds == 0.6


def test_each_reply_waits_the_configured_round_trip() -> None:
    dobot = DobotDashboard("dobot1", Endpoint("127.0.0.1", 0), EventBus(), ArmOptions(),
                           flange=FlangeBus(), feedback_enabled=False, reply_seconds=0.01)
    with patch.object(dobot_module.time, "sleep") as sleep:
        dobot.handle_line("RobotMode()")
    sleep.assert_called_once_with(0.01)


def test_replies_do_not_wait_unless_configured(dobot: DobotDashboard) -> None:
    with patch.object(dobot_module.time, "sleep") as sleep:
        dobot.handle_line("RobotMode()")
    sleep.assert_not_called()


SERVO = "ServoP(100,200,300,0,0,0,t=0.03)"


def test_servop_queues_behind_the_move_ahead_of_it(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    assert _value(dobot.handle_line(SERVO)) == 2
    assert _reported_id(dobot) == 1


def test_servop_moves_the_arm_to_its_point(dobot: DobotDashboard) -> None:
    dobot.handle_line("ServoP(100,200,300,90,0,0,t=0.03)")
    assert dobot.arm._move is not None
    assert dobot.arm._move.requested_pose == pytest.approx((0.1, 0.2, 0.3, 1.5708, 0, 0), abs=1e-4)


def test_servop_takes_its_run_time_whatever_the_speed_factor(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(10)")
    dobot.handle_line(SERVO)
    assert dobot.arm._move is not None
    assert dobot.arm._move.duration == pytest.approx(0.03)


def test_servop_without_a_run_time_takes_the_default(dobot: DobotDashboard) -> None:
    dobot.handle_line("ServoP(100,200,300,0,0,0)")
    assert dobot.arm._move is not None
    assert dobot.arm._move.duration == pytest.approx(0.1)


def test_servop_rejects_a_run_time_out_of_range(dobot: DobotDashboard) -> None:
    assert dobot.handle_line("ServoP(100,200,300,0,0,0,t=0.001)").startswith("-40001,")


def _move_seconds(dobot: DobotDashboard) -> float:
    assert dobot.arm._move is not None, "no move under way"
    return dobot.arm._move.duration


def test_relmovltool_waits_for_the_move_ahead_of_it(dobot: DobotDashboard) -> None:
    dobot.handle_line(FIRST)
    dobot.handle_line("RelMovLTool(10,0,0,0,0,0)")
    assert _reported_id(dobot) == 1


def test_relmovltool_takes_the_time_its_distance_needs_at_v(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(100)")
    dobot.handle_line("RelMovLTool(20,0,0,0,0,0,v=50)")
    assert _move_seconds(dobot) == pytest.approx(0.02)  # 20 mm at 50% of 2000 mm/s


def test_relmovltool_without_v_runs_at_full_ratio(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(100)")
    dobot.handle_line("RelMovLTool(20,0,0,0,0,0)")
    assert _move_seconds(dobot) == pytest.approx(0.01)


def test_relmovltool_is_slowed_by_the_global_speed_ratio(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(20)")
    dobot.handle_line("RelMovLTool(20,0,0,0,0,0,v=50)")
    assert _move_seconds(dobot) == pytest.approx(0.1)


def test_relmovltool_at_an_absolute_speed_ignores_the_global_ratio(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(20)")
    dobot.handle_line("RelMovLTool(20,0,0,0,0,0,speed=100)")
    assert _move_seconds(dobot) == pytest.approx(0.2)  # 20 mm at 100 mm/s


def test_relmovltool_takes_the_time_its_rotation_needs(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(100)")
    dobot.handle_line("RelMovLTool(0,0,0,0,0,90)")
    assert _move_seconds(dobot) == pytest.approx(0.5)  # 90 degrees at 180 degrees/s


def test_relmovltool_moves_along_the_tool_axes(dobot: DobotDashboard) -> None:
    dobot.handle_line("SetTool(1,{0,0,0,0,0,90})")
    dobot.handle_line("Tool(1)")
    dobot.handle_line("RelMovLTool(10,0,0,0,0,0)")
    assert dobot.arm._move is not None
    # the tool's x axis is the base's y axis
    assert dobot.arm._move.requested_pose[:3] == pytest.approx((0, 0.01, 0), abs=1e-9)


def test_movl_takes_the_time_its_distance_needs_at_v(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(100)")
    start = dobot.arm.state.view.pose
    x, y, z = (round(c * 1000 + d, 6) for c, d in zip(start[:3], (40, 0, 0), strict=True))
    rx, ry, rz = (math.degrees(a) for a in start[3:])
    dobot.handle_line(f"MovL(pose={{{x},{y},{z},{rx},{ry},{rz}}},v=50)")
    assert _move_seconds(dobot) == pytest.approx(0.04)  # 40 mm at 50% of 2000 mm/s


def test_reljointmovj_takes_the_time_its_slowest_joint_needs(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(100)")
    dobot.handle_line("RelJointMovJ(60,0,0,90,0,0,v=50)")
    assert _move_seconds(dobot) == pytest.approx(1.0)  # J1: 60 degrees at 50% of 120 degrees/s


def test_movj_to_joints_is_slowed_by_the_global_speed_ratio(dobot: DobotDashboard) -> None:
    dobot.handle_line("SpeedFactor(50)")
    dobot.handle_line("MovJ(joint={0,0,0,0,0,90})")
    assert _move_seconds(dobot) == pytest.approx(1.0)  # J6: 90 degrees at 50% of 180 degrees/s


@pytest.mark.parametrize("verb", ["RelMovLTool(5,0,0,0,0,0", "RelJointMovJ(5,0,0,0,0,0"])
def test_a_move_with_v_below_one_percent_is_refused(dobot: DobotDashboard, verb: str) -> None:
    assert dobot.handle_line(f"{verb},v=0)").startswith("-40001,")
    assert dobot.arm._move is None


@pytest.fixture
def cr20a() -> DobotDashboard:
    """A CR20A with its real geometry, so moves exercise inverse kinematics."""
    d = DobotDashboard("cr20a", Endpoint("127.0.0.1", 0), EventBus(),
                       ArmOptions(backend="dh", dh_params=DOBOT_ROBOT_MODELS["cr20a"].dh_params),
                       flange=FlangeBus(), feedback_enabled=False, model_info=DOBOT_ROBOT_MODELS["cr20a"])
    d.handle_line("EnableRobot()")
    return d


#: Where a jog on the robot met the wrist singularity: J5 at -0.18 degrees.
WRIST_STRAIGHT = "MovJ(joint={-181.064,20.8947,120.0009,-31.6578,-0.1807,-89.3492})"


#: The servo points a jog sent from WRIST_STRAIGHT (x, z in mm), up to where it met the singularity.
JOG_POINTS = [(892.04, 379.44), (892.33, 380.20), (892.79, 381.39), (893.51, 383.26), (894.31, 385.36),
              (895.29, 387.90), (896.43, 390.87), (897.91, 394.74), (899.62, 399.18), (901.26, 403.45),
              (902.89, 407.70), (904.52, 411.95), (906.43, 416.91)]


def _servo_point(x: float, z: float) -> str:
    return f"ServoP({x},295.587353,{z},-90.181381,19.888580,-1.066193,t=0.030303)"


def _jog_from_wrist_straight(dobot: DobotDashboard) -> None:
    dobot.handle_line(WRIST_STRAIGHT)
    _finish_move(dobot)
    for x, z in JOG_POINTS:
        dobot.handle_line(_servo_point(x, z))
        if dobot.arm._move is None:
            return
        _finish_move(dobot)


def test_a_servo_point_near_the_wrist_singularity_moves(cr20a: DobotDashboard) -> None:
    cr20a.handle_line(WRIST_STRAIGHT)
    _finish_move(cr20a)
    cr20a.handle_line(_servo_point(*JOG_POINTS[0]))
    assert cr20a.arm.state.view.mode is ArmMode.MOVING


def test_a_jog_into_the_wrist_singularity_raises_the_ik_alarm(cr20a: DobotDashboard) -> None:
    _jog_from_wrist_straight(cr20a)
    assert cr20a.handle_line("GetErrorID()").startswith("0,{[2]}")


def test_a_jog_into_the_wrist_singularity_does_not_flip_the_wrist(cr20a: DobotDashboard) -> None:
    _jog_from_wrist_straight(cr20a)
    assert math.degrees(cr20a.arm.state.view.joints[4]) == pytest.approx(-0.18, abs=1)
