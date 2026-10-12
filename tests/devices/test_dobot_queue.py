"""The Dobot runs moves and settings from a queue, in the order they were sent."""
from __future__ import annotations

from unittest.mock import patch

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots import arm as arm_module
from machinist.devices.robots.arm import ArmMode, ArmOptions
from machinist.devices.robots.dobot import DobotDashboard, DobotFeedbackPacket
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
