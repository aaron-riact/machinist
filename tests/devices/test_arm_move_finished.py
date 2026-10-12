"""A robot can hear when its arm finishes a move, to start the next one."""
from __future__ import annotations

import time
from unittest.mock import Mock, patch

from machinist.devices.robots import arm as arm_module
from machinist.devices.robots.arm import ArmMode, RobotArm
from machinist.kinematics.units import Radians

TARGET = (Radians(0.5), Radians(0.0), Radians(0.0))


def _arm_with_listener() -> tuple[RobotArm, Mock]:
    arm = RobotArm(joint_count=3)
    listener = Mock()
    arm.on_move_finished(listener)
    return arm, listener


def _tick_at(arm: RobotArm, seconds_from_now: float) -> None:
    """One physics tick, as if *seconds_from_now* had passed."""
    later = time.monotonic() + seconds_from_now
    with patch.object(arm_module.time, "monotonic", return_value=later):
        arm._tick()


def test_a_move_that_runs_to_its_end_tells_the_listeners() -> None:
    arm, listener = _arm_with_listener()
    arm.movej(TARGET, duration=1.0)
    _tick_at(arm, 2.0)
    listener.assert_called_once_with()


def test_a_move_still_under_way_does_not() -> None:
    arm, listener = _arm_with_listener()
    arm.movej(TARGET, duration=1.0)
    _tick_at(arm, 0.5)
    listener.assert_not_called()


def test_a_stopped_move_does_not() -> None:
    arm, listener = _arm_with_listener()
    arm.movej(TARGET, duration=1.0)
    arm.stop()
    _tick_at(arm, 2.0)
    listener.assert_not_called()


def test_a_listener_can_start_the_next_move() -> None:
    arm = RobotArm(joint_count=3)
    arm.on_move_finished(lambda: arm.movej((Radians(0.0),) * 3, duration=1.0))
    arm.movej(TARGET, duration=1.0)
    _tick_at(arm, 2.0)  # would deadlock if the listener ran under the arm's lock
    assert arm.state.view.mode is ArmMode.MOVING
