"""Every change to an arm is published as an ArmChanged carrying the new view."""

from __future__ import annotations

import dataclasses
import time

import pytest

from machinist.core.events import Event
from machinist.devices.robots.arm import ArmChanged, ArmMode, RobotArm
from machinist.kinematics.units import Radians


def _recording_arm(joint_count: int = 3) -> tuple[RobotArm, list[ArmChanged]]:
    events: list[Event] = []
    arm = RobotArm(joint_count=joint_count, owner="ur1", publish=events.append)
    return arm, events  # type: ignore[return-value]


def test_commands_publish_the_new_view() -> None:
    arm, events = _recording_arm()
    arm.set_servo(False)
    arm.estop()

    assert [e.view.servo_on for e in events] == [False, False]
    assert events[-1].view.mode is ArmMode.ESTOPPED
    assert events[-1].device == "ur1"
    assert all(isinstance(e, ArmChanged) for e in events)


def test_a_command_that_changes_nothing_is_silent() -> None:
    arm, events = _recording_arm()
    arm.set_servo(True)  # already on
    arm.reset()  # not e-stopped
    assert events == []


def test_the_view_cannot_be_written_directly() -> None:
    arm, _ = _recording_arm()
    with pytest.raises(dataclasses.FrozenInstanceError):
        arm.state.view.servo_on = False  # type: ignore[misc]


def test_a_move_publishes_its_progress_and_completion() -> None:
    arm, events = _recording_arm()
    arm.start_ticker()
    try:
        arm.movej((Radians(0.5), Radians(0.0), Radians(0.0)), duration=0.05)
        time.sleep(0.2)
    finally:
        arm.stop_ticker()

    assert events[0].view.mode is ArmMode.MOVING and events[0].view.current_command == "movej"
    assert len(events) > 2, "each tick that moved the arm was announced"
    final = events[-1].view
    assert final.mode is ArmMode.IDLE and final.current_command is None
    assert final.joints[0] == Radians(0.5)
    assert arm.state.view is final
