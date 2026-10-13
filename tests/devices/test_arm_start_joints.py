"""An arm starts at all-zero joints unless its options name others."""
from __future__ import annotations

import math

import pytest

from machinist.devices.robots.arm import ArmOptions, RobotArm, arm_from_options


def test_an_arm_starts_at_zero_by_default() -> None:
    assert arm_from_options(ArmOptions()).state.view.joints == (0.0,) * 6


def test_an_arm_starts_at_the_joints_its_options_name() -> None:
    arm = arm_from_options(ArmOptions(start_joints_deg=(10, 20, 30, 40, 50, 60)))
    assert arm.state.view.joints == pytest.approx([math.radians(d) for d in (10, 20, 30, 40, 50, 60)])


def test_start_joints_for_another_joint_count_are_refused() -> None:
    with pytest.raises(ValueError):
        RobotArm(joint_count=6, start_joints=(0.0,) * 5)  # type: ignore[arg-type]
