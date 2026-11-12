"""A straight move keeps the tool point on its line all the way, not only at its ends."""

from __future__ import annotations

import time
from unittest.mock import patch

import numpy as np
import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots import arm as arm_module
from machinist.devices.robots.arm import RobotArm
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.kinematics.api import Pose
from machinist.kinematics.dh_backend import pose_to_mat
from machinist.kinematics.units import Meters


def _tick_at(arm: RobotArm, seconds_from_now: float) -> None:
    later = time.monotonic() + seconds_from_now
    with patch.object(arm_module.time, "monotonic", return_value=later):
        arm._tick()


@pytest.fixture
def arm() -> RobotArm:
    options = YaskawaOptions(controller="dx200", robot="up50n",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    return YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options).arm


@pytest.mark.parametrize("fraction", [0.25, 0.5, 0.8])
def test_part_way_through_a_straight_move_the_tool_point_is_on_the_line(
        arm: RobotArm, fraction: float) -> None:
    start = np.array(arm.state.view.pose)
    target: Pose = (Meters(start[0] + 0.6), Meters(start[1] + 0.4), *arm.state.view.pose[2:])
    arm.movel(target, duration=1.0)
    _tick_at(arm, fraction)
    here = np.array(arm.state.view.pose)
    expected = start[:3] + fraction * (np.array(target[:3]) - start[:3])
    assert np.linalg.norm(here[:3] - expected) < 0.002
    turned = pose_to_mat(arm.state.view.pose)[:3, :3]
    assert np.allclose(turned, pose_to_mat(tuple(start))[:3, :3], atol=1e-3)


def test_a_straight_move_ends_at_its_target(arm: RobotArm) -> None:
    start = arm.state.view.pose
    target: Pose = (Meters(start[0] - 0.2), start[1], Meters(start[2] + 0.1), *start[3:])
    arm.movel(target, duration=1.0)
    _tick_at(arm, 2.0)
    assert np.allclose(arm.state.view.pose[:3], target[:3], atol=1e-4)
