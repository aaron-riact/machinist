"""What is mounted on an arm hears it move, and may stop it while it does."""

from __future__ import annotations

import time
from unittest.mock import patch

import numpy as np

from machinist.core.capabilities import HasMount
from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots import arm as arm_module
from machinist.devices.robots.arm import ArmMode, RobotArm
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.kinematics.api import Pose
from machinist.kinematics.dh_backend import pose_to_mat
from machinist.kinematics.units import Radians

TARGET = (Radians(0.5), Radians(0.0), Radians(0.0))


def _tick_at(arm: RobotArm, seconds_from_now: float) -> None:
    later = time.monotonic() + seconds_from_now
    with patch.object(arm_module.time, "monotonic", return_value=later):
        arm._tick()


def test_each_tick_of_a_move_tells_the_new_pose() -> None:
    arm = RobotArm(joint_count=3)
    poses: list[Pose] = []
    arm.on_moved(poses.append)
    arm.movej(TARGET, duration=1.0)
    _tick_at(arm, 0.5)
    _tick_at(arm, 2.0)
    halfway = arm.forward((Radians(0.25), Radians(0.0), Radians(0.0)))
    assert len(poses) == 2
    assert np.allclose(poses[0], halfway, atol=1e-4)
    assert poses[1] == arm.forward(TARGET)


def test_a_still_arm_tells_nothing() -> None:
    arm = RobotArm(joint_count=3)
    poses: list[Pose] = []
    arm.on_moved(poses.append)
    arm._tick()
    assert poses == []


def test_a_jog_step_tells_the_new_pose() -> None:
    arm = RobotArm(joint_count=3)
    poses: list[Pose] = []
    arm.on_moved(poses.append)
    arm.jog_cartesian(np.array([0.01, 0, 0, 0, 0, 0]))
    assert poses == [arm.state.view.pose]


def test_a_listener_may_stop_the_arm_it_hears() -> None:
    arm = RobotArm(joint_count=3)
    arm.on_moved(lambda _: arm.stop())
    arm.movej(TARGET, duration=1.0)
    _tick_at(arm, 0.5)
    assert arm.state.view.mode is ArmMode.IDLE


def test_an_arm_device_is_a_mount_at_its_pose() -> None:
    options = YaskawaOptions(controller="dx200", robot="up50n")
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options)
    assert isinstance(robot, HasMount)
    assert np.allclose(robot.mount_transform(), pose_to_mat(robot.arm.state.view.pose))
    heard: list[np.ndarray] = []
    robot.on_mount_moved(heard.append)
    robot.arm.movej(tuple(Radians(0.1) for _ in range(6)), duration=1.0)
    _tick_at(robot.arm, 2.0)
    assert len(heard) == 1
    assert np.allclose(heard[0], robot.mount_transform())
