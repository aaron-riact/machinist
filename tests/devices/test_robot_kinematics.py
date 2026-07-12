"""Smoke test: robots accept YAML kinematics options."""

from __future__ import annotations

import math
import time

import pytest

from machinist.kinematics.units import Radians

pytest.importorskip("numpy")

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions, DHParams, KinematicsOptions, arm_from_options
from machinist.devices.robots.ur import URDashboardServer


def _ur5_dh() -> DHParams:
    return DHParams(
        a=(0, -0.425, -0.3922, 0, 0, 0),
        d=(0.089159, 0, 0, 0.10915, 0.09465, 0.0823),
        alpha=(math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0),
    )


def test_robot_uses_configured_kinematics_backend() -> None:
    bus = EventBus()
    options = ArmOptions(
        kinematics=KinematicsOptions(
            backend="dh",
            dh=_ur5_dh(),
        ),
    )
    ur = URDashboardServer("ur1", Endpoint("127.0.0.1", 0), bus, options)
    try:
        pose = ur.arm._kinematics.forward(tuple(Radians(0.0) for _ in range(6)))  # noqa: SLF001
        # Non-identity pose because DH parameters are substantial.
        assert any(abs(x) > 0.01 for x in pose[:3])
    finally:
        ur.arm.stop_ticker()


def test_arm_accepts_top_level_dh_options() -> None:
    arm = arm_from_options(
        ArmOptions(
            joint_count=6,
            dh_params=_ur5_dh(),
        )
    )
    try:
        home = arm.state.snapshot()
        assert any(abs(value) > 0.01 for value in home.pose[:3])
        arm.start_ticker()
        arm.movej(tuple(Radians(v) for v in (0.1, -0.4, 0.2, -0.1, 0.3, 0.2)), duration=0.01)
        time.sleep(0.03)
        moved = arm.state.snapshot()
        assert moved.pose != moved.joints[:6]
    finally:
        arm.stop_ticker()
