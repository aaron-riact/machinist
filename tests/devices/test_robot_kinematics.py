"""Smoke test: robots accept YAML kinematics options."""

from __future__ import annotations

import math
import time

import pytest

pytest.importorskip("numpy")

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions, arm_from_options
from machinist.devices.robots.ur import URDashboardServer


def test_robot_uses_configured_kinematics_backend() -> None:
    bus = EventBus()
    options = ArmOptions(
        kinematics={
            "backend": "dh",
            "dh_params": {
                "a": [0, -0.425, -0.3922, 0, 0, 0],
                "d": [0.089159, 0, 0, 0.10915, 0.09465, 0.0823],
                "alpha": [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0],
            },
        },
    )
    ur = URDashboardServer("ur1", Endpoint("127.0.0.1", 0), bus, options=options)
    try:
        pose = ur.arm._kinematics.forward((0.0,) * 6)  # noqa: SLF001
        # Non-identity pose because DH parameters are substantial.
        assert any(abs(x) > 0.01 for x in pose[:3])
    finally:
        ur.arm.stop_ticker()


def test_arm_accepts_top_level_dh_options() -> None:
    arm = arm_from_options(
        ArmOptions(
            joint_count=6,
            dh_params={
                "a": [0, -0.425, -0.3922, 0, 0, 0],
                "d": [0.089159, 0, 0, 0.10915, 0.09465, 0.0823],
                "alpha": [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0],
            },
        )
    )
    try:
        home = arm.state.snapshot()
        assert any(abs(value) > 0.01 for value in home.pose[:3])
        arm.start_ticker()
        arm.movej((0.1, -0.4, 0.2, -0.1, 0.3, 0.2), duration=0.01)
        time.sleep(0.03)
        moved = arm.state.snapshot()
        assert moved.pose != moved.joints[:6]
    finally:
        arm.stop_ticker()
