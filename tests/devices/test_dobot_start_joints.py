"""A Dobot starts in its natural pose, unless the scene names other joints."""
from __future__ import annotations

import math

import pytest

from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.robots.dobot import DobotDashboard


def _scene_dobot(**options: object) -> DobotDashboard:
    device = default_registry.create(
        "dobot_dashboard", "d", Endpoint("127.0.0.1", 0), EventBus(),
        {"robot_type": "cr20a", "feedback_ports": False, "flange_gateway_ports": False, **options},
    )
    assert isinstance(device, DobotDashboard)
    return device


def _degrees(dobot: DobotDashboard) -> list[float]:
    return [math.degrees(j) for j in dobot.arm.state.view.joints]


def test_a_dobot_starts_in_its_natural_pose() -> None:
    assert _degrees(_scene_dobot()) == pytest.approx([0, 0, 90, 0, -90, 0])


def test_a_scene_can_start_a_dobot_elsewhere() -> None:
    dobot = _scene_dobot(start_joints_deg=[170, 36, -78, 31, 94, -84])
    assert _degrees(dobot) == pytest.approx([170, 36, -78, 31, 94, -84])
