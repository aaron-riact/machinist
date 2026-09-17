"""OnRobot 3FG25: registers drive the fingers, and every change announces the panel."""

from __future__ import annotations

import time

from machinist.core.events import EventBus
from machinist.core.panel import PanelChanged
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.grippers.onrobot_3fg25 import (
    REG_CONTROL,
    REG_RAW_DIAMETER,
    REG_STATUS,
    REG_TARGET_DIAMETER,
    STATUS_BUSY,
    OnRobot3FG25,
)

from ..conftest import free_port


def _gripper(bus: EventBus | None = None, **options: object) -> OnRobot3FG25:
    device = default_registry.create(
        "onrobot_3fg25", "g1", Endpoint("127.0.0.1", free_port()), bus or EventBus(), options
    )
    assert isinstance(device, OnRobot3FG25)
    return device


def _settle(gripper: OnRobot3FG25, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not gripper.register_port.read(REG_STATUS, 1)[0] & STATUS_BUSY:
            return
        time.sleep(0.01)
    raise AssertionError("gripper never settled")


def test_starts_near_the_configured_diameter() -> None:
    gripper = _gripper(initial_diameter_mm=75.0)
    raw = gripper.register_port.read(REG_RAW_DIAMETER, 1)[0]
    assert abs(raw - 750) <= 2  # tenths of a mm, via the finger-angle geometry


def test_a_grip_moves_the_fingers_to_the_target_and_settles() -> None:
    gripper = _gripper(initial_diameter_mm=75.0)
    gripper.register_port.write(REG_TARGET_DIAMETER, [500])
    gripper.register_port.write(REG_CONTROL, [1])
    _settle(gripper)
    raw = gripper.register_port.read(REG_RAW_DIAMETER, 1)[0]
    assert abs(raw - 500) <= 2


def test_every_state_change_announces_the_panel() -> None:
    bus = EventBus()
    panels: list[PanelChanged] = []
    bus.subscribe(lambda e: panels.append(e) if isinstance(e, PanelChanged) else None)
    gripper = _gripper(bus, initial_diameter_mm=75.0)

    gripper.register_port.write(REG_TARGET_DIAMETER, [500])
    gripper.register_port.write(REG_CONTROL, [1])
    _settle(gripper)

    assert panels
    final = {f.signal: f.value for f in panels[-1].panel.status_fields}
    assert final["BUSY"] == "0"
    assert abs(float(final["DIAMETER"]) - 50.0) <= 0.2
