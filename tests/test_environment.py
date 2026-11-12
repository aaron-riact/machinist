"""The scene: parts in the cell, and what a mounted sensor sees of them."""

from __future__ import annotations

import math
import time
from pathlib import Path
from unittest.mock import patch

import numpy as np
import pytest
import yaml
from pydantic import ValidationError

from machinist.core.config import (
    DeviceConfig,
    IOLink,
    MountConfig,
    PartsConfig,
    SystemConfig,
    load_config,
)
from machinist.core.environment import Box, Cylinder, Scene
from machinist.core.world import WorldBuilder
from machinist.devices.robots import arm as arm_module
from machinist.devices.robots.arm import RobotArm
from machinist.devices.robots.yaskawa import YaskawaRobot
from machinist.devices.sensors.laser import Laser
from machinist.kinematics.units import Radians

DOWN = np.array([0.0, 0.0, -1.0])


def _beam(origin: tuple[float, float, float], direction: np.ndarray) -> np.ndarray:
    """A transform whose +Z axis is *direction*, at *origin*."""
    z = direction / np.linalg.norm(direction)
    x = np.cross([0.0, 1.0, 0.0], z) if abs(z[1]) < 0.9 else np.cross([1.0, 0.0, 0.0], z)
    x /= np.linalg.norm(x)
    beam = np.eye(4)
    beam[:3, 0], beam[:3, 1], beam[:3, 2], beam[:3, 3] = x, np.cross(z, x), z, origin
    return beam


# --- Shapes ---------------------------------------------------------------------

CYLINDER = Cylinder(np.array([1.0, 0.0, 0.0]), radius=0.05, height=0.08)


def test_a_beam_from_above_meets_the_cylinders_top() -> None:
    assert CYLINDER.hit(np.array([1.02, 0.0, 0.5]), DOWN) == pytest.approx(0.42)


def test_a_beam_from_the_side_meets_the_cylinders_wall() -> None:
    assert CYLINDER.hit(np.array([0.0, 0.0, 0.04]), np.array([1.0, 0.0, 0.0])) == \
        pytest.approx(0.95)


def test_a_beam_past_the_cylinder_misses_it() -> None:
    assert CYLINDER.hit(np.array([1.06, 0.0, 0.5]), DOWN) is None
    assert CYLINDER.hit(np.array([0.0, 0.0, 0.1]), np.array([1.0, 0.0, 0.0])) is None


def test_a_beam_from_inside_the_cylinder_meets_it_at_once() -> None:
    assert CYLINDER.hit(np.array([1.0, 0.0, 0.04]), DOWN) == 0.0


def test_a_turned_box_is_met_where_its_turned_sides_are() -> None:
    box = Box(np.array([0.0, 0.0, 0.0]), half_x=0.1, half_y=0.01, height=0.05,
              rz=math.radians(90))
    # Turned a quarter, its long side runs along Y.
    assert box.hit(np.array([0.0, 0.09, 0.3]), DOWN) == pytest.approx(0.25)
    assert box.hit(np.array([0.09, 0.0, 0.3]), DOWN) is None


def test_the_scene_gives_the_nearest_part_along_the_beam() -> None:
    near = Cylinder(np.array([0.0, 0.0, 0.2]), radius=0.05, height=0.1)
    scene = Scene((CYLINDER, near))
    assert scene.distance_along(_beam((0.0, 0.0, 1.0), DOWN)) == pytest.approx(0.7)
    assert scene.distance_along(_beam((0.5, 0.0, 1.0), DOWN)) is None


# --- Parts in the scene ------------------------------------------------------------


def test_a_grid_lays_parts_out_along_its_turned_rows() -> None:
    parts = PartsConfig.model_validate({
        "name": "pallet", "cylinder": {"diameter_mm": 100, "height_mm": 80},
        "grid": {"first_mm": [500, 0, 10], "count": [2, 2], "pitch_mm": [150, 120]},
        "rz_deg": 90,
    })
    # Turned a quarter, the rows run along +Y and the next row lies toward -X.
    assert np.allclose(parts.bottoms_mm(),
                       [(500, 0, 10), (380, 0, 10), (500, 150, 10), (380, 150, 10)])


@pytest.mark.parametrize(("raw", "complaint"), [
    ({"name": "p", "at_mm": [[0, 0, 0]]}, "give one shape"),
    ({"name": "p", "at_mm": [[0, 0, 0]], "cylinder": {"diameter_mm": 1, "height_mm": 1},
      "box": {"size_mm": [1, 1, 1]}}, "give one shape"),
    ({"name": "p", "cylinder": {"diameter_mm": 1, "height_mm": 1}}, "place them"),
    ({"name": "p", "at_mm": [[0, 0, 0]], "box": {"size_mm": [1, 0, 1]}}, "longer than 0"),
])
def test_parts_that_cannot_stand_in_the_cell_are_refused(raw: dict[str, object],
                                                          complaint: str) -> None:
    with pytest.raises(ValidationError, match=complaint):
        PartsConfig.model_validate(raw)


def _tick_at(arm: RobotArm, seconds_from_now: float) -> None:
    later = time.monotonic() + seconds_from_now
    with patch.object(arm_module.time, "monotonic", return_value=later):
        arm._tick()


# --- A laser on an arm ---------------------------------------------------------------

ROBOT = DeviceConfig(name="r1", kind="yaskawa", host="127.0.0.1", port=0, options={
    "controller": "dx200", "robot": "up50n", "start_joints_deg": [0, 0, 0, 0, -90, 0],
    "signals": ["in10"]})
LASER = DeviceConfig(name="laser1", kind="laser", options={"switches": {"q1": 350}})


def _flange() -> np.ndarray:
    world = WorldBuilder().build(SystemConfig(devices=(ROBOT,)))
    robot = world.devices[0]
    assert isinstance(robot, YaskawaRobot)
    return robot.mount_transform()


def _cell(*, part_below_flange_mm: float) -> tuple[YaskawaRobot, Laser]:
    """The arm with the laser on its flange, its beam straight down onto a part."""
    flange = _flange()
    assert np.allclose(flange[:3, 2], DOWN, atol=1e-6)
    x, y, z = flange[:3, 3] * 1000
    world = WorldBuilder().build(SystemConfig(
        devices=(ROBOT, LASER),
        io_links=(IOLink(source="laser1.q1", target="r1.in10"),),
        parts=(PartsConfig(name="part", cylinder={"diameter_mm": 100, "height_mm": 80},
                           at_mm=((x, y, z - part_below_flange_mm - 80),)),),
        mounts=(MountConfig(device="laser1", carrier="r1", at_mm_deg=(0, 0, 0, 0, 0, 0)),),
    ))
    robot, laser = world.devices
    assert isinstance(robot, YaskawaRobot)
    assert isinstance(laser, Laser)
    laser.io["on"].set(True)
    return robot, laser


def test_a_laser_on_the_flange_sees_the_part_below_it_and_the_robot_hears() -> None:
    robot, laser = _cell(part_below_flange_mm=300)
    assert laser.io["q1"].value is True
    assert robot.controller.io[20] == 0b10


def test_the_laser_stops_seeing_the_part_when_the_arm_moves_away() -> None:
    robot, laser = _cell(part_below_flange_mm=300)
    # Turned 30 degrees about S, the flange is far from the part.
    robot.arm.movej(tuple(Radians(math.radians(a)) for a in (30, 0, 0, 0, -90, 0)),
                    duration=0.01)
    _tick_at(robot.arm, 1.0)
    assert laser.io["q1"].value is False
    assert robot.controller.io[20] == 0


def test_a_fixed_laser_sees_the_part_under_it_once() -> None:
    world = WorldBuilder().build(SystemConfig(
        devices=(LASER,),
        parts=(PartsConfig(name="part", box={"size_mm": [100, 100, 100]}, at_mm=((0, 0, 0),)),),
        mounts=(MountConfig(device="laser1", at_mm_deg=(0, 0, 300, 180, 0, 0)),),
    ))
    laser = world.devices[0]
    assert isinstance(laser, Laser)
    laser.io["on"].set(True)
    assert laser.io["q1"].value is True


@pytest.mark.parametrize(("mount", "error", "complaint"), [
    (MountConfig(device="nobody", at_mm_deg=(0,) * 6), KeyError, "unknown device 'nobody'"),
    (MountConfig(device="r1", at_mm_deg=(0,) * 6), TypeError, "senses no distance"),
    (MountConfig(device="laser1", carrier="nobody", at_mm_deg=(0,) * 6), KeyError,
     "unknown device 'nobody'"),
    (MountConfig(device="laser1", carrier="laser1", at_mm_deg=(0,) * 6), TypeError,
     "no mount point"),
])
def test_a_mount_that_cannot_be_made_is_refused(mount: MountConfig, error: type[Exception],
                                                complaint: str) -> None:
    with pytest.raises(error, match=complaint):
        WorldBuilder().build(SystemConfig(devices=(ROBOT, LASER), mounts=(mount,)))


def test_a_scene_file_mounts_a_laser_on_an_arm(tmp_path: Path) -> None:
    scene = tmp_path / "scene.yaml"
    scene.write_text(yaml.safe_dump({
        "devices": [ROBOT.model_dump(), LASER.model_dump()],
        "mounts": [{"device": "laser1", "carrier": "r1", "at_mm_deg": [0, 0, 0, 0, 0, 0]}],
        "parts": [{"name": "p", "box": {"size_mm": [10, 10, 10]}, "at_mm": [[0, 0, 0]]}],
    }))
    config = load_config([scene])
    assert config.mounts == (MountConfig(device="laser1", carrier="r1", at_mm_deg=(0,) * 6),)
    assert len(WorldBuilder().build(config).scene.solids) == 1
