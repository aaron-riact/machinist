"""Searches: a move that stops on its input, and records where ($PX002, $PX003, $B002)."""

from __future__ import annotations

from collections.abc import Callable, Iterator

import numpy as np
import pytest

from machinist.core.config import DeviceConfig, IOLink, MountConfig, PartsConfig, SystemConfig
from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.core.world import WorldBuilder
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.controller import YaskawaController
from machinist.devices.robots.yaskawa.expression import Literal, Variable
from machinist.devices.robots.yaskawa.inform import (
    IncrementalMove,
    Move,
    PositionRef,
    Watch,
    parse_instruction,
)
from machinist.devices.robots.yaskawa.positions import DataType, Position, cartesian_axes
from machinist.devices.sensors.laser import Laser
from machinist.kinematics.api import Pose

from .test_yaskawa_inform import _job

START_DEG = (0, 0, 0, 0, -90, 0)


@pytest.mark.parametrize(("text", "watch"), [
    ("MOVL P011 V=20.0 PL=0 SRCH RIN#(1)=ON DIS=2.0",
     Watch(True, Literal(1), True, Literal(0), Literal(2.0))),
    ("MOVL P011 V=20.0 SRCH RIN#(1)=ON T=0.20 DIS=0.0",
     Watch(True, Literal(1), True, Literal(0.2), Literal(0))),
    ("MOVL P011 V=20.0 PL=0 UNTIL IN#(D030)=OFF",
     Watch(False, Variable("D", 30), False)),
])
def test_a_watched_move_parses_with_its_input_and_when_it_starts(text: str,
                                                                  watch: Watch) -> None:
    assert parse_instruction(text) == Move(True, PositionRef(False, 11), Literal(20), watch)


def test_an_incremental_search_parses_too() -> None:
    assert parse_instruction("IMOV P011 V=20.0 UF#(5) SRCH RIN#(1)=OFF DIS=300.0") == \
        IncrementalMove(11, Literal(20), DataType.USER, Literal(5),
                        Watch(True, Literal(1), False, Literal(0), Literal(300)))


@pytest.fixture
def controller() -> Iterator[YaskawaController]:
    options = YaskawaOptions(controller="dx200", robot="up50n", start_joints_deg=START_DEG)
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options)
    robot.arm.start_ticker()
    try:
        yield robot.controller
    finally:
        robot.arm.stop_ticker()


def _flange_x_mm(controller: YaskawaController) -> float:
    return controller.arm.state.view.pose[0] * 1000


def _switch_when_past(controller: YaskawaController, x_mm: float,
                      switch: Callable[[bool], None]) -> None:
    """Switch an input on once the flange has passed *x_mm* along base X."""
    controller.arm.on_moved(lambda pose: switch(pose[0] * 1000 > x_mm))


def _along_x(controller: YaskawaController, mm: float) -> Position:
    """The flange's pose (tool 0) moved *mm* along base X."""
    pose = controller.arm.state.view.pose
    return Position(DataType.BASE, 0, 0, cartesian_axes([
        pose[0] * 1000 + mm, pose[1] * 1000, pose[2] * 1000, *np.degrees(pose[3:])]))


def _run(controller: YaskawaController, *lines: str, target: Position) -> None:
    controller.positions["P"] = {1: target.record()}
    controller.files["J.JBI"] = _job(*lines).encode()
    controller.start_job("J")
    assert controller.wait_job(10)


def test_a_search_stops_where_its_input_switches_and_records_it(
        controller: YaskawaController) -> None:
    x0 = _flange_x_mm(controller)
    _switch_when_past(controller, x0 + 30, lambda on: controller.set_direct_input(1, on))
    _run(controller, "MOVL P0001 V=100.0 SRCH RIN#(1)=ON DIS=2.0", "GETS B000 $B002",
         "GETS PX0002 $PX003", target=_along_x(controller, 100))
    assert controller.variables["B"][0] == 1
    found = Position.from_record(controller.positions["P"][2])
    assert found.axes[0] / 1000 == pytest.approx(x0 + 30, abs=3)
    assert _flange_x_mm(controller) == pytest.approx(x0 + 30, abs=3)


def test_a_search_that_finds_nothing_runs_to_its_end(controller: YaskawaController) -> None:
    x0 = _flange_x_mm(controller)
    controller.search_found = True
    _run(controller, "MOVL P0001 V=500.0 SRCH RIN#(1)=ON", "GETS B000 $B002",
         target=_along_x(controller, 50))
    assert controller.variables["B"][0] == 0
    assert _flange_x_mm(controller) == pytest.approx(x0 + 50, abs=0.1)


def test_a_search_watches_only_after_its_distance(controller: YaskawaController) -> None:
    """The input is on from the start: the search stops once it has run DIS=."""
    x0 = _flange_x_mm(controller)
    controller.set_direct_input(1, True)
    _run(controller, "MOVL P0001 V=100.0 SRCH RIN#(1)=ON DIS=20.0",
         target=_along_x(controller, 100))
    assert _flange_x_mm(controller) == pytest.approx(x0 + 20, abs=3)


def test_until_stops_on_a_general_input_and_records_nothing(
        controller: YaskawaController) -> None:
    x0 = _flange_x_mm(controller)
    _switch_when_past(controller, x0 + 40, lambda on: controller.set_input(10, on))
    _run(controller, "MOVL P0001 V=100.0 PL=0 UNTIL IN#(10)=ON", "GETS B000 $B002",
         target=_along_x(controller, 100))
    assert _flange_x_mm(controller) == pytest.approx(x0 + 40, abs=3)
    assert controller.variables["B"][0] == 0
    assert controller.search_pulses is None


# --- With the scene: a laser on the flange finds a part's edge ---------------------


def test_a_laser_search_stops_at_the_parts_edge() -> None:
    """The beam points down from the flange onto the table. A cylinder 100 mm across
    stands 10 mm ahead along X; the search runs at 20 mm/s."""
    robot_config = DeviceConfig(name="r1", kind="yaskawa", host="127.0.0.1", port=0, options={
        "controller": "dx200", "robot": "up50n", "start_joints_deg": list(START_DEG),
        "signals": ["rin1", "ot9"]})
    probe = WorldBuilder().build(SystemConfig(devices=(robot_config,))).devices[0]
    assert isinstance(probe, YaskawaRobot)
    flange: Pose = probe.arm.state.view.pose
    x, y, z = (v * 1000 for v in flange[:3])
    edge_x = x + 10
    world = WorldBuilder().build(SystemConfig(
        devices=(robot_config, DeviceConfig(name="laser1", kind="laser",
                                            options={"switches": {"q1": 300}})),
        io_links=(IOLink(source="r1.ot9", target="laser1.on"),
                  IOLink(source="laser1.q1", target="r1.rin1")),
        parts=(PartsConfig(name="raw", cylinder={"diameter_mm": 100, "height_mm": 80},
                           at_mm=((edge_x + 50, y, z - 130 - 80),)),),
        mounts=(MountConfig(device="laser1", carrier="r1", at_mm_deg=(0, 0, 0, 0, 0, 0)),),
    ))
    robot, laser = world.devices
    assert isinstance(robot, YaskawaRobot)
    assert isinstance(laser, Laser)
    controller = robot.controller
    robot.arm.start_ticker()
    try:
        _run(controller, "DOUT OT#(9) ON", "MOVL P0001 V=20.0 PL=0 SRCH RIN#(1)=ON DIS=2.0",
             "GETS B000 $B002", "GETS PX0002 $PX003", target=_along_x(controller, 300))
    finally:
        robot.arm.stop_ticker()
    assert controller.variables["B"][0] == 1
    found_x = Position.from_record(controller.positions["P"][2]).axes[0] / 1000
    assert found_x == pytest.approx(edge_x, abs=0.5)
