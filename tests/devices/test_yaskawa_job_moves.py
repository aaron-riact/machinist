"""A job's moves: MOVJ, MOVL and IMOV, each waited for before the job goes on."""

from __future__ import annotations

from collections.abc import Iterator

import numpy as np
import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.controller import YaskawaController
from machinist.devices.robots.yaskawa.positions import (
    DataType,
    Position,
    cartesian_axes,
    position_at,
    pulse_axes,
)

from .test_yaskawa_frames import TOOL_CND
from .test_yaskawa_inform import _job

START_DEG = (0, 0, 0, 0, -90, 0)


@pytest.fixture
def controller() -> Iterator[YaskawaController]:
    options = YaskawaOptions(controller="dx200", robot="up50n", start_joints_deg=START_DEG)
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options)
    robot.controller.files["TOOL.CND"] = TOOL_CND.encode()
    robot.arm.start_ticker()
    try:
        yield robot.controller
    finally:
        robot.arm.stop_ticker()


def _run(controller: YaskawaController, *lines: str, positions: dict[int, Position],
         pos_section: tuple[str, ...] = ()) -> list[tuple[str, dict[str, object]]]:
    events: list[tuple[str, dict[str, object]]] = []
    controller.note = lambda kind, **fields: events.append((kind, fields))
    for number, position in positions.items():
        controller.positions.setdefault("P", {})[number] = position.record()
    text = _job(*lines)
    if pos_section:
        text = text.replace("//INST", "\r\n".join(["//POS", *pos_section, "//INST"]))
    controller.files["J.JBI"] = text.encode()
    controller.start_job("J")
    assert controller.wait_job(10)
    return [event for event in events if event[0].startswith("job_")]


def _tool_point(controller: YaskawaController, tool: int) -> Position:
    return position_at(controller.flange_at(tuple(controller.pulses())),
                       data_type=DataType.BASE, user=0, tool=tool, frames=controller)


def _start_point(controller: YaskawaController, tool: int) -> np.ndarray:
    return np.array(_tool_point(controller, tool).axes[:3]) / 1000


def _down(x: float, y: float, z: float, tool: int) -> Position:
    """Tool point at x, y, z mm in base, the tool pointing straight down."""
    return Position(DataType.BASE, tool, 0, cartesian_axes([x, y, z, 180, 0, 0]))


def test_movj_to_pulses_turns_the_axes_there(controller: YaskawaController) -> None:
    target = [p + 2000 for p in controller.pulses()]
    events = _run(controller, "MOVJ P0001 VJ=100.00", "DOUT OT#(1) ON",
                  positions={1: Position(DataType.PULSE, 0, 0, pulse_axes(target))})
    assert events == [("job_ran", {"job": "J"})]
    assert np.allclose(controller.pulses(), target, atol=2)
    assert controller.output(1)


def test_movl_takes_the_tool_point_of_the_selected_tool_there(
        controller: YaskawaController) -> None:
    x, y, z = _start_point(controller, 1)
    events = _run(controller, "TCPON TL#(1)", "MOVL P0001 V=1000.0 PL=0",
                  positions={1: _down(x + 50, y - 30, z - 20, tool=0)})
    assert events == [("job_ran", {"job": "J"})]
    assert np.allclose(_start_point(controller, 1), (x + 50, y - 30, z - 20), atol=0.1)


def test_a_move_to_the_jobs_own_position(controller: YaskawaController) -> None:
    target = [p - 1500 for p in controller.pulses()]
    _run(controller, "MOVJ C00000 VJ=100.00", positions={}, pos_section=(
        "///NPOS 1,0,0,0,0,0", "///TOOL 0", "///POSTYPE PULSE", "///PULSE",
        "C00000=" + ",".join(str(p) for p in target)))
    assert np.allclose(controller.pulses(), target, atol=2)


def test_imov_moves_along_a_user_frames_axes(controller: YaskawaController) -> None:
    """User frame 5's X axis runs along base +Y: 30 mm along it is 30 mm along base Y."""
    x, y, z = _start_point(controller, 1)
    frame = {10: _down(1000, 0, 0, 0), 11: _down(1000, 500, 0, 0), 12: _down(900, 0, 0, 0)}
    step = Position(DataType.USER, 1, 5, cartesian_axes([30, 0, -10, 0, 0, 0]))
    _run(controller, "MFRAME UF#(5) PX0010 PX0011 PX0012", "TCPON TL#(1)",
         "IMOV P0001 V=1000.0 UF#(5)", positions={**frame, 1: step})
    assert np.allclose(_start_point(controller, 1), (x, y + 30, z - 10), atol=0.1)


def test_a_hold_during_a_move_stops_the_job_there(controller: YaskawaController) -> None:
    target = [p + 200_000 for p in controller.pulses()]
    controller.positions["P"] = {1: Position(DataType.PULSE, 0, 0, pulse_axes(target)).record()}
    controller.files["J.JBI"] = _job("MOVJ P0001 VJ=1.00", "DOUT OT#(1) ON").encode()
    events: list[tuple[str, dict[str, object]]] = []
    controller.note = lambda kind, **fields: events.append((kind, fields))
    controller.start_job("J")
    controller.set_hold(True)
    assert controller.wait_job(5)
    assert ("job_stopped", {"job": "J", "line": 1, "reason": "hold"}) in events
    assert not controller.output(1)


def test_a_move_past_a_soft_limit_stops_the_job_on_its_alarm(
        controller: YaskawaController) -> None:
    controller.soft_limit_pulses = tuple((p - 5000, p + 5000) for p in controller.pulses())
    past = [controller.pulses()[0] + 6000, *controller.pulses()[1:]]
    events = _run(controller, "MOVJ P0001 VJ=50.00", "DOUT OT#(1) ON",
                  positions={1: Position(DataType.PULSE, 0, 0, pulse_axes(past))})
    assert events == [("job_stopped", {"job": "J", "line": 1, "reason": "alarm 4980"})]
    assert not controller.output(1)
