"""Positions in space: system positions, CNVRT, MFRAME, MULMAT and the tool instructions."""

from __future__ import annotations

import numpy as np
import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.controller import YaskawaController
from machinist.devices.robots.yaskawa.inform import JobStoppedError, run_job
from machinist.devices.robots.yaskawa.positions import (
    DataType,
    Position,
    cartesian_axes,
    position_at,
)

from .test_yaskawa_inform import _job

#: Tool 1 reaches 100 mm along the flange's Z; tool 2 has no offset.
TOOL_CND = "\r\n".join([
    "//TOOL 0", "///NAME", "0.000,0.000,0.000,0.0000,0.0000,0.0000", "0,0,0", "0", "0,0,0", "0,0,1",
    "//TOOL 1", "///NAME LASER", "0.000,0.000,100.000,0.0000,0.0000,0.0000", "0,0,0", "0",
    "0,0,0", "0,0,1",
    "//TOOL 2", "///NAME", "0.000,0.000,0.000,0.0000,0.0000,0.0000", "0,0,0", "0", "0,0,0", "0,0,1",
]) + "\r\n"


def _base(x: float, y: float, z: float, rz: float = 0.0, tool: int = 0) -> Position:
    return Position(DataType.BASE, tool, 0, cartesian_axes([x, y, z, 0, 0, rz]))


@pytest.fixture
def controller() -> YaskawaController:
    options = YaskawaOptions(controller="dx200", robot="up50n",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options)
    robot.controller.files["TOOL.CND"] = TOOL_CND.encode()
    return robot.controller


def _run(controller: YaskawaController, *lines: str,
         positions: dict[int, Position] | None = None) -> None:
    for number, position in (positions or {}).items():
        controller.positions.setdefault("P", {})[number] = position.record()
    controller.files["J.JBI"] = _job(*lines).encode()
    run_job(controller, "J")


def _p(controller: YaskawaController, number: int) -> Position:
    return Position.from_record(controller.positions["P"][number])


def _close(a: Position, b: Position, *, um: int = 2) -> bool:
    return (a.data_type, a.tool, a.user) == (b.data_type, b.tool, b.user) and np.allclose(
        a.axes, b.axes, atol=um)


def test_px000_is_the_arms_pulses_and_px001_its_tool_point_in_base(
        controller: YaskawaController) -> None:
    _run(controller, "TCPON TL#(1)", "GETS PX0010 $PX000", "GETS PX0011 $PX001")
    assert _p(controller, 10).axes[:6] == tuple(controller.pulses())
    flange = controller.flange_at(tuple(controller.pulses()))
    expected = position_at(flange, data_type=DataType.BASE, user=0, tool=1, frames=controller)
    assert _p(controller, 11) == expected
    # The flange points down, so tool 1's point lies 100 mm below it.
    assert _p(controller, 11).axes[2] == pytest.approx(flange[2, 3] * 1e6 - 100_000, abs=2)


def test_before_any_search_nothing_was_found(controller: YaskawaController) -> None:
    _run(controller, "SET B000 7", "GETS B000 $B002")
    assert controller.variables["B"][0] == 0


def test_a_user_frame_from_three_points_converts_positions_into_it(
        controller: YaskawaController) -> None:
    """User frame 5 at (1000, 0, 0), its X axis along base +Y."""
    _run(controller,
         "MFRAME UF#(5) PX0001 PX0002 PX0003",
         "CNVRT PX0011 PX0010 UF#(5)",
         "CNVRT PX0012 PX0011 BF",
         positions={1: _base(1000, 0, 0), 2: _base(1000, 500, 0), 3: _base(900, 0, 0),
                    10: _base(1000, 100, 50, rz=90)})
    assert _close(_p(controller, 11),
                  Position(DataType.USER, 0, 5, cartesian_axes([100, 0, 50, 0, 0, 0])))
    assert _close(_p(controller, 12), _base(1000, 100, 50, rz=90))


def test_cnvrt_with_another_tool_gives_that_tools_point(controller: YaskawaController) -> None:
    """Tool 0's point straight down from the flange; tool 1's lies 100 mm further."""
    down = Position(DataType.BASE, 0, 0, cartesian_axes([1000, 0, 500, 180, 0, 0]))
    _run(controller, "CNVRT PX0011 PX0010 BF TL#(1)", positions={10: down})
    assert _close(_p(controller, 11),
                  Position(DataType.BASE, 1, 0, cartesian_axes([1000, 0, 400, 180, 0, 0])))


def test_mulmat_multiplies_the_two_transforms(controller: YaskawaController) -> None:
    _run(controller, "MULMAT P0012 P0010 P0011",
         positions={10: _base(100, 0, 0, rz=90), 11: _base(10, 0, 0)})
    assert _close(_p(controller, 12), _base(100, 10, 0, rz=90))


def test_settool_sets_the_offset_gettool_reads_and_tool_cnd_keeps(
        controller: YaskawaController) -> None:
    _run(controller, "SETTOOL TL#(2) P0010", "GETTOOL P0011 TL#(2)",
         positions={10: _base(-300.0, 0.0, 100.0, rz=90)})
    assert controller.tool_offset(2) == pytest.approx((-300.0, 0.0, 100.0, 0, 0, 90))
    assert _p(controller, 11).axes[:6] == (-300000, 0, 100000, 0, 0, 900000)
    assert b"-300.000,0.000,100.000,0.0000,0.0000,90.0000" in controller.files["TOOL.CND"]


@pytest.mark.parametrize(("lines", "reason"), [
    (("MFRAME UF#(5) PX0001 PX0001 PX0001",), "do not span a plane"),
    (("CNVRT PX0011 PX0001 UF#(9)",), "user frame 9 is not defined"),
    (("TCPON TL#(99)",), "tool 99"),
])
def test_what_the_controller_refuses_stops_the_job(
        controller: YaskawaController, lines: tuple[str, ...], reason: str) -> None:
    with pytest.raises(JobStoppedError, match=reason):
        _run(controller, *lines, positions={1: _base(1000, 0, 0)})
