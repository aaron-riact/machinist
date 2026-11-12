"""The controller's signals a scene wires to other devices: inputs in, outputs out."""

from __future__ import annotations

import pytest

from machinist.core.capabilities import HasIO
from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.events import EventBus
from machinist.core.io import Direction
from machinist.core.options import OptionsError, parse_options
from machinist.core.types import Endpoint
from machinist.core.world import WorldBuilder
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.signals import ControllerSignal, SignalKind

from ..conftest import free_port


def _robot(*signals: str, ladder: tuple[str, ...] = ()) -> YaskawaRobot:
    options = YaskawaOptions.model_validate(
        {"controller": "dx200", "robot": "up50n", "signals": list(signals), "ladder": ladder})
    return YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)


@pytest.mark.parametrize(("name", "signal"), [
    ("in10", ControllerSignal(SignalKind.IN, 10)),
    ("ot9", ControllerSignal(SignalKind.OT, 9)),
    ("rin1", ControllerSignal(SignalKind.RIN, 1)),
    ("s20021", ControllerSignal(SignalKind.EXTERNAL, 20021)),
    ("s30020", ControllerSignal(SignalKind.EXTERNAL, 30020)),
])
def test_a_name_gives_the_kind_and_number(name: str, signal: ControllerSignal) -> None:
    assert ControllerSignal.parse(name) == signal
    assert signal.name == name


@pytest.mark.parametrize("name", ["IN10", "in0", "in", "out5", "rin1a", "s20018", "s10010",
                                  "s2001"])
def test_a_name_that_is_no_signal_is_refused(name: str) -> None:
    with pytest.raises(ValueError, match="expected in<n>, ot<n>, rin<n>, or s and"):
        ControllerSignal.parse(name)


def test_a_signal_listed_twice_is_refused() -> None:
    with pytest.raises(ValueError, match="lists one twice"):
        _robot("in10", "in10")


def test_only_outputs_are_the_controllers_to_switch() -> None:
    robot = _robot("in10", "ot9", "rin1", "s20021", "s30020")
    assert isinstance(robot, HasIO)
    directions = {signal.name: signal.direction for signal in robot.io}
    assert directions == {"in10": Direction.INPUT, "ot9": Direction.OUTPUT,
                          "rin1": Direction.INPUT, "s20021": Direction.INPUT,
                          "s30020": Direction.OUTPUT}


def test_an_external_input_reaches_the_jobs_through_the_ladder() -> None:
    """A cell's ladder copies terminals #20020-#20027 to IN#9-IN#16."""
    robot = _robot("s20021", ladder=("GSTR #20020 GOUT #00020",))
    robot.io["s20021"].set(True)
    assert robot.controller.io[20] == 0b10


def test_an_external_output_follows_what_the_ladder_passes_on() -> None:
    """A cell's ladder copies OT#9-OT#16 to terminals #30020-#30027."""
    robot = _robot("s30020", ladder=("GSTR #10020 GOUT #30020",))
    robot.controller.files["LASER_ON.JBI"] = (
        b"/JOB\r\n//NAME LASER_ON\r\n//INST\r\nNOP\r\nDOUT OT#(9) ON\r\nEND\r\n")
    robot.controller.start_job("LASER_ON")
    assert robot.controller.wait_job(2)
    assert robot.io["s30020"].value is True


def test_an_input_switches_the_controllers_general_input() -> None:
    robot = _robot("in10", ladder=("GSTR #00020 GOUT #30010",))
    robot.io["in10"].set(True)
    # IN#10 is bit 1 of #00020; the ladder ran and copied it on.
    assert robot.controller.io[20] == 0b10
    assert robot.controller.io[30010] == 0b10
    robot.io["in10"].set(False)
    assert robot.controller.io[20] == 0


def test_a_direct_input_switches_the_controllers_direct_input() -> None:
    robot = _robot("rin1")
    robot.io["rin1"].set(True)
    assert robot.controller.direct_inputs == {1: True}


def test_an_output_follows_what_a_job_switches() -> None:
    robot = _robot("ot9")
    robot.controller.files["LASER_ON.JBI"] = (
        b"/JOB\r\n//NAME LASER_ON\r\n//INST\r\nNOP\r\nDOUT OT#(9) ON\r\nEND\r\n")
    robot.controller.start_job("LASER_ON")
    assert robot.controller.wait_job(2)
    assert robot.io["ot9"].value is True


def test_a_scene_wires_another_devices_output_to_a_robot_input() -> None:
    config = SystemConfig(
        devices=(
            DeviceConfig(name="io1", kind="weidmuller_ur20", host="127.0.0.1", port=0,
                         options={"inputs": 1, "outputs": 1}),
            DeviceConfig(name="r1", kind="yaskawa", host="127.0.0.1", port=0,
                         options={"controller": "dx200", "robot": "up50n",
                                  "signals": ["in10"]}),
        ),
        io_links=(IOLink(source="io1.o1", target="r1.in10"),),
    )
    world = WorldBuilder().build(config)
    world.io_map.bank("io1")["o1"].set(True)
    robot = next(d for d in world.devices if isinstance(d, YaskawaRobot))
    assert robot.controller.io[20] == 0b10


def test_options_that_name_no_signal_are_refused_at_the_boundary() -> None:
    with pytest.raises(OptionsError, match="signals"):
        parse_options(YaskawaOptions, {"controller": "dx200", "robot": "up50n",
                                       "signals": ["door"]}, kind="yaskawa")
