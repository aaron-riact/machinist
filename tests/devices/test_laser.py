"""A laser distance switch: outputs by distance, while it is on."""

from __future__ import annotations

import pytest

from machinist.core.capabilities import HasIO, SensesDistance
from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.options import OptionsError, parse_options
from machinist.core.types import Endpoint
from machinist.devices.sensors.laser import Laser, LaserOptions


def _laser(switches: dict[str, object]) -> Laser:
    options = parse_options(LaserOptions, {"switches": switches}, kind="laser")
    return Laser("l1", Endpoint("127.0.0.1", 0), EventBus(), options,
                 io=SignalBank(owner="l1"))


def _outputs(laser: Laser) -> dict[str, bool]:
    return {signal.name: signal.value for signal in laser.io if signal.name != "on"}


def test_it_is_an_io_device_the_scene_hands_distances_to() -> None:
    laser = _laser({"q1": 180})
    assert isinstance(laser, HasIO)
    assert isinstance(laser, SensesDistance)
    assert [signal.name for signal in laser.io] == ["on", "ok", "q1"]


def test_off_it_switches_nothing_whatever_it_sees() -> None:
    laser = _laser({"q1": 180})
    laser.sense_distance(0.1)
    assert _outputs(laser) == {"ok": False, "q1": False}


def test_on_it_is_ok_and_each_output_is_on_closer_than_its_switch_point() -> None:
    laser = _laser({"q1": 180, "q2": 600})
    laser.io["on"].set(True)
    laser.sense_distance(None)
    assert _outputs(laser) == {"ok": True, "q1": False, "q2": False}
    laser.sense_distance(0.4)
    assert _outputs(laser) == {"ok": True, "q1": False, "q2": True}
    laser.sense_distance(0.13)
    assert _outputs(laser) == {"ok": True, "q1": True, "q2": True}


def test_what_it_saw_while_off_shows_once_it_is_switched_on() -> None:
    laser = _laser({"q1": 180})
    laser.sense_distance(0.13)
    laser.io["on"].set(True)
    assert _outputs(laser) == {"ok": True, "q1": True}
    laser.io["on"].set(False)
    assert _outputs(laser) == {"ok": False, "q1": False}


def test_an_output_with_hysteresis_goes_off_only_beyond_its_off_point() -> None:
    laser = _laser({"q1": {"on_within_mm": 175, "off_beyond_mm": 200}})
    laser.io["on"].set(True)
    laser.sense_distance(0.19)
    assert laser.io["q1"].value is False
    laser.sense_distance(0.17)
    assert laser.io["q1"].value is True
    laser.sense_distance(0.19)
    assert laser.io["q1"].value is True
    laser.sense_distance(0.21)
    assert laser.io["q1"].value is False


@pytest.mark.parametrize(("switches", "complaint"), [
    ({}, "switches is empty"),
    ({"ok": 180}, "takes the name of the laser's own signal"),
    ({"q1": 0}, "is not positive"),
    ({"q1": {"on_within_mm": 200, "off_beyond_mm": 180}}, "is closer than on_within_mm"),
])
def test_switch_points_that_cannot_work_are_refused(switches: dict[str, object],
                                                     complaint: str) -> None:
    with pytest.raises(OptionsError, match=complaint):
        _laser(switches)
