from __future__ import annotations

import pytest

from machinist.core.events import Event
from machinist.core.io import Direction, IOMap, SignalBank, SignalChanged


def test_link_propagates_value() -> None:
    io = IOMap()
    src = io.bank("ctrl").declare("out_5")
    dst = io.bank("machine").declare("door_open_cmd")
    io.link("ctrl.out_5", "machine.door_open_cmd")
    src.set(True)
    assert dst.value is True


def test_signal_direction_defaults_to_input_and_is_recorded() -> None:
    bank = IOMap().bank("ctrl")
    cmd = bank.declare("door_open_cmd")
    status = bank.declare("door_is_open", Direction.OUTPUT)
    assert cmd.direction is Direction.INPUT
    assert status.direction is Direction.OUTPUT


def test_unknown_signal_raises() -> None:
    io = IOMap()
    io.bank("ctrl").declare("out_5")
    with pytest.raises(KeyError):
        io.link("ctrl.out_5", "nope.x")


def test_invalid_path() -> None:
    io = IOMap()
    with pytest.raises(ValueError, match="must be"):
        io.link("ctrl", "machine.door")


def test_a_bank_with_a_publisher_announces_every_signal_change() -> None:
    events: list[Event] = []
    bank = SignalBank(owner="io1", publish=events.append)
    out = bank.declare("o1", Direction.OUTPUT)

    out.set(True)
    out.set(True)  # no change, no event
    out.set(False)

    assert [(e.signal, e.direction, e.value) for e in events if isinstance(e, SignalChanged)] == [
        ("o1", Direction.OUTPUT, True),
        ("o1", Direction.OUTPUT, False),
    ]
    assert all(e.device == "io1" for e in events)


def test_a_bank_without_a_publisher_stays_silent() -> None:
    bank = SignalBank(owner="io1")
    bank.declare("i1").set(True)  # nothing to assert but that it does not blow up
    assert bank["i1"].value is True
