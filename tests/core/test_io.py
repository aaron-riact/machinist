from __future__ import annotations

import pytest

from machinist.core.io import IOMap


def test_link_propagates_value() -> None:
    io = IOMap()
    src = io.bank("ctrl").declare("out_5")
    dst = io.bank("machine").declare("door_open_cmd")
    io.link("ctrl.out_5", "machine.door_open_cmd")
    src.set(True)
    assert dst.value is True


def test_unknown_signal_raises() -> None:
    io = IOMap()
    io.bank("ctrl").declare("out_5")
    with pytest.raises(KeyError):
        io.link("ctrl.out_5", "nope.x")


def test_invalid_path() -> None:
    io = IOMap()
    with pytest.raises(ValueError, match="must be"):
        io.link("ctrl", "machine.door")
