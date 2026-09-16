"""A flange bus decodes a slave id to a device's registers."""

from __future__ import annotations

import pytest

from machinist.transport.flange_bus import FlangeBus, NoSlaveError
from machinist.transport.registers import RegisterPort


def _port() -> tuple[RegisterPort, dict[int, int]]:
    cells: dict[int, int] = {}
    return RegisterPort(on_read=lambda a: cells.get(a, 0), on_write=cells.__setitem__), cells


def test_a_new_line_has_nothing_on_it() -> None:
    assert FlangeBus().slave_ids == ()


def test_read_reaches_the_attached_slave() -> None:
    bus = FlangeBus()
    port, cells = _port()
    cells.update({0x010B: 500, 0x010C: 1})
    bus.attach(0x41, port)

    assert bus.read_holding(0x41, 0x010B, 2) == [500, 1]


def test_write_reaches_the_attached_slave() -> None:
    bus = FlangeBus()
    port, cells = _port()
    bus.attach(0x41, port)

    bus.write_holding(0x41, 0, [400, 1100, 1])

    assert cells == {0: 400, 1: 1100, 2: 1}


def test_two_grippers_share_one_line() -> None:
    """An OnRobot Dual Quick Changer puts two tools on the same flange."""
    bus = FlangeBus()
    first, first_cells = _port()
    second, second_cells = _port()
    bus.attach(0x41, first)
    bus.attach(0x42, second)

    bus.write_holding(0x41, 0, [1])
    bus.write_holding(0x42, 0, [2])

    assert first_cells == {0: 1}
    assert second_cells == {0: 2}
    assert bus.slave_ids == (0x41, 0x42)


def test_reading_an_absent_slave_fails() -> None:
    bus = FlangeBus()

    with pytest.raises(NoSlaveError, match="slave id 65"):
        bus.read_holding(0x41, 0x010B, 2)


def test_writing_an_absent_slave_fails() -> None:
    bus = FlangeBus()

    with pytest.raises(NoSlaveError):
        bus.write_holding(0x41, 0, [1])


def test_two_devices_cannot_share_a_slave_id() -> None:
    bus = FlangeBus()
    first, _ = _port()
    second, _ = _port()
    bus.attach(0x41, first)

    with pytest.raises(ValueError, match="already taken"):
        bus.attach(0x41, second)


def test_detaching_takes_a_slave_off_the_line() -> None:
    bus = FlangeBus()
    port, _ = _port()
    bus.attach(0x41, port)

    bus.detach(0x41)

    assert bus.slave_ids == ()
    with pytest.raises(NoSlaveError):
        bus.read_holding(0x41, 0, 1)
