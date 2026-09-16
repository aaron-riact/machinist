"""A RegisterPort is a device's register model, independent of any wire."""

from __future__ import annotations

from machinist.transport.registers import RegisterPort


def _port() -> tuple[RegisterPort, dict[int, int]]:
    cells: dict[int, int] = {}
    return RegisterPort(on_read=lambda a: cells.get(a, 0), on_write=cells.__setitem__), cells


def test_read_returns_consecutive_registers() -> None:
    port, cells = _port()
    cells.update({10: 1, 11: 2, 12: 3})

    assert port.read(10, 3) == [1, 2, 3]


def test_read_masks_values_to_16_bits() -> None:
    port, cells = _port()
    cells[10] = -1

    assert port.read(10, 1) == [0xFFFF]


def test_write_fans_out_to_consecutive_addresses() -> None:
    port, cells = _port()

    port.write(10, [7, 8])

    assert cells == {10: 7, 11: 8}


def test_write_masks_values_to_16_bits() -> None:
    port, cells = _port()

    port.write(10, [0x1FFFF])

    assert cells == {10: 0xFFFF}


def test_reading_nothing_returns_nothing() -> None:
    port, _ = _port()

    assert port.read(10, 0) == []
