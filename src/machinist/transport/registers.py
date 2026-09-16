"""A device's holding-register model, separated from any one wire format.

An emulated Modbus slave owns a mapping from register address to its own
state. Which wire that mapping is reached over is a separate question: a
gripper answers the same registers whether a PLC reads them over Modbus/TCP
or a robot reads them over the RS485 line on its tool flange.

:class:`RegisterPort` is that mapping. A device exposes one; a front end --
:class:`~machinist.transport.modbus_server.HoldingRegisterServer` for TCP,
:class:`~machinist.transport.flange_bus.FlangeBus` for a flange -- consumes it.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

ReadCallback = Callable[[int], int]
WriteCallback = Callable[[int, int], None]


@dataclass(frozen=True, slots=True)
class RegisterPort:
    """One slave's holding registers, as a read and a write callback."""

    on_read: ReadCallback
    on_write: WriteCallback

    def read(self, address: int, count: int) -> list[int]:
        """Read *count* consecutive registers, masked to 16 bits."""
        return [self.on_read(address + offset) & 0xFFFF for offset in range(count)]

    def write(self, address: int, values: list[int]) -> None:
        """Write consecutive registers starting at *address*."""
        for offset, value in enumerate(values):
            self.on_write(address + offset, value & 0xFFFF)
