"""The serial line on a robot's tool flange, as an in-process bus.

A robot arm with a Modbus RTU master on its tool flange reaches a gripper
over RS485: one line, one or more slaves, each answering on its own slave
id. An OnRobot Dual Quick Changer puts two grippers on that one line.

There is no serial port to emulate here -- both ends are objects in the
same process -- so the bus is just the address decode: a slave id picks a
device's :class:`~machinist.transport.registers.RegisterPort`, and the read
or write goes straight to it. What it does model faithfully is the part
that matters for testing a driver: a master can exist on a line where
nothing answers, and talking to an absent slave fails.
"""

from __future__ import annotations

from .registers import RegisterPort


class NoSlaveError(LookupError):
    """Raised when addressing a slave id that nothing on the line answers to."""


class FlangeBus:
    """The RS485 line on one arm's tool flange, addressed by slave id."""

    def __init__(self) -> None:
        self._slaves: dict[int, RegisterPort] = {}

    def attach(self, slave_id: int, registers: RegisterPort) -> None:
        """Wire a device onto the line as *slave_id*."""
        if slave_id in self._slaves:
            raise ValueError(f"slave id {slave_id} is already taken on this flange")
        self._slaves[slave_id] = registers

    def detach(self, slave_id: int) -> None:
        self._slaves.pop(slave_id, None)

    @property
    def slave_ids(self) -> tuple[int, ...]:
        return tuple(sorted(self._slaves))

    def read_holding(self, slave_id: int, address: int, count: int) -> list[int]:
        return self._slave(slave_id).read(address, count)

    def write_holding(self, slave_id: int, address: int, values: list[int]) -> None:
        self._slave(slave_id).write(address, values)

    def _slave(self, slave_id: int) -> RegisterPort:
        try:
            return self._slaves[slave_id]
        except KeyError:
            raise NoSlaveError(f"no device answers to slave id {slave_id} on this flange") from None
