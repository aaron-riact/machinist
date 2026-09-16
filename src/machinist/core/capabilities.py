"""Optional capabilities a :class:`~machinist.core.device.Device` may declare.

A device is more than a lifecycle and a listener. Some carry discrete IO,
some expose Modbus holding registers, some have a tool flange, some hold a
library of programs. The framework (World wiring, the TUI, the web API)
needs to know which is which.

Each capability here is a small abstract base class. A device declares a
capability by inheriting from it and providing the named attributes::

    class WeidmullerUR20(Device, HasIO, HasRegisters):
        ...

Consumers then ask with ``isinstance(device, HasIO)`` and get a typed
attribute back, rather than probing with ``getattr``.

Capabilities whose payload type lives in the device layer are declared next
to that type: :class:`~machinist.devices.robots.arm.HasArm` and
:class:`~machinist.devices.machines.state.HasMachineState`.
"""

from __future__ import annotations

from abc import ABC, abstractmethod

from ..transport.flange_bus import FlangeBus
from ..transport.registers import RegisterPort
from .io import SignalBank
from .programs import ProgramLibrary


class HasIO(ABC):
    """A device with a bank of discrete IO signals."""

    io: SignalBank


class HasRegisters(ABC):
    """A device whose state is reachable as Modbus holding registers."""

    register_port: RegisterPort


class HasFlange(ABC):
    """A device (an arm) that carries a serial line on its tool flange."""

    flange: FlangeBus


class HasPrograms(ABC):
    """A device that stores programs and can run one by name."""

    programs: ProgramLibrary

    @abstractmethod
    def run_program(self, name: str) -> None:
        """Start *name* from :attr:`programs`. Raises if it cannot start."""
