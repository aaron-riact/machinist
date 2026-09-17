"""The device-specific detail panel, as typed rows.

Most of a device's state is generic: lifecycle, discrete IO, an arm, a
machine. What is left is particular to the emulated product -- a Modbus
register map, an EtherNet/IP I/O block, a controller's alarm list. A
device presents that as a :class:`Panel`: rows of :class:`Field` grouped
into inputs, outputs and derived values, plus a word on the transport.

The panel is a frozen value. A device publishes :class:`PanelChanged`
whenever the state behind it moves.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import ClassVar

from .events import Event


@dataclass(frozen=True, slots=True)
class Field:
    """One row of a panel: a short signal tag, a name, where it lives, and its value."""

    signal: str
    name: str
    offset: str = ""
    type: str = "str"
    value: str = ""
    #: For a bit or bool row, its state as a boolean, so a UI can paint an
    #: indicator without parsing ``value``. ``None`` for every other row.
    on: bool | None = None


@dataclass(frozen=True, slots=True)
class Panel:
    """What a device shows beyond its generic state."""

    #: How the device is reached: ``io`` (signals only), ``modbus``,
    #: ``adapter`` or ``scanner`` (EtherNet/IP), ...
    mode: str = "io"
    transport_ready: bool = True
    peer_connected: bool = True
    clients: int | None = None
    input_block_hex: str = ""
    output_block_hex: str = ""
    input_fields: tuple[Field, ...] = ()
    output_fields: tuple[Field, ...] = ()
    derived_fields: tuple[Field, ...] = ()


@dataclass(frozen=True, slots=True, kw_only=True)
class PanelChanged(Event):
    """A device's detail panel now reads *panel*."""

    KIND: ClassVar[str] = "panel"

    panel: Panel
