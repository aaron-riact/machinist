"""The controller's signals that a scene wires to other devices, by name.

A name gives a kind and a number, as the jobs write them:

* ``in10``: general input IN#10, which the wiring outside the controller switches.
* ``ot9``: general output OT#9, which the controller switches.
* ``rin1``: direct input RIN#1, the fast input a search (``SRCH RIN#(1)=ON``) stops on.
* ``s20021``: external input #20021, a terminal the cell's wiring switches; the
  ladder passes it on (to IN#10 in a cell whose ladder copies #20020 to #00020).
* ``s30020``: external output #30020, a terminal the ladder switches.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from enum import StrEnum

from ....core.io import Direction


class SignalKind(StrEnum):
    IN = "in"
    OT = "ot"
    RIN = "rin"
    EXTERNAL = "s"


_NAME = re.compile(r"(in|ot|rin)([1-9]\d*)|s([23]\d{3}[0-7])")


@dataclass(frozen=True, slots=True)
class ControllerSignal:
    kind: SignalKind
    number: int

    @classmethod
    def parse(cls, name: str) -> ControllerSignal:
        match = _NAME.fullmatch(name)
        if match is None:
            raise ValueError(f"signal {name!r}: expected in<n>, ot<n>, rin<n>, or s and an "
                             f"external input (#2xxxx) or output (#3xxxx)")
        if match[3] is not None:
            return cls(SignalKind.EXTERNAL, int(match[3]))
        return cls(SignalKind(match[1]), int(match[2]))

    @property
    def name(self) -> str:
        return f"{self.kind}{self.number}"

    @property
    def direction(self) -> Direction:
        """Outputs are the controller's to switch; the rest are switched from outside."""
        output = self.kind is SignalKind.OT or (
            self.kind is SignalKind.EXTERNAL and self.number >= 30000)
        return Direction.OUTPUT if output else Direction.INPUT
