"""Shared state for CNC machines.

A machine has:

* an arbitrary set of *doors* (each can be opened/closed)
* an arbitrary set of *chucks* (each can be opened/closed/clamped)
* a *cycle* state (idle / running / paused / aborted)
* a key/value *variable store* (for macro variables, DPRINT, etc.)

Per-vendor modules expose this state through their wire protocol.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from enum import StrEnum, auto


class CycleState(StrEnum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    ABORTED = auto()


@dataclass(slots=True)
class Toggle:
    """A two-position actuator (door, chuck)."""

    name: str
    open: bool = False

    def set(self, *, open: bool) -> None:  # noqa: A002 - matches industrial vocab
        self.open = open


@dataclass(slots=True)
class MachineState:
    cycle: CycleState = CycleState.IDLE
    program: str = ""
    doors: dict[str, Toggle] = field(default_factory=dict)
    chucks: dict[str, Toggle] = field(default_factory=dict)
    variables: dict[str, float | str] = field(default_factory=dict)
    dprint_log: list[str] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def door(self, name: str) -> Toggle:
        return self.doors.setdefault(name, Toggle(name=name))

    def chuck(self, name: str) -> Toggle:
        return self.chucks.setdefault(name, Toggle(name=name))

    def dprint(self, text: str) -> None:
        with self._lock:
            self.dprint_log.append(text)
