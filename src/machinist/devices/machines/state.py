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
from collections.abc import Callable
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
class CartesianPosition:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def move_to(
        self,
        *,
        x: float | None = None,
        y: float | None = None,
        z: float | None = None,
    ) -> None:
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if z is not None:
            self.z = z


@dataclass(slots=True)
class MachineState:
    cycle: CycleState = CycleState.IDLE
    program: str = ""
    doors: dict[str, Toggle] = field(default_factory=dict)
    chucks: dict[str, Toggle] = field(default_factory=dict)
    variables: dict[str, float | str] = field(default_factory=dict)
    dprint_log: list[str] = field(default_factory=list)
    dprint_subscribers: list[Callable[[str], None]] = field(default_factory=list)
    # Spindle / tooling / production telemetry (generic across CNCs).
    spindle_rpm: float = 0.0
    feed: float = 0.0
    tool: int = 0
    tool_changes: int = 0
    parts: int = 0
    position: CartesianPosition = field(default_factory=CartesianPosition)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def door(self, name: str) -> Toggle:
        return self.doors.setdefault(name, Toggle(name=name))

    def chuck(self, name: str) -> Toggle:
        return self.chucks.setdefault(name, Toggle(name=name))

    def dprint(self, text: str) -> None:
        with self._lock:
            self.dprint_log.append(text)
            subs = list(self.dprint_subscribers)
        for sub in subs:
            sub(text)


def machine_readers(state: MachineState) -> dict[str, Callable[[], object]]:
    """Zero-arg readers exposing machine state (e.g. for an OPC-UA server).

    Doors and chucks are captured by name at call time, so the set of
    nodes reflects whatever the machine declared; values stay live.
    """
    readers: dict[str, Callable[[], object]] = {
        "cycle": lambda: str(state.cycle),
        "program": lambda: state.program.splitlines()[0] if state.program else "",
        "spindle_rpm": lambda: state.spindle_rpm,
        "feed": lambda: state.feed,
        "tool": lambda: state.tool,
        "parts": lambda: state.parts,
        "x": lambda: state.position.x,
        "y": lambda: state.position.y,
        "z": lambda: state.position.z,
    }
    for name in state.doors:
        readers[f"door_{name}_open"] = _toggle_reader(state.door, name)
    for name in state.chucks:
        readers[f"chuck_{name}_open"] = _toggle_reader(state.chuck, name)
    return readers


def _toggle_reader(get: Callable[[str], Toggle], name: str) -> Callable[[], object]:
    """Bind ``name`` so the reader stays a clean zero-arg closure."""
    return lambda: get(name).open
