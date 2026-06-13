"""Tiny G-code interpreter focussed on emulation needs.

We deliberately do **not** try to be a full G-code engine: the goal is
to drive the emulator's :class:`MachineState` (cycle progression,
DPRINT lines, variable writes, dwell delays). Movement is acknowledged
but not simulated kinematically.

Supported subset:

* ``M0/M1``                program stop
* ``M30``                  program end (increments the parts counter)
* ``M3/M4 Sn``             spindle on clockwise/counter-clockwise at ``n`` RPM
* ``M5``                   spindle stop
* ``M6`` / ``Tn``          tool change (sets active tool, counts the change)
* ``G4 Pn``                dwell ``n`` seconds
* ``G0/G1 X Y Z Fn``       movement (acknowledged; ``F`` sets feed rate)
* ``#n=value``             variable assignment
* ``DPRINT[...]``          append to DPRINT log

We use the public ``pygcode`` library if available; otherwise the
hand-rolled parser below covers everything we need without a dependency.
"""

from __future__ import annotations

import re
import time
from collections.abc import Iterator
from dataclasses import dataclass

from .state import CycleState, MachineState

_TOKEN = re.compile(r"\s*([A-Z#][A-Z0-9_]*=?|\S+)")
_VAR_ASSIGN = re.compile(r"#(\d+)\s*=\s*(.+)")
_DPRINT = re.compile(r"DPRINT\[(.*)\]", re.IGNORECASE)
_NUM = re.compile(r"-?\d+(\.\d+)?")


@dataclass(frozen=True, slots=True)
class GCodeLine:
    """Typed representation of a parsed G-code line."""

    g: int | None = None
    m: int | None = None
    f: float | None = None
    s: float | None = None
    t: int | None = None
    x: float | None = None
    y: float | None = None
    z: float | None = None
    p: float | None = None


@dataclass(slots=True)
class Interpreter:
    state: MachineState

    def run(self, program: str) -> Iterator[str]:
        """Execute the program, yielding human-readable progress lines."""
        with self.state._lock:
            self.state.program = program
            self.state.cycle = CycleState.RUNNING
        try:
            for line in program.splitlines():
                yield from self._exec_line(line.strip())
        finally:
            with self.state._lock:
                if self.state.cycle is CycleState.RUNNING:
                    self.state.cycle = CycleState.IDLE

    def _exec_line(self, line: str) -> Iterator[str]:
        if not line or line.startswith(";") or line.startswith("("):
            return
        if (m := _DPRINT.search(line)) is not None:
            text = m.group(1)
            self.state.dprint(text)
            yield f"DPRINT {text!r}"
            return
        if (m := _VAR_ASSIGN.match(line)) is not None:
            self.state.variables[m.group(1)] = _coerce(m.group(2).strip())
            yield f"#{m.group(1)} = {m.group(2)}"
            return
        words = _words(line)
        yield from self._apply_tooling(words)
        if words.m == 30:
            self.state.cycle = CycleState.IDLE
            self.state.parts += 1
            yield "M30 program end"
            return
        if words.m in (0, 1):
            self.state.cycle = CycleState.PAUSED
            yield "program stop"
            return
        if words.g == 4 and words.p is not None:
            duration = words.p
            time.sleep(duration)
            yield f"dwell {duration}s"
            return
        if words.g in (0, 1):
            self._apply_position(words)
            yield f"G{words.g} move"
            return

    def _apply_tooling(self, words: GCodeLine) -> Iterator[str]:
        """Update spindle, feed and tool state from a parsed line."""
        if words.f is not None:
            self.state.feed = words.f
        if words.t is not None:
            self.state.tool = words.t
        if words.m == 6:
            self.state.tool_changes += 1
            yield f"tool change T{self.state.tool}"
        if words.m in (3, 4):
            self.state.spindle_rpm = words.s if words.s is not None else self.state.spindle_rpm
            direction = "CW" if words.m == 3 else "CCW"
            yield f"spindle {direction} {self.state.spindle_rpm:g}"
        elif words.m == 5:
            self.state.spindle_rpm = 0.0
            yield "spindle stop"

    def _apply_position(self, words: GCodeLine) -> None:
        self.state.position.move_to(
            x=words.x,
            y=words.y,
            z=words.z,
        )


def _words(line: str) -> GCodeLine:
    """Parse G-code line into a typed :class:`GCodeLine`."""
    kw: dict[str, object] = {}
    i = 0
    while i < len(line):
        ch = line[i].upper()
        if not ch.isalpha():
            i += 1
            continue
        i += 1
        m = _NUM.match(line, i)
        if m is None:
            i += 1
            continue
        raw = m.group(0)
        if ch in "MG":
            kw[f"{ch.lower()}"] = int(raw)
        elif ch == "F":
            kw["f"] = float(raw)
        elif ch == "S":
            kw["s"] = float(raw)
        elif ch == "T":
            kw["t"] = int(float(raw))
        elif ch == "P":
            kw["p"] = float(raw)
        elif ch in "XYZ":
            kw[ch.lower()] = float(raw)
        i = m.end()
    return GCodeLine(**kw)


def _coerce(value: str) -> float | str:
    try:
        return float(value)
    except ValueError:
        return value
