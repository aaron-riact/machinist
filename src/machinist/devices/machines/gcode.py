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
        if "M30" in words:
            self.state.cycle = CycleState.IDLE
            self.state.parts += 1
            yield "M30 program end"
            return
        if any(w in words for w in ("M0", "M1")):
            self.state.cycle = CycleState.PAUSED
            yield "program stop"
            return
        if "G4" in words and "P" in words:
            duration = float(words["P"])
            time.sleep(duration)
            yield f"dwell {duration}s"
            return
        if any(w in words for w in ("G0", "G1")):
            yield " ".join(f"{k}{v}" for k, v in words.items())
            return

    def _apply_tooling(self, words: dict[str, str]) -> Iterator[str]:
        """Update spindle, feed and tool state from a parsed line."""
        if "F" in words:
            self.state.feed = float(words["F"])
        if "T" in words:
            self.state.tool = int(float(words["T"]))
        if "M6" in words:
            self.state.tool_changes += 1
            yield f"tool change T{self.state.tool}"
        if any(w in words for w in ("M3", "M4")):
            self.state.spindle_rpm = float(words.get("S", self.state.spindle_rpm))
            direction = "CW" if "M3" in words else "CCW"
            yield f"spindle {direction} {self.state.spindle_rpm:g}"
        elif "M5" in words:
            self.state.spindle_rpm = 0.0
            yield "spindle stop"


def _words(line: str) -> dict[str, str]:
    """Parse G-code line into a ``{letter: value}`` mapping.

    M- and G-codes become canonical keys (``M03`` and ``M3`` both map to
    ``M3``) so callers can match them without worrying about zero-padding.
    """
    out: dict[str, str] = {}
    i = 0
    while i < len(line):
        ch = line[i].upper()
        if not ch.isalpha():
            i += 1
            continue
        i += 1
        m = _NUM.match(line, i)
        if m is None:
            out[ch] = ""
            continue
        if ch in "MG":
            out[f"{ch}{_canonical(m.group(0))}"] = m.group(0)
        else:
            out[ch] = m.group(0)
        i = m.end()
    return out


def _canonical(number: str) -> str:
    """Strip zero-padding from an integer code (``03`` -> ``3``)."""
    try:
        return str(int(number))
    except ValueError:
        return number


def _coerce(value: str) -> float | str:
    try:
        return float(value)
    except ValueError:
        return value
