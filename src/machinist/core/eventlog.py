"""Persist the event stream as JSON Lines.

One line per event: the envelope (``device``, ``kind``, ``timestamp``) and the
event's own fields under ``payload``. Frozen views inside a payload (an arm
view, a machine view, a panel) are written out as nested objects. The file
is the fleet's history; the projection can be rebuilt from it.
"""

from __future__ import annotations

import json
import threading
from collections.abc import Collection, Mapping
from dataclasses import asdict, is_dataclass
from enum import Enum
from math import degrees
from pathlib import Path
from typing import Any, TextIO

from .events import Event, EventBus


def to_jsonable(value: Any) -> Any:
    """Turn events, frozen views, enums and mappings into JSON-ready data."""
    if is_dataclass(value) and not isinstance(value, type):
        return {k: to_jsonable(v) for k, v in asdict(value).items()}
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Mapping):
        return {str(k): to_jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set, frozenset)):
        return [to_jsonable(v) for v in value]
    return value


def event_to_record(event: Event) -> dict[str, Any]:
    return {
        "timestamp": event.timestamp,
        "device": event.device,
        "kind": event.kind,
        "payload": to_jsonable(event.payload),
    }


class EventLog:
    """Append every event on a bus to a JSON Lines file."""

    def __init__(self, path: str | Path) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.path.open("a", encoding="utf-8", buffering=1)
        self._lock = threading.Lock()
        self._unsubscribe = None

    def attach(self, bus: EventBus) -> None:
        """Start recording *bus*. Call :meth:`close` to stop."""
        self._unsubscribe = bus.subscribe(self.record)

    def record(self, event: Event) -> None:
        line = json.dumps(event_to_record(event), separators=(",", ":"))
        with self._lock:
            self._file.write(line + "\n")

    def close(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
        with self._lock:
            if not self._file.closed:
                self._file.close()


#: Lines a device sends and receives; a device that prints its own is spared them.
LINE_KINDS = frozenset({"rx", "tx"})


class ConsoleLog:
    """Print the events on a bus that say what is happening, one line each.

    Polls make most of the traffic: a driver asking for status and position many
    times a second. The lines a device sends and receives (rx, tx) are therefore
    left out unless *debug*. An arm reports its state at its tick rate, so it is
    shown only when its mode or servo changes (started or stopped moving, faulted,
    servo off), unless *include_arm* shows every report. Devices named in
    *own_lines* print their own received lines, so their rx and tx are left out
    rather than shown twice.
    """

    def __init__(self, stream: TextIO, *, debug: bool = False, include_arm: bool = False,
                 own_lines: Collection[str] = ()) -> None:
        self._stream = stream
        self._debug = debug
        self._include_arm = include_arm
        self._own_lines = frozenset(own_lines)
        self._arm_states: dict[str, tuple[Any, Any]] = {}
        self._lock = threading.Lock()
        self._unsubscribe: Any = None

    def attach(self, bus: EventBus) -> None:
        """Start printing *bus*. Call :meth:`close` to stop."""
        self._unsubscribe = bus.subscribe(self.record)

    def record(self, event: Event) -> None:
        if event.kind in LINE_KINDS and (not self._debug or event.device in self._own_lines):
            return
        if event.kind == "arm" and not self._include_arm:
            line = self._arm_change(event)
        else:
            payload = json.dumps(to_jsonable(event.payload), separators=(",", ":"))
            line = f"[{event.device}] {event.kind} {payload}"
        if line is None:
            return
        with self._lock:
            print(line, file=self._stream, flush=True)

    def _arm_change(self, event: Event) -> str | None:
        """A line when the arm's mode or servo changed since its last report, else None."""
        view = event.payload.get("view")
        if view is None:
            return None
        state = (view.mode, view.servo_on)
        with self._lock:
            if self._arm_states.get(event.device) == state:
                return None
            self._arm_states[event.device] = state
        x, y, z, rx, ry, rz = view.pose
        pose = (f"{x * 1000:.1f} {y * 1000:.1f} {z * 1000:.1f} mm "
                f"{degrees(rx):.1f} {degrees(ry):.1f} {degrees(rz):.1f} deg")
        joints = " ".join(f"{degrees(j):.1f}" for j in view.joints)
        servo = "on" if view.servo_on else "off"
        return (f"[{event.device}] arm {to_jsonable(view.mode)}, servo {servo}, "
                f"at {pose}, joints {joints} deg")

    def close(self) -> None:
        if self._unsubscribe is not None:
            self._unsubscribe()
            self._unsubscribe = None
