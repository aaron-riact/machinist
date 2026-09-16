"""Persist the event stream as JSON Lines.

One line per event: the envelope (``device``, ``kind``, ``timestamp``) and the
event's own fields under ``payload``. Frozen views inside a payload (an arm
view, a machine view, a panel) are written out as nested objects. The file
is the fleet's history; the projection can be rebuilt from it.
"""

from __future__ import annotations

import json
import threading
from collections.abc import Mapping
from dataclasses import asdict, is_dataclass
from enum import Enum
from pathlib import Path
from typing import Any

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
