"""Status events broadcast by emulators to the UI and other observers.

The bus is intentionally simple: a thread-safe pub/sub of *immutable*
``Event`` objects. We do not couple producers to a particular runtime
(threads vs asyncio); subscribers receive callbacks synchronously and are
expected to be cheap (e.g. push onto a queue).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

EventHandler = Callable[["Event"], None]


@dataclass(frozen=True, slots=True)
class Event:
    """A timestamped status update from a device."""

    device: str
    kind: str
    payload: dict[str, Any] = field(default_factory=dict)
    timestamp: float = field(default_factory=time.time)


class EventBus:
    """Trivial thread-safe pub/sub bus.

    Handlers are called in registration order while holding no lock,
    after the subscriber list has been snapshot under the lock. This
    keeps publishers fast and removes the risk of re-entrancy deadlocks.
    """

    def __init__(self) -> None:
        self._handlers: list[EventHandler] = []
        self._lock = threading.Lock()

    def subscribe(self, handler: EventHandler) -> Callable[[], None]:
        """Subscribe a handler; returns an unsubscribe callable."""
        with self._lock:
            self._handlers.append(handler)

        def unsubscribe() -> None:
            with self._lock:
                if handler in self._handlers:
                    self._handlers.remove(handler)

        return unsubscribe

    def publish(self, event: Event) -> None:
        with self._lock:
            handlers = list(self._handlers)
        for handler in handlers:
            handler(event)
