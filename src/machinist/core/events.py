"""Status events broadcast by emulators to the UI and other observers.

Every event is a frozen dataclass carrying the *device* it came from and a
*timestamp*. Two families exist:

* **Typed state events** (:class:`LifecycleChanged` and friends) describe a
  change of state with real fields. The UI projection reduces over these.
* :class:`Note` is the one untyped event: a name plus a free-form payload,
  for things worth logging but not worth modelling -- protocol traffic,
  program steps, debug breadcrumbs.

Both expose ``kind`` and ``payload`` so a log renderer or a JSON feed can
treat them alike.

The bus is intentionally simple: a thread-safe pub/sub of immutable events.
We do not couple producers to a particular runtime (threads vs asyncio);
subscribers receive callbacks synchronously and are expected to be cheap
(e.g. push onto a queue).
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field, fields
from typing import Any, ClassVar

from .types import DeviceState

EventHandler = Callable[["Event"], None]

_ENVELOPE = frozenset({"device", "timestamp"})


@dataclass(frozen=True, slots=True, kw_only=True)
class Event:
    """Base of every event: who it is about, and when."""

    #: Short name for logs and feeds. Typed subclasses set this once.
    KIND: ClassVar[str] = ""

    device: str
    timestamp: float = field(default_factory=time.time)

    @property
    def kind(self) -> str:
        return type(self).KIND

    @property
    def payload(self) -> dict[str, Any]:
        """The event's own fields, without the envelope."""
        return {f.name: getattr(self, f.name) for f in fields(self) if f.name not in _ENVELOPE}


@dataclass(frozen=True, slots=True, kw_only=True)
class Note(Event):
    """An untyped, log-only event: a name and whatever the emitter attached."""

    name: str
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def kind(self) -> str:
        return self.name

    @property
    def payload(self) -> dict[str, Any]:
        return self.data


@dataclass(frozen=True, slots=True, kw_only=True)
class LifecycleChanged(Event):
    """The framework lifecycle of a device moved to *state*."""

    KIND: ClassVar[str] = "state"

    state: DeviceState


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
