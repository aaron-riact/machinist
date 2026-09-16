"""The one lifecycle every transport server shares.

A device runs several servers side by side: a line-protocol listener, an
MTConnect agent, an OPC-UA publisher, a Modbus slave. Each is started on
its own thread, signals when it has bound its port, and is told to stop
when the device shuts down. :class:`Service` names that contract so a
device can hold a list of them and drive all of them the same way.
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from collections.abc import Callable
from typing import ClassVar


class Service(ABC):
    """Something that serves on a thread until told to stop."""

    #: Seconds a device waits for :meth:`serve_forever` to signal *ready*
    #: before it gives up and faults. Slow stacks raise this.
    BIND_TIMEOUT: ClassVar[float] = 2.0

    @abstractmethod
    def serve_forever(self, ready: threading.Event | None = None) -> None:
        """Block and serve. Set *ready* once the listener is bound and accepting."""

    @abstractmethod
    def shutdown(self) -> None:
        """Ask :meth:`serve_forever` to return. Safe to call from another thread."""


class Poller(Service):
    """Call *poll* every *interval* seconds until shut down.

    For state the process cannot be told about and has to go and look at:
    a directory another program writes into, a file a user edits.
    """

    def __init__(self, poll: Callable[[], None], *, interval: float) -> None:
        self._poll = poll
        self._interval = interval
        self._stop = threading.Event()

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        if ready is not None:
            ready.set()
        while not self._stop.is_set():
            self._poll()
            self._stop.wait(self._interval)

    def shutdown(self) -> None:
        self._stop.set()
