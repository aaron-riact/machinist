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
