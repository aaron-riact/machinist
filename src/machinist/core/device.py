"""The :class:`Device` abstract base class.

Every emulated machine — robot arm, CNC, gripper, IO controller —
implements this interface. The base class is intentionally thin: it owns
*lifecycle* (start/stop) and *status reporting*, and delegates everything
else to subclasses.

We do not commit to threads vs. asyncio: the framework spins up each
device in a worker thread and each device decides internally whether to
run an asyncio loop, native sockets, simpy etc. This keeps the framework
hospitable to any concurrency model without infecting devices with the
choice of any other.
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from typing import Any

from .events import Event, EventBus
from .types import DeviceState, Endpoint


class Device(ABC):
    """Abstract emulated device.

    Subclasses implement :meth:`_run` (long-running listener) and
    :meth:`_shutdown` (graceful teardown). They publish status updates
    via :meth:`emit`.
    """

    #: Human-readable device kind (e.g. ``"ur_robot"``). Subclasses set this.
    kind: str = "device"

    def __init__(self, name: str, endpoint: Endpoint, bus: EventBus) -> None:
        self.name = name
        self.endpoint = endpoint
        self._bus = bus
        self._state = DeviceState.CREATED
        self._state_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    # ----- public API --------------------------------------------------

    @property
    def state(self) -> DeviceState:
        with self._state_lock:
            return self._state

    def start(self) -> None:
        """Spawn the device worker thread."""
        with self._state_lock:
            if self._state is not DeviceState.CREATED:
                raise RuntimeError(f"{self.name} already started ({self._state})")
            self._state = DeviceState.STARTING
        self._thread = threading.Thread(
            target=self._thread_main, name=f"machinist-{self.name}", daemon=True
        )
        self._thread.start()

    def stop(self, *, timeout: float = 5.0) -> None:
        """Request shutdown and wait for the worker to exit."""
        with self._state_lock:
            if self._state in (DeviceState.STOPPED, DeviceState.CREATED):
                return
            self._state = DeviceState.STOPPING
        self._stop_event.set()
        self._shutdown()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def emit(self, kind: str, **payload: Any) -> None:
        """Publish a status :class:`Event` for this device."""
        self._bus.publish(Event(device=self.name, kind=kind, payload=payload))

    # ----- subclass hooks ---------------------------------------------

    @abstractmethod
    def _run(self, stop: threading.Event) -> None:
        """Long-running worker. Should return when ``stop`` is set."""

    def _shutdown(self) -> None:
        """Optional hook for releasing OS resources before joining."""

    # ----- internals ---------------------------------------------------

    def _thread_main(self) -> None:
        with self._state_lock:
            self._state = DeviceState.RUNNING
        self.emit("state", state=self._state)
        try:
            self._run(self._stop_event)
        except Exception as exc:  # pragma: no cover - exercised in tests
            with self._state_lock:
                self._state = DeviceState.FAULTED
            self.emit("error", message=str(exc))
        else:
            with self._state_lock:
                self._state = DeviceState.STOPPED
        self.emit("state", state=self.state)
