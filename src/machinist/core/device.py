"""The :class:`Device` abstract base class.

Every emulated machine — robot arm, CNC, gripper, IO controller —
implements this interface. The base class is intentionally thin: it owns
*lifecycle* (start/stop) and *status reporting*, and delegates everything
else to subclasses.

Concurrency model: one worker thread per device. What the device runs
*inside* that thread (threads, asyncio, simpy, …) is its own business.

Two names are carefully distinguished:

* ``lifecycle``  — the framework's DeviceState (created/starting/
  running/stopping/stopped/faulted). Owned here.
* ``state``      — reserved for the *domain* state of the device
  (machine state, gripper state, …). Owned by subclasses, if at all.
"""

from __future__ import annotations

import threading
from abc import ABC, abstractmethod
from typing import Any

from .events import Event, EventBus
from .types import DeviceState, Endpoint


class Device(ABC):
    """Abstract emulated device."""

    #: Human-readable kind (e.g. ``"ur_dashboard"``). Subclasses set this.
    kind: str = "device"

    def __init__(self, name: str, endpoint: Endpoint, bus: EventBus) -> None:
        self.name = name
        self.endpoint = endpoint
        self._bus = bus
        self._lifecycle = DeviceState.CREATED
        self._lifecycle_lock = threading.Lock()
        self._ready = threading.Event()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()

    # ----- public API --------------------------------------------------

    @property
    def lifecycle(self) -> DeviceState:
        """Framework-owned lifecycle phase (see :class:`DeviceState`)."""
        with self._lifecycle_lock:
            return self._lifecycle

    def start(self) -> None:
        """Spawn the device worker thread."""
        with self._lifecycle_lock:
            if self._lifecycle is not DeviceState.CREATED:
                raise RuntimeError(f"{self.name} already started ({self._lifecycle})")
            self._lifecycle = DeviceState.STARTING
        self._thread = threading.Thread(
            target=self._thread_main, name=f"machinist-{self.name}", daemon=True
        )
        self._thread.start()

    def stop(self, *, timeout: float = 5.0) -> None:
        """Request shutdown and wait for the worker to exit."""
        with self._lifecycle_lock:
            if self._lifecycle in (DeviceState.STOPPED, DeviceState.CREATED):
                return
            self._lifecycle = DeviceState.STOPPING
        self._stop_event.set()
        self._shutdown()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def wait_ready(self, *, timeout: float = 2.0) -> bool:
        """Block until the device has bound its listener(s)."""
        return self._ready.wait(timeout=timeout)

    def emit(self, kind: str, **payload: Any) -> None:
        """Publish a status :class:`Event` for this device."""
        self._bus.publish(Event(device=self.name, kind=kind, payload=payload))

    # ----- subclass hooks ---------------------------------------------

    @abstractmethod
    def _run(self, stop: threading.Event) -> None:
        """Long-running worker; must return when ``stop`` is set.

        Subclasses MUST call :meth:`_mark_running` once their listener
        (TCP socket, HTTP server, …) is bound and accepting connections.
        """

    def _shutdown(self) -> None:
        """Optional hook for releasing OS resources before joining."""

    # ----- subclass helpers -------------------------------------------

    def _mark_running(self) -> None:
        """Announce that the device is fully operational."""
        with self._lifecycle_lock:
            if self._lifecycle is DeviceState.STARTING:
                self._lifecycle = DeviceState.RUNNING
        self._ready.set()
        self.emit("state", state=str(self._lifecycle))

    # ----- internals ---------------------------------------------------

    def _thread_main(self) -> None:
        try:
            self._run(self._stop_event)
        except Exception as exc:
            with self._lifecycle_lock:
                self._lifecycle = DeviceState.FAULTED
            self._ready.set()
            self.emit("error", message=str(exc))
            self.emit("state", state=str(self._lifecycle))
            return
        with self._lifecycle_lock:
            self._lifecycle = DeviceState.STOPPED
        self.emit("state", state=str(self._lifecycle))
