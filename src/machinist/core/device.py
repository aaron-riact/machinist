"""The :class:`Device` abstract base class.

Every emulated machine - robot arm, CNC, gripper, IO controller -
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
    via :meth:`emit`. They mark themselves *RUNNING* by calling
    :meth:`_mark_running` once their external listeners are bound.
    """

    #: Human-readable device kind (e.g. ``"ur_robot"``). Subclasses set this.
    kind: str = "device"

    def __init__(self, name: str, endpoint: Endpoint, bus: EventBus) -> None:
        self.name = name
        self.endpoint = endpoint
        self._bus = bus
        self._lifecycle = DeviceState.CREATED
        self._lifecycle_lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._ready = threading.Event()

    # ----- public API --------------------------------------------------

    @property
    def lifecycle(self) -> DeviceState:
        """Current lifecycle state. Distinct from any *domain* state."""
        with self._lifecycle_lock:
            return self._lifecycle

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._lifecycle is not DeviceState.CREATED:
                raise RuntimeError(f"{self.name} already started ({self._lifecycle})")
            self._lifecycle = DeviceState.STARTING
        self._thread = threading.Thread(
            target=self._thread_main, name=f"machinist-{self.name}", daemon=True
        )
        self._thread.start()

    def wait_ready(self, timeout: float = 5.0) -> bool:
        """Block until :meth:`_mark_running` has been called."""
        return self._ready.wait(timeout=timeout)

    def stop(self, *, timeout: float = 5.0) -> None:
        with self._lifecycle_lock:
            if self._lifecycle in (DeviceState.STOPPED, DeviceState.CREATED):
                return
            self._lifecycle = DeviceState.STOPPING
        self._stop_event.set()
        self._shutdown()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def emit(self, kind: str, **payload: Any) -> None:
        self._bus.publish(Event(device=self.name, kind=kind, payload=payload))

    # ----- subclass hooks ---------------------------------------------

    @abstractmethod
    def _run(self, stop: threading.Event) -> None:
        """Long-running worker. Returns when ``stop`` is set."""

    def _shutdown(self) -> None:
        """Optional hook for releasing OS resources before joining."""

    def _mark_running(self) -> None:
        """Subclasses call this once external listeners are bound."""
        with self._lifecycle_lock:
            self._lifecycle = DeviceState.RUNNING
        self._ready.set()
        self.emit("state", state=DeviceState.RUNNING)

    # ----- internals ---------------------------------------------------

    def _thread_main(self) -> None:
        try:
            self._run(self._stop_event)
        except Exception as exc:
            with self._lifecycle_lock:
                self._lifecycle = DeviceState.FAULTED
            self.emit("error", message=str(exc))
            self._ready.set()
            return
        with self._lifecycle_lock:
            self._lifecycle = DeviceState.STOPPED
        self.emit("state", state=DeviceState.STOPPED)
