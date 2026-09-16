"""The :class:`Device` abstract base class.

Every emulated machine — robot arm, CNC, gripper, IO controller —
implements this interface. The base class is intentionally thin: it owns
*lifecycle* (start/stop), the *services* the device serves, and *status
reporting*, and delegates everything else to subclasses.

Concurrency model: one worker thread per device. That thread starts each
registered :class:`~machinist.transport.service.Service` on a thread of
its own, waits for every one to bind, marks the device running, then runs
:meth:`Device._serve` until asked to stop. What a device does inside
``_serve`` (poll a bus, tick a simulation, or just wait) is its own
business.

Two names are carefully distinguished:

* ``lifecycle``  — the framework's DeviceState (created/starting/
  running/stopping/stopped/faulted). Owned here.
* ``state``      — reserved for the *domain* state of the device
  (machine state, gripper state, …). Owned by subclasses, if at all.
"""

from __future__ import annotations

import threading
from abc import ABC
from typing import Any

from ..transport.service import Service
from .capabilities import HasIO
from .events import DeviceFaulted, Event, EventBus, LifecycleChanged, Note
from .io import Direction, SignalBank
from .panel import Field, Panel
from .types import DeviceState, Endpoint

#: How long a device waits for a service thread to exit after shutdown.
SERVICE_JOIN_TIMEOUT = 2.0


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
        self._services: list[Service] = []

    # ----- public API --------------------------------------------------

    @property
    def services(self) -> tuple[Service, ...]:
        """The servers this device runs, in start order."""
        return tuple(self._services)

    def add_service(self, service: Service) -> None:
        """Register a server to start with the device. Call before :meth:`start`."""
        with self._lifecycle_lock:
            if self._lifecycle is not DeviceState.CREATED:
                raise RuntimeError(f"{self.name} already started; add services before start()")
        self._services.append(service)

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

    def publish(self, event: Event) -> None:
        """Put a typed event about this device on the bus."""
        self._bus.publish(event)

    def emit(self, kind: str, **payload: Any) -> None:
        """Publish an untyped :class:`Note` about this device (log-only)."""
        self._bus.publish(Note(device=self.name, name=kind, data=payload))

    def _publish_lifecycle(self) -> None:
        self.publish(LifecycleChanged(device=self.name, state=self.lifecycle))

    def build_detail(self) -> Panel:
        """The device's detail :class:`Panel`.

        The default lists the device's discrete IO as bit fields. Devices
        with a register map or an I/O block override it.
        """
        bank = self._signal_bank()
        if bank is None:
            return Panel()
        rows = {
            Direction.INPUT: [],
            Direction.OUTPUT: [],
        }
        for sig in bank:
            rows[sig.direction].append(
                Field(
                    signal=sig.name.upper(),
                    name=sig.name,
                    type="bit",
                    value="ON" if sig.value else "OFF",
                )
            )
        return Panel(
            input_fields=tuple(rows[Direction.INPUT]),
            output_fields=tuple(rows[Direction.OUTPUT]),
        )

    def _signal_bank(self) -> SignalBank | None:
        """The device's IO bank if it declares :class:`HasIO`, else ``None``."""
        return self.io if isinstance(self, HasIO) else None

    # ----- subclass hooks ---------------------------------------------

    def _serve(self, stop: threading.Event) -> None:
        """The device's own work, run once every service is up.

        Must return when *stop* is set. The default has no work of its own
        and simply waits; a device that polls or simulates overrides this.
        """
        stop.wait()

    def _shutdown(self) -> None:
        """Optional hook for releasing OS resources before joining."""

    def _run(self, stop: threading.Event) -> None:
        """The worker thread: start every service, serve, then tear down.

        Subclasses normally leave this alone and override :meth:`_serve`.
        """
        threads: list[threading.Thread] = []
        try:
            for service in self._services:
                threads.append(self._start_service(service))
            self._mark_running()
            self._serve(stop)
        finally:
            for service in self._services:
                service.shutdown()
            for thread in threads:
                thread.join(timeout=SERVICE_JOIN_TIMEOUT)

    def _start_service(self, service: Service) -> threading.Thread:
        """Serve *service* on its own thread and wait until it has bound."""
        label = type(service).__name__
        ready = threading.Event()
        thread = threading.Thread(
            target=service.serve_forever,
            args=(ready,),
            name=f"machinist-{self.name}-{label}",
            daemon=True,
        )
        thread.start()
        if not ready.wait(timeout=service.BIND_TIMEOUT):
            raise RuntimeError(f"{self.name}: {label} failed to bind")
        return thread

    # ----- subclass helpers -------------------------------------------

    def _mark_running(self) -> None:
        """Announce that the device is fully operational."""
        with self._lifecycle_lock:
            if self._lifecycle is DeviceState.STARTING:
                self._lifecycle = DeviceState.RUNNING
        self._ready.set()
        self._publish_lifecycle()

    # ----- internals ---------------------------------------------------

    def _thread_main(self) -> None:
        try:
            self._run(self._stop_event)
        except Exception as exc:
            with self._lifecycle_lock:
                self._lifecycle = DeviceState.FAULTED
            self._ready.set()
            self.publish(DeviceFaulted(device=self.name, message=str(exc)))
            self._publish_lifecycle()
            return
        with self._lifecycle_lock:
            self._lifecycle = DeviceState.STOPPED
        self._publish_lifecycle()
