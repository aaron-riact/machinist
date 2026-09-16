"""The base Device runs every registered Service from one loop."""

from __future__ import annotations

import threading

import pytest

from machinist.core.device import Device
from machinist.core.events import DeviceFaulted, Event, EventBus, LifecycleChanged
from machinist.core.types import DeviceState, Endpoint
from machinist.transport.service import Service


class _RecordingService(Service):
    """Binds at once (or never), and records the calls it receives."""

    def __init__(self, *, binds: bool = True) -> None:
        self.binds = binds
        self.served = threading.Event()
        self.shut_down = threading.Event()
        self._stop = threading.Event()

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        self.served.set()
        if self.binds and ready is not None:
            ready.set()
        self._stop.wait()

    def shutdown(self) -> None:
        self.shut_down.set()
        self._stop.set()


class _Idle(Device):
    kind = "idle"


class _Polling(Device):
    kind = "polling"

    def __init__(self, *args: object) -> None:
        super().__init__(*args)  # type: ignore[arg-type]
        self.polls = 0

    def _serve(self, stop: threading.Event) -> None:
        while not stop.is_set():
            self.polls += 1
            stop.wait(0.005)


def _device(cls: type[Device] = _Idle, bus: EventBus | None = None) -> Device:
    return cls("dev1", Endpoint("127.0.0.1", 0), bus or EventBus())


@pytest.mark.timeout(5)
def test_services_are_served_then_shut_down_in_order() -> None:
    device = _device()
    first, second = _RecordingService(), _RecordingService()
    device.add_service(first)
    device.add_service(second)
    assert device.services == (first, second)

    device.start()
    assert device.wait_ready(timeout=2.0)
    assert device.lifecycle is DeviceState.RUNNING
    assert first.served.is_set() and second.served.is_set()

    device.stop()
    assert first.shut_down.is_set() and second.shut_down.is_set()
    assert device.lifecycle is DeviceState.STOPPED


@pytest.mark.timeout(5)
def test_lifecycle_moves_are_published_as_typed_events() -> None:
    events: list[Event] = []
    bus = EventBus()
    bus.subscribe(events.append)
    device = _device(bus=bus)
    device.start()
    assert device.wait_ready(timeout=2.0)
    device.stop()
    states = [e.state for e in events if isinstance(e, LifecycleChanged)]
    assert states == [DeviceState.RUNNING, DeviceState.STOPPED]


@pytest.mark.timeout(5)
def test_a_device_with_no_services_still_runs_and_stops() -> None:
    device = _device()
    device.start()
    assert device.wait_ready(timeout=2.0)
    device.stop()
    assert device.lifecycle is DeviceState.STOPPED


@pytest.mark.timeout(5)
def test_serve_hook_runs_between_startup_and_shutdown() -> None:
    device = _device(_Polling)
    device.start()
    assert device.wait_ready(timeout=2.0)
    deadline = threading.Event()
    deadline.wait(0.05)
    device.stop()
    assert device.polls > 1  # type: ignore[attr-defined]


@pytest.mark.timeout(5)
def test_a_service_that_never_binds_faults_the_device(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(_RecordingService, "BIND_TIMEOUT", 0.05)
    events: list[Event] = []
    bus = EventBus()
    bus.subscribe(events.append)
    device = _device(bus=bus)
    good, stuck = _RecordingService(), _RecordingService(binds=False)
    device.add_service(good)
    device.add_service(stuck)

    device.start()
    device.wait_ready(timeout=2.0)
    assert device._thread is not None
    device._thread.join(timeout=2.0)

    assert device.lifecycle is DeviceState.FAULTED
    faults = [e for e in events if isinstance(e, DeviceFaulted)]
    assert len(faults) == 1 and "failed to bind" in faults[0].message
    assert good.shut_down.is_set(), "services that did start are torn down again"


def test_services_cannot_be_added_after_start() -> None:
    device = _device()
    device.start()
    try:
        with pytest.raises(RuntimeError, match="before start"):
            device.add_service(_RecordingService())
    finally:
        device.stop()


@pytest.mark.timeout(5)
def test_a_slow_service_may_declare_a_longer_bind_timeout() -> None:
    class Slow(_RecordingService):
        BIND_TIMEOUT = 1.0

        def serve_forever(self, ready: threading.Event | None = None) -> None:
            self._stop.wait(0.2)  # slower than the default would allow
            super().serve_forever(ready)

    device = _device()
    device.add_service(Slow())
    device.start()
    try:
        assert device.wait_ready(timeout=2.0)
        assert device.lifecycle is DeviceState.RUNNING
    finally:
        device.stop()
