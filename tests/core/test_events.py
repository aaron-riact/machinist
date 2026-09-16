from __future__ import annotations

from machinist.core.events import Event, EventBus, LifecycleChanged, Note
from machinist.core.types import DeviceState


def test_publish_invokes_subscribers() -> None:
    bus = EventBus()
    received: list[Event] = []
    bus.subscribe(received.append)
    bus.publish(Note(device="d", name="rx", data={"x": 1}))
    assert received[0].device == "d"
    assert received[0].payload == {"x": 1}


def test_unsubscribe() -> None:
    bus = EventBus()
    received: list[Event] = []
    unsub = bus.subscribe(received.append)
    unsub()
    bus.publish(Note(device="d", name="rx"))
    assert received == []


def test_note_reports_its_name_and_data_as_kind_and_payload() -> None:
    note = Note(device="d", name="rx", data={"line": "hello"})
    assert note.kind == "rx"
    assert note.payload == {"line": "hello"}


def test_typed_event_reports_its_fields_as_payload() -> None:
    event = LifecycleChanged(device="d", state=DeviceState.RUNNING, timestamp=1.0)
    assert event.kind == "state"
    assert event.payload == {"state": DeviceState.RUNNING}
    assert event.device == "d" and event.timestamp == 1.0
