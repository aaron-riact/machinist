from __future__ import annotations

from machinist.core.events import Event, EventBus


def test_publish_invokes_subscribers() -> None:
    bus = EventBus()
    received: list[Event] = []
    bus.subscribe(received.append)
    bus.publish(Event(device="d", kind="state", payload={"x": 1}))
    assert received[0].device == "d"
    assert received[0].payload == {"x": 1}


def test_unsubscribe() -> None:
    bus = EventBus()
    received: list[Event] = []
    unsub = bus.subscribe(received.append)
    unsub()
    bus.publish(Event(device="d", kind="state"))
    assert received == []
