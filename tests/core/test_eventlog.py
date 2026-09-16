from __future__ import annotations

import json
from pathlib import Path

from machinist.core.eventlog import EventLog, to_jsonable
from machinist.core.events import EventBus, LifecycleChanged, Note
from machinist.core.io import Direction, SignalChanged
from machinist.core.types import DeviceState
from machinist.devices.robots.arm import ArmChanged, RobotArm


def test_events_are_appended_as_json_lines(tmp_path: Path) -> None:
    bus = EventBus()
    log = EventLog(tmp_path / "fleet.jsonl")
    log.attach(bus)
    bus.publish(LifecycleChanged(device="g1", state=DeviceState.RUNNING, timestamp=1.0))
    bus.publish(SignalChanged(device="g1", signal="o1", direction=Direction.OUTPUT, value=True, timestamp=2.0))
    bus.publish(Note(device="g1", name="rx", data={"line": "hi"}, timestamp=3.0))
    log.close()
    bus.publish(Note(device="g1", name="rx", data={"line": "after close"}))

    lines = [json.loads(line) for line in (tmp_path / "fleet.jsonl").read_text().splitlines()]
    assert [r["kind"] for r in lines] == ["state", "signal", "rx"]
    assert lines[0] == {"timestamp": 1.0, "device": "g1", "kind": "state", "payload": {"state": "running"}}
    assert lines[1]["payload"] == {"signal": "o1", "direction": "output", "value": True}
    assert lines[2]["payload"] == {"line": "hi"}


def test_nested_views_are_written_as_plain_objects() -> None:
    arm = RobotArm(joint_count=2)
    record = to_jsonable(ArmChanged(device="a", view=arm.state.view).payload)
    assert record["view"]["mode"] == "idle"
    assert record["view"]["joints"] == [0.0, 0.0]
