from __future__ import annotations

import io
import json
from dataclasses import replace
from pathlib import Path

from machinist.core.eventlog import ConsoleLog, EventLog, to_jsonable
from machinist.core.events import EventBus, LifecycleChanged, Note
from machinist.core.io import Direction, SignalChanged
from machinist.core.types import DeviceState
from machinist.devices.robots.arm import ArmChanged, ArmMode, RobotArm


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


def test_the_console_log_prints_each_event_on_a_line() -> None:
    bus, out = EventBus(), io.StringIO()
    ConsoleLog(out).attach(bus)
    bus.publish(Note(device="up50n", name="command", data={"command": "SVON", "data": "1"}))
    assert out.getvalue() == '[up50n] command {"command":"SVON","data":"1"}\n'


def test_the_console_log_shows_lines_sent_and_received_only_when_debugging() -> None:
    for debug, lines in ((False, 0), (True, 1)):
        bus, out = EventBus(), io.StringIO()
        ConsoleLog(out, debug=debug).attach(bus)
        bus.publish(Note(device="up50n", name="rx", data={"line": "HOSTCTRL_REQUEST RSTATS 0"}))
        assert len(out.getvalue().splitlines()) == lines


def test_the_console_log_shows_the_arm_when_its_mode_changes() -> None:
    arm = RobotArm(joint_count=2)
    bus, out = EventBus(), io.StringIO()
    ConsoleLog(out).attach(bus)
    idle = arm.state.view
    moving = replace(idle, mode=ArmMode.MOVING, joints=(0.5, 0.0))
    for view in (idle, idle, moving, replace(moving, joints=(0.6, 0.0)), idle):
        bus.publish(ArmChanged(device="a", view=view))
    lines = out.getvalue().splitlines()
    assert [line.split(",")[0] for line in lines] == [
        "[a] arm idle", "[a] arm moving", "[a] arm idle",
    ]


def test_the_console_log_shows_every_arm_report_when_asked() -> None:
    arm = RobotArm(joint_count=2)
    bus, out = EventBus(), io.StringIO()
    ConsoleLog(out, include_arm=True).attach(bus)
    for _ in range(3):
        bus.publish(ArmChanged(device="a", view=arm.state.view))
    assert len(out.getvalue().splitlines()) == 3


def test_the_console_log_spares_a_device_that_prints_its_own_lines() -> None:
    bus, out = EventBus(), io.StringIO()
    ConsoleLog(out, debug=True, own_lines=["dobot"]).attach(bus)
    bus.publish(Note(device="dobot", name="rx", data={"line": "GetPose()"}))
    bus.publish(Note(device="dobot", name="quirk", data={}))
    assert out.getvalue() == "[dobot] quirk {}\n"
