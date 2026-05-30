from __future__ import annotations

import queue
from types import SimpleNamespace

from machinist.core.events import Event
from machinist.core.types import DeviceState
from machinist.tui.app import (
    _arm_summary,
    _cmd_ls,
    _cmd_run,
    _detail_header,
    _format_event,
    _machine_summary,
    MachinistApp,
    _paint_lifecycle,
)
from machinist.devices.robots.arm import RobotArm
from machinist.devices.machines.state import MachineState, Toggle


def test_format_event_is_compact_and_deterministic() -> None:
    ev = Event(device="ur1", kind="rx", payload={"line": "power on"}, timestamp=1234.567)
    out = _format_event(ev)
    assert "ur1" in out and "rx" in out and "power on" in out
    assert "         rx" not in out


def test_paint_lifecycle_uses_expected_colours() -> None:
    assert _paint_lifecycle(DeviceState.RUNNING).startswith("[green]")
    assert _paint_lifecycle(DeviceState.FAULTED).startswith("[red]")


def test_arm_summary_is_empty_for_non_robot() -> None:
    assert _arm_summary(SimpleNamespace(arm=None)) == ""


def test_arm_summary_reports_estop_and_pose() -> None:
    arm = RobotArm(joint_count=6)
    arm.estop()
    out = _arm_summary(SimpleNamespace(arm=arm))
    assert "estopped" in out
    assert "ENGAGED" in out
    assert "joints" in out and "pose" in out


def test_machine_summary_is_empty_for_non_machine() -> None:
    assert _machine_summary(SimpleNamespace(state=None)) == ""
    assert _machine_summary(SimpleNamespace()) == ""


def test_machine_summary_reports_cycle_and_tooling() -> None:
    state = MachineState()
    state.doors["main"] = Toggle(name="main", open=True)
    state.program = "O0001\nG0 X0"
    state.spindle_rpm = 1500.0
    state.tool = 3
    state.parts = 7
    out = _machine_summary(SimpleNamespace(state=state))
    assert "O0001" in out
    assert "1500" in out
    assert "T3" in out
    assert "parts 7" in out
    assert "main:" in out


def test_detail_header_combines_static_and_dynamic_sections() -> None:
    state = MachineState()
    state.program = "O0001"
    device = SimpleNamespace(
        name="mill",
        kind="haas_ngc",
        endpoint="127.0.0.1:5051",
        lifecycle=DeviceState.RUNNING,
        state=state,
    )
    out = _detail_header(device)
    assert "mill" in out
    assert "haas_ngc" in out
    assert "program" in out


def test_drain_refreshes_selected_header_even_without_events() -> None:
    calls: list[str] = []
    app = SimpleNamespace(
        _events=queue.Queue(),
        _selected="robot1",
        _log=SimpleNamespace(write=lambda _msg: None),
        _refresh_devices_table=lambda: calls.append("table"),
        _refresh_detail=lambda: calls.append("detail"),
        _refresh_detail_header=lambda: calls.append("header"),
    )
    MachinistApp._drain(app)
    assert calls == ["header"]


class _FakeApp:
    def __init__(self, device) -> None:
        self._selected = device.name
        self._device = device
        self.writes: list[str] = []
        self._log = SimpleNamespace(write=self.writes.append)

    def _lookup(self, name):
        return self._device if (name in (None, self._device.name)) else None


def _fake_device(*, programs=None, run=None):
    return SimpleNamespace(name="haas1", programs=programs, run_program=run)


def test_cmd_ls_lists_programs() -> None:
    programs = SimpleNamespace(list=lambda: ["O0001.nc", "O0002.nc"])
    app = _FakeApp(_fake_device(programs=programs))
    _cmd_ls(app, "")
    assert any("O0001.nc" in w for w in app.writes)


def test_cmd_ls_complains_when_no_library() -> None:
    app = _FakeApp(_fake_device(programs=None))
    _cmd_ls(app, "")
    assert any("no program library" in w for w in app.writes)


def test_cmd_run_dispatches_program_name() -> None:
    called: list[str] = []
    device = _fake_device(
        programs=SimpleNamespace(list=lambda: []),
        run=called.append,
    )
    app = _FakeApp(device)
    _cmd_run(app, "haas1 O0001.nc")
    assert called == ["O0001.nc"]


def test_cmd_run_reports_errors() -> None:
    def boom(_name: str) -> None:
        raise RuntimeError("already running")
    app = _FakeApp(_fake_device(programs=None, run=boom))
    _cmd_run(app, "haas1 X.nc")
    assert any("already running" in w for w in app.writes)
