from __future__ import annotations

import queue
from types import SimpleNamespace

from textual.widgets._data_table import ColumnKey, RowKey

from machinist.core.events import Event
from machinist.core.types import DeviceState
from machinist.devices.machines.state import MachineState, Toggle
from machinist.devices.robots.arm import RobotArm
from machinist.tui.app import (
    MachinistApp,
    _arm_summary,
    _cmd_ls,
    _cmd_run,
    _detail_header,
    _snapshot_summary,
    _format_event,
    _machine_summary,
    _paint_lifecycle,
)


def test_format_event_is_compact_and_deterministic() -> None:
    ev = Event(device="ur1", kind="rx", payload={"line": "power on"}, timestamp=1234.567)
    out = _format_event(ev)
    assert "ur1" in out
    assert "rx" in out
    assert "power on" in out
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
    assert "joints" in out
    assert "pose" in out


def test_machine_summary_is_empty_for_non_machine() -> None:
    assert _machine_summary(SimpleNamespace(state=None)) == ""
    assert _machine_summary(SimpleNamespace()) == ""


def test_machine_summary_reports_cycle_and_tooling() -> None:
    state = MachineState()
    state.doors["main"] = Toggle(name="main", open=True)
    state.program = "O0001\nG0 X0"
    state.position.x = 12.0
    state.position.y = -3.5
    state.position.z = 8.25
    state.spindle_rpm = 1500.0
    state.tool = 3
    state.parts = 7
    out = _machine_summary(SimpleNamespace(state=state))
    assert "O0001" in out
    assert "+12.000" in out
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


def test_snapshot_summary_reports_mode_and_link_state() -> None:
    device = SimpleNamespace(
        ethernetip_snapshot=lambda: {
            "mode": "adapter",
            "transport_ready": True,
            "peer_connected": False,
        }
    )
    out = _snapshot_summary(device)
    assert "adapter" in out
    assert "waiting" in out


def test_drain_does_not_refresh_when_no_events() -> None:
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
    assert calls == []


def test_drain_refreshes_detail_on_event_for_selected_device() -> None:
    q: queue.Queue[Event] = queue.Queue()
    q.put(Event(device="robot1", kind="moving", payload={"diameter_mm": 42.0}))
    calls: list[str] = []
    app = SimpleNamespace(
        _events=q,
        _selected="robot1",
        _log=SimpleNamespace(write=lambda _msg: None),
        _refresh_devices_table=lambda: calls.append("table"),
        _refresh_detail=lambda: calls.append("detail"),
    )
    MachinistApp._drain(app)
    assert calls == ["detail"]


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


class _MockTable:
    def __init__(self) -> None:
        self.rows: dict[RowKey, object] = {}
        self.row_count = 0
        self.cursor_row = -1
        self.log: list[tuple] = []

    def clear(self) -> None:
        self.rows.clear()
        self.row_count = 0
        self.log.append(("clear",))

    def add_row(self, *cells: object) -> RowKey:
        rk = RowKey()
        self.rows[rk] = object()
        self.row_count = len(self.rows)
        self.log.append(("add_row",) + cells)
        return rk

    def update_cell(self, row_key: object, column_key: object, value: object) -> None:
        self.log.append(("update_cell", row_key, column_key, value))

    def move_cursor(self, row: int = 0) -> None:
        pass


def test_refresh_detail_populates_then_increments_then_rebuilds_on_switch() -> None:
    """First call: clear + add_row.
    Second call (same device): update_cell only (no clear/add_row).
    Third call (different device): clear + add_row again.
    Fourth call (None selected): clears everything.
    """
    from machinist.tui.app import Direction

    sigs = [
        SimpleNamespace(name="i1", value=True, direction=Direction.INPUT),
        SimpleNamespace(name="o1", value=False, direction=Direction.OUTPUT),
    ]
    device1 = SimpleNamespace(
        name="dev1", kind="test", endpoint="ep1",
        lifecycle=DeviceState.RUNNING, io=sigs,
    )
    device2 = SimpleNamespace(
        name="dev2", kind="test", endpoint="ep2",
        lifecycle=DeviceState.RUNNING, io=sigs,
    )

    in_label = ColumnKey("input")
    in_value = ColumnKey("value")
    out_label = ColumnKey("output")
    out_value = ColumnKey("value")
    der_value = ColumnKey("value")

    inputs = _MockTable()
    outputs = _MockTable()
    derived = _MockTable()

    app = SimpleNamespace(
        _selected="dev1",
        _lookup=lambda name: (device1 if name == "dev1" else device2) if name else None,
        _last_selected=None,
        inputs=inputs,
        outputs=outputs,
        derived=derived,
        files=_MockTable(),
        detail_header=SimpleNamespace(update=lambda _: None),
        _inputs_col_label=in_label,
        _inputs_col_value=in_value,
        _outputs_col_label=out_label,
        _outputs_col_value=out_value,
        _derived_col_field=ColumnKey("field"),
        _derived_col_value=der_value,
        _refresh_detail_header=lambda _device: None,
        _refresh_files=lambda _device: None,
    )

    # --- first call: populate ---
    MachinistApp._refresh_detail(app)
    assert ("clear",) in inputs.log
    assert any(c[0] == "add_row" for c in inputs.log)
    assert any(c[0] == "add_row" for c in outputs.log)
    assert inputs.row_count == 1
    assert outputs.row_count == 1

    # --- second call: same device → incremental update_cell ---
    inputs.log.clear()
    outputs.log.clear()
    derived.log.clear()
    MachinistApp._refresh_detail(app)

    assert not any(c[0] == "clear" for c in inputs.log)
    assert not any(c[0] == "add_row" for c in inputs.log)
    input_updates = [c for c in inputs.log if c[0] == "update_cell"]
    output_updates = [c for c in outputs.log if c[0] == "update_cell"]
    assert len(input_updates) == 2  # label + value
    assert len(output_updates) == 2
    # ColumnKey objects passed by identity (not strings)
    assert input_updates[0][2] is in_label
    assert input_updates[1][2] is in_value
    assert output_updates[0][2] is out_label
    assert output_updates[1][2] is out_value

    # --- third call: different device → rebuild ---
    app._selected = "dev2"
    inputs.log.clear()
    outputs.log.clear()
    derived.log.clear()
    MachinistApp._refresh_detail(app)
    last_clear = max(
        (i for i, c in enumerate(inputs.log) if c[0] == "clear"), default=-1
    )
    last_add = max(
        (i for i, c in enumerate(inputs.log) if c[0] == "add_row"), default=-1
    )
    assert last_clear >= 0 and last_add >= 0
    assert any(c[0] == "clear" for c in inputs.log)  # rebuild: clear called
    assert any(c[0] == "add_row" for c in inputs.log)

    # --- fourth call: no device selected ---
    app._selected = None
    app._last_selected = None
    inputs.log.clear()
    outputs.log.clear()
    derived.log.clear()
    MachinistApp._refresh_detail(app)
    assert ("clear",) in inputs.log
    assert ("clear",) in outputs.log
    assert ("clear",) in derived.log
    assert app._last_selected is None
