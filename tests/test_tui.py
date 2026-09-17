from __future__ import annotations

import queue
from types import MappingProxyType, SimpleNamespace

from textual.widgets._data_table import ColumnKey, RowKey

from machinist.commands import Commands
from machinist.core.events import Event, LifecycleChanged, Note
from machinist.core.io import Direction
from machinist.core.panel import Field, Panel
from machinist.core.types import DeviceState
from machinist.devices.machines.state import MachineState
from machinist.devices.robots.arm import RobotArm
from machinist.projection import DeviceView, FleetState, SignalView
from machinist.tui.app import (
    MachinistApp,
    _arm_summary,
    _detail_header,
    _format_event,
    _io_rows,
    _machine_summary,
    _paint_lifecycle,
    _panel_summary,
)

from .fakes import RecordingDobot


def _view(name: str = "dev1", kind: str = "fake", **fields: object) -> DeviceView:
    return DeviceView(name=name, kind=kind, endpoint="127.0.0.1:1", **fields)  # type: ignore[arg-type]


def _signals(**values: bool) -> MappingProxyType:
    return MappingProxyType(
        {
            name: SignalView(name=name, direction=Direction.INPUT if name.startswith("i") else Direction.OUTPUT, value=v)
            for name, v in values.items()
        }
    )


# --- pure renderers -------------------------------------------------------


def test_format_event_is_compact_and_deterministic() -> None:
    ev = Note(device="ur1", name="rx", data={"line": "power on"}, timestamp=1234.567)
    out = _format_event(ev)
    assert "ur1" in out
    assert "rx" in out
    assert "power on" in out
    assert "         rx" not in out


def test_paint_lifecycle_uses_expected_colours() -> None:
    assert _paint_lifecycle(DeviceState.RUNNING).startswith("[green]")
    assert _paint_lifecycle(DeviceState.FAULTED).startswith("[red]")


def test_arm_summary_is_empty_without_an_arm() -> None:
    assert _arm_summary(None) == ""


def test_arm_summary_reports_estop_and_pose() -> None:
    arm = RobotArm(joint_count=6)
    arm.estop()
    out = _arm_summary(arm.state.view)
    assert "estopped" in out
    assert "ENGAGED" in out
    assert "joints" in out
    assert "pose" in out


def test_machine_summary_is_empty_without_a_machine() -> None:
    assert _machine_summary(None) == ""


def test_machine_summary_reports_cycle_and_tooling() -> None:
    state = MachineState()
    state.set_door("main", open=True)
    state.update(program="O0001\nG0 X0", spindle_rpm=1500.0, tool=3, parts=7)
    state.move_to(x=12.0, y=-3.5, z=8.25)
    out = _machine_summary(state.view)
    assert "O0001" in out
    assert "+12.000" in out
    assert "1500" in out
    assert "T3" in out
    assert "parts 7" in out
    assert "main:" in out


def test_detail_header_combines_static_and_dynamic_sections() -> None:
    state = MachineState()
    state.update(program="O0001")
    view = _view("mill", kind="haas_ngc", machine=state.view, fault="boom")
    out = _detail_header(view)
    assert "mill" in out
    assert "haas_ngc" in out
    assert "program" in out
    assert "boom" in out


def test_panel_summary_reports_mode_and_link_state() -> None:
    out = _panel_summary(Panel(mode="adapter", transport_ready=True, peer_connected=False))
    assert "adapter" in out
    assert "waiting" in out
    assert _panel_summary(Panel()) == ""


def test_io_rows_fall_back_to_signals_when_the_panel_has_none() -> None:
    view = _view(signals=_signals(i1=True, o1=False))
    inputs, outputs = _io_rows(view)
    assert [(f.signal, f.value) for f in inputs] == [("I1", "ON")]
    assert [(f.signal, f.value) for f in outputs] == [("O1", "OFF")]


def test_io_rows_prefer_the_panels_own_rows() -> None:
    panel = Panel(input_fields=(Field("T_FORCE", "Target force", value="400"),))
    inputs, outputs = _io_rows(_view(panel=panel, signals=_signals(i1=True)))
    assert [f.signal for f in inputs] == ["T_FORCE"]
    assert outputs == ()


# --- the tick: log, then paint only when the projection moved --------------


def _drain_app(q: queue.Queue | None = None, *, version: int = 0, painted: int = 0):
    painted_states: list[FleetState] = []
    logged: list[str] = []
    app = SimpleNamespace(
        _events=q if q is not None else queue.Queue(),
        _log=SimpleNamespace(write=logged.append),
        projection=SimpleNamespace(state=FleetState(version=version)),
        _painted_version=painted,
        _paint=painted_states.append,
    )
    return app, painted_states, logged


def test_drain_paints_nothing_when_the_projection_did_not_move() -> None:
    app, painted, _ = _drain_app(version=3, painted=3)
    MachinistApp._drain(app)
    assert painted == []


def test_drain_paints_when_the_projection_moved() -> None:
    app, painted, _ = _drain_app(version=4, painted=3)
    MachinistApp._drain(app)
    assert len(painted) == 1 and painted[0].version == 4


def test_drain_logs_the_queued_events() -> None:
    q: queue.Queue[Event] = queue.Queue()
    q.put(Note(device="robot1", name="rx", data={"line": "hi"}))
    q.put(LifecycleChanged(device="robot1", state=DeviceState.RUNNING))
    app, _, logged = _drain_app(q)
    MachinistApp._drain(app)
    assert len(logged) == 2 and "hi" in logged[0] and "running" in logged[1]


# --- the detail panel: populate, then update in place, then rebuild --------


class _MockTable:
    def __init__(self) -> None:
        self.rows: dict[RowKey, object] = {}
        self.row_count = 0
        self.log: list[tuple] = []

    def clear(self) -> None:
        self.rows.clear()
        self.row_count = 0
        self.log.append(("clear",))

    def add_row(self, *cells: object, key: str | None = None) -> RowKey:
        rk = RowKey(key)
        self.rows[rk] = object()
        self.row_count = len(self.rows)
        self.log.append(("add_row",) + cells)
        return rk

    def update_cell(self, row_key: object, column_key: object, value: object) -> None:
        self.log.append(("update_cell", row_key, column_key, value))


def _detail_app(selected: str | None) -> SimpleNamespace:
    return SimpleNamespace(
        _selected=selected,
        _painted=None,
        inputs=_MockTable(),
        outputs=_MockTable(),
        derived=_MockTable(),
        files=_MockTable(),
        detail_header=SimpleNamespace(update=lambda _: None),
        _inputs_col_label=ColumnKey("input"),
        _inputs_col_value=ColumnKey("value"),
        _outputs_col_label=ColumnKey("output"),
        _outputs_col_value=ColumnKey("value"),
        _derived_col_field=ColumnKey("field"),
        _derived_col_value=ColumnKey("value"),
        _selected_view=lambda state: state.devices.get(selected) if selected else None,
        _refresh_files=lambda view: None,
    )


def test_refresh_detail_populates_then_updates_in_place_then_rebuilds_on_switch() -> None:
    dev1 = _view("dev1", signals=_signals(i1=True, o1=False))
    dev2 = _view("dev2", signals=_signals(i1=False, o1=True))
    state = FleetState.of([dev1, dev2])
    app = _detail_app("dev1")

    MachinistApp._refresh_detail(app, state)
    assert ("clear",) in app.inputs.log
    assert app.inputs.row_count == 1 and app.outputs.row_count == 1

    # same device, a value changed: cells are updated, rows are not rebuilt
    app.inputs.log.clear(); app.outputs.log.clear()
    changed = state.with_device(dev1.with_signal(SignalView("i1", Direction.INPUT, False)))
    MachinistApp._refresh_detail(app, changed)
    assert not any(c[0] in ("clear", "add_row") for c in app.inputs.log)
    updates = [c for c in app.inputs.log if c[0] == "update_cell"]
    assert len(updates) == 2 and updates[0][2] is app._inputs_col_label

    # another device: rebuilt
    app._selected = "dev2"
    app._selected_view = lambda st: st.devices.get("dev2")
    app.inputs.log.clear()
    MachinistApp._refresh_detail(app, changed)
    assert any(c[0] == "clear" for c in app.inputs.log)
    assert any(c[0] == "add_row" for c in app.inputs.log)

    # nothing selected: everything cleared
    app._selected = None
    app._selected_view = lambda st: None
    app.derived.log.clear()
    MachinistApp._refresh_detail(app, changed)
    assert ("clear",) in app.derived.log
    assert app._painted is None


def test_refresh_devices_table_updates_rows_in_place_and_never_clears() -> None:
    """Rebuilding the table resets its cursor, so a highlighted device would jump away."""
    a = _view("a", lifecycle=DeviceState.CREATED)
    b = _view("b", lifecycle=DeviceState.CREATED)
    app = SimpleNamespace(devices_table=_MockTable(), _devices_col_state=ColumnKey("state"))

    MachinistApp._refresh_devices_table(app, FleetState.of([a, b]))
    assert [c[0] for c in app.devices_table.log] == ["add_row", "add_row"]
    assert {k.value for k in app.devices_table.rows} == {"a", "b"}

    app.devices_table.log.clear()
    running = FleetState.of([a, DeviceView(name="b", kind="fake", endpoint="e", lifecycle=DeviceState.RUNNING)])
    MachinistApp._refresh_devices_table(app, running)

    assert [c[0] for c in app.devices_table.log] == ["update_cell", "update_cell"]
    assert ("clear",) not in app.devices_table.log
    updated = [c for c in app.devices_table.log if c[1] == "b"][0]
    assert "running" in updated[3]


def test_refresh_files_lists_the_programs_of_the_view() -> None:
    app = SimpleNamespace(files=_MockTable())
    MachinistApp._refresh_files(app, _view("haas1", programs=("O0001.nc", "O0002.nc")))
    assert [c[1] for c in app.files.log if c[0] == "add_row"] == ["O0001.nc", "O0002.nc"]
    MachinistApp._refresh_files(app, _view("io1"))
    assert app.files.log[-1] == ("clear",)


# --- the command bar delegates to the shared dispatcher --------------------


def _command_app(device) -> SimpleNamespace:
    writes: list[str] = []
    exits: list[bool] = []
    app = SimpleNamespace(
        commands=Commands(SimpleNamespace(devices=[device])),
        _selected=device.name,
        _log=SimpleNamespace(write=writes.append),
        writes=writes,
        exit=lambda: exits.append(True),
        exits=exits,
    )
    app._dispatch_command = lambda line: MachinistApp._dispatch_command(app, line)
    return app


def test_dispatch_command_runs_the_shared_verbs_with_the_selection() -> None:
    dobot = RecordingDobot()
    app = _command_app(dobot)
    MachinistApp._dispatch_command(app, "pstop error ids=17,116 sticky")
    assert dobot.stops == [{"robot_mode": 9, "controller_ids": (17, 116), "sticky": True}]
    assert any("protective stop error" in w for w in app.writes)


def test_dispatch_command_logs_errors_instead_of_raising() -> None:
    app = _command_app(RecordingDobot())
    MachinistApp._dispatch_command(app, "pstop dobot1 sideways")
    assert any("unknown pstop option" in w for w in app.writes)
    MachinistApp._dispatch_command(app, "frobnicate")
    assert any("unknown command" in w for w in app.writes)


def test_dispatch_command_owns_quit_and_ignores_blank_lines() -> None:
    app = _command_app(RecordingDobot())
    MachinistApp._dispatch_command(app, "   ")
    assert app.writes == []
    MachinistApp._dispatch_command(app, "quit")
    assert app.exits == [True]


def test_keybindings_go_through_the_same_dispatcher() -> None:
    dobot = RecordingDobot()
    app = _command_app(dobot)
    MachinistApp.action_estop(app)
    assert dobot.arm.state.view.estopped
    MachinistApp.action_reset(app)
    assert not dobot.arm.state.view.estopped
