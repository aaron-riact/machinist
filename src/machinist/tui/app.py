"""Claude-code styled Textual UI for Machinist.

Layout::

    +------------------------------------------------------------+
    |   ◇ Machinist                                              |
    +---------------+--------------------------------------------+
    | devices table | detail header (kind/endpoint/lifecycle)    |
    |               +--------------------------------------------+
    |               | signals grid (scrollable, multi-column)    |
    +---------------+--------------------------------------------+
    | event log (rich-formatted, auto-scrolls)                   |
    +------------------------------------------------------------+
    | ◇ command bar                                              |
    +------------------------------------------------------------+

Design decisions worth knowing:

* **The screen is a projection.** Everything painted comes from the
  :class:`~machinist.projection.Projection`'s :class:`FleetState`, never
  from a device. Devices announce every change on the bus; the projection
  folds them; the UI repaints when the fleet's version moved.
* **RichLog** (not ``Log``) for the event panel — it renders Rich
  markup faithfully, whereas ``Log`` has highlighting quirks that
  produced wide, ragged columns.
* **Signals as DataTable** — natively scrollable and navigable; copes
  with hundreds of IOs without blowing past the panel's bounds.
* **Bounded queue + drain-per-tick** — publisher threads (from device
  ``EventBus``) never block on the UI. The tick is only the hand-off to
  the UI thread; it paints nothing unless something changed.
"""

from __future__ import annotations

import queue
from collections.abc import Callable
from contextlib import suppress
from typing import ClassVar

from rich.style import Style
from rich.text import Text
from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, Header, Input, RichLog, Static

from ..core.capabilities import HasPrograms
from ..core.device import Device
from ..core.events import DeviceFaulted, Event, LifecycleChanged, Note
from ..core.io import Direction, SignalChanged
from ..core.panel import Field, Panel
from ..core.types import DeviceState
from ..core.world import World
from ..devices.machines.state import MachineView
from ..devices.robots.arm import ArmStateView, HasArm, RobotArm
from ..projection import DeviceView, FleetState, Projection
from ..web.api import CommandError, dispatch_command

#: Event types worth a line in the log. Continuous state (arm ticks, panel
#: and machine views) is painted, not logged.
_LOGGED = (Note, LifecycleChanged, DeviceFaulted, SignalChanged)


class MachinistApp(App[None]):
    """Live, interactive view of a Machinist :class:`World`."""

    CSS = """
    #top { height: 60%; }
    #top.log-small { height: 1fr; }
    #top.log-large { height: 20%; }
    #devices { width: 44; border: round #6e6cd1; }
    #detail-pane { border: round #6e6cd1; }
    #detail-header { height: auto; padding: 0 1; }
    #signals-row { height: 1fr; }
    #inputs, #outputs { width: 1fr; }
    #detail-lower { height: 40%; }
    #files, #derived { width: 1fr; border-top: dashed #6e6cd1; }
    #detail-lower.hidden { display: none; }
    RichLog#log { height: 1fr; border: round #6e6cd1; padding: 0 1; }
    RichLog#log.log-small { height: 3; }
    Input#cmd { dock: bottom; height: 3; border: round #6e6cd1; }
    """

    BINDINGS: ClassVar[list[Binding]] = [
        Binding("ctrl+c", "quit", "Quit"),
        Binding("e", "estop", "E-Stop selected"),
        Binding("r", "reset", "Reset selected"),
        Binding("f", "toggle_files", "Files panel"),
        Binding("l", "toggle_log", "Log size"),
    ]

    def __init__(self, world: World) -> None:
        super().__init__()
        self.world = world
        self.projection = Projection(world)
        self._events: queue.Queue[Event] = queue.Queue(maxsize=4096)
        self._selected: str | None = (
            world.devices[0].name if world.devices else None
        )
        self._painted: DeviceView | None = None  # the detail as last drawn
        self._painted_version = -1
        self._log_size: int = 0  # 0=medium, 1=small, 2=large

    # ----- widgets -----------------------------------------------------

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="top"):
            self.devices_table = DataTable(id="devices", cursor_type="row")
            yield self.devices_table
            with Vertical(id="detail-pane"):
                self.detail_header = Static(id="detail-header")
                yield self.detail_header
                with Horizontal(id="signals-row"):
                    self.inputs = DataTable(
                        id="inputs", cursor_type="row", zebra_stripes=True,
                        cursor_foreground_priority="renderable",
                    )
                    yield self.inputs
                    self.outputs = DataTable(
                        id="outputs", cursor_type="row", zebra_stripes=True,
                        cursor_foreground_priority="renderable",
                    )
                    yield self.outputs
                with Horizontal(id="detail-lower"):
                    self.files = DataTable(
                        id="files", cursor_type="row", zebra_stripes=True,
                    )
                    yield self.files
                    self.derived = DataTable(
                        id="derived", cursor_type="row", zebra_stripes=True,
                        cursor_foreground_priority="renderable",
                    )
                    yield self.derived
        self._log = RichLog(id="log", wrap=False, max_lines=2000, highlight=False, markup=True)
        yield self._log
        self.cmd = Input(placeholder="◇ command (type 'help' for ideas)", id="cmd")
        yield self.cmd
        yield Footer()

    def on_mount(self) -> None:
        self.title = "◇ Machinist"
        self.sub_title = f"fleet of {len(self.world.devices)} device(s)"
        self.devices_table.add_columns("name", "kind", "state")
        (
            self._inputs_col_label,
            _,
            self._inputs_col_value,
        ) = self.inputs.add_columns("input", "offset", "value")
        (
            self._outputs_col_label,
            _,
            self._outputs_col_value,
        ) = self.outputs.add_columns("output", "offset", "value")
        self.files.add_columns("program")
        self._derived_col_field, self._derived_col_value = self.derived.add_columns("field", "value")
        self.world.bus.subscribe(self._enqueue)
        self.set_interval(0.05, self._drain)
        self._paint(self.projection.state)

    # ----- bus → UI -----------------------------------------------------

    def _enqueue(self, event: Event) -> None:
        if isinstance(event, _LOGGED):
            with suppress(queue.Full):  # pragma: no cover
                self._events.put_nowait(event)

    def _drain(self) -> None:
        """Log what arrived, then repaint if the projection moved on."""
        for _ in range(50):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            self._log.write(_format_event(event))
        state = self.projection.state
        if state.version != self._painted_version:
            self._paint(state)

    def _paint(self, state: FleetState) -> None:
        self._painted_version = state.version
        self._refresh_devices_table(state)
        self._refresh_detail(state)

    # ----- selection / detail -------------------------------------------

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        if event.control is self.devices_table:
            row = self.devices_table.get_row_at(event.cursor_row)
            self._selected = str(row[0])
            self._refresh_detail(self.projection.state)
            return
        if event.control is self.files:
            program = str(self.files.get_row_at(event.cursor_row)[0])
            _cmd_run(self, f"{self._selected or ''} {program}")
            return

    def _refresh_devices_table(self, state: FleetState) -> None:
        self.devices_table.clear()
        for view in state.devices.values():
            self.devices_table.add_row(view.name, view.kind, _paint_lifecycle(view.lifecycle))

    def _selected_view(self, state: FleetState) -> DeviceView | None:
        return state.devices.get(self._selected) if self._selected else None

    def _refresh_detail(self, state: FleetState) -> None:
        view = self._selected_view(state)
        if view is None:
            self.detail_header.update("[dim]no device selected[/]")
            for table in (self.inputs, self.outputs, self.files, self.derived):
                table.clear()
            self._painted = None
            return
        self.detail_header.update(_detail_header(view))

        inputs, outputs = _io_rows(view)
        derived = view.panel.derived_fields
        previous = self._painted
        same_shape = (
            previous is not None
            and previous.name == view.name
            and len(_io_rows(previous)[0]) == len(inputs)
            and len(_io_rows(previous)[1]) == len(outputs)
            and len(previous.panel.derived_fields) == len(derived)
            and self.inputs.row_count == len(inputs)
        )
        dot = _dot_painter(view)

        if not same_shape:
            self.inputs.clear()
            self.outputs.clear()
            self.derived.clear()
            for field in inputs:
                self.inputs.add_row(dot(field) + " " + field.signal + " " + field.name, field.offset, field.value)
            for field in outputs:
                self.outputs.add_row(dot(field) + " " + field.signal + " " + field.name, field.offset, field.value)
            for field in derived:
                self.derived.add_row(f"{field.signal} {field.name}", field.value)
        else:
            input_keys = list(self.inputs.rows.keys())
            output_keys = list(self.outputs.rows.keys())
            derived_keys = list(self.derived.rows.keys())
            for i, field in enumerate(inputs):
                self.inputs.update_cell(input_keys[i], self._inputs_col_label, dot(field) + " " + field.signal + " " + field.name)
                self.inputs.update_cell(input_keys[i], self._inputs_col_value, field.value)
            for i, field in enumerate(outputs):
                self.outputs.update_cell(output_keys[i], self._outputs_col_label, dot(field) + " " + field.signal + " " + field.name)
                self.outputs.update_cell(output_keys[i], self._outputs_col_value, field.value)
            for i, field in enumerate(derived):
                self.derived.update_cell(derived_keys[i], self._derived_col_value, field.value)

        if previous is None or previous.name != view.name or previous.programs != view.programs:
            self._refresh_files(view)
        self._painted = view

    def _refresh_files(self, view: DeviceView) -> None:
        self.files.clear()
        for name in view.programs or ():
            self.files.add_row(name)

    def _lookup(self, name: str | None) -> Device | None:
        if name is None:
            return None
        return next((d for d in self.world.devices if d.name == name), None)

    # ----- command bar -------------------------------------------------

    def on_input_submitted(self, event: Input.Submitted) -> None:
        self._dispatch_command(event.value.strip())
        self.cmd.value = ""

    def _dispatch_command(self, line: str) -> None:
        if not line:
            return
        verb, _, rest = line.partition(" ")
        handler = _COMMANDS.get(verb)
        if handler is None:
            self._log.write(f"[red]unknown command[/]: {verb}")
            return
        handler(self, rest)

    def _set_signal(self, target: str, value: bool) -> None:
        try:
            self.world.io_map.signal(target).set(value)
            self._log.write(f"set [cyan]{target}[/] = {value}")
        except (KeyError, ValueError) as exc:
            self._log.write(f"[red]error[/]: {exc}")

    def _with_arm(self, name: str, fn: Callable[[RobotArm], None]) -> None:
        device = self._lookup(name)
        if not isinstance(device, HasArm):
            self._log.write(f"[red]{name}[/] has no arm")
            return
        fn(device.arm)
        self._log.write(f"applied to [cyan]{name}[/]")

    # ----- keybindings --------------------------------------------------

    def action_estop(self) -> None:
        if self._selected is not None:
            self._with_arm(self._selected, lambda arm: arm.estop())

    def action_reset(self) -> None:
        if self._selected is not None:
            self._with_arm(self._selected, lambda arm: arm.reset())

    def action_toggle_files(self) -> None:
        self.query_one("#detail-lower").toggle_class("hidden")

    def action_toggle_log(self) -> None:
        top = self.query_one("#top")
        log = self.query_one(RichLog)
        top.remove_class("log-small", "log-large")
        log.remove_class("log-small", "log-large")
        self._log_size = (self._log_size + 1) % 3
        if self._log_size == 1:
            top.add_class("log-small")
            log.add_class("log-small")
        elif self._log_size == 2:
            top.add_class("log-large")
            log.add_class("log-large")


# --- stateless helpers --------------------------------------------------


_LIFECYCLE_COLOURS: dict[DeviceState, str] = {
    DeviceState.RUNNING: "green",
    DeviceState.FAULTED: "red",
    DeviceState.STOPPED: "grey50",
    DeviceState.STARTING: "yellow",
}


def _paint_lifecycle(state: DeviceState) -> str:
    return f"[{_LIFECYCLE_COLOURS.get(state, 'white')}]{state}[/]"


def _detail_header(view: DeviceView) -> str:
    fault = f"\n[red]fault[/] {view.fault}" if view.fault else ""
    return (
        f"[bold]{view.name}[/]  [dim]({view.kind})[/]\n"
        f"endpoint [magenta]{view.endpoint}[/]   "
        f"lifecycle {_paint_lifecycle(view.lifecycle)}{fault}"
        f"{_arm_summary(view.arm)}"
        f"{_machine_summary(view.machine)}"
        f"{_panel_summary(view.panel)}"
    )


def _arm_summary(s: ArmStateView | None) -> str:
    """One-line-per-fact robot status, or '' for non-robot devices."""
    if s is None:
        return ""
    mode = s.mode
    mode_colour = "red" if mode in ("estopped", "faulted") else "green"
    joints = "  ".join(f"{j:+.3f}" for j in s.joints)
    pose = "  ".join(f"{p:+.3f}" for p in s.pose)
    command = s.current_command or "[dim]none[/]"
    estop = "[red]ENGAGED[/]" if mode == "estopped" else "[green]clear[/]"
    return (
        f"\nmode [{mode_colour}]{mode}[/]   servo {'on' if s.servo_on else 'off'}   "
        f"e-stop {estop}   command [cyan]{command}[/]\n"
        f"joints [yellow]{joints}[/]\n"
        f"pose   [magenta]{pose}[/]"
    )


def _machine_summary(state: MachineView | None) -> str:
    """One-line CNC status (cycle/program/spindle/tool/parts), or '' otherwise."""
    if state is None:
        return ""
    cycle = str(state.cycle)
    cycle_colour = (
        "green" if cycle == "running" else "yellow" if cycle == "paused" else "grey50"
    )
    program = state.program.splitlines()[0] if state.program else "[dim]none[/]"
    xyz = f"{state.position.x:+.3f}  {state.position.y:+.3f}  {state.position.z:+.3f}"
    doors = "  ".join(
        f"{name}:{'[red]open[/]' if door_open else '[green]shut[/]'}"
        for name, door_open in state.doors.items()
    )
    return (
        f"\ncycle [{cycle_colour}]{cycle}[/]   program [cyan]{program}[/]\n"
        f"xyz [yellow]{xyz}[/]\n"
        f"spindle [yellow]{state.spindle_rpm:g}[/] rpm   feed {state.feed:g}   "
        f"tool [magenta]T{state.tool}[/]   parts {state.parts}\n"
        f"doors  {doors or '[dim]none[/]'}"
    )


def _panel_summary(panel: Panel) -> str:
    if panel.mode == "io":
        return ""
    if panel.clients is not None:
        return f"\n{panel.mode}   {panel.clients} client(s)"
    peer = "peer up" if panel.peer_connected else "waiting"
    ready = "ready" if panel.transport_ready else "offline"
    return f"\n{panel.mode}   transport {ready}   link {peer}"


def _io_rows(view: DeviceView) -> tuple[tuple[Field, ...], tuple[Field, ...]]:
    """The input and output tables: the panel's own rows, else the device's signals."""
    if view.panel.input_fields or view.panel.output_fields:
        return view.panel.input_fields, view.panel.output_fields
    rows = {Direction.INPUT: [], Direction.OUTPUT: []}
    for sig in view.signals.values():
        rows[sig.direction].append(
            Field(signal=sig.name.upper(), name=sig.name, type="bit", value="ON" if sig.value else "OFF")
        )
    return tuple(rows[Direction.INPUT]), tuple(rows[Direction.OUTPUT])


def _dot_painter(view: DeviceView) -> Callable[[Field], Text]:
    """Green/red dot for bit fields, looked up case-insensitively in the device's signals."""
    values = {name.lower(): sig.value for name, sig in view.signals.items()}

    def dot(field: Field) -> Text:
        if field.type not in ("bit", "bool"):
            return Text(" ")
        on = values.get(field.signal.lower())
        if on is None:
            on = field.value.upper() in ("ON", "1", "TRUE")
        t = Text("●")
        t.stylize(Style(color="green" if on else "red"))
        return t

    return dot


def _format_event(event: Event) -> str:
    payload = " ".join(f"{k}={v}" for k, v in event.payload.items())
    return (
        f"[dim]{event.timestamp:12.3f}[/] "
        f"[cyan]{event.device:<12}[/] "
        f"[magenta]{event.kind:<6}[/] {payload}"
    )


def _cmd_fault(app: MachinistApp, verb: str, rest: str) -> None:
    """Run a fault-injection verb through the shared web dispatcher.

    Delegating rather than reimplementing keeps the two command surfaces
    from drifting: there is one parser for ``pstop``/``failenable``, and the
    TUI only has to render the result.
    """
    target, _, tail = rest.strip().partition(" ")
    if not target:
        target = app._selected or ""
    try:
        result = dispatch_command(app.world, f"{verb} {target} {tail}".strip())
    except CommandError as exc:
        app._log.write(f"[red]error[/]: {exc}")
        return
    app._log.write(result["message"])


def _cmd_set(app: MachinistApp, rest: str) -> None:
    target, _, value = rest.partition(" ")
    app._set_signal(target, value.strip() in ("1", "true", "on"))


def _cmd_ls(app: MachinistApp, rest: str) -> None:
    device = app._lookup(rest.strip() or app._selected)
    if not isinstance(device, HasPrograms):
        app._log.write(f"[red]{rest or 'selected'}[/] has no program library")
        return
    names = device.programs.list() or ["(empty)"]
    app._log.write(f"[cyan]{device.name}[/] programs: {', '.join(names)}")


def _cmd_run(app: MachinistApp, rest: str) -> None:
    target, _, program = rest.partition(" ")
    device = app._lookup(target.strip() or app._selected)
    if not isinstance(device, HasPrograms):
        app._log.write(f"[red]{target or 'selected'}[/] cannot run programs")
        return
    try:
        device.run_program(program.strip())
        app._log.write(f"started [cyan]{program.strip()}[/] on {device.name}")
    except (FileNotFoundError, RuntimeError) as exc:
        app._log.write(f"[red]error[/]: {exc}")


_COMMANDS: dict[str, Callable[[MachinistApp, str], None]] = {
    "help": lambda app, _: app._log.write(
        "[bold]commands[/]  estop <device> | reset <device> | "
        "set <device.signal> 0|1 | ls <device> | run <device> <program> | "
        "pstop <device> [clear|collision|error|disabled] [ids=17,116] [sticky] | "
        "failenable <device> stuck|error|off | quit"
    ),
    "quit": lambda app, _: app.exit(),
    "estop": lambda app, rest: app._with_arm(rest.strip(), lambda arm: arm.estop()),
    "reset": lambda app, rest: app._with_arm(rest.strip(), lambda arm: arm.reset()),
    "set": _cmd_set,
    "ls": _cmd_ls,
    "run": _cmd_run,
    "pstop": lambda app, rest: _cmd_fault(app, "pstop", rest),
    "failenable": lambda app, rest: _cmd_fault(app, "failenable", rest),
}
