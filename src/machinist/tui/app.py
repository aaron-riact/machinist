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

* **RichLog** (not ``Log``) for the event panel — it renders Rich
  markup faithfully, whereas ``Log`` has highlighting quirks that
  produced wide, ragged columns.
* **Signals as DataTable** — natively scrollable and navigable; copes
  with hundreds of IOs without blowing past the panel's bounds.
* **Bounded queue + drain-per-tick** — publisher threads (from device
  ``EventBus``) never block on the UI.
"""

from __future__ import annotations

import queue
from collections.abc import Callable
from contextlib import suppress
from typing import ClassVar

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, Header, Input, RichLog, Static

from ..core.device import Device
from ..core.events import Event
from ..core.io import Direction
from ..core.types import DeviceState
from ..core.world import World


class MachinistApp(App[None]):
    """Live, interactive view of a Machinist :class:`World`."""

    CSS = """
    #top { height: 60%; }
    #devices { width: 44; border: round #6e6cd1; }
    #detail-pane { border: round #6e6cd1; }
    #detail-header { height: auto; padding: 0 1; }
    #signals-row { height: 1fr; }
    #inputs, #outputs { width: 1fr; }
    #detail-lower { height: 40%; }
    #files, #registers { width: 1fr; border-top: dashed #6e6cd1; }
    #detail-lower.hidden { display: none; }
    RichLog#log { height: 1fr; border: round #6e6cd1; padding: 0 1; }
    Input#cmd { dock: bottom; height: 3; border: round #6e6cd1; }
    """

    BINDINGS: ClassVar[list[Binding]] = [
        Binding("ctrl+c", "quit", "Quit"),
        Binding("e", "estop", "E-Stop selected"),
        Binding("r", "reset", "Reset selected"),
        Binding("f", "toggle_files", "Files panel"),
    ]

    def __init__(self, world: World) -> None:
        super().__init__()
        self.world = world
        self._events: queue.Queue[Event] = queue.Queue(maxsize=4096)
        self._selected: str | None = (
            world.devices[0].name if world.devices else None
        )

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
                    )
                    yield self.inputs
                    self.outputs = DataTable(
                        id="outputs", cursor_type="row", zebra_stripes=True,
                    )
                    yield self.outputs
                with Horizontal(id="detail-lower"):
                    self.files = DataTable(
                        id="files", cursor_type="row", zebra_stripes=True,
                    )
                    yield self.files
                    self.registers = DataTable(
                        id="registers", cursor_type="row", zebra_stripes=True,
                    )
                    yield self.registers
        self._log = RichLog(id="log", wrap=False, max_lines=2000, highlight=False, markup=True)
        yield self._log
        self.cmd = Input(placeholder="◇ command (type 'help' for ideas)", id="cmd")
        yield self.cmd
        yield Footer()

    def on_mount(self) -> None:
        self.title = "◇ Machinist"
        self.sub_title = f"fleet of {len(self.world.devices)} device(s)"
        self.devices_table.add_columns("name", "kind", "state")
        self.inputs.add_columns("input", "value")
        self.outputs.add_columns("output", "value")
        self.files.add_columns("program")
        self.registers.add_columns("dir", "field", "offset", "type", "value")
        self._refresh_devices_table()
        self.world.bus.subscribe(self._enqueue)
        self.set_interval(0.1, self._drain)
        self._refresh_detail()

    # ----- bus → UI -----------------------------------------------------

    def _enqueue(self, event: Event) -> None:
        with suppress(queue.Full):  # pragma: no cover
            self._events.put_nowait(event)

    def _drain(self) -> None:
        for _ in range(50):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            self._log.write(_format_event(event))
            if event.kind == "state":
                self._refresh_devices_table()
            if event.device == self._selected:
                self._refresh_detail()
                return
        self._refresh_detail_header()

    # ----- selection / detail -------------------------------------------

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        if event.control is self.devices_table:
            row = self.devices_table.get_row_at(event.cursor_row)
            self._selected = str(row[0])
            self._refresh_detail()
            return
        if event.control is self.files:
            program = str(self.files.get_row_at(event.cursor_row)[0])
            _cmd_run(self, f"{self._selected or ''} {program}")
            return

    def _refresh_devices_table(self) -> None:
        self.devices_table.clear()
        for d in self.world.devices:
            self.devices_table.add_row(
                d.name, d.kind, _paint_lifecycle(d.lifecycle)
            )

    def _refresh_detail(self) -> None:
        device = self._lookup(self._selected)
        if device is None:
            self.detail_header.update("[dim]no device selected[/]")
            self.inputs.clear()
            self.outputs.clear()
            self.files.clear()
            self.registers.clear()
            return
        self._refresh_detail_header(device)
        self.inputs.clear()
        self.outputs.clear()
        bank = getattr(device, "io", None)
        if bank is not None:
            for sig in bank:
                dot = "[green]●[/]" if sig.value else "[red]●[/]"
                table = (
                    self.outputs if sig.direction is Direction.OUTPUT else self.inputs
                )
                table.add_row(f"{dot} {sig.name}", str(sig.value))
        self._refresh_files(device)
        self._refresh_registers(device)

    def _refresh_detail_header(self, device: Device | None = None) -> None:
        current = device or self._lookup(self._selected)
        if current is None:
            self.detail_header.update("[dim]no device selected[/]")
            return
        self.detail_header.update(_detail_header(current))

    def _refresh_files(self, device: Device) -> None:
        self.files.clear()
        programs = getattr(device, "programs", None)
        if programs is None:
            return
        for name in programs.list():
            self.files.add_row(name)

    def _refresh_registers(self, device: Device) -> None:
        self.registers.clear()
        snapshot = _ethernetip_snapshot(device)
        if snapshot is None:
            return
        for direction, fields in [
            ("IN", snapshot["input_fields"]),
            ("OUT", snapshot["output_fields"]),
            ("DRV", snapshot["derived_fields"]),
        ]:
            for field in fields:
                self.registers.add_row(
                    direction,
                    f"{field['signal']} {field['name']}",
                    field["offset"],
                    field["type"],
                    field["value"],
                )

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
            self.world.io_map._resolve(target).set(value)
            self._log.write(f"set [cyan]{target}[/] = {value}")
        except (KeyError, ValueError) as exc:
            self._log.write(f"[red]error[/]: {exc}")

    def _with_arm(self, name: str, fn: Callable) -> None:  # type: ignore[type-arg]
        device = self._lookup(name)
        arm = getattr(device, "arm", None)
        if arm is None:
            self._log.write(f"[red]{name}[/] has no arm")
            return
        fn(arm)
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


# --- stateless helpers --------------------------------------------------


_LIFECYCLE_COLOURS: dict[DeviceState, str] = {
    DeviceState.RUNNING: "green",
    DeviceState.FAULTED: "red",
    DeviceState.STOPPED: "grey50",
    DeviceState.STARTING: "yellow",
}


def _paint_lifecycle(state: DeviceState) -> str:
    return f"[{_LIFECYCLE_COLOURS.get(state, 'white')}]{state}[/]"


def _detail_header(device: Device) -> str:
    return (
        f"[bold]{device.name}[/]  [dim]({device.kind})[/]\n"
        f"endpoint [magenta]{device.endpoint}[/]   "
        f"lifecycle {_paint_lifecycle(device.lifecycle)}"
        f"{_arm_summary(device)}"
        f"{_machine_summary(device)}"
        f"{_ethernetip_summary(device)}"
    )


def _arm_summary(device: Device) -> str:
    """One-line-per-fact robot status, or '' for non-robot devices."""
    arm = getattr(device, "arm", None)
    if arm is None:
        return ""
    s = arm.state.snapshot()
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


def _machine_summary(device: Device) -> str:
    """One-line CNC status (cycle/program/spindle/tool/parts), or '' otherwise."""
    state = getattr(device, "state", None)
    if state is None or not hasattr(state, "cycle"):
        return ""
    cycle = str(state.cycle)
    cycle_colour = (
        "green" if cycle == "running" else "yellow" if cycle == "paused" else "grey50"
    )
    program = state.program.splitlines()[0] if state.program else "[dim]none[/]"
    xyz = f"{state.position.x:+.3f}  {state.position.y:+.3f}  {state.position.z:+.3f}"
    doors = "  ".join(
        f"{name}:{'[red]open[/]' if door.open else '[green]shut[/]'}"
        for name, door in state.doors.items()
    )
    return (
        f"\ncycle [{cycle_colour}]{cycle}[/]   program [cyan]{program}[/]\n"
        f"xyz [yellow]{xyz}[/]\n"
        f"spindle [yellow]{state.spindle_rpm:g}[/] rpm   feed {state.feed:g}   "
        f"tool [magenta]T{state.tool}[/]   parts {state.parts}\n"
        f"doors  {doors or '[dim]none[/]'}"
    )


def _ethernetip_summary(device: Device) -> str:
    snapshot = _ethernetip_snapshot(device)
    if snapshot is None:
        return ""
    peer = "peer up" if snapshot["peer_connected"] else "waiting"
    ready = "ready" if snapshot["transport_ready"] else "offline"
    return (
        f"\neip mode [cyan]{snapshot['mode']}[/]   transport {ready}   "
        f"link {peer}"
    )


def _ethernetip_snapshot(device: Device) -> dict[str, object] | None:
    snapshot = getattr(device, "ethernetip_snapshot", None)
    if snapshot is None or not callable(snapshot):
        return None
    return snapshot()


def _format_event(event: Event) -> str:
    payload = " ".join(f"{k}={v}" for k, v in event.payload.items())
    return (
        f"[dim]{event.timestamp:12.3f}[/] "
        f"[cyan]{event.device:<12}[/] "
        f"[magenta]{event.kind:<6}[/] {payload}"
    )


def _cmd_set(app: MachinistApp, rest: str) -> None:
    target, _, value = rest.partition(" ")
    app._set_signal(target, value.strip() in ("1", "true", "on"))


def _cmd_ls(app: MachinistApp, rest: str) -> None:
    device = app._lookup(rest.strip() or app._selected)
    programs = getattr(device, "programs", None)
    if programs is None:
        app._log.write(f"[red]{rest or 'selected'}[/] has no program library")
        return
    names = programs.list() or ["(empty)"]
    app._log.write(f"[cyan]{device.name}[/] programs: {', '.join(names)}")


def _cmd_run(app: MachinistApp, rest: str) -> None:
    target, _, program = rest.partition(" ")
    device = app._lookup(target.strip() or app._selected)
    run_program = getattr(device, "run_program", None)
    if run_program is None:
        app._log.write(f"[red]{target or 'selected'}[/] cannot run programs")
        return
    try:
        run_program(program.strip())
        app._log.write(f"started [cyan]{program.strip()}[/] on {device.name}")
    except (FileNotFoundError, RuntimeError) as exc:
        app._log.write(f"[red]error[/]: {exc}")


_COMMANDS: dict[str, Callable[[MachinistApp, str], None]] = {
    "help": lambda app, _: app._log.write(
        "[bold]commands[/]  estop <device> | reset <device> | "
        "set <device.signal> 0|1 | ls <device> | run <device> <program> | quit"
    ),
    "quit": lambda app, _: app.exit(),
    "estop": lambda app, rest: app._with_arm(rest.strip(), lambda arm: arm.estop()),
    "reset": lambda app, rest: app._with_arm(rest.strip(), lambda arm: arm.reset()),
    "set": _cmd_set,
    "ls": _cmd_ls,
    "run": _cmd_run,
}
