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
from typing import ClassVar

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, Header, Input, RichLog, Static

from ..core.device import Device
from ..core.events import Event
from ..core.types import DeviceState
from ..core.world import World


class MachinistApp(App[None]):
    """Live, interactive view of a Machinist :class:`World`."""

    CSS = """
    #top { height: 60%; }
    #devices { width: 36; border: round #6e6cd1; }
    #detail-pane { border: round #6e6cd1; }
    #detail-header { height: 3; padding: 0 1; }
    #signals { height: 1fr; }
    RichLog#log { height: 30%; border: round #6e6cd1; padding: 0 1; }
    Input#cmd { dock: bottom; height: 3; border: round #6e6cd1; }
    """

    BINDINGS: ClassVar[list[Binding]] = [
        Binding("ctrl+c", "quit", "Quit"),
        Binding("e", "estop", "E-Stop selected"),
        Binding("r", "reset", "Reset selected"),
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
                self.signals = DataTable(id="signals", cursor_type="row", zebra_stripes=True)
                yield self.signals
        self.log = RichLog(id="log", wrap=False, max_lines=2000, highlight=False, markup=True)
        yield self.log
        self.cmd = Input(placeholder="◇ command (type 'help' for ideas)", id="cmd")
        yield self.cmd
        yield Footer()

    def on_mount(self) -> None:
        self.title = "◇ Machinist"
        self.sub_title = f"fleet of {len(self.world.devices)} device(s)"
        self.devices_table.add_columns("name", "kind", "endpoint", "state")
        self.signals.add_columns("signal", "value")
        self._refresh_devices_table()
        self.world.bus.subscribe(self._enqueue)
        self.set_interval(0.1, self._drain)
        self._refresh_detail()

    # ----- bus → UI -----------------------------------------------------

    def _enqueue(self, event: Event) -> None:
        try:
            self._events.put_nowait(event)
        except queue.Full:  # pragma: no cover
            pass

    def _drain(self) -> None:
        for _ in range(50):
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                return
            self.log.write(_format_event(event))
            if event.kind == "state":
                self._refresh_devices_table()
            if event.device == self._selected:
                self._refresh_detail()

    # ----- selection / detail -------------------------------------------

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        # Only react to the *devices* table; the signals table is read-only.
        if event.control is not self.devices_table:
            return
        row = self.devices_table.get_row_at(event.cursor_row)
        self._selected = str(row[0])
        self._refresh_detail()

    def _refresh_devices_table(self) -> None:
        self.devices_table.clear()
        for d in self.world.devices:
            self.devices_table.add_row(
                d.name, d.kind, str(d.endpoint), _paint_lifecycle(d.lifecycle)
            )

    def _refresh_detail(self) -> None:
        device = self._lookup(self._selected)
        if device is None:
            self.detail_header.update("[dim]no device selected[/]")
            self.signals.clear()
            return
        self.detail_header.update(
            f"[bold]{device.name}[/]  [dim]({device.kind})[/]\n"
            f"endpoint [magenta]{device.endpoint}[/]   "
            f"lifecycle {_paint_lifecycle(device.lifecycle)}"
        )
        self.signals.clear()
        bank = getattr(device, "io", None)
        if bank is None:
            return
        for sig in bank:
            dot = "[green]●[/]" if sig.value else "[red]●[/]"
            self.signals.add_row(f"{dot} {sig.name}", str(sig.value))

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
            self.log.write(f"[red]unknown command[/]: {verb}")
            return
        handler(self, rest)

    def _set_signal(self, target: str, value: bool) -> None:
        try:
            self.world.io_map._resolve(target).set(value)  # noqa: SLF001
            self.log.write(f"set [cyan]{target}[/] = {value}")
        except (KeyError, ValueError) as exc:
            self.log.write(f"[red]error[/]: {exc}")

    def _with_arm(self, name: str, fn: Callable) -> None:  # type: ignore[type-arg]
        device = self._lookup(name)
        arm = getattr(device, "arm", None)
        if arm is None:
            self.log.write(f"[red]{name}[/] has no arm")
            return
        fn(arm)
        self.log.write(f"applied to [cyan]{name}[/]")

    # ----- keybindings --------------------------------------------------

    def action_estop(self) -> None:
        if self._selected is not None:
            self._with_arm(self._selected, lambda arm: arm.estop())

    def action_reset(self) -> None:
        if self._selected is not None:
            self._with_arm(self._selected, lambda arm: arm.reset())


# --- stateless helpers --------------------------------------------------


_LIFECYCLE_COLOURS: dict[DeviceState, str] = {
    DeviceState.RUNNING: "green",
    DeviceState.FAULTED: "red",
    DeviceState.STOPPED: "grey50",
    DeviceState.STARTING: "yellow",
}


def _paint_lifecycle(state: DeviceState) -> str:
    return f"[{_LIFECYCLE_COLOURS.get(state, 'white')}]{state}[/]"


def _format_event(event: Event) -> str:
    payload = " ".join(f"{k}={v}" for k, v in event.payload.items())
    return (
        f"[dim]{event.timestamp:12.3f}[/] "
        f"[cyan]{event.device:<12}[/] "
        f"[magenta]{event.kind:<6}[/] {payload}"
    )


_COMMANDS: dict[str, Callable[[MachinistApp, str], None]] = {
    "help": lambda app, _: app.log.write(
        "[bold]commands[/]  estop <device> | reset <device> | "
        "set <device.signal> 0|1 | ls <device> | run <device> <program> | quit"
    ),
    "quit": lambda app, _: app.exit(),
    "estop": lambda app, rest: app._with_arm(rest.strip(), lambda arm: arm.estop()),
    "reset": lambda app, rest: app._with_arm(rest.strip(), lambda arm: arm.reset()),
    "set": lambda app, rest: _cmd_set(app, rest),
    "ls": lambda app, rest: _cmd_ls(app, rest),
    "run": lambda app, rest: _cmd_run(app, rest),
}


def _cmd_set(app: MachinistApp, rest: str) -> None:
    target, _, value = rest.partition(" ")
    app._set_signal(target, value.strip() in ("1", "true", "on"))


def _cmd_ls(app: MachinistApp, rest: str) -> None:
    device = app._lookup(rest.strip() or app._selected)
    programs = getattr(device, "programs", None)
    if programs is None:
        app.log.write(f"[red]{rest or 'selected'}[/] has no program library")
        return
    names = programs.list() or ["(empty)"]
    app.log.write(f"[cyan]{device.name}[/] programs: {', '.join(names)}")


def _cmd_run(app: MachinistApp, rest: str) -> None:
    target, _, program = rest.partition(" ")
    device = app._lookup(target.strip() or app._selected)
    run_program = getattr(device, "run_program", None)
    if run_program is None:
        app.log.write(f"[red]{target or 'selected'}[/] cannot run programs")
        return
    try:
        run_program(program.strip())
        app.log.write(f"started [cyan]{program.strip()}[/] on {device.name}")
    except (FileNotFoundError, RuntimeError) as exc:
        app.log.write(f"[red]error[/]: {exc}")
