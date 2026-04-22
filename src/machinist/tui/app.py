"""Claude-code styled Textual UI.

Layout:

    +-------------------------------------------------------------+
    |  ◇ Machinist  ─ a fleet of N device(s)                      |
    +---------------+---------------------------------------------+
    | devices table |     selected device detail                  |
    |               |     - endpoint, lifecycle                   |
    |               |     - signals (live grid)                   |
    |               |     - last events stream                    |
    +---------------+---------------------------------------------+
    | event log (all devices) — scrolling                         |
    +-------------------------------------------------------------+

Single-pane "command bar" at the bottom mirrors a CLI for power users
(``estop ur1``, ``set io1.o5 1`` …). Updates are pushed onto an internal
queue from the bus and drained on Textual's next tick so the UI never
blocks publisher threads.
"""

from __future__ import annotations

import queue
from typing import ClassVar

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal, Vertical
from textual.widgets import DataTable, Footer, Header, Input, Log, Static

from ..core.events import Event
from ..core.types import DeviceState
from ..core.world import World


class MachinistApp(App[None]):
    """Live, interactive view of a Machinist :class:`World`."""

    CSS = """
    Screen { layout: vertical; }
    #top { height: 60%; }
    #devices { width: 40%; border: round #6e6cd1; }
    #detail { width: 60%; border: round #6e6cd1; padding: 0 1; }
    #log { height: 30%; border: round #6e6cd1; }
    #cmd { dock: bottom; height: 3; }
    Header { background: #16161e; color: #c0caf5; }
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

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="top"):
            self.devices_table = DataTable(id="devices", cursor_type="row")
            yield self.devices_table
            self.detail = Static(id="detail")
            yield self.detail
        self._log = Log(id="log", highlight=True)
        yield self._log
        self.cmd = Input(placeholder="◇ command (type 'help' for ideas)", id="cmd")
        yield self.cmd
        yield Footer()

    def on_mount(self) -> None:
        self.title = "◇ Machinist"
        self.sub_title = f"a fleet of {len(self.world.devices)} device(s)"
        self.devices_table.add_columns("name", "kind", "endpoint", "state")
        for d in self.world.devices:
            self.devices_table.add_row(d.name, d.kind, str(d.endpoint), str(d.lifecycle))
        self.world.bus.subscribe(self._enqueue)
        self.set_interval(0.1, self._drain)
        self._refresh_detail()

    # ----- bus -> UI ---------------------------------------------------

    def _enqueue(self, event: Event) -> None:
        try:
            self._events.put_nowait(event)
        except queue.Full:  # pragma: no cover
            pass

    def _drain(self) -> None:
        drained = 0
        while drained < 50:
            try:
                event = self._events.get_nowait()
            except queue.Empty:
                break
            self._log.write_line(self._format(event))
            if event.kind == "state":
                self._refresh_devices_table()
            if event.device == self._selected:
                self._refresh_detail()
            drained += 1

    @staticmethod
    def _format(event: Event) -> str:
        ts = f"[{event.timestamp:.3f}]"
        payload = " ".join(f"{k}={v}" for k, v in event.payload.items())
        return f"{ts} {event.device:>16} · {event.kind:<10} {payload}"

    # ----- selection / detail ------------------------------------------

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        row = self.devices_table.get_row_at(event.cursor_row)
        self._selected = str(row[0])
        self._refresh_detail()

    def _refresh_devices_table(self) -> None:
        self.devices_table.clear()
        for d in self.world.devices:
            self.devices_table.add_row(d.name, d.kind, str(d.endpoint), str(d.lifecycle))

    def _refresh_detail(self) -> None:
        d = self._lookup(self._selected)
        if d is None:
            self.detail.update("No device selected")
            return
        lines = [
            f"[bold]{d.name}[/]  ({d.kind})",
            f"endpoint:  [magenta]{d.endpoint}[/]",
            f"lifecycle: {self._lifecycle_color(d.lifecycle)}",
        ]
        bank = getattr(d, "io", None)
        if bank is not None:
            lines.append("\n[bold]signals[/]")
            for sig in bank:
                color = "green" if sig.value else "red"
                lines.append(f"  [{color}]●[/] {sig.name:<24} = {sig.value}")
        self.detail.update("\n".join(lines))

    @staticmethod
    def _lifecycle_color(state: DeviceState) -> str:
        return {
            DeviceState.RUNNING: f"[green]{state}[/]",
            DeviceState.FAULTED: f"[red]{state}[/]",
            DeviceState.STOPPED: f"[grey]{state}[/]",
        }.get(state, str(state))

    def _lookup(self, name: str | None):
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
        match verb:
            case "help":
                self._log.write_line(
                    "[bold]commands[/]  estop <device> | reset <device> | "
                    "set <device.signal> 0|1 | quit"
                )
            case "quit":
                self.exit()
            case "estop":
                self._with_arm(rest, lambda arm: arm.estop())
            case "reset":
                self._with_arm(rest, lambda arm: arm.reset())
            case "set":
                target, _, value = rest.partition(" ")
                self._set_signal(target, value.strip() in ("1", "true", "on"))
            case _:
                self._log.write_line(f"[red]unknown command[/]: {verb}")

    def _with_arm(self, name: str, fn) -> None:  # type: ignore[no-untyped-def]
        d = self._lookup(name)
        arm = getattr(d, "arm", None)
        if arm is None:
            self._log.write_line(f"[red]{name}[/] has no arm")
            return
        fn(arm)
        self._log.write_line(f"applied to [cyan]{name}[/]")

    def _set_signal(self, target: str, value: bool) -> None:
        try:
            self.world.io_map._resolve(target).set(value)  # noqa: SLF001
            self._log.write_line(f"set [cyan]{target}[/] = {value}")
        except (KeyError, ValueError) as exc:
            self._log.write_line(f"[red]error[/]: {exc}")

    # ----- bindings ---------------------------------------------------

    def action_estop(self) -> None:
        if self._selected is not None:
            self._with_arm(self._selected, lambda arm: arm.estop())

    def action_reset(self) -> None:
        if self._selected is not None:
            self._with_arm(self._selected, lambda arm: arm.reset())
