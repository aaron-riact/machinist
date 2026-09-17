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
  folds them; the panels in :mod:`machinist.tui.widgets` paint the views.
* **Pushed, not polled.** The projection's listener posts one
  :class:`FleetMoved` message (coalesced: at most one in flight), and each
  loggable bus event posts an :class:`EventArrived`. Textual delivers both
  on the UI thread; nothing wakes on a timer.
* **One command surface.** The bar, the file table and the keybindings all
  go through :class:`~machinist.commands.Commands`, the same object the web
  endpoint uses.
"""

from __future__ import annotations

import threading
from typing import ClassVar

from textual.app import App, ComposeResult
from textual.binding import Binding
from textual.containers import Horizontal
from textual.message import Message
from textual.widgets import DataTable, Footer, Header, Input

from ..commands import HELP, CommandError, Commands
from ..core.events import DeviceFaulted, Event, LifecycleChanged, Note
from ..core.io import SignalChanged
from ..core.world import World
from ..projection import FleetState, Projection
from .widgets import DetailPane, DeviceList, EventLogPanel, ProgramList

#: Event types worth a line in the log. Continuous state (arm ticks, panel
#: and machine views) is painted, not logged.
_LOGGED = (Note, LifecycleChanged, DeviceFaulted, SignalChanged)


class FleetMoved(Message):
    """The projection has a newer fleet state to paint."""


class EventArrived(Message):
    def __init__(self, event: Event) -> None:
        super().__init__()
        self.event = event


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
    #files { height: 40%; border-top: dashed #6e6cd1; }
    EventLogPanel#log { height: 1fr; border: round #6e6cd1; padding: 0 1; }
    EventLogPanel#log.log-small { height: 3; }
    Input#cmd { dock: bottom; height: 3; border: round #6e6cd1; }
    """

    BINDINGS: ClassVar[list[Binding]] = [
        Binding("ctrl+c", "quit", "Quit"),
        Binding("e", "estop", "E-Stop selected"),
        Binding("r", "reset", "Reset selected"),
        Binding("f", "toggle_programs", "Programs"),
        Binding("l", "toggle_log", "Log size"),
        Binding("colon,slash", "focus_command", "Command", key_display=":"),
        Binding("escape", "focus_devices", "Devices", show=False),
    ]

    def __init__(self, world: World) -> None:
        super().__init__()
        self.world = world
        self.projection = Projection(world)
        self.commands = Commands(world)
        self._selected: str | None = world.devices[0].name if world.devices else None
        self._repaint_pending = threading.Event()
        self._log_size: int = 0  # 0=medium, 1=small, 2=large
        self._unsubscribe: list = []

    # ----- widgets -----------------------------------------------------

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)
        with Horizontal(id="top"):
            self.devices = DeviceList(id="devices")
            yield self.devices
            self.detail = DetailPane(id="detail-pane")
            yield self.detail
        self.log_panel = EventLogPanel(id="log")
        yield self.log_panel
        self.cmd = Input(placeholder="◇ command (type 'help' for ideas)", id="cmd")
        yield self.cmd
        yield Footer()

    def on_mount(self) -> None:
        self.title = "◇ Machinist"
        self.sub_title = f"fleet of {len(self.world.devices)} device(s)"
        self._paint(self.projection.state)
        self._unsubscribe = [
            self.world.bus.subscribe(self._bus_listener),
            self.projection.subscribe(self._projection_listener),
        ]

    def on_unmount(self) -> None:
        for unsubscribe in self._unsubscribe:
            unsubscribe()
        self.projection.close()

    # ----- bus / projection → UI thread ---------------------------------
    # These two run on device threads and only post. They must not be named
    # ``_on_<something>``: Textual dispatches messages to ``_on_<name>`` too.

    def _bus_listener(self, event: Event) -> None:
        """Runs on the publishing device's thread: only post."""
        if isinstance(event, _LOGGED):
            self.post_message(EventArrived(event))

    def _projection_listener(self, _state: FleetState) -> None:
        """Runs on the publishing device's thread: post one repaint, not one per event."""
        if not self._repaint_pending.is_set():
            self._repaint_pending.set()
            self.post_message(FleetMoved())

    def on_event_arrived(self, message: EventArrived) -> None:
        self.log_panel.log_event(message.event)

    def on_fleet_moved(self, _message: FleetMoved) -> None:
        self._repaint_pending.clear()
        self._paint(self.projection.state)

    def _paint(self, state: FleetState) -> None:
        self.devices.show(state)
        self.detail.view = state.devices.get(self._selected) if self._selected else None

    # ----- selection ------------------------------------------------------

    def on_data_table_row_highlighted(self, event: DataTable.RowHighlighted) -> None:
        if event.control is self.devices and event.row_key is not None:
            self._select(str(event.row_key.value))

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        if event.control is self.devices:
            self._select(str(event.row_key.value))

    def _select(self, name: str) -> None:
        if name == self._selected:
            return
        self._selected = name
        self.detail.view = self.projection.state.devices.get(name)

    def on_program_list_run(self, message: ProgramList.Run) -> None:
        self._dispatch_command(f"run {self._selected or ''} {message.program}")

    # ----- command bar -------------------------------------------------

    def on_input_submitted(self, event: Input.Submitted) -> None:
        self._dispatch_command(event.value.strip())
        self.cmd.value = ""

    def _dispatch_command(self, line: str) -> None:
        """Run one command line through the shared dispatcher and log the outcome.

        ``quit`` and ``help`` are the TUI's own; everything else is the same
        :class:`~machinist.commands.Commands` the web endpoint uses, with the
        selected device filling in for a name left out.
        """
        line = line.strip()
        if not line:
            return
        if line == "quit":
            self.exit()
            return
        if line == "help":
            self.log_panel.note(HELP + " | quit")
            return
        try:
            result = self.commands.dispatch(line, selected=self._selected)
        except CommandError as exc:
            self.log_panel.note(f"[red]error[/]: {exc}")
            return
        self.log_panel.note(result.message)

    # ----- keybindings --------------------------------------------------

    def action_estop(self) -> None:
        self._dispatch_command("estop")

    def action_reset(self) -> None:
        self._dispatch_command("reset")

    def action_focus_command(self) -> None:
        self.cmd.focus()

    def action_focus_devices(self) -> None:
        self.devices.focus()

    def action_toggle_programs(self) -> None:
        self.detail.show_programs = not self.detail.show_programs

    def action_toggle_log(self) -> None:
        top = self.query_one("#top")
        top.remove_class("log-small", "log-large")
        self.log_panel.remove_class("log-small", "log-large")
        self._log_size = (self._log_size + 1) % 3
        if self._log_size == 1:
            top.add_class("log-small")
            self.log_panel.add_class("log-small")
        elif self._log_size == 2:
            top.add_class("log-large")
            self.log_panel.add_class("log-large")
