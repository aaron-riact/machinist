"""Pure renderers: fleet views in, Rich markup and table rows out.

Nothing here touches a widget, so every function is testable with a plain
:class:`~machinist.projection.DeviceView`.
"""

from __future__ import annotations

import time

from rich.style import Style
from rich.text import Text

from ..core.events import Event
from ..core.io import Direction
from ..core.panel import Field, Panel
from ..core.types import DeviceState
from ..devices.machines.state import MachineView
from ..devices.robots.arm import ArmStateView
from ..projection import DeviceView

_LIFECYCLE_COLOURS: dict[DeviceState, str] = {
    DeviceState.RUNNING: "green",
    DeviceState.FAULTED: "red",
    DeviceState.STOPPED: "grey50",
    DeviceState.STARTING: "yellow",
}


def paint_lifecycle(state: DeviceState) -> str:
    return f"[{_LIFECYCLE_COLOURS.get(state, 'white')}]{state}[/]"


def detail_header(view: DeviceView) -> str:
    fault = f"\n[red]fault[/] {view.fault}" if view.fault else ""
    return (
        f"[bold]{view.name}[/]  [dim]({view.kind})[/]\n"
        f"endpoint [magenta]{view.endpoint}[/]   "
        f"lifecycle {paint_lifecycle(view.lifecycle)}{fault}"
        f"{arm_summary(view.arm)}"
        f"{machine_summary(view.machine)}"
        f"{panel_summary(view.panel)}"
    )


def arm_summary(s: ArmStateView | None) -> str:
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


def machine_summary(state: MachineView | None) -> str:
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


def panel_summary(panel: Panel) -> str:
    if panel.mode == "io":
        return ""
    if panel.clients is not None:
        return f"\n{panel.mode}   {panel.clients} client(s)"
    peer = "peer up" if panel.peer_connected else "waiting"
    ready = "ready" if panel.transport_ready else "offline"
    return f"\n{panel.mode}   transport {ready}   link {peer}"


def io_rows(view: DeviceView) -> tuple[tuple[Field, ...], tuple[Field, ...]]:
    """The input and output tables: the panel's own rows, else the device's signals."""
    if view.panel.input_fields or view.panel.output_fields:
        return view.panel.input_fields, view.panel.output_fields
    rows = {Direction.INPUT: [], Direction.OUTPUT: []}
    for sig in view.signals.values():
        rows[sig.direction].append(
            Field(
                signal=sig.name.upper(), name=sig.name, type="bit",
                value="ON" if sig.value else "OFF", on=sig.value,
            )
        )
    return tuple(rows[Direction.INPUT]), tuple(rows[Direction.OUTPUT])


def dot(field: Field) -> Text:
    """A green or red dot for a bit row; a blank for anything else."""
    if field.on is None:
        return Text(" ")
    t = Text("●")
    t.stylize(Style(color="green" if field.on else "red"))
    return t


def format_event(event: Event) -> str:
    payload = " ".join(f"{k}={v}" for k, v in event.payload.items())
    return (
        f"[dim]{clock(event.timestamp)}[/] "
        f"[cyan]{event.device:<12}[/] "
        f"[magenta]{event.kind:<6}[/] {payload}"
    )


def clock(timestamp: float) -> str:
    """Local wall-clock time with milliseconds, e.g. ``14:05:32.123``."""
    local = time.localtime(timestamp)
    return time.strftime("%H:%M:%S", local) + f".{int(timestamp * 1000) % 1000:03d}"
