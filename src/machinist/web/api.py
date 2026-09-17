"""Pure, IO-free serialization for the web UI.

It turns the :class:`~machinist.projection.FleetState` (or one
:class:`~machinist.projection.DeviceView`) into plain JSON-able dicts. It
never touches a device: what the browser sees is exactly what the
projection reduced from the event stream, the same facts the TUI paints.

Commands are not here: both UIs share :mod:`machinist.commands`. There is
deliberately **no** HTTP, sockets or threading in this module, so it is
unit-testable in microseconds.
"""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

from ..devices.machines.state import MachineView
from ..devices.robots.arm import ArmStateView
from ..projection import DeviceView, FleetState


def snapshot_world(state: FleetState) -> dict[str, Any]:
    """Serialize the whole fleet into a JSON-able snapshot."""
    return {"devices": [view_to_dict(view) for view in state.devices.values()]}


def view_to_dict(view: DeviceView) -> dict[str, Any]:
    """Serialize one device view, including any arm/machine/IO it carries."""
    snap: dict[str, Any] = {
        "name": view.name,
        "kind": view.kind,
        "endpoint": view.endpoint,
        "lifecycle": str(view.lifecycle),
        "signals": [
            {"name": sig.name, "direction": str(sig.direction), "value": sig.value}
            for sig in view.signals.values()
        ],
    }
    snap[_panel_slot(view.panel.mode)] = asdict(view.panel)
    if view.arm is not None:
        snap["arm"] = _arm_snapshot(view.arm)
    if view.machine is not None:
        snap["machine"] = _machine_snapshot(view.machine)
    if view.programs is not None:
        snap["programs"] = list(view.programs)
    if view.fault is not None:
        snap["fault"] = view.fault
    return snap


def _panel_slot(mode: str) -> str:
    """Where the browser expects a panel: its Modbus tile or its generic (EtherNet/IP-shaped) tile."""
    return "modbus" if mode == "modbus" else "ethernetip"


def _arm_snapshot(s: ArmStateView) -> dict[str, Any]:
    return {
        "mode": str(s.mode),
        "servo_on": s.servo_on,
        "estopped": s.estopped,
        "faulted": s.faulted,
        "moving": s.moving,
        "command": s.current_command,
        "speed_fraction": s.speed_fraction,
        "joints": list(s.joints),
        "pose": list(s.pose),
    }


def _machine_snapshot(state: MachineView) -> dict[str, Any]:
    program = state.program.splitlines()[0] if state.program else ""
    return {
        "cycle": str(state.cycle),
        "program": program,
        "spindle_rpm": state.spindle_rpm,
        "feed": state.feed,
        "tool": state.tool,
        "parts": state.parts,
        "position": {
            "x": state.position.x,
            "y": state.position.y,
            "z": state.position.z,
        },
        "doors": dict(state.doors),
        "chucks": dict(state.chucks),
    }
