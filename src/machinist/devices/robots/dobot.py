"""Dobot TCP/IP remote control protocol emulator.

Reference: *Dobot TCP/IP Remote Control Interface Guide V4.6.2*.

Dashboard port is ``29999``. A command on the wire is::

    MessageName(Param1,Param2,…)

The message *ends at the closing paren* — there is no newline or any
other terminator. Responses carry their own terminator, a semicolon::

    0,{value1,…},MessageName(args);

We therefore use :data:`PAREN` framing rather than trying to abuse a
line-oriented server (``;`` only marks *reply* boundaries, never
incoming-message boundaries).
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import PAREN
from .arm import RobotArm

DOBOT_DASHBOARD_PORT = 29999


class DobotDashboard(LineServerDevice):
    """Emulated Dobot TCP/IP dashboard (port 29999)."""

    kind = "dobot_dashboard"
    DEFAULT_PORT = DOBOT_DASHBOARD_PORT
    FRAMER = PAREN

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = RobotArm(joint_count=int(options.get("joint_count", 6)))
        self.arm.start_ticker()

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        verb, args = _parse(line)
        s = self.arm.state.snapshot()
        match verb.lower():
            case "enablerobot":
                self.arm.set_servo(True); return _ok(verb, args)
            case "disablerobot":
                self.arm.set_servo(False); return _ok(verb, args)
            case "emergencystop":
                self.arm.estop(); return _ok(verb, args)
            case "clearerror":
                self.arm.reset(); return _ok(verb, args)
            case "getpose":
                return _ok(verb, args, value=",".join(f"{p:.4f}" for p in s.pose))
            case "getangle":
                return _ok(verb, args, value=",".join(f"{j:.4f}" for j in s.joints))
            case "movj":
                self.arm.movej(tuple(_parse_floats(args, count=len(s.joints))))
                return _ok(verb, args)
            case "movl":
                self.arm.movel(tuple(_parse_floats(args, count=6)))  # type: ignore[arg-type]
                return _ok(verb, args)
            case _:
                return f"-10000,{{}},{verb}({args})"

    def _shutdown(self) -> None:
        super()._shutdown()
        self.arm.stop_ticker()


# --- helpers ---------------------------------------------------------


def _parse(line: str) -> tuple[str, str]:
    """Split ``Verb(args)`` into ``(verb, args)``."""
    line = line.strip()
    if "(" not in line or not line.endswith(")"):
        return line, ""
    verb, rest = line.split("(", 1)
    return verb.strip(), rest[:-1]


def _parse_floats(text: str, *, count: int) -> list[float]:
    parts = [p for p in text.split(",") if p.strip()]
    if len(parts) != count:
        raise ValueError(f"expected {count} floats, got {len(parts)}")
    return [float(p) for p in parts]


def _ok(verb: str, args: str, *, value: str = "") -> str:
    return f"0,{{{value}}},{verb}({args})"


@register("dobot_dashboard", default_port=DOBOT_DASHBOARD_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    return DobotDashboard(name, endpoint, bus, options)
