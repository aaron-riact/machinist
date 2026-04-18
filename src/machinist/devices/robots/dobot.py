"""Dobot TCP/IP remote control protocol emulator.

Reference: ``Dobot TCP_IP Remote Control Interface Guide V4.6.2`` PDF in
the repo root. Two ports are exposed by real Dobots:

* ``29999`` – dashboard (text commands, semicolon-terminated)
* ``30003`` – feedback / RT data (binary)

We implement the dashboard text channel here. Every command has the
form ``Verb(arg1,arg2,...)`` and replies are ``code,{value};`` strings.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from .arm import RobotArm

DOBOT_DASHBOARD_PORT = 29999


class DobotDashboard(LineServerDevice):
    kind = "dobot_dashboard"
    DEFAULT_PORT = DOBOT_DASHBOARD_PORT
    TERMINATOR = ";"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = RobotArm(joint_count=int(options.get("joint_count", 6)))
        self.arm.start_ticker()

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        verb, args = _split(line.strip())
        s = self.arm.state.snapshot()
        match verb.lower():
            case "enablerobot":
                self.arm.set_servo(True)
                return "0,{};EnableRobot()"
            case "disablerobot":
                self.arm.set_servo(False)
                return "0,{};DisableRobot()"
            case "emergencystop":
                self.arm.estop()
                return "0,{};EmergencyStop()"
            case "clearerror":
                self.arm.reset()
                return "0,{};ClearError()"
            case "getpose":
                return f"0,{{{','.join(f'{p:.4f}' for p in s.pose)}}};GetPose()"
            case "getangle":
                return f"0,{{{','.join(f'{j:.4f}' for j in s.joints)}}};GetAngle()"
            case "movj":
                joints = _parse_floats(args, count=len(s.joints))
                self.arm.movej(tuple(joints))
                return "0,{};MovJ()"
            case "movl":
                pose = tuple(_parse_floats(args, count=6))
                self.arm.movel(pose)  # type: ignore[arg-type]
                return "0,{};MovL()"
            case _:
                return f"-1,{{}};{verb}() unknown"

    def _shutdown(self) -> None:
        super()._shutdown()
        self.arm.stop_ticker()


def _split(line: str) -> tuple[str, str]:
    if "(" not in line:
        return line, ""
    verb, rest = line.split("(", 1)
    args = rest.rsplit(")", 1)[0]
    return verb, args


def _parse_floats(text: str, *, count: int) -> list[float]:
    parts = [p for p in text.split(",") if p.strip()]
    if len(parts) != count:
        raise ValueError(f"expected {count} floats, got {len(parts)}")
    return [float(p) for p in parts]


@register("dobot_dashboard", default_port=DOBOT_DASHBOARD_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    return DobotDashboard(name, endpoint, bus, options)
