"""Universal Robots Dashboard server emulator.

Implements the well-known text-based Dashboard protocol on port 29999
that ``ur-rtde`` and many other clients speak. We cover the verbs that
exercise the parts of the robot most useful to scripts:

* ``polyscope`` greeting on connect
* ``robotmode`` / ``programState`` / ``safetymode`` / ``running``
* ``power on|off``, ``brake release``
* ``stop``, ``pause``, ``play``
* ``load <path>``
* ``unlock protective stop``

All pose/joint queries and movement live on the *primary* interface
(port 30001) which speaks a binary protocol. We expose a simplified
text variant here too for tests; a future commit will add the binary
secondary/RTDE channels.
"""

from __future__ import annotations

import threading
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from ...kinematics.api import DHParams, KinematicsOptions

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import NEWLINE
from .arm import ArmOptions, ArmMode, RobotArm, arm_from_options

UR_DASHBOARD_PORT = 29999

GREETING = "Connected: Universal Robots Dashboard Server"


@dataclass(slots=True)
class _LoadedProgram:
    name: str = ""


class URDashboardServer(LineServerDevice):
    """Universal Robots Dashboard text protocol on port 29999."""

    kind = "ur_dashboard"
    DEFAULT_PORT = UR_DASHBOARD_PORT
    # UR Dashboard is newline-terminated ASCII.
    FRAMER = NEWLINE

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: ArmOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options)
        self.arm.start_ticker()
        self._loaded = _LoadedProgram()

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        verb, _, _ = line.strip().partition(" ")
        return self._dispatch(verb.lower(), line.strip())

    def _dispatch(self, verb: str, raw: str) -> str:
        state = self.arm.state.snapshot()
        match verb:
            case "polyscope":
                return GREETING
            case "robotmode":
                return f"Robotmode: {self._robotmode(state.mode)}"
            case "safetymode":
                return f"Safetymode: {'PROTECTIVE_STOP' if state.mode is ArmMode.ESTOPPED else 'NORMAL'}"
            case "programstate":
                return f"STATE: {'PLAYING' if state.program_running else 'STOPPED'} {self._loaded.name or 'no program'}"
            case "running":
                return f"Program running: {str(state.program_running).lower()}"
            case "power":
                self.arm.set_servo(raw.lower().endswith(" on"))
                return "Powering on" if raw.lower().endswith(" on") else "Powering off"
            case "brake":
                return "Brake releasing"
            case "stop":
                self._loaded = _LoadedProgram(self._loaded.name)
                return "Stopped"
            case "pause":
                return "Paused"
            case "play":
                return "Starting program"
            case "load":
                _, _, path = raw.partition(" ")
                self._loaded = _LoadedProgram(name=path.strip())
                return f"Loading program: {self._loaded.name}"
            case "unlock":
                self.arm.reset()
                return "Protective stop releasing"
            case "quit":
                return "Disconnected"
            case _:
                return f"Unknown command: {verb}"

    @staticmethod
    def _robotmode(mode: ArmMode) -> str:
        return {
            ArmMode.IDLE: "RUNNING",
            ArmMode.MOVING: "RUNNING",
            ArmMode.ESTOPPED: "PROTECTIVE_STOP",
            ArmMode.FAULTED: "FAULT",
        }[mode]

    def _shutdown(self) -> None:
        super()._shutdown()
        self.arm.stop_ticker()


@register("ur_dashboard", default_port=UR_DASHBOARD_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    raw = dict(options)
    dh = DHParams(**raw.pop("dh_params")) if "dh_params" in raw else None
    kin = KinematicsOptions(**raw.pop("kinematics")) if "kinematics" in raw else None
    return URDashboardServer(name, endpoint, bus, ArmOptions(kinematics=kin, dh_params=dh, **raw))
