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

from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

from pydantic import field_validator

from ...core.capabilities import HasFlange
from ...core.device import Device
from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.flange_bus import FlangeBus
from ...transport.framing import NEWLINE
from ...transport.modbus_rtu_gateway import ModbusRtuGateway, parse_gateway_ports
from .arm import ArmMode, ArmOptions, HasArm, arm_from_options

UR_DASHBOARD_PORT = 29999

#: Where a UR lets the network at the tool flange's RS485 line, once the
#: tool communication interface has been handed over. Off unless the scene
#: asks for it, because a bare UR does not forward the line at all.
UR_FLANGE_GATEWAY_PORT = 12345

GREETING = "Connected: Universal Robots Dashboard Server"


class UROptions(ArmOptions):
    """The ``ur_dashboard`` block: an arm, and the doors onto its tool flange."""

    #: TCP doors onto the flange's RS485 line. Shut unless the scene asks,
    #: because a UR forwards the tool line only once the tool communication
    #: interface has been handed over. ``true`` asks for 12345.
    flange_gateway_ports: tuple[int, ...] = ()

    @field_validator("flange_gateway_ports", mode="before")
    @classmethod
    def _read_gateway_ports(cls, raw: Any) -> object:
        return parse_gateway_ports(
            raw, ports=(UR_FLANGE_GATEWAY_PORT,), on_by_default=False
        )


@dataclass(slots=True)
class _LoadedProgram:
    name: str = ""


class URDashboardServer(LineServerDevice, HasArm, HasFlange):
    """Universal Robots Dashboard text protocol on port 29999."""

    kind = "ur_dashboard"
    DEFAULT_PORT = UR_DASHBOARD_PORT
    # UR Dashboard is newline-terminated ASCII.
    FRAMER = NEWLINE

    def __init__(
        self,
        name: str,
        endpoint: Endpoint,
        bus: EventBus,
        options: ArmOptions,
        *,
        flange: FlangeBus,
        gateway: ModbusRtuGateway | None = None,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options, name=name, publish=self.publish)
        self.add_service(self.arm)
        self._loaded = _LoadedProgram()
        self.flange = flange
        self._gateway = gateway
        if gateway is not None:
            self.add_service(gateway)

    @property
    def flange_gateway_ports(self) -> tuple[int, ...]:
        """TCP ports that open straight onto the flange line, if any."""
        return () if self._gateway is None else self._gateway.ports

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


@register("ur_dashboard", default_port=UR_DASHBOARD_PORT, options=UROptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: UROptions) -> Device:
    flange = FlangeBus()
    gateway = (
        ModbusRtuGateway(host=endpoint.host, ports=options.flange_gateway_ports, line=flange)
        if options.flange_gateway_ports
        else None
    )
    return URDashboardServer(name, endpoint, bus, options, flange=flange, gateway=gateway)
