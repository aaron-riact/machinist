"""Yaskawa Motoman NX100/DX100 telnet HSE-style command emulator.

Reference: NX100 HTTP / Telnet network command guide
(``NX100_http_network_command.pdf`` in the repo root).

Commands are short ASCII text terminated by ``\\r\\n``. Every reply
starts with either ``OK`` or ``ERROR:<code>``. We implement enough to
let a typical monitoring client work end-to-end: ``RPOSJ`` (joint
position), ``RPOSC`` (cartesian pose), ``CANCEL``, ``HOLD``,
``SVON``/``SVOFF``, and ``MOVJ``/``MOVL``.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from .arm import ArmMode, RobotArm

MOTOMAN_PORT = 80  # HSE web/console port on NX100


class MotomanNX100(LineServerDevice):
    kind = "motoman_nx100"
    DEFAULT_PORT = MOTOMAN_PORT
    TERMINATOR = "\r\n"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = RobotArm(joint_count=int(options.get("joint_count", 6)))
        self.arm.start_ticker()

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        verb, _, args = line.strip().partition(" ")
        v = verb.upper()
        s = self.arm.state.snapshot()
        match v:
            case "RPOSJ":
                return ",".join(f"{j:.4f}" for j in s.joints)
            case "RPOSC":
                return ",".join(f"{p:.4f}" for p in s.pose)
            case "SVON":
                self.arm.set_servo(True)
                return "OK"
            case "SVOFF":
                self.arm.set_servo(False)
                return "OK"
            case "HOLD" | "CANCEL":
                self.arm.estop()
                return "OK"
            case "RESET":
                self.arm.reset()
                return "OK"
            case "MOVJ":
                joints = _parse_floats(args, count=len(s.joints))
                self.arm.movej(tuple(joints))
                return "OK"
            case "MOVL":
                pose = tuple(_parse_floats(args, count=6))
                self.arm.movel(pose)  # type: ignore[arg-type]
                return "OK"
            case "STATE":
                return _state_word(s.mode)
            case _:
                return "ERROR:E2010"

    def _shutdown(self) -> None:
        super()._shutdown()
        self.arm.stop_ticker()


def _parse_floats(text: str, *, count: int) -> list[float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != count:
        raise ValueError(f"expected {count} floats, got {len(parts)}")
    return [float(p) for p in parts]


def _state_word(mode: ArmMode) -> str:
    return {
        ArmMode.IDLE: "READY",
        ArmMode.MOVING: "RUNNING",
        ArmMode.ESTOPPED: "ESTOP",
        ArmMode.FAULTED: "ALARM",
    }[mode]


@register("motoman_nx100", default_port=MOTOMAN_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    return MotomanNX100(name, endpoint, bus, options)
