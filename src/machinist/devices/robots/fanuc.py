"""Fanuc R-30iA/B controller text-command emulator (``Fanucpy``-compatible).

The community ``fanucpy`` library and FaRoC use a simple newline-based
text protocol over a Karel-served TCP socket. We emulate the verbs that
exercise our shared :class:`RobotArm` model: ``getjpos``, ``getlpos``,
``movej``, ``movel``, ``setdo``, ``getdi``, ``stop``, ``reset``.

IO is exposed via the device's :class:`SignalBank` so other devices can
wire to it through ``io_links``.
"""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any

from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import NEWLINE
from .arm import RobotArm, arm_from_options

FANUC_PORT = 18735  # fanucpy default Karel port


class FanucKarelServer(LineServerDevice):
    kind = "fanuc_r30ib"
    DEFAULT_PORT = FANUC_PORT
    FRAMER = NEWLINE

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]
    ) -> None:
        super().__init__(name, endpoint, bus)
        digital_outputs = int(options.get("digital_outputs", 16))
        digital_inputs = int(options.get("digital_inputs", 16))
        self.arm = arm_from_options(options)
        self.arm.start_ticker()
        self.io = SignalBank(owner=name)
        for i in range(1, digital_outputs + 1):
            self.io.declare(f"do{i}", Direction.OUTPUT)
        for i in range(1, digital_inputs + 1):
            self.io.declare(f"di{i}", Direction.INPUT)

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        verb, _, args = line.strip().partition(" ")
        s = self.arm.state.snapshot()
        match verb.lower():
            case "getjpos":
                return ",".join(f"{j:.4f}" for j in s.joints)
            case "getlpos":
                return ",".join(f"{p:.4f}" for p in s.pose)
            case "movej":
                joints = _parse_floats(args, count=len(s.joints))
                self.arm.movej(tuple(joints))
                return "OK"
            case "movel":
                pose = tuple(_parse_floats(args, count=6))
                self.arm.movel(pose)  # type: ignore[arg-type]
                return "OK"
            case "setdo":
                idx, val = args.split(",")
                self.io[f"do{int(idx)}"].set(bool(int(val)))
                return "OK"
            case "getdi":
                idx = int(args)
                return "1" if self.io[f"di{idx}"].value else "0"
            case "stop":
                self.arm.estop()
                return "OK"
            case "reset":
                self.arm.reset()
                return "OK"
            case _:
                return f"ERR:unknown verb {verb!r}"

    def _shutdown(self) -> None:
        super()._shutdown()
        self.arm.stop_ticker()


def _parse_floats(text: str, *, count: int) -> list[float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != count:
        raise ValueError(f"expected {count} floats, got {len(parts)}")
    return [float(p) for p in parts]


@register("fanuc_r30ib", default_port=FANUC_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    return FanucKarelServer(name, endpoint, bus, options)
