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
from dataclasses import dataclass
from typing import Any

from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import NEWLINE
from ...kinematics.api import DHParams, Joints, KinematicsOptions, Pose
from ...kinematics.units import Meters, Radians
from .arm import ArmOptions, RobotArm, arm_from_options

FANUC_PORT = 18735  # fanucpy default Karel port


@dataclass(frozen=True, slots=True)
class FanucKarelServerOptions:
    joint_count: int = 6
    kinematics: dict[str, Any] | None = None
    backend: str | None = None
    dh_params: dict[str, list[float]] | None = None
    urdf: str | None = None
    digital_outputs: int = 16
    digital_inputs: int = 16


class FanucKarelServer(LineServerDevice):
    kind = "fanuc_r30ib"
    DEFAULT_PORT = FANUC_PORT
    FRAMER = NEWLINE

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: FanucKarelServerOptions,
        *, arm: RobotArm, io: SignalBank,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm
        self.arm.start_ticker()
        self.io = io
        for i in range(1, options.digital_outputs + 1):
            self.io.declare(f"do{i}", Direction.OUTPUT)
        for i in range(1, options.digital_inputs + 1):
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
                self.arm.movej(tuple(Radians(j) for j in joints))
                return "OK"
            case "movel":
                raw = _parse_floats(args, count=6)
                self.arm.movel((Meters(raw[0]), Meters(raw[1]), Meters(raw[2]),
                                Radians(raw[3]), Radians(raw[4]), Radians(raw[5])))
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
    opts = FanucKarelServerOptions(**options)
    dh = DHParams(**opts.dh_params) if opts.dh_params is not None else None
    kin = KinematicsOptions(**opts.kinematics) if opts.kinematics is not None else None
    arm = arm_from_options(ArmOptions(
        joint_count=opts.joint_count,
        kinematics=kin,
        backend=opts.backend,
        dh_params=dh,
        urdf=opts.urdf,
    ))
    return FanucKarelServer(name, endpoint, bus, opts, arm=arm, io=SignalBank(owner=name))
