"""Fanuc R-30iA/B controller text-command emulator (``Fanucpy``-compatible).

The community ``fanucpy`` library and FaRoC use a simple newline-based
text protocol over a Karel-served TCP socket. We emulate the verbs that
exercise our shared :class:`RobotArm` model: ``getjpos``, ``getlpos``,
``movej``, ``movel``, ``setdo``, ``getdi``, ``stop``, ``reset``.

IO is exposed via the device's :class:`SignalBank` so other devices can
wire to it through ``io_links``.
"""

from __future__ import annotations

import struct
import threading
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import DHParams, Joints, KinematicsOptions, Pose
from ...kinematics.units import Meters, Radians
from ...transport.focas import FocasSubpacket
from ...transport.focas_server import FocasServer
from ...transport.framing import NEWLINE
from ...transport.line_server import LineServer, stateless
from .arm import ArmOptions, RobotArm, arm_from_options

FANUC_PORT = 18735  # fanucpy default Karel port
FOCAS_PORT = 8193  # standard FOCAS1/2 port


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


# ----- Dual-protocol robot (FOCAS + Karel) -----------------------------------


@dataclass(frozen=True, slots=True)
class FanucFocasRobotOptions:
    joint_count: int = 6
    kinematics: dict[str, Any] | None = None
    backend: str | None = None
    dh_params: dict[str, list[float]] | None = None
    urdf: str | None = None
    digital_outputs: int = 16
    digital_inputs: int = 16
    model: str = "R-30iB"
    focas_port: int = FOCAS_PORT
    karel_port: int = FANUC_PORT


class FanucFocasRobot(Device):
    kind = "fanuc_focas_robot"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: FanucFocasRobotOptions,
        *, arm: RobotArm, io: SignalBank,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm
        self.io = io
        for i in range(1, options.digital_outputs + 1):
            self.io.declare(f"do{i}", Direction.OUTPUT)
        for i in range(1, options.digital_inputs + 1):
            self.io.declare(f"di{i}", Direction.INPUT)
        self._options = options
        self._server_focas = FocasServer(
            host=endpoint.host,
            port=options.focas_port,
            on_request=self._focas_handler,
        )
        self._server_karel = LineServer(
            endpoint.host,
            options.karel_port,
            session_factory=stateless(self._karel_handler),
            framer=NEWLINE,
        )

    def _karel_handler(self, line: str) -> Iterable[str] | str | None:
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

    def _focas_handler(self, sp: FocasSubpacket) -> bytes:
        key = (sp.c1, sp.c2, sp.c3)
        handler = _FOCAS_ROBOT_HANDLERS.get(key)
        if handler is None:
            return sp.encode_response_error(1)
        try:
            return handler(self, sp)
        except Exception:
            return sp.encode_response_error(6)

    def _focas_sysinfo(self, sp: FocasSubpacket) -> bytes:
        opts = self._options
        data = struct.pack(
            ">HH2s2s4s4s2s",
            0,
            opts.joint_count,
            opts.model.encode("ascii").ljust(2)[:2],
            b"RB",
            opts.model.encode("ascii").ljust(4)[:4],
            b"1.00".ljust(4)[:4],
            b"\x20\x20",
        )
        return sp.encode_response_ok(data)

    def _focas_status(self, sp: FocasSubpacket) -> bytes:
        s = self.arm.state.snapshot()
        aut = 1 if s.servo_on else 0
        run = 1 if s.program_running else 0
        motion = 1 if s.moving else 0
        data = struct.pack(">HHHHHHH", aut, run, motion, 0, 0, 0, 0)
        return sp.encode_response_ok(data)

    def _focas_axes(self, sp: FocasSubpacket) -> bytes:
        s = self.arm.state.snapshot()
        joints = s.joints
        values = b"".join(
            struct.pack(">ii", int(j * 10000), 0x0002000A)
            for j in joints
        )
        count = len(joints)
        data = struct.pack(">HH", 0, count) + values
        return sp.encode_response_ok(data)

    def _focas_alarm(self, sp: FocasSubpacket) -> bytes:
        return sp.encode_response_ok(struct.pack(">I", 0))

    def _focas_get_time(self, sp: FocasSubpacket) -> bytes:
        import time
        t = time.localtime()
        data = struct.pack(">HHH", t.tm_year, t.tm_mon, t.tm_mday)
        if sp.v1 == 1:
            data = struct.pack(">HHH", t.tm_hour, t.tm_min, t.tm_sec)
        return sp.encode_response_ok(struct.pack(">H", len(data) + 2) + data)

    def _focas_read_pmc(self, sp: FocasSubpacket) -> bytes:
        section = sp.v3 & 0xFF
        address = sp.v1 & 0xFFFF
        count = max(sp.v4, 1)
        if section == 2:  # Y section → digital outputs
            values = bytearray()
            for offset in range(count):
                try:
                    val = self.io[f"do{address + offset}"].value
                except KeyError:
                    val = 0
                values.append(1 if val else 0)
            return sp.encode_response_ok(bytes(values))
        if section == 3:  # X section → digital inputs
            values = bytearray()
            for offset in range(count):
                try:
                    val = self.io[f"di{address + offset}"].value
                except KeyError:
                    val = 0
                values.append(1 if val else 0)
            return sp.encode_response_ok(bytes(values))
        return sp.encode_response_ok(bytes(count))

    def _focas_write_pmc(self, sp: FocasSubpacket) -> bytes:
        section = sp.v3 & 0xFF
        address = sp.v1 & 0xFFFF
        if section == 2:  # Y section → digital outputs
            for i, b in enumerate(sp.payload):
                self.io[f"do{address + i}"].set(bool(b))
            self.emit("io", section="Y", address=address, data=sp.payload.hex())
        return sp.encode_response_ok()

    def _run(self, stop: threading.Event) -> None:
        self.arm.start_ticker()
        threads: list[threading.Thread] = []

        for server in (self._server_focas, self._server_karel):
            ready = threading.Event()
            t = threading.Thread(
                target=server.serve_forever, args=(ready,), daemon=True,
            )
            t.start()
            if not ready.wait(timeout=2.0):
                raise RuntimeError(f"{self.name} server failed to bind")
            threads.append(t)

        self._mark_running()
        stop.wait()

        for server in (self._server_focas, self._server_karel):
            server.shutdown()
        for t in threads:
            t.join(timeout=2.0)

    def _shutdown(self) -> None:
        self._server_focas.shutdown()
        self._server_karel.shutdown()
        self.arm.stop_ticker()


_FOCAS_ROBOT_HANDLERS: dict[tuple[int, int, int], Any] = {
    (1, 1, 0x18): FanucFocasRobot._focas_sysinfo,
    (1, 1, 0x19): FanucFocasRobot._focas_status,
    (1, 1, 0x1a): FanucFocasRobot._focas_alarm,
    (1, 1, 0x26): FanucFocasRobot._focas_axes,
    (1, 1, 0x45): FanucFocasRobot._focas_get_time,
    (2, 1, 0x8001): FanucFocasRobot._focas_read_pmc,
    (2, 1, 0x8002): FanucFocasRobot._focas_write_pmc,
}


@register("fanuc_focas_robot", default_port=FOCAS_PORT)
def _robot_factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    opt = FanucFocasRobotOptions(**opts)
    dh = DHParams(**opt.dh_params) if opt.dh_params is not None else None
    kin = KinematicsOptions(**opt.kinematics) if opt.kinematics is not None else None
    arm = arm_from_options(ArmOptions(
        joint_count=opt.joint_count,
        kinematics=kin,
        backend=opt.backend,
        dh_params=dh,
        urdf=opt.urdf,
    ))
    return FanucFocasRobot(name, endpoint, bus, opt, arm=arm, io=SignalBank(owner=name))
