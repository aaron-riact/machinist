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

import ctypes
import socket
import threading
import time

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import DHParams, KinematicsOptions
from ...transport.framing import PAREN
from .arm import ArmMode, ArmOptions, ArmStateView, RobotArm, arm_from_options

DOBOT_DASHBOARD_PORT = 29999
DOBOT_FEEDBACK_FAST_PORT = 30004
DOBOT_FEEDBACK_MED_PORT = 30005
DOBOT_FEEDBACK_SLOW_PORT = 30006


class DobotFeedbackPacket(ctypes.Structure):
    """1440-byte binary feedback packet, layout matches ``MyType`` from the
    Dobot TCP/IP client library at ``dobot_api.py:MyType``."""

    _layout_ = "ms"
    _fields_ = [
        ("len", ctypes.c_uint16),
        ("reserve", ctypes.c_byte * 6),
        ("DigitalInputs", ctypes.c_uint64),
        ("DigitalOutputs", ctypes.c_uint64),
        ("RobotMode", ctypes.c_uint64),
        ("TimeStamp", ctypes.c_uint64),
        ("RunTime", ctypes.c_uint64),
        ("TestValue", ctypes.c_uint64),
        ("reserve2", ctypes.c_byte * 8),
        ("SpeedScaling", ctypes.c_double),
        ("reserve3", ctypes.c_byte * 16),
        ("VRobot", ctypes.c_double),
        ("IRobot", ctypes.c_double),
        ("ProgramState", ctypes.c_double),
        ("SafetyOIn", ctypes.c_uint16),
        ("SafetyOOut", ctypes.c_uint16),
        ("reserve4", ctypes.c_byte * 76),
        ("QTarget", ctypes.c_double * 6),
        ("QDTarget", ctypes.c_double * 6),
        ("QDDTarget", ctypes.c_double * 6),
        ("ITarget", ctypes.c_double * 6),
        ("MTarget", ctypes.c_double * 6),
        ("QActual", ctypes.c_double * 6),
        ("QDActual", ctypes.c_double * 6),
        ("IActual", ctypes.c_double * 6),
        ("ActualTCPForce", ctypes.c_double * 6),
        ("ToolVectorActual", ctypes.c_double * 6),
        ("TCPSpeedActual", ctypes.c_double * 6),
        ("TCPForce", ctypes.c_double * 6),
        ("ToolVectorTarget", ctypes.c_double * 6),
        ("TCPSpeedTarget", ctypes.c_double * 6),
        ("MotorTemperatures", ctypes.c_double * 6),
        ("JointModes", ctypes.c_double * 6),
        ("VActual", ctypes.c_double * 6),
        ("HandType", ctypes.c_byte * 4),
        ("User", ctypes.c_byte),
        ("Tool", ctypes.c_byte),
        ("RunQueuedCmd", ctypes.c_byte),
        ("PauseCmdFlag", ctypes.c_byte),
        ("VelocityRatio", ctypes.c_byte),
        ("AccelerationRatio", ctypes.c_byte),
        ("reserve5", ctypes.c_byte),
        ("XYZVelocityRatio", ctypes.c_byte),
        ("RVelocityRatio", ctypes.c_byte),
        ("XYZAccelerationRatio", ctypes.c_byte),
        ("RAccelerationRatio", ctypes.c_byte),
        ("reserve6", ctypes.c_byte * 2),
        ("BrakeStatus", ctypes.c_byte),
        ("EnableStatus", ctypes.c_byte),
        ("DragStatus", ctypes.c_byte),
        ("RunningStatus", ctypes.c_byte),
        ("ErrorStatus", ctypes.c_byte),
        ("JogStatusCR", ctypes.c_byte),
        ("CRRobotType", ctypes.c_byte),
        ("DragButtonSignal", ctypes.c_byte),
        ("EnableButtonSignal", ctypes.c_byte),
        ("RecordButtonSignal", ctypes.c_byte),
        ("ReappearButtonSignal", ctypes.c_byte),
        ("JawButtonSignal", ctypes.c_byte),
        ("SixForceOnline", ctypes.c_byte),
        ("CollisionState", ctypes.c_byte),
        ("ArmApproachState", ctypes.c_byte),
        ("J4ApproachState", ctypes.c_byte),
        ("J5ApproachState", ctypes.c_byte),
        ("J6ApproachState", ctypes.c_byte),
        ("reserve7", ctypes.c_byte * 61),
        ("VibrationDisZ", ctypes.c_double),
        ("CurrentCommandId", ctypes.c_uint64),
        ("MActual", ctypes.c_double * 6),
        ("Load", ctypes.c_double),
        ("CenterX", ctypes.c_double),
        ("CenterY", ctypes.c_double),
        ("CenterZ", ctypes.c_double),
        ("UserValue", ctypes.c_double * 6),
        ("ToolValue", ctypes.c_double * 6),
        ("reserve8", ctypes.c_byte * 8),
        ("SixForceValue", ctypes.c_double * 6),
        ("TargetQuaternion", ctypes.c_double * 4),
        ("ActualQuaternion", ctypes.c_double * 4),
        ("AutoManualMode", ctypes.c_uint16),
        ("ExportStatus", ctypes.c_uint16),
        ("SafetyState", ctypes.c_byte),
        ("reserve9", ctypes.c_byte * 19),
    ]


assert ctypes.sizeof(DobotFeedbackPacket) == 1440

_ARM_MODE_TO_ROBOT_MODE: dict[ArmMode, int] = {
    ArmMode.IDLE: 5,
    ArmMode.MOVING: 7,
    ArmMode.ESTOPPED: 9,
    ArmMode.FAULTED: 9,
}


def _update_feedback_packet(
    pkt: DobotFeedbackPacket,
    state: ArmStateView,
    *,
    now_us: int = 0,
    command_id: int = 0,
) -> None:
    pkt.len = 1440
    pkt.TestValue = 0x123456789abcdef
    pkt.RobotMode = _ARM_MODE_TO_ROBOT_MODE.get(state.mode, 9)
    pkt.TimeStamp = now_us
    pkt.SpeedScaling = state.speed_fraction
    pkt.QActual[:] = state.joints
    pkt.ToolVectorActual[:] = state.pose
    pkt.EnableStatus = 1 if state.servo_on else 0
    pkt.BrakeStatus = 1 if state.mode in (ArmMode.IDLE, ArmMode.ESTOPPED) else 0
    pkt.ErrorStatus = 1 if state.mode in (ArmMode.FAULTED, ArmMode.ESTOPPED) else 0
    pkt.RunningStatus = 1 if state.mode is ArmMode.MOVING else 0
    pkt.CurrentCommandId = command_id


def _send_to_all(clients: list[socket.socket], data: bytes) -> None:
    dead: list[socket.socket] = []
    for c in clients:
        try:
            c.sendall(data)
        except OSError:
            dead.append(c)
    for c in dead:
        clients.remove(c)
        c.close()


def _accept_loop(sock: socket.socket, clients: list[socket.socket], running: threading.Event) -> None:
    sock.settimeout(1.0)
    while running.is_set():
        try:
            client, _addr = sock.accept()
        except socket.timeout:
            continue
        clients.append(client)
    sock.close()


def _feedback_writer(
    arm: RobotArm,
    fast: list[socket.socket],
    med: list[socket.socket],
    slow: list[socket.socket],
    command_id: list[int],
    running: threading.Event,
) -> None:
    pkt = DobotFeedbackPacket()
    tick = 0
    period = 0.008
    while running.is_set():
        deadline = time.monotonic() + period
        s = arm.state.snapshot()
        _update_feedback_packet(pkt, s, now_us=time.monotonic_ns() // 1000, command_id=command_id[0])
        data = bytes(pkt)
        _send_to_all(fast, data)
        if tick % 25 == 0:
            _send_to_all(med, data)
            if tick % 125 == 0:
                _send_to_all(slow, data)
        tick += 1
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)


class DobotDashboard(LineServerDevice):
    """Emulated Dobot TCP/IP dashboard (port 29999)."""

    kind = "dobot_dashboard"
    DEFAULT_PORT = DOBOT_DASHBOARD_PORT
    FRAMER = PAREN

    _FEEDBACK_PORTS = (DOBOT_FEEDBACK_FAST_PORT, DOBOT_FEEDBACK_MED_PORT, DOBOT_FEEDBACK_SLOW_PORT)

    def __init__(
        self,
        name: str,
        endpoint: Endpoint,
        bus: EventBus,
        options: ArmOptions,
        *,
        feedback_enabled: bool = True,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options)
        self.arm.start_ticker()

        self._current_command_id: list[int] = [0]
        self._running = threading.Event()
        self._running.set()
        self._feedback_socks: list[socket.socket] = []
        self._writer: threading.Thread | None = None

        if feedback_enabled:
            self._clients_fast: list[socket.socket] = []
            self._clients_med: list[socket.socket] = []
            self._clients_slow: list[socket.socket] = []

            for port, clients in zip(self._FEEDBACK_PORTS, (self._clients_fast, self._clients_med, self._clients_slow)):
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                sock.bind(("", port))
                sock.listen()
                self._feedback_socks.append(sock)
                t = threading.Thread(
                    target=_accept_loop, args=(sock, clients, self._running), daemon=True
                )
                t.start()

            self._writer = threading.Thread(
                target=_feedback_writer,
                args=(self.arm, self._clients_fast, self._clients_med, self._clients_slow, self._current_command_id, self._running),
                daemon=True,
            )
            self._writer.start()

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
            case "robotmode":
                return _ok(verb, args, value=str(_ARM_MODE_TO_ROBOT_MODE.get(s.mode, 9)))
            case "movj":
                self.arm.movej(tuple(_parse_floats(args, count=len(s.joints))))
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case "movl":
                self.arm.movel(tuple(_parse_floats(args, count=6)))  # type: ignore[arg-type]
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case _:
                return f"-10000,{{}},{verb}({args})"

    def _shutdown(self) -> None:
        super()._shutdown()
        self._running.clear()
        for sock in self._feedback_socks:
            sock.close()
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
    raw = dict(options)
    dh = DHParams(**raw.pop("dh_params")) if "dh_params" in raw else None
    kin = KinematicsOptions(**raw.pop("kinematics")) if "kinematics" in raw else None
    feedback = raw.pop("feedback_ports", None)
    feedback_enabled = feedback is not False
    return DobotDashboard(
        name, endpoint, bus, ArmOptions(kinematics=kin, dh_params=dh, **raw),
        feedback_enabled=feedback_enabled,
    )
