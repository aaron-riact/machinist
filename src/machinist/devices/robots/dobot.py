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

import ast
import math
from collections.abc import Iterable
from dataclasses import dataclass
from typing import Any

import ctypes
import os
import socket
import sys
import threading
import time

import numpy as np
from numpy.typing import NDArray

from ...core.device import DeviceDetail, DetailField
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import DHParams, Joints, KinematicsOptions, Pose
from ...kinematics.units import Meters, Radians
from ...transport.framing import PAREN
from .arm import ArmMode, ArmOptions, ArmStateView, RobotArm, arm_from_options

DOBOT_DASHBOARD_PORT = 29999
DOBOT_FEEDBACK_FAST_PORT = 30004
DOBOT_FEEDBACK_MED_PORT = 30005
DOBOT_FEEDBACK_SLOW_PORT = 30006


@dataclass(frozen=True, slots=True)
class _RobotModelInfo:
    type_code: int
    tool_di_count: int = 4
    tool_do_count: int = 4
    tool_ai_count: int = 2
    ai_count: int = 2
    ao_count: int = 2
    dh_params: DHParams | None = None


_CR5_DH = DHParams(
    a=(0.0, 0.0, 0.427, 0.357, 0.0, 0.0),
    d=(0.147, 0.147, 0.122, -0.116, 0.116, 0.0),
    alpha=(0.0, math.pi / 2, math.pi, math.pi, -math.pi / 2, math.pi / 2),
    theta_offset=(0.0, math.pi / 2, 0.0, math.pi / 2, 0.0, 0.0),
)

_CR10A_DH = DHParams(
    a=(0.0, 0.0, -0.607, -0.568, 0.0, 0.0),
    d=(0.1765, 0.0, 0.0, 0.191, 0.125, 0.1084),
    alpha=(0.0, math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2),
    theta_offset=(0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0),
)

_CR20A_DH = DHParams(
    a=(0.0, 0.0, -0.8252, -0.746, 0.0, 0.0),
    d=(0.23, 0.0, 0.0468, 0.1288, 0.1288, 0.1365),
    alpha=(0.0, math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2),
    theta_offset=(0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0),
)

DOBOT_ROBOT_MODELS: dict[str, _RobotModelInfo] = {
    "cr3": _RobotModelInfo(type_code=3),
    "cr5": _RobotModelInfo(type_code=5, tool_di_count=2, tool_do_count=2, dh_params=_CR5_DH),
    "cr7": _RobotModelInfo(type_code=7),
    "cr10": _RobotModelInfo(type_code=10, tool_di_count=2, tool_do_count=2, dh_params=_CR10A_DH),
    "cr12": _RobotModelInfo(type_code=12),
    "cr16": _RobotModelInfo(type_code=16),
    "nova2": _RobotModelInfo(type_code=101),
    "nova5": _RobotModelInfo(type_code=103),
    "cr3a": _RobotModelInfo(type_code=113),
    "cr5a": _RobotModelInfo(type_code=115),
    "cr5af": _RobotModelInfo(type_code=116),
    "cr7a": _RobotModelInfo(type_code=117),
    "cr10a": _RobotModelInfo(type_code=120, tool_di_count=2, tool_do_count=2, dh_params=_CR10A_DH),
    "cr10af": _RobotModelInfo(type_code=121),
    "cr12a": _RobotModelInfo(type_code=122),
    "cr16a": _RobotModelInfo(type_code=126),
    "cr20af": _RobotModelInfo(type_code=127),
    "cr20a": _RobotModelInfo(type_code=130, tool_di_count=4, tool_do_count=4, dh_params=_CR20A_DH),
    "cr20": _RobotModelInfo(type_code=20, tool_di_count=4, tool_do_count=4, dh_params=_CR20A_DH),
    "magician_e6": _RobotModelInfo(type_code=150),
}

_DEFAULT_MODEL = _RobotModelInfo(type_code=5, tool_di_count=4, tool_do_count=4, dh_params=_CR5_DH)

DOBOT_ROBOT_TYPES: dict[str, int] = {
    name: info.type_code for name, info in DOBOT_ROBOT_MODELS.items()
}


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
    robot_type_code: int = 5,
) -> None:
    pkt.len = 1440
    pkt.TestValue = 0x123456789abcdef
    pkt.RobotMode = _ARM_MODE_TO_ROBOT_MODE[state.mode]
    pkt.TimeStamp = now_us
    pkt.CRRobotType = robot_type_code
    pkt.SpeedScaling = state.speed_fraction
    pkt.QActual[:] = [math.degrees(j) for j in state.joints]
    pkt.ToolVectorActual[:] = (
        state.pose[0] * 1000,
        state.pose[1] * 1000,
        state.pose[2] * 1000,
        math.degrees(state.pose[3]),
        math.degrees(state.pose[4]),
        math.degrees(state.pose[5]),
    )
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
        except OSError:
            return
        clients.append(client)
    sock.close()


def _feedback_writer(
    arm: RobotArm,
    fast: list[socket.socket],
    med: list[socket.socket],
    slow: list[socket.socket],
    command_id: list[int],
    io: SignalBank,
    robot_type_code: int,
    tool_di_count: int,
    tool_do_count: int,
    running: threading.Event,
    tool_frames: dict[int, Pose] | None = None,
    active_tool: list[int] | None = None,
) -> None:
    pkt = DobotFeedbackPacket()
    tick = 0
    period = 0.008
    while running.is_set():
        deadline = time.monotonic() + period
        s = arm.state.snapshot()
        _update_feedback_packet(pkt, s, now_us=time.monotonic_ns() // 1000, command_id=command_id[0], robot_type_code=robot_type_code)
        pkt.DigitalInputs = sum(
            (1 << (i - 1)) for i in range(1, tool_di_count + 1) if io[f"tooldi{i}"].value
        )
        pkt.DigitalOutputs = sum(
            (1 << (i - 1)) for i in range(1, tool_do_count + 1) if io[f"tooldo{i}"].value
        )
        idx = active_tool[0] if active_tool else 0
        if idx > 0 and tool_frames and idx in tool_frames:
            tva = pkt.ToolVectorActual
            tva_m: Pose = (
                Meters(tva[0] * 1e-3), Meters(tva[1] * 1e-3), Meters(tva[2] * 1e-3),
                Radians(math.radians(tva[3])), Radians(math.radians(tva[4])), Radians(math.radians(tva[5])),
            )
            T_f = _pose_to_mat(tva_m)
            T_t = _pose_to_mat(tool_frames[idx])
            tcp = _mat_to_pose(T_f @ T_t)
            pkt.ToolVectorActual[:] = (
                tcp[0] * 1000, tcp[1] * 1000, tcp[2] * 1000,
                math.degrees(tcp[3]), math.degrees(tcp[4]), math.degrees(tcp[5]),
            )
        data = bytes(pkt)
        _send_to_all(fast, data)
        if tick % 25 == 0:
            _send_to_all(med, data)
            if tick % 125 == 0:
                _send_to_all(slow, data)
                qa = [f"{pkt.QActual[i]:+.4f}" for i in range(6)]
                tv = [f"{pkt.ToolVectorActual[i]:+.4f}" for i in range(6)]
                print(f"[fb] QActual=({','.join(qa)})  ToolVec=({','.join(tv)})", file=sys.stderr, flush=True)
        tick += 1
        remaining = deadline - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)


class DobotDashboard(LineServerDevice):
    """Emulated Dobot TCP/IP dashboard (port 29999)."""

    kind = "dobot_dashboard"
    DEFAULT_PORT = DOBOT_DASHBOARD_PORT
    FRAMER = PAREN
    _quiet_commands = frozenset({"tooldi", "gettooldo", "ai", "getao", "toolai", "geterrorid"})

    _FEEDBACK_PORTS = (DOBOT_FEEDBACK_FAST_PORT, DOBOT_FEEDBACK_MED_PORT, DOBOT_FEEDBACK_SLOW_PORT)

    def __init__(
        self,
        name: str,
        endpoint: Endpoint,
        bus: EventBus,
        options: ArmOptions,
        *,
        feedback_enabled: bool = True,
        robot_type_code: int = 5,
        model_info: _RobotModelInfo | None = None,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options)
        self.arm.start_ticker()
        self._model_info = model_info or _RobotModelInfo(type_code=robot_type_code)
        self._robot_type_code = self._model_info.type_code

        self._tool_di_count = self._model_info.tool_di_count
        self._tool_do_count = self._model_info.tool_do_count
        self._tool_ai_count = self._model_info.tool_ai_count
        self._ai_count = self._model_info.ai_count
        self._ao_count = self._model_info.ao_count

        self._current_command_id: list[int] = [0]
        self._running = threading.Event()
        self._running.set()
        self._error_ids: list[int] = []
        self._ai: list[float] = [0.0] * self._ai_count
        self._tool_ai: list[float] = [0.0] * self._tool_ai_count
        self._ao: list[float] = [0.0] * self._ao_count

        self._tool_frames: dict[int, Pose] = {}
        self._active_tool: list[int] = [0]

        self.io = SignalBank(name)
        for i in range(1, self._tool_di_count + 1):
            self.io.declare(f"tooldi{i}", direction=Direction.INPUT)
        for i in range(1, self._tool_do_count + 1):
            self.io.declare(f"tooldo{i}", direction=Direction.OUTPUT)
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
                args=(self.arm, self._clients_fast, self._clients_med, self._clients_slow, self._current_command_id, self.io, self._robot_type_code, self._tool_di_count, self._tool_do_count, self._running, self._tool_frames, self._active_tool),
                daemon=True,
            )
            self._writer.start()

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        verb, args = _parse(line)
        if os.environ.get("MACHINIST_LOG_STDERR"):
            try:
                log_level = int(os.environ["MACHINIST_LOG_STDERR"])
            except ValueError:
                log_level = 1
            if log_level >= 2 or verb.lower() not in self._quiet_commands:
                print(f"[dobot/{self.name}] {line}", file=sys.stderr, flush=True)
        s = self.arm.state.snapshot()
        match verb.lower():
            case "enablerobot":
                self.arm.set_servo(True); return _ok(verb, args)
            case "disablerobot":
                self.arm.set_servo(False); return _ok(verb, args)
            case "emergencystop":
                self.arm.estop()
                self._error_ids = [1]
                return _ok(verb, args)
            case "clearerror":
                self.arm.reset()
                self._error_ids.clear()
                return _ok(verb, args)
            case "stop":
                self.arm.stop()
                return _ok(verb, args)
            case "geterrorid":
                return _ok(verb, args, value="[" + ",".join(str(e) for e in self._error_ids) + "]")
            case "getpose":
                tool_idx = None
                if args.strip():
                    for part in args.split(","):
                        part = part.strip()
                        if "=" not in part:
                            continue
                        key, _, val = part.partition("=")
                        key = key.strip()
                        val = val.strip()
                        if key == "tool":
                            try:
                                tool_idx = int(val)
                            except ValueError:
                                return f"-30001,{{}},{verb}({args})"
                            if tool_idx < 0 or tool_idx > 50:
                                return f"-40001,{{}},{verb}({args})"
                            if tool_idx != 0 and tool_idx not in self._tool_frames:
                                return f"-1,{{}},{verb}({args})"
                        elif key == "user":
                            try:
                                user_idx = int(val)
                            except ValueError:
                                return f"-30001,{{}},{verb}({args})"
                            if user_idx < 0 or user_idx > 50:
                                return f"-40001,{{}},{verb}({args})"

                pose = s.pose
                if tool_idx is not None and tool_idx != 0:
                    T_f = _pose_to_mat(pose)
                    T_t = _pose_to_mat(self._tool_frames[tool_idx])
                    pose = _mat_to_pose(T_f @ T_t)
                pose_mm = (
                    pose[0] * 1000,
                    pose[1] * 1000,
                    pose[2] * 1000,
                    math.degrees(pose[3]),
                    math.degrees(pose[4]),
                    math.degrees(pose[5]),
                )
                return _ok(verb, args, value=",".join(f"{p:.4f}" for p in pose_mm))
            case "getangle":
                return _ok(verb, args, value=",".join(f"{math.degrees(j):.4f}" for j in s.joints))
            case "robotmode":
                return _ok(verb, args, value=str(_ARM_MODE_TO_ROBOT_MODE[s.mode]))
            case "tooldi":
                idx, err = _int_arg(args, verb, hi=self._tool_di_count)
                if err:
                    return err
                return _ok(verb, args, value=str(int(self.io[f"tooldi{idx}"].value)))
            case "gettooldo":
                idx, err = _int_arg(args, verb, hi=self._tool_do_count)
                if err:
                    return err
                return _ok(verb, args, value=str(int(self.io[f"tooldo{idx}"].value)))
            case "ai":
                idx, err = _int_arg(args, verb, hi=len(self._ai))
                if err:
                    return err
                return _ok(verb, args, value=str(self._ai[idx - 1]))
            case "getao":
                idx, err = _int_arg(args, verb, hi=len(self._ao))
                if err:
                    return err
                return _ok(verb, args, value=str(self._ao[idx - 1]))
            case "toolai":
                idx, err = _int_arg(args, verb, hi=len(self._tool_ai))
                if err:
                    return err
                return _ok(verb, args, value=str(self._tool_ai[idx - 1]))
            case "speedfactor":
                ratio, err = _int_arg(args, verb, hi=100)
                if err:
                    return err
                self.arm.set_speed_factor(ratio / 100)
                return _ok(verb, args)
            case "settool":
                try:
                    vals = _literal_eval_braced(args)
                    index = int(vals[0])
                    if index < 1 or index > 50:
                        return f"-40001,{{}},{verb}({args})"
                    pose = tuple(float(v) for v in vals[1])
                    if len(pose) != 6:
                        return f"-30001,{{}},{verb}({args})"
                except Exception:
                    return f"-30001,{{}},{verb}({args})"
                self._tool_frames[index] = (
                    Meters(pose[0] * 1e-3),
                    Meters(pose[1] * 1e-3),
                    Meters(pose[2] * 1e-3),
                    Radians(math.radians(pose[3])),
                    Radians(math.radians(pose[4])),
                    Radians(math.radians(pose[5])),
                )
                return _ok(verb, args)
            case "tool":
                idx, err = _int_arg(args, verb, lo=0, hi=50)
                if err:
                    return err
                if idx != 0 and idx not in self._tool_frames:
                    return f"-1,{{}},Tool({idx})"
                self._active_tool[0] = idx
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case "reljointmovj":
                try:
                    deltas = _parse_required_floats(args, count=len(s.joints))
                except ValueError:
                    return f"-30001,{{}},{verb}({args})"
                target: Joints = tuple(Radians(j + math.radians(d)) for j, d in zip(s.joints, deltas))
                self.arm.movej(target)
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case "relmovltool":
                try:
                    delta = _parse_required_floats(args, count=6)
                except ValueError:
                    return f"-30001,{{}},{verb}({args})"
                delta_m = np.array([delta[0] * 1e-3, delta[1] * 1e-3, delta[2] * 1e-3,
                                    math.radians(delta[3]), math.radians(delta[4]), math.radians(delta[5])], dtype=float)
                T_cur = _pose_to_mat(s.pose)
                R = T_cur[:3, :3]
                tool_pose = self._tool_frames.get(self._active_tool[0], (0.0,) * 6)
                T_tool = _pose_to_mat(tool_pose)  # type: ignore[arg-type]
                R_tool = T_tool[:3, :3]
                R_tcp = R @ R_tool
                p = R @ np.array([tool_pose[0], tool_pose[1], tool_pose[2]], dtype=float)
                twist = np.zeros(6, dtype=float)
                twist[:3] = R_tcp @ delta_m[:3]
                twist[3:] = R_tcp @ delta_m[3:]
                skew_p = np.array([[0, -p[2], p[1]],
                                   [p[2], 0, -p[0]],
                                   [-p[1], p[0], 0]], dtype=float)
                twist[:3] = twist[:3] + skew_p @ twist[3:]
                print(f"[dobot/{self.name}] RelMovLTool delta_m=({','.join(f'{v:.4f}' for v in delta_m)})", file=sys.stderr, flush=True)
                print(f"[dobot/{self.name}]   pose=({','.join(f'{v:.4f}' for v in s.pose)})  tool={self._active_tool[0]}  tool_pose=({','.join(f'{v:.4f}' for v in tool_pose)})", file=sys.stderr, flush=True)
                print(f"[dobot/{self.name}]   R_tcp=[[{R_tcp[0,0]:.4f},{R_tcp[0,1]:.4f},{R_tcp[0,2]:.4f}]...]  p=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})", file=sys.stderr, flush=True)
                print(f"[dobot/{self.name}]   world_flange_twist=({','.join(f'{v:.6f}' for v in twist)})", file=sys.stderr, flush=True)
                self.arm.jog_cartesian(twist, dt=1.0)
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case "movj":
                try:
                    joints = _parse_motion_args(args, count=len(s.joints))
                except ValueError:
                    return f"-30001,{{}},{verb}({args})"
                self.arm.movej(tuple(Radians(math.radians(j)) for j in joints))
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case "movl":
                try:
                    pose_mm = _parse_motion_args(args, count=6)
                except ValueError:
                    return f"-30001,{{}},{verb}({args})"
                self.arm.movel((
                    Meters(pose_mm[0] * 1e-3), Meters(pose_mm[1] * 1e-3), Meters(pose_mm[2] * 1e-3),
                    Radians(math.radians(pose_mm[3])), Radians(math.radians(pose_mm[4])), Radians(math.radians(pose_mm[5])),
                ))
                self._current_command_id[0] += 1
                return _ok(verb, args, value=str(self._current_command_id[0]))
            case _:
                return f"-10000,{{}},{verb}({args})"

    def build_detail(self) -> DeviceDetail:
        detail = super().build_detail()
        s = self.arm.state.snapshot()
        detail["derived_fields"] = [
            DetailField(signal="robottype", name="Robot type", offset="0", type="int", value=str(self._robot_type_code)),
            DetailField(signal="speedfactor", name="Speed factor", offset="0", type="int", value=f"{int(s.speed_fraction * 100)}%"),
        ] + [
            DetailField(signal=f"ai{i+1}", name=f"AI-{i+1}", offset=str(i), type="float", value=str(v))
            for i, v in enumerate(self._ai)
        ] + [
            DetailField(signal=f"ao{i+1}", name=f"AO-{i+1}", offset=str(i), type="float", value=str(v))
            for i, v in enumerate(self._ao)
        ] + [
            DetailField(signal=f"toolai{i+1}", name=f"ToolAI-{i+1}", offset=str(i), type="float", value=str(v))
            for i, v in enumerate(self._tool_ai)
        ]
        return detail

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


def _literal_eval_braced(text: str) -> tuple:
    """Parse comma‑separated args, converting ``{x,y,z}`` Set nodes to tuples.

    For example ``"1,{10,20,30,0,0,0}"`` becomes ``(1, (10.0, 20.0, …, 0.0))``.
    """
    tree = ast.parse(f"({text},)", mode="eval")
    assert isinstance(tree.body, ast.Tuple)
    out: list[Any] = []
    for elt in tree.body.elts:
        if isinstance(elt, ast.Set):
            out.append(tuple(ast.literal_eval(e) for e in elt.elts))
        else:
            out.append(ast.literal_eval(elt))
    return tuple(out)


def _parse_required_floats(text: str, *, count: int) -> list[float]:
    """Extract the first *count* float tokens from comma‑separated *text*.

    Stops at the first non-float token (e.g. ``"keyword=value"``), so
    trailing optional keyword arguments are silently ignored.
    """
    parts = [p.strip() for p in text.split(",") if p.strip()]
    floats: list[float] = []
    for p in parts:
        try:
            floats.append(float(p))
        except ValueError:
            break
        if len(floats) == count:
            break
    if len(floats) != count:
        raise ValueError(f"expected {count} floats, got {len(floats)} from {text!r}")
    return floats


def _parse_motion_args(text: str, *, count: int) -> list[float]:
    """Parse MovL/J motion arguments.

    Supports ``pose={x,y,z,rx,ry,rz}`` / ``joint={j1..j6}`` keyword format
    and bare positional floats.  Trailing ``keyword=value`` tokens are
    silently ignored in the bare-float form.
    """
    stripped = text.strip()
    if stripped.startswith("pose=") or stripped.startswith("joint="):
        eq = stripped.index("=")
        rest = stripped[eq + 1:].lstrip()
        if not rest.startswith("{") or "}" not in rest:
            raise ValueError(f"expected brace expression in {text!r}")
        end = rest.index("}")
        result = _literal_eval_braced(rest[:end + 1])
        vals = result[0]
        if len(vals) != count:
            raise ValueError(f"expected {count} values, got {len(vals)} from {text!r}")
        return [float(v) for v in vals]
    return _parse_required_floats(text, count=count)


def _int_arg(args: str, verb: str, *, lo: int = 1, hi: int) -> tuple[int | None, str | None]:
    if not args:
        return None, f"-20000,{{}},{verb}()"
    try:
        val = int(args.strip())
    except ValueError:
        return None, f"-30001,{{}},{verb}({args})"
    if val < lo or val > hi:
        return None, f"-40001,{{}},{verb}({args})"
    return val, None


def _ok(verb: str, args: str, *, value: str = "") -> str:
    return f"0,{{{value}}},{verb}({args})"


def _pose_to_mat(pose: Pose) -> NDArray[np.float64]:
    """4×4 homogeneous from ``(x,y,z,rx,ry,rz)`` (ZYX RPY)."""
    x, y, z, rx, ry, rz = pose
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = np.array([
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
        [-sy,     cy * sx,                 cx * cy],
    ])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [x, y, z]
    return T


def _mat_to_pose(T: NDArray[np.float64]) -> Pose:
    """Inverse of :func:`_pose_to_mat`."""
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    rz = math.atan2(T[1, 0], T[0, 0])
    ry = math.atan2(-T[2, 0], math.hypot(T[2, 1], T[2, 2]))
    rx = math.atan2(T[2, 1], T[2, 2])
    return (Meters(float(x)), Meters(float(y)), Meters(float(z)),
            Radians(rx), Radians(ry), Radians(rz))


@register("dobot_dashboard", default_port=DOBOT_DASHBOARD_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    raw = dict(options)
    robot_type_raw = raw.pop("robot_type", "cr5")
    model_info = DOBOT_ROBOT_MODELS.get(robot_type_raw, _DEFAULT_MODEL)
    dh = DHParams(**raw.pop("dh_params")) if "dh_params" in raw else model_info.dh_params
    kin = KinematicsOptions(**raw.pop("kinematics")) if "kinematics" in raw else None
    feedback = raw.pop("feedback_ports", None)
    feedback_enabled = feedback is not False
    return DobotDashboard(
        name, endpoint, bus, ArmOptions(kinematics=kin, dh_params=dh, **raw),
        feedback_enabled=feedback_enabled,
        model_info=model_info,
    )
