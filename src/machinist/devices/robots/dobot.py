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
import ctypes
import math
import os
import socket
import sys
import threading
import time
from collections.abc import Callable, Iterable, Sequence
from dataclasses import dataclass
from enum import StrEnum
from typing import Any

import numpy as np
from numpy.typing import NDArray
from pydantic import field_validator

from ...core.capabilities import HasFlange, HasIO
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, Signal, SignalBank
from ...core.line_device import LineServerDevice
from ...core.panel import Field, Panel, PanelChanged
from ...core.registry import register
from ...core.state import StateCell
from ...core.types import Endpoint
from ...kinematics.api import DHParams, Joints, Pose
from ...kinematics.units import Meters, Radians
from ...transport.flange_bus import FlangeBus, NoSlaveError
from ...transport.framing import PAREN
from ...transport.modbus_rtu_gateway import ModbusRtuGateway
from ...transport.line_server import Reply
from ...transport.service import Service
from .arm import ArmMode, ArmOptions, ArmStateView, HasArm, arm_from_options

DOBOT_DASHBOARD_PORT = 29999
DOBOT_FEEDBACK_FAST_PORT = 30004
DOBOT_FEEDBACK_MED_PORT = 30005
DOBOT_FEEDBACK_SLOW_PORT = 30006

#: Where the controller lets the network at the flange's RS485 line. A
#: client connects here and speaks RTU; the slave id picks the tool.
DOBOT_FLANGE_GATEWAY_PORT = 60000


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

# Robot mode codes, "RobotMode" in the interface guide. Only the ones the
# emulator can report are named here.
ROBOT_MODE_DISABLED = 4
ROBOT_MODE_ENABLE = 5
ROBOT_MODE_RUNNING = 7
ROBOT_MODE_ERROR = 9
ROBOT_MODE_COLLISION = 11

#: Modes an injected protective stop may report. A real controller reaches
#: COLLISION on a detected impact, ERROR on an uncleared alarm, and DISABLED
#: when it has dropped the servos -- the driver under test treats all three as
#: a protective stop, but takes different paths to get there.
PROTECTIVE_STOP_MODES = (ROBOT_MODE_COLLISION, ROBOT_MODE_ERROR, ROBOT_MODE_DISABLED)

#: Reply ErrorID for a command refused because the robot is alarmed, as
#: decoded by the vendor client ("The robot is in an error state").
ERR_ROBOT_IN_ERROR_STATE = -2

#: Reply ErrorID for a command the controller could not carry out.
ERR_COMMAND_FAILED = -1

#: The only Modbus value type the emulator serves. U32/F32/F64 repack values
#: across register pairs; refusing them keeps a future caller from silently
#: getting U16 data back under another name.
MODBUS_VALUE_TYPE = "U16"

class EnableFailure(StrEnum):
    """How an injected ``EnableRobot`` failure presents itself.

    ``STUCK`` is the realistic one: the controller accepts the command but
    never leaves DISABLED, because the alarm cause is still present. ``ERROR``
    is the blunter case where the command itself is rejected.
    """

    STUCK = "stuck"
    ERROR = "error"


#: How many Modbus masters a controller will hold open at once, per the
#: interface guide ("A maximum of 5 devices can be connected at the same time").
MAX_MODBUS_MASTERS = 5


@dataclass(slots=True)
class _ModbusMaster:
    """One Modbus master session, bound to a slave id on the flange line.

    ``ModbusRTUCreate`` binds a session to a single slave id, so two grippers
    on a Dual Quick Changer mean two sessions and two indices.
    """

    slave_id: int
    baud: int
    #: The serial arguments exactly as they arrived. The vendor client omits
    #: optional ones it considers default, which shifts the remaining
    #: positions -- there is no reliable way to tell a trailing data_bit from
    #: a trailing stop_bit. Nothing here drives a real UART, so they are kept
    #: verbatim for display rather than guessed at.
    serial: str = ""


#: Verbs that start motion. A robot in a protective stop refuses them all,
#: so a queued move cannot quietly resume while the stop is engaged.
_MOTION_VERBS = frozenset({"movj", "movl", "reljointmovj", "relmovltool"})

#: Protective-stop modes by the name the ``pstop`` command accepts.
PROTECTIVE_STOP_MODE_BY_NAME: dict[str, int] = {
    "collision": ROBOT_MODE_COLLISION,
    "error": ROBOT_MODE_ERROR,
    "disabled": ROBOT_MODE_DISABLED,
}

_ROBOT_MODE_NAMES: dict[int, str] = {
    ROBOT_MODE_DISABLED: "DISABLED",
    ROBOT_MODE_ENABLE: "ENABLE",
    ROBOT_MODE_RUNNING: "RUNNING",
    ROBOT_MODE_ERROR: "ERROR",
    ROBOT_MODE_COLLISION: "COLLISION",
}

_ARM_MODE_TO_ROBOT_MODE: dict[ArmMode, int] = {
    ArmMode.IDLE: ROBOT_MODE_ENABLE,
    ArmMode.MOVING: ROBOT_MODE_RUNNING,
    ArmMode.ESTOPPED: ROBOT_MODE_ERROR,
    ArmMode.FAULTED: ROBOT_MODE_ERROR,
}


@dataclass(frozen=True, slots=True)
class _FaultState:
    """An injected controller fault: what the robot reports while stopped.

    Held in the dashboard's StateCell and shared with the feedback-writer
    thread, so raising a fault is seen on both the dashboard port and the
    feedback stream without restarting anything.

    The reported mode is an override rather than an arm mode because COLLISION
    and DISABLED have no counterpart in the shared arm state machine, and no
    other vendor's robot would use them.
    """

    active: bool = False
    robot_mode: int = ROBOT_MODE_COLLISION
    sticky: bool = False
    enable_failure: EnableFailure | None = None
    #: What ``GetErrorID`` answers while the stop is engaged.
    alarm_ids: tuple[int, ...] = ()

    @property
    def robot_mode_override(self) -> int | None:
        if self.active:
            return self.robot_mode
        if self.enable_failure is EnableFailure.STUCK:
            # Refusing to enable *is* being disabled, so report it even with no
            # protective stop engaged.
            return ROBOT_MODE_DISABLED
        return None


def _robot_mode(state: ArmStateView, fault: _FaultState | None = None) -> int:
    """The mode code to report. A real e-stop outranks an injected fault."""
    if state.mode is ArmMode.ESTOPPED or fault is None:
        return _ARM_MODE_TO_ROBOT_MODE[state.mode]
    override = fault.robot_mode_override
    return _ARM_MODE_TO_ROBOT_MODE[state.mode] if override is None else override


def _update_feedback_packet(
    pkt: DobotFeedbackPacket,
    state: ArmStateView,
    *,
    now_us: int = 0,
    command_id: int = 0,
    robot_type_code: int = 5,
    tool: int = 0,
    payload: Sequence[float] | None = None,
    fault: _FaultState | None = None,
) -> None:
    pkt.len = 1440
    pkt.TestValue = 0x123456789abcdef
    pkt.RobotMode = _robot_mode(state, fault)
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
    pkt.RunQueuedCmd = 1 if state.mode is ArmMode.MOVING else 0
    pkt.CurrentCommandId = command_id
    pkt.Tool = tool
    if payload is not None:
        pkt.Load, pkt.CenterX, pkt.CenterY, pkt.CenterZ = payload


class _FeedbackStream(Service):
    """The controller's three feedback ports, served as one :class:`Service`.

    A real Dobot streams the same 1440-byte packet on 30004 (every 8 ms),
    30005 (every 200 ms) and 30006 (every second). The packet is built from
    whatever the dashboard holds at that instant, so the stream only needs a
    reference to it.
    """

    PORTS = (DOBOT_FEEDBACK_FAST_PORT, DOBOT_FEEDBACK_MED_PORT, DOBOT_FEEDBACK_SLOW_PORT)
    PERIOD = 0.008

    def __init__(self, dashboard: DobotDashboard) -> None:
        self._dashboard = dashboard
        self._stop = threading.Event()
        self._listeners: list[socket.socket] = []
        self._clients: tuple[list[socket.socket], ...] = ([], [], [])

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        for port, clients in zip(self.PORTS, self._clients, strict=True):
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("", port))
            sock.listen()
            sock.settimeout(1.0)
            self._listeners.append(sock)
            threading.Thread(target=self._accept, args=(sock, clients), daemon=True).start()
        if ready is not None:
            ready.set()
        self._write_loop()

    def shutdown(self) -> None:
        self._stop.set()
        for sock in self._listeners:
            sock.close()

    def _accept(self, sock: socket.socket, clients: list[socket.socket]) -> None:
        while not self._stop.is_set():
            try:
                client, _addr = sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            clients.append(client)

    def _write_loop(self) -> None:
        pkt = DobotFeedbackPacket()
        fast, med, slow = self._clients
        tick = 0
        while not self._stop.is_set():
            deadline = time.monotonic() + self.PERIOD
            self._dashboard.fill_feedback(pkt)
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


class DobotOptions(ArmOptions):
    """The ``dobot_dashboard`` block: an arm, which Dobot model it is, and the feedback ports."""

    robot_type: str = "cr5"
    #: Serve the 30004/30005/30006 feedback streams. Off for tests that only need the dashboard.
    feedback_ports: bool = True
    #: TCP doors onto the flange's RS485 line. Left out means the port a real
    #: controller answers on; ``false`` shuts the passthrough; a number or a
    #: list of them names the ports outright.
    flange_gateway_ports: tuple[int, ...] = (DOBOT_FLANGE_GATEWAY_PORT,)

    @field_validator("flange_gateway_ports", mode="before")
    @classmethod
    def _read_gateway_ports(cls, raw: Any) -> object:
        return _parse_gateway_ports(raw)


@dataclass(frozen=True, slots=True)
class _Request:
    """One dashboard command, with the state it is judged against."""

    verb: str
    args: str
    arm: ArmStateView
    fault: _FaultState


class DobotDashboard(LineServerDevice, HasArm, HasIO, HasFlange):
    """Emulated Dobot TCP/IP dashboard (port 29999)."""

    kind = "dobot_dashboard"
    DEFAULT_PORT = DOBOT_DASHBOARD_PORT
    FRAMER = PAREN
    _quiet_commands = frozenset(
        {"tooldi", "gettooldo", "ai", "getao", "toolai", "geterrorid", "getholdregs"}
    )

    def __init__(
        self,
        name: str,
        endpoint: Endpoint,
        bus: EventBus,
        options: ArmOptions,
        *,
        flange: FlangeBus,
        gateway: ModbusRtuGateway | None = None,
        feedback_enabled: bool = True,
        robot_type_code: int = 5,
        model_info: _RobotModelInfo | None = None,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options, name=name, publish=self.publish)
        self.add_service(self.arm)
        self._model_info = model_info or _RobotModelInfo(type_code=robot_type_code)
        self._robot_type_code = self._model_info.type_code

        self._tool_di_count = self._model_info.tool_di_count
        self._tool_do_count = self._model_info.tool_do_count
        self._tool_ai_count = self._model_info.tool_ai_count
        self._ai_count = self._model_info.ai_count
        self._ao_count = self._model_info.ao_count

        self._current_command_id = 0
        self._fault = StateCell(_FaultState(), on_change=lambda _view: self.announce_panel())
        self.flange = flange
        self._gateway = gateway
        if gateway is not None:
            self.add_service(gateway)
        self._masters: dict[int, _ModbusMaster] = {}
        self._ai: list[float] = [0.0] * self._ai_count
        self._tool_ai: list[float] = [0.0] * self._tool_ai_count
        self._ao: list[float] = [0.0] * self._ao_count

        self._tool_frames: dict[int, Pose] = {}
        self._active_tool = 0
        self._payload: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)  # load, cx, cy, cz

        self.io = SignalBank(name, publish=self.publish)
        for i in range(1, self._tool_di_count + 1):
            self.io.declare(f"tooldi{i}", direction=Direction.INPUT)
        for i in range(1, self._tool_do_count + 1):
            self.io.declare(f"tooldo{i}", direction=Direction.OUTPUT)
        if feedback_enabled:
            self.add_service(_FeedbackStream(self))

    def fill_feedback(self, pkt: DobotFeedbackPacket) -> None:
        """Write the controller's current state into one feedback packet."""
        active_tool = self._active_tool
        _update_feedback_packet(
            pkt,
            self.arm.state.view,
            now_us=time.monotonic_ns() // 1000,
            command_id=self._current_command_id,
            robot_type_code=self._robot_type_code,
            tool=active_tool,
            payload=self._payload,
            fault=self._fault.view,
        )
        pkt.DigitalInputs = sum(
            (1 << (i - 1)) for i in range(1, self._tool_di_count + 1) if self.io[f"tooldi{i}"].value
        )
        pkt.DigitalOutputs = sum(
            (1 << (i - 1)) for i in range(1, self._tool_do_count + 1) if self.io[f"tooldo{i}"].value
        )
        if active_tool > 0 and active_tool in self._tool_frames:
            t = self._tool_frames[active_tool]
            pkt.ToolValue[:] = (
                t[0] * 1000, t[1] * 1000, t[2] * 1000,
                math.degrees(t[3]), math.degrees(t[4]), math.degrees(t[5]),
            )

    def handle_line(self, line: str) -> Reply:
        verb, args = _parse(line)
        if os.environ.get("MACHINIST_LOG_STDERR"):
            try:
                log_level = int(os.environ["MACHINIST_LOG_STDERR"])
            except ValueError:
                log_level = 1
            if log_level >= 2 or verb.lower() not in self._quiet_commands:
                print(f"[dobot/{self.name}] {line}", file=sys.stderr, flush=True)
        req = _Request(verb=verb, args=args, arm=self.arm.state.view, fault=self._fault.view)
        if verb.lower() in _MOTION_VERBS and req.fault.active:
            return f"{ERR_ROBOT_IN_ERROR_STATE},{{}},{verb}({args})"
        handler = _VERBS.get(verb.lower())
        if handler is None:
            return f"-10000,{{}},{verb}({args})"
        return handler(self, req)

    # ----- dashboard verbs, one method each ----------------------------------

    def _verb_enablerobot(self, req: _Request) -> Reply:
        if req.fault.enable_failure is EnableFailure.ERROR:
            return f"{ERR_ROBOT_IN_ERROR_STATE},{{}},{req.verb}({req.args})"
        if req.fault.enable_failure is EnableFailure.STUCK:
            # Accepted, but the servos never come on -- RobotMode keeps
            # reporting DISABLED, so a driver'req.arm enable loop times out.
            return _ok(req.verb, req.args)
        self.arm.set_servo(True); return _ok(req.verb, req.args)

    def _verb_disablerobot(self, req: _Request) -> Reply:
        self.arm.set_servo(False); return _ok(req.verb, req.args)

    def _verb_emergencystop(self, req: _Request) -> Reply:
        self.arm.estop()
        self._fault.update(alarm_ids=(1,))
        return _ok(req.verb, req.args)

    def _verb_modbusrtucreate(self, req: _Request) -> Reply:
        try:
            slave_id, baud, serial = _parse_modbus_rtu_create(req.args)
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        index = self._next_master_index()
        if index is None:
            return f"{ERR_COMMAND_FAILED},{{}},{req.verb}({req.args})"
        self._masters[index] = _ModbusMaster(slave_id=slave_id, baud=baud, serial=serial)
        self.announce_panel()
        return _ok(req.verb, req.args, value=str(index))

    def _verb_modbusclose(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, lo=0, hi=MAX_MODBUS_MASTERS - 1)
        if err:
            return err
        if self._masters.pop(idx, None) is None:
            return f"{ERR_COMMAND_FAILED},{{}},{req.verb}({req.args})"
        self.announce_panel()
        return _ok(req.verb, req.args)

    def _verb_getholdregs(self, req: _Request) -> Reply:
        try:
            index, addr, count, val_type = _parse_hold_regs_read(req.args)
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        if val_type != MODBUS_VALUE_TYPE:
            return f"-40001,{{}},{req.verb}({req.args})"
        master = self._masters.get(index)
        if master is None:
            return f"{ERR_COMMAND_FAILED},{{}},{req.verb}({req.args})"
        try:
            values = self.flange.read_holding(master.slave_id, addr, count)
        except NoSlaveError:
            return f"{ERR_COMMAND_FAILED},{{}},{req.verb}({req.args})"
        return _ok(req.verb, req.args, value=",".join(str(v) for v in values))

    def _verb_setholdregs(self, req: _Request) -> Reply:
        try:
            index, addr, count, values, val_type = _parse_hold_regs_write(req.args)
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        if val_type != MODBUS_VALUE_TYPE:
            return f"-40001,{{}},{req.verb}({req.args})"
        if len(values) != count:
            return f"-30001,{{}},{req.verb}({req.args})"
        master = self._masters.get(index)
        if master is None:
            return f"{ERR_COMMAND_FAILED},{{}},{req.verb}({req.args})"
        try:
            self.flange.write_holding(master.slave_id, addr, values)
        except NoSlaveError:
            return f"{ERR_COMMAND_FAILED},{{}},{req.verb}({req.args})"
        return _ok(req.verb, req.args)

    def _verb_clearerror(self, req: _Request) -> Reply:
        self.arm.reset()
        if req.fault.active and req.fault.sticky:
            # The alarm cause is still present, so the controller
            # reports the same alarm straight back. ClearError itself
            # still succeeds -- the guide says to re-read RobotMode to
            # find out whether the robot is actually clear.
            return _ok(req.verb, req.args)
        self.clear_protective_stop()
        return _ok(req.verb, req.args)

    def _verb_stop(self, req: _Request) -> Reply:
        self.arm.stop()
        return _ok(req.verb, req.args)

    def _verb_geterrorid(self, req: _Request) -> Reply:
        return _ok(req.verb, req.args, value="[" + ",".join(str(e) for e in req.fault.alarm_ids) + "]")

    def _verb_getpose(self, req: _Request) -> Reply:
        tool_idx = None
        if req.args.strip():
            for part in req.args.split(","):
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
                        return f"-30001,{{}},{req.verb}({req.args})"
                    if tool_idx < 0 or tool_idx > 50:
                        return f"-40001,{{}},{req.verb}({req.args})"
                    if tool_idx != 0 and tool_idx not in self._tool_frames:
                        return f"-1,{{}},{req.verb}({req.args})"
                elif key == "user":
                    try:
                        user_idx = int(val)
                    except ValueError:
                        return f"-30001,{{}},{req.verb}({req.args})"
                    if user_idx < 0 or user_idx > 50:
                        return f"-40001,{{}},{req.verb}({req.args})"

        pose = req.arm.pose
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
        return _ok(req.verb, req.args, value=",".join(f"{p:.4f}" for p in pose_mm))

    def _verb_getangle(self, req: _Request) -> Reply:
        return _ok(req.verb, req.args, value=",".join(f"{math.degrees(j):.4f}" for j in req.arm.joints))

    def _verb_robotmode(self, req: _Request) -> Reply:
        return _ok(req.verb, req.args, value=str(_robot_mode(req.arm, req.fault)))

    def _verb_tooldi(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, hi=self._tool_di_count)
        if err:
            return err
        return _ok(req.verb, req.args, value=str(int(self.io[f"tooldi{idx}"].value)))

    def _verb_gettooldo(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, hi=self._tool_do_count)
        if err:
            return err
        return _ok(req.verb, req.args, value=str(int(self.io[f"tooldo{idx}"].value)))

    def _verb_ai(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, hi=len(self._ai))
        if err:
            return err
        return _ok(req.verb, req.args, value=str(self._ai[idx - 1]))

    def _verb_getao(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, hi=len(self._ao))
        if err:
            return err
        return _ok(req.verb, req.args, value=str(self._ao[idx - 1]))

    def _verb_toolai(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, hi=len(self._tool_ai))
        if err:
            return err
        return _ok(req.verb, req.args, value=str(self._tool_ai[idx - 1]))

    def _verb_speedfactor(self, req: _Request) -> Reply:
        ratio, err = _int_arg(req.args, req.verb, hi=100)
        if err:
            return err
        self.arm.set_speed_factor(ratio / 100)
        self.announce_panel()  # the panel shows the speed factor
        return _ok(req.verb, req.args)

    def _verb_settool(self, req: _Request) -> Reply:
        try:
            vals = _literal_eval_braced(req.args)
            index = int(vals[0])
            if index < 1 or index > 50:
                return f"-40001,{{}},{req.verb}({req.args})"
            pose = tuple(float(v) for v in vals[1])
            if len(pose) != 6:
                return f"-30001,{{}},{req.verb}({req.args})"
        except Exception:
            return f"-30001,{{}},{req.verb}({req.args})"
        self._tool_frames[index] = (
            Meters(pose[0] * 1e-3),
            Meters(pose[1] * 1e-3),
            Meters(pose[2] * 1e-3),
            Radians(math.radians(pose[3])),
            Radians(math.radians(pose[4])),
            Radians(math.radians(pose[5])),
        )
        # SetTool silently bumps CurrentCommandId even though the
        # response carries no value field (it'req.arm "immediate" per the
        # protocol docs, but the real Dobot queues it internally).
        self._current_command_id += 1
        return _ok(req.verb, req.args)

    def _verb_setpayload(self, req: _Request) -> Reply:
        parts = [p.strip() for p in req.args.split(",")]
        try:
            load = float(parts[0])
        except (ValueError, IndexError):
            return f"-30001,{{}},{req.verb}({req.args})"
        if len(parts) == 4:
            try:
                cx, cy, cz = float(parts[1]), float(parts[2]), float(parts[3])
            except ValueError:
                return f"-30001,{{}},{req.verb}({req.args})"
            self._payload = (load, cx, cy, cz)
        elif len(parts) == 1:
            self._payload = (load, 0.0, 0.0, 0.0)
        else:
            return f"-30001,{{}},{req.verb}({req.args})"
        self._current_command_id += 1
        return _ok(req.verb, req.args, value=str(self._current_command_id))

    def _verb_tool(self, req: _Request) -> Reply:
        idx, err = _int_arg(req.args, req.verb, lo=0, hi=50)
        if err:
            return err
        if idx != 0 and idx not in self._tool_frames:
            return f"-1,{{}},Tool({idx})"
        self._active_tool = idx
        self._current_command_id += 1
        return _ok(req.verb, req.args, value=str(self._current_command_id))

    def _verb_reljointmovj(self, req: _Request) -> Reply:
        try:
            deltas = _parse_required_floats(req.args, count=len(req.arm.joints))
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        target: Joints = tuple(Radians(j + math.radians(d)) for j, d in zip(req.arm.joints, deltas))
        self.arm.movej(target)
        self._current_command_id += 1
        return _ok(req.verb, req.args, value=str(self._current_command_id))

    def _verb_relmovltool(self, req: _Request) -> Reply:
        try:
            delta = _parse_required_floats(req.args, count=6)
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        delta_m = np.array([delta[0] * 1e-3, delta[1] * 1e-3, delta[2] * 1e-3,
                            math.radians(delta[3]), math.radians(delta[4]), math.radians(delta[5])], dtype=float)
        T_cur = _pose_to_mat(req.arm.pose)
        R = T_cur[:3, :3]
        tool_pose = self._tool_frames.get(self._active_tool, (0.0,) * 6)
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
        print(f"[dobot/{self.name}]   pose=({','.join(f'{v:.4f}' for v in req.arm.pose)})  tool={self._active_tool}  tool_pose=({','.join(f'{v:.4f}' for v in tool_pose)})", file=sys.stderr, flush=True)
        print(f"[dobot/{self.name}]   R_tcp=[[{R_tcp[0,0]:.4f},{R_tcp[0,1]:.4f},{R_tcp[0,2]:.4f}]...]  p=({p[0]:.4f},{p[1]:.4f},{p[2]:.4f})", file=sys.stderr, flush=True)
        print(f"[dobot/{self.name}]   world_flange_twist=({','.join(f'{v:.6f}' for v in twist)})", file=sys.stderr, flush=True)
        self.arm.jog_cartesian(twist, dt=1.0)
        self._current_command_id += 1
        return _ok(req.verb, req.args, value=str(self._current_command_id))

    def _verb_movj(self, req: _Request) -> Reply:
        try:
            vals, form = _parse_motion_args(req.args, count=len(req.arm.joints))
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        if form == "joint":
            # MovJ(joint={j1..j6}): target is joint coordinates (degrees).
            self.arm.movej(tuple(Radians(math.radians(j)) for j in vals))
        else:
            # MovJ(pose={…}) or bare floats: target is a Cartesian pose
            # (mm + deg); joint-interpolated motion to it via IK. This is
            # the default point type per the TCP/IP interface guide.
            self.arm.movej_pose((
                Meters(vals[0] * 1e-3), Meters(vals[1] * 1e-3), Meters(vals[2] * 1e-3),
                Radians(math.radians(vals[3])), Radians(math.radians(vals[4])), Radians(math.radians(vals[5])),
            ))
        self._current_command_id += 1
        return _ok(req.verb, req.args, value=str(self._current_command_id))

    def _verb_movl(self, req: _Request) -> Reply:
        try:
            pose_mm, _form = _parse_motion_args(req.args, count=6)
        except ValueError:
            return f"-30001,{{}},{req.verb}({req.args})"
        self.arm.movel((
            Meters(pose_mm[0] * 1e-3), Meters(pose_mm[1] * 1e-3), Meters(pose_mm[2] * 1e-3),
            Radians(math.radians(pose_mm[3])), Radians(math.radians(pose_mm[4])), Radians(math.radians(pose_mm[5])),
        ))
        self._current_command_id += 1
        return _ok(req.verb, req.args, value=str(self._current_command_id))

    # ----- modbus masters ------------------------------------------------

    @property
    def flange_gateway_ports(self) -> tuple[int, ...]:
        """TCP ports that open straight onto the flange line, if any."""
        return () if self._gateway is None else self._gateway.ports

    def _next_master_index(self) -> int | None:
        return next(
            (i for i in range(MAX_MODBUS_MASTERS) if i not in self._masters), None
        )

    # ----- fault injection ---------------------------------------------

    def inject_protective_stop(
        self,
        *,
        robot_mode: int = ROBOT_MODE_COLLISION,
        controller_ids: Iterable[int] = (),
        sticky: bool = False,
    ) -> None:
        """Put the robot into a protective stop, as a collision would.

        *robot_mode* is what ``RobotMode`` and the feedback stream report --
        one of :data:`PROTECTIVE_STOP_MODES`. *controller_ids* become the
        ``GetErrorID`` reply, and may be changed by calling again while the
        stop is still engaged. A *sticky* stop survives ``ClearError``, the way
        an alarm whose cause is still present does on a real controller.
        """
        if robot_mode not in PROTECTIVE_STOP_MODES:
            raise ValueError(
                f"robot_mode {robot_mode} is not a protective stop; "
                f"expected one of {PROTECTIVE_STOP_MODES}"
            )
        self.arm.fault()
        self._fault.update(
            active=True,
            robot_mode=robot_mode,
            sticky=sticky,
            alarm_ids=tuple(int(code) for code in controller_ids),
        )
        self.emit(
            "fault",
            state="engaged",
            mode=_ROBOT_MODE_NAMES.get(robot_mode, str(robot_mode)),
            alarms=self._alarm_ids_detail(),
            sticky=sticky,
        )

    def set_enable_failure(self, failure: EnableFailure | None) -> None:
        """Make ``EnableRobot`` fail, so an unlock attempt cannot succeed.

        Independent of :meth:`inject_protective_stop`, because a robot can
        refuse to enable without being in a protective stop. Pass ``None`` to
        let it enable normally again.
        """
        self._fault.update(enable_failure=failure)
        self.emit("fault", enable_failure=failure.value if failure else "none")

    def clear_protective_stop(self) -> None:
        """Release an injected protective stop, whether or not it is sticky."""
        self._fault.update(active=False, sticky=False, alarm_ids=())
        self.arm.clear_fault()
        self.emit("fault", state="clear")

    def clear_faults(self) -> None:
        """Release everything injected: the stop and any enable failure."""
        self.clear_protective_stop()
        self.set_enable_failure(None)

    # ----- detail panel -------------------------------------------------

    def announce_panel(self) -> None:
        """Publish the detail panel; called whenever something it shows has changed."""
        self.publish(PanelChanged(device=self.name, panel=self.build_detail()))

    def _protective_stop_detail(self) -> str:
        fault = self._fault.view
        if not fault.active:
            return "clear"
        name = _ROBOT_MODE_NAMES.get(fault.robot_mode, "?")
        sticky = ", sticky" if fault.sticky else ""
        return f"ENGAGED: {name} ({fault.robot_mode}){sticky}"

    def _alarm_ids_detail(self) -> str:
        return ",".join(str(code) for code in self._fault.view.alarm_ids) or "-"

    def _enable_failure_detail(self) -> str:
        failure = self._fault.view.enable_failure
        if failure is None:
            return "-"
        if failure is EnableFailure.STUCK:
            return "stuck (stays DISABLED)"
        return "error (EnableRobot rejected)"

    def _flange_detail(self) -> str:
        ids = self.flange.slave_ids
        return ", ".join(f"0x{slave_id:02X}" for slave_id in ids) if ids else "-"

    def _master_details(self) -> list[tuple[int, str]]:
        rows = []
        for index in sorted(self._masters):
            master = self._masters[index]
            answers = "" if master.slave_id in self.flange.slave_ids else ", no answer"
            serial = f" {master.serial}" if master.serial else ""
            rows.append(
                (index, f"slave 0x{master.slave_id:02X} @ {master.baud}{serial}{answers}")
            )
        return rows

    def build_detail(self) -> Panel:
        """The controller's IO, digital and analogue, with its standing state as status rows.

        A panel that lists inputs and outputs replaces the plain signal rows, so
        the tool DI/DO bits are listed here too, next to the analogue channels.
        """
        s = self.arm.state.snapshot()
        status = [
            Field(signal="robottype", name="Robot type", type="int", value=str(self._robot_type_code)),
            Field(signal="speedfactor", name="Speed factor", type="int", value=f"{int(s.speed_fraction * 100)}%"),
            Field(signal="pstop", name="Protective stop", type="str", value=self._protective_stop_detail()),
            Field(signal="alarmids", name="Alarm IDs", type="str", value=self._alarm_ids_detail()),
            Field(signal="enablefail", name="Enable failure", type="str", value=self._enable_failure_detail()),
            Field(signal="flange", name="Flange slaves", type="str", value=self._flange_detail()),
        ] + [
            Field(signal=f"master{index}", name=f"Modbus master {index}", type="str", value=value)
            for index, value in self._master_details()
        ]
        inputs = [_bit_row(sig) for sig in self.io if sig.direction is Direction.INPUT] + [
            Field(signal=f"AI{i + 1}", name=f"AI-{i + 1}", type="float", value=str(v))
            for i, v in enumerate(self._ai)
        ] + [
            Field(signal=f"TOOLAI{i + 1}", name=f"ToolAI-{i + 1}", type="float", value=str(v))
            for i, v in enumerate(self._tool_ai)
        ]
        outputs = [_bit_row(sig) for sig in self.io if sig.direction is Direction.OUTPUT] + [
            Field(signal=f"AO{i + 1}", name=f"AO-{i + 1}", type="float", value=str(v))
            for i, v in enumerate(self._ao)
        ]
        return Panel(
            mode="dashboard",
            input_fields=tuple(inputs),
            output_fields=tuple(outputs),
            status_fields=tuple(status),
        )


#: Dashboard verb (lower case) -> the method that answers it.
_VERBS: dict[str, Callable[[DobotDashboard, _Request], Reply]] = {
    "enablerobot": DobotDashboard._verb_enablerobot,
    "disablerobot": DobotDashboard._verb_disablerobot,
    "emergencystop": DobotDashboard._verb_emergencystop,
    "modbusrtucreate": DobotDashboard._verb_modbusrtucreate,
    "modbusclose": DobotDashboard._verb_modbusclose,
    "getholdregs": DobotDashboard._verb_getholdregs,
    "setholdregs": DobotDashboard._verb_setholdregs,
    "clearerror": DobotDashboard._verb_clearerror,
    "stop": DobotDashboard._verb_stop,
    "geterrorid": DobotDashboard._verb_geterrorid,
    "getpose": DobotDashboard._verb_getpose,
    "getangle": DobotDashboard._verb_getangle,
    "robotmode": DobotDashboard._verb_robotmode,
    "tooldi": DobotDashboard._verb_tooldi,
    "gettooldo": DobotDashboard._verb_gettooldo,
    "ai": DobotDashboard._verb_ai,
    "getao": DobotDashboard._verb_getao,
    "toolai": DobotDashboard._verb_toolai,
    "speedfactor": DobotDashboard._verb_speedfactor,
    "settool": DobotDashboard._verb_settool,
    "setpayload": DobotDashboard._verb_setpayload,
    "tool": DobotDashboard._verb_tool,
    "reljointmovj": DobotDashboard._verb_reljointmovj,
    "relmovltool": DobotDashboard._verb_relmovltool,
    "movj": DobotDashboard._verb_movj,
    "movl": DobotDashboard._verb_movl,
}


def _bit_row(sig: Signal) -> Field:
    return Field(signal=sig.name.upper(), name=sig.name, type="bit", value="ON" if sig.value else "OFF", on=sig.value)


# --- helpers ---------------------------------------------------------


def _parse(line: str) -> tuple[str, str]:
    """Split ``Verb(args)`` into ``(verb, args)``."""
    line = line.strip()
    if "(" not in line or not line.endswith(")"):
        return line, ""
    verb, rest = line.split("(", 1)
    return verb.strip(), rest[:-1]


def _parse_gateway_ports(raw: Any) -> tuple[int, ...]:
    """Read the ``flange_gateway_ports`` option into the ports to listen on.

    Left out means the port a real controller answers on; ``false`` shuts the
    passthrough; a number or a list of them names the ports outright.
    """
    if raw is None:
        return (DOBOT_FLANGE_GATEWAY_PORT,)
    if raw is False:
        return ()
    if isinstance(raw, int) and not isinstance(raw, bool):
        return (raw,)
    if isinstance(raw, list | tuple):
        return tuple(int(port) for port in raw)
    raise ValueError(f"flange_gateway_ports wants false, a port or a list of them, got {raw!r}")


def _parse_modbus_rtu_create(args: str) -> tuple[int, int, str]:
    """Split ``ModbusRTUCreate`` into (slave_id, baud, remaining serial args).

    Only the first two are positionally reliable. The vendor client drops any
    optional argument whose value matches its default, so what follows is
    ambiguous -- ``ModbusRTUCreate(1,115200,E,1)`` carries a stop bit in the
    position the documented signature gives to a data bit.
    """
    parts = [p.strip() for p in args.split(",") if p.strip()]
    if len(parts) < 2:
        raise ValueError(f"expected at least slave_id and baud, got {args!r}")
    return int(parts[0]), int(parts[1]), ",".join(parts[2:])


def _parse_hold_regs_read(args: str) -> tuple[int, int, int, str]:
    """Split ``GetHoldRegs(index,addr,count[,valType])``."""
    parts = [p.strip() for p in args.split(",") if p.strip()]
    if len(parts) not in (3, 4):
        raise ValueError(f"expected index, addr, count[, valType], got {args!r}")
    val_type = parts[3] if len(parts) == 4 else MODBUS_VALUE_TYPE
    return int(parts[0]), int(parts[1]), int(parts[2]), val_type


def _parse_hold_regs_write(args: str) -> tuple[int, int, int, list[int], str]:
    """Split ``SetHoldRegs(index,addr,count,{v,...}[,valType])``.

    The value table is brace-wrapped, so it cannot be split on commas along
    with the rest.
    """
    head, brace, rest = args.partition("{")
    if not brace:
        raise ValueError(f"expected a braced value table in {args!r}")
    table, close, tail = rest.partition("}")
    if not close:
        raise ValueError(f"unterminated value table in {args!r}")

    leading = [p.strip() for p in head.split(",") if p.strip()]
    if len(leading) != 3:
        raise ValueError(f"expected index, addr, count before the table in {args!r}")
    trailing = [p.strip() for p in tail.split(",") if p.strip()]
    if len(trailing) > 1:
        raise ValueError(f"unexpected arguments after the value table in {args!r}")
    val_type = trailing[0] if trailing else MODBUS_VALUE_TYPE

    values = [int(v.strip()) for v in table.split(",") if v.strip()]
    return int(leading[0]), int(leading[1]), int(leading[2]), values, val_type


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


def _parse_motion_args(text: str, *, count: int) -> tuple[list[float], str]:
    """Parse MovL/J motion arguments into ``(values, form)``.

    Supports ``pose={x,y,z,rx,ry,rz}`` / ``joint={j1..j6}`` keyword format
    and bare positional floats.  Trailing ``keyword=value`` tokens are
    silently ignored in the bare-float form.

    ``form`` is ``"joint"`` for ``joint={…}``, ``"pose"`` for ``pose={…}``,
    otherwise ``"bare"``.  Callers need this because ``MovJ`` accepts *either*
    a Cartesian pose (``pose={…}`` / bare floats — the default point type in
    the interface guide) or joint coordinates (``joint={…}``); the two must be
    interpreted differently (IK vs. direct joint targets).
    """
    stripped = text.strip()
    if stripped.startswith("pose=") or stripped.startswith("joint="):
        form = "joint" if stripped.startswith("joint=") else "pose"
        eq = stripped.index("=")
        rest = stripped[eq + 1:].lstrip()
        if not rest.startswith("{") or "}" not in rest:
            raise ValueError(f"expected brace expression in {text!r}")
        end = rest.index("}")
        result = _literal_eval_braced(rest[:end + 1])
        vals = result[0]
        if len(vals) != count:
            raise ValueError(f"expected {count} values, got {len(vals)} from {text!r}")
        return [float(v) for v in vals], form
    return _parse_required_floats(text, count=count), "bare"


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


@register("dobot_dashboard", default_port=DOBOT_DASHBOARD_PORT, options=DobotOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: DobotOptions) -> Device:
    model_info = DOBOT_ROBOT_MODELS.get(options.robot_type, _DEFAULT_MODEL)
    if options.dh_params is None:
        # the model's own geometry, unless the scene gives explicit DH parameters
        options = options.model_copy(update={"dh_params": model_info.dh_params})
    flange = FlangeBus()
    gateway = (
        ModbusRtuGateway(host=endpoint.host, ports=options.flange_gateway_ports, line=flange)
        if options.flange_gateway_ports
        else None
    )
    return DobotDashboard(
        name, endpoint, bus, options,
        flange=flange,
        gateway=gateway,
        feedback_enabled=options.feedback_ports,
        model_info=model_info,
    )
