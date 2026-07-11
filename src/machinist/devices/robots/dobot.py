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

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import DHParams, KinematicsOptions
from ...transport.framing import PAREN
from .arm import ArmOptions, RobotArm, arm_from_options

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


class DobotDashboard(LineServerDevice):
    """Emulated Dobot TCP/IP dashboard (port 29999)."""

    kind = "dobot_dashboard"
    DEFAULT_PORT = DOBOT_DASHBOARD_PORT
    FRAMER = PAREN

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: ArmOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options)
        self.arm.start_ticker()

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
            case "movj":
                self.arm.movej(tuple(_parse_floats(args, count=len(s.joints))))
                return _ok(verb, args)
            case "movl":
                self.arm.movel(tuple(_parse_floats(args, count=6)))  # type: ignore[arg-type]
                return _ok(verb, args)
            case _:
                return f"-10000,{{}},{verb}({args})"

    def _shutdown(self) -> None:
        super()._shutdown()
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
    return DobotDashboard(name, endpoint, bus, ArmOptions(kinematics=kin, dh_params=dh, **raw))
