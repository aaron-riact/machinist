"""Yaskawa Motoman Ethernet-server emulator (NX100, DX200, YRC1000).

References: *NX100 Options Instructions for Ethernet Server Function*
(HW0483202), in the repository as ``NX100_http_network_command.pdf``, and
the DX200's (HW1481823). The two describe the same protocol; what differs
between controllers is in :data:`CONTROLLERS`.

The controller serves on TCP port 80 but the protocol is **not** HTTP. A
session runs::

    CONNECT Robot_access[ Keep-Alive:<n>]<CR><LF>
    ← OK: NX Information Server (3.00)[ Keep-Alive:<n>].<CR><LF>   (NX100's name)
    HOSTCTRL_REQUEST <Command> <Size><CR><LF>
    ← OK: <Command><CR><LF>                 (or NG: <Message><CR><LF>, then close)
    [if Size > 0] <Command data><CR>
    ← <Data><CR>                            (an answer that carries data)
    ← 0000<CR><LF>                          (an answer without data)
    ← ERROR:<Command> is not successful (<code>).<CR><LF>, then close

Without ``Keep-Alive`` the controller answers one command and closes the
socket; with ``Keep-Alive:<n>`` it answers *n* (``-1``: no limit).
"""

from __future__ import annotations

import math
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import Enum, IntEnum, IntFlag, auto
from pathlib import Path
from typing import Any, Literal

import numpy as np
from numpy.typing import NDArray
from pydantic import Field, model_validator

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import Joints, Pose
from ...kinematics.dh_backend import _mat_to_pose, pose_to_mat
from ...kinematics.units import Meters, Radians
from ...transport.framing import CR
from ...transport.line_server import Hangup, LineServer, Reply, SessionHandler
from .arm import ArmOptions, HasArm, NoLinearPath, RobotArm, arm_from_options

MOTOMAN_PORT = 80
#: Where the High-Speed Ethernet Server (HSES) listens, over UDP.
HSES_PORT = 10040
#: The reply to a START request the controller cannot accept, before it closes.
START_ERROR = "NG: HTTP Error Response"
NORMAL_ANSWER = "0000"
MAX_KEEP_ALIVE = 32767
#: Position answers carry this many axes: the robot's six and six external ones.
AXES_ON_THE_WIRE = 12
EXTERNAL_AXES = 6
#: Coordinate numbers: base, robot, then user frames from 2 on.
BASE_COORDINATE, ROBOT_COORDINATE = 0, 1
#: The alarm RALARM reports while the arm is faulted. A real controller names
#: the cause; the arm only knows that it faulted, so this is a stand-in.
FAULT_ALARM = 4000
#: RALARM answers an error and four alarms, each a code and its data.
ALARM_SLOTS = 4



@dataclass(frozen=True, slots=True)
class YaskawaArm:
    """What a scene gets by naming a Yaskawa arm."""

    #: The arm's kinematic chain, rooted at the controller's robot base frame.
    urdf: Path
    #: Encoder pulses per degree of each axis (S, L, U, R, B, T).
    pulses_per_degree: tuple[float, ...]
    #: Each axis's top speed in degrees per second.
    max_joint_speed_deg_s: tuple[float, ...]


_URDF_DIR = Path(__file__).parent / "urdf"

#: The arms by name.
ROBOTS: dict[str, YaskawaArm] = {
    # Pulses: 4096 per motor turn through each gear ratio, except L, whose
    # ratio did not match the arm and is measured from its pulse limits; the
    # riact Motoman driver converts with the same values. Speeds: the
    # specification sheet.
    "up50n": YaskawaArm(
        urdf=_URDF_DIR / "yr_up50n.urdf",
        pulses_per_degree=(1416.552, 1945.413, 1994.362, 895.052, 814.519, 418.627),
        max_joint_speed_deg_s=(170.0, 170.0, 170.0, 250.0, 250.0, 350.0),
    ),
}
#: Joint speed in percent of the maximum, and linear speed in mm/s or degrees/s.
MIN_JOINT_SPEED, MAX_JOINT_SPEED_PERCENT, MIN_LINEAR_SPEED = 0.01, 100.0, 0.1
#: A move takes at least this long, so a move to where the arm is still runs.
MIN_MOVE_SECONDS = 0.02
#: Signal numbers count eight to a byte: #22010 to #22017 are byte 2201's bits.
SIGNALS_PER_BYTE = 8


@dataclass(frozen=True, slots=True)
class ControllerProfile:
    """What one Yaskawa controller generation does differently on the Ethernet server."""

    #: The START reply, before the Keep-Alive count and the closing dot.
    server_name: str
    #: Decimals of Rx, Ry, Rz in answers (X, Y, Z always have 3).
    rotation_decimals: int
    #: Tool files 0 to max_tool.
    max_tool: int
    #: User frames 1 to user_frames, as coordinates 2 to user_frames + 1.
    user_frames: int
    #: The network inputs, the only signals IOWRITE may write.
    network_inputs: range

    @property
    def last_user_coordinate(self) -> int:
        return self.user_frames + 1

    @property
    def tool_coordinate(self) -> int:
        """IMOV may also step in the tool's own frame, numbered after the user frames."""
        return self.user_frames + 2


#: The controllers by name. NX100 and DX200 follow their Ethernet server
#: manuals; the YRC1000 has not been checked against one, so it takes the
#: DX200's values until a real controller says otherwise.
CONTROLLERS: dict[str, ControllerProfile] = {
    "nx100": ControllerProfile(
        server_name="OK: NX Information Server (3.00)",
        rotation_decimals=2,
        max_tool=23,
        user_frames=24,
        network_inputs=range(22010, 23288),
    ),
    "dx200": ControllerProfile(
        server_name="OK: DX Information Server (1.00)",
        rotation_decimals=4,
        max_tool=63,
        user_frames=64,
        network_inputs=range(27010, 29568),
    ),
}
CONTROLLERS["yrc1000"] = CONTROLLERS["dx200"]


Protocol = Literal["ethernet_server", "hses"]


class YaskawaOptions(ArmOptions):
    """The ``yaskawa`` block: the arm, its controller, and the protocols it serves."""

    #: Which controller generation answers: nx100, dx200 or yrc1000.
    controller: Literal["nx100", "dx200", "yrc1000"]
    #: The arm, from :data:`ROBOTS`. It supplies urdf, pulses_per_degree and
    #: max_joint_speed_deg_s wherever the scene does not give them.
    robot: str | None = None
    #: What the controller serves: the text Ethernet server on the device's
    #: port (TCP 80 by default), and HSES on hses_port (UDP).
    protocols: tuple[Protocol, ...] = ("ethernet_server",)
    hses_port: int = HSES_PORT

    #: Encoder pulses per degree of each joint, in joint order.
    pulses_per_degree: tuple[float, ...]
    #: Each joint's top speed in degrees per second, which 100 % of a joint
    #: move's speed means.
    max_joint_speed_deg_s: tuple[float, ...]

    #: What the cell's ladder copies: each network-input byte to another
    #: byte, as ``{22010: 30040}`` mirrors #22010-#22017 on #30040-#30047.
    #: Signal numbers name the first signal of each byte.
    ladder_copies: dict[int, int] = Field(default_factory=dict)

    @model_validator(mode="before")
    @classmethod
    def _fill_in_the_robot(cls, raw: Any) -> Any:
        if not isinstance(raw, dict):
            return raw
        name = raw.get("robot", cls.model_fields["robot"].default)
        if name is None:
            return raw
        if name not in ROBOTS:
            raise ValueError(f"unknown robot {name!r}; known: {', '.join(sorted(ROBOTS))}")
        arm = ROBOTS[name]
        return {
            "urdf": str(arm.urdf),
            "pulses_per_degree": arm.pulses_per_degree,
            "max_joint_speed_deg_s": arm.max_joint_speed_deg_s,
            **raw,
        }

    @model_validator(mode="after")
    def _protocols_the_controller_has(self) -> YaskawaOptions:
        if not self.protocols:
            raise ValueError("protocols is empty; the controller would serve nothing")
        if len(set(self.protocols)) != len(self.protocols):
            raise ValueError(f"protocols lists one twice: {list(self.protocols)}")
        if "hses" in self.protocols and self.controller == "nx100":
            raise ValueError("the NX100 has no HSES")
        return self

    @model_validator(mode="after")
    def _ladder_copies_name_bytes(self) -> YaskawaOptions:
        network_inputs = CONTROLLERS[self.controller].network_inputs
        for source, target in self.ladder_copies.items():
            if source not in network_inputs:
                raise ValueError(f"ladder copies from #{source}, not a network input")
            for signal in (source, target):
                if signal % 10:
                    raise ValueError(f"#{signal} is not the first signal of a byte")
        return self

    @model_validator(mode="after")
    def _one_value_per_joint(self) -> YaskawaOptions:
        for name in ("pulses_per_degree", "max_joint_speed_deg_s"):
            values = getattr(self, name)
            if len(values) != self.joint_count:
                raise ValueError(f"{len(values)} {name} for {self.joint_count} joints")
        return self


class MotomanNX100Options(YaskawaOptions):
    """The ``motoman_nx100`` block: a ``yaskawa`` block for an NX100 with a UP50N."""

    controller: Literal["nx100", "dx200", "yrc1000"] = "nx100"
    robot: str | None = "up50n"


class Refusal(Enum):
    """Why the controller refuses a command, whichever protocol carried it."""

    BAD_DATA = auto()
    UNDEFINED_USER_FRAME = auto()
    EMERGENCY_STOP = auto()
    SERVO_OFF = auto()
    HELD = auto()
    NO_LINEAR_PATH = auto()
    NOT_WRITABLE = auto()


#: The code in an Ethernet server ``ERROR:<Command> is not successful (<code>).``
#: answer. The manual leaves the list to the Data Transmission manual
#: (RE-CKI-A445), which is not at hand, so these codes are stand-ins that only
#: name why a command failed.
INTERPRETER_MESSAGES: dict[Refusal, int] = {
    Refusal.BAD_DATA: 9001,
    Refusal.UNDEFINED_USER_FRAME: 9002,
    Refusal.EMERGENCY_STOP: 9003,
    Refusal.SERVO_OFF: 9004,
    Refusal.HELD: 9005,
    Refusal.NO_LINEAR_PATH: 9006,
    Refusal.NOT_WRITABLE: 9007,
}


class Status1(IntFlag):
    """RSTATS Data-1: mode, cycle and operation."""

    STEP = 0x01
    ONE_CYCLE = 0x02
    AUTO = 0x04
    RUNNING = 0x08
    SAFETY_SPEED = 0x10
    TEACH = 0x20
    PLAY = 0x40
    COMMAND_REMOTE = 0x80


class Status2(IntFlag):
    """RSTATS Data-2: holds, alarm, error and servo."""

    HOLD_PENDANT = 0x02
    HOLD_EXTERNAL = 0x04
    HOLD_COMMAND = 0x08
    ALARM = 0x10
    ERROR = 0x20
    SERVO_ON = 0x40


class CommandRefusedError(Exception):
    """A command the controller refuses; each protocol answers it its own way."""

    def __init__(self, refusal: Refusal, reason: str) -> None:
        super().__init__(reason)
        self.refusal = refusal


Command = Callable[[str], str | None]


@dataclass(slots=True)
class YaskawaController:
    """The controller's state and what it does, whichever protocol asks.

    Every server a device runs (the Ethernet server, HSES) acts on this one
    controller, as they do on a real one: a HOLD over one shows in the
    status the other reads.
    """

    arm: RobotArm
    profile: ControllerProfile
    pulses_per_degree: tuple[float, ...]
    max_joint_speed_deg_s: tuple[float, ...]
    cycle: Status1 = Status1.ONE_CYCLE
    #: The form (posture) type and tool number of the last move, which
    #: position reads report. The arm has no tool files, so the tool only
    #: travels along.
    form: int = 0
    tool: int = 0
    #: HOLD from a command: the arm stops and takes no move until it is released.
    hold: bool = False
    ladder_copies: dict[int, int] = field(default_factory=dict)
    #: Each IO byte by the number of its first signal; an unset byte reads 0.
    io: dict[int, int] = field(default_factory=dict)

    # --- status -----------------------------------------------------------

    def status(self) -> tuple[Status1, Status2]:
        view = self.arm.state.view
        data1 = Status1.COMMAND_REMOTE | Status1.PLAY | self.cycle
        if view.moving:
            data1 |= Status1.RUNNING
        data2 = Status2(0)
        if view.servo_on and not view.estopped:
            data2 |= Status2.SERVO_ON
        if view.faulted:
            data2 |= Status2.ALARM
        if self.hold:
            data2 |= Status2.HOLD_COMMAND
        return data1, data2

    def alarms(self) -> list[tuple[int, int]]:
        """The alarms showing, newest first, each as (code, data)."""
        return [(FAULT_ALARM, 0)] if self.arm.state.view.faulted else []

    def pulses(self) -> list[int]:
        """Each robot axis's position in encoder pulses."""
        return [
            round(math.degrees(joint) * ratio)
            for joint, ratio in zip(self.arm.state.view.joints, self.pulses_per_degree, strict=True)
        ]

    def joints_at(self, pulses: list[float]) -> Joints:
        """The joint angles at the robot axes' *pulses* (any external axes after them ignored)."""
        robot_pulses = pulses[: len(self.pulses_per_degree)]
        return tuple(
            Radians(math.radians(p / ratio))
            for p, ratio in zip(robot_pulses, self.pulses_per_degree, strict=True)
        )

    # --- operation ----------------------------------------------------------

    def set_servo(self, on: bool) -> None:
        if on:
            if self.arm.state.view.estopped:
                raise CommandRefusedError(Refusal.EMERGENCY_STOP, "emergency stop is engaged")
            self.arm.set_servo(True)
        else:
            self.arm.stop()
            self.arm.set_servo(False)

    def set_hold(self, on: bool) -> None:
        self.hold = on
        if on:
            self.arm.stop()

    def reset_alarm(self) -> None:
        self.arm.clear_fault()

    def cancel_error(self) -> None:
        """Cancels an error. The emulator raises none, so there is nothing to cancel."""

    def check_tool(self, tool: int) -> int:
        if not 0 <= tool <= self.profile.max_tool:
            raise CommandRefusedError(Refusal.BAD_DATA, f"tool {tool}")
        return tool

    def check_user_frame(self, number: int) -> None:
        """User frames exist by number, but none is taught."""
        if 1 <= number <= self.profile.user_frames:
            raise CommandRefusedError(
                Refusal.UNDEFINED_USER_FRAME, f"user frame {number} is not defined"
            )
        raise CommandRefusedError(Refusal.BAD_DATA, f"user frame {number}")

    def move_joints(self, target: Joints, speed_percent: float) -> None:
        """Joint-space move at *speed_percent* of each joint's top speed."""
        self._check_can_move()
        self.arm.movej(target, duration=self._joint_move_seconds(target, speed_percent))

    def move_linear(self, target: Pose, selection: SpeedSelection, speed: float) -> None:
        """Straight-line move, *speed* in mm/s (V) or degrees/s (VR)."""
        self._check_can_move()
        start = pose_to_mat(self.arm.state.view.pose)
        end = pose_to_mat(target)
        if selection is SpeedSelection.V:
            travel = float(np.linalg.norm(end[:3, 3] - start[:3, 3])) * 1000
        else:
            turn = start[:3, :3].T @ end[:3, :3]
            travel = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(turn) - 1) / 2))))
        try:
            self.arm.movel(target, duration=max(travel / speed, MIN_MOVE_SECONDS),
                           follows_speed_factor=False)
        except NoLinearPath as exc:
            raise CommandRefusedError(Refusal.NO_LINEAR_PATH, str(exc)) from exc

    def read_io(self, signals: list[int]) -> list[int]:
        """The bytes whose first signals are *signals*."""
        return [self.io.get(signal, 0) for signal in signals]

    def write_network_inputs(self, signals: list[int], values: list[int]) -> None:
        """Write bytes to network inputs, the only signals a host may write."""
        network_inputs = self.profile.network_inputs
        if signals[0] not in network_inputs or signals[-1] + 7 not in network_inputs:
            raise CommandRefusedError(Refusal.NOT_WRITABLE, f"#{signals[0]} is not a network input")
        for signal, value in zip(signals, values, strict=True):
            if not 0 <= value <= 0xFF:
                raise CommandRefusedError(Refusal.BAD_DATA, f"byte {value}")
            self.io[signal] = value
            if signal in self.ladder_copies:
                self.io[self.ladder_copies[signal]] = value

    def _check_can_move(self) -> None:
        view = self.arm.state.view
        if view.estopped:
            raise CommandRefusedError(Refusal.EMERGENCY_STOP, "emergency stop is engaged")
        if not view.servo_on:
            raise CommandRefusedError(Refusal.SERVO_OFF, "servo power is off")
        if self.hold:
            raise CommandRefusedError(Refusal.HELD, "the controller is held")

    def _joint_move_seconds(self, target: Joints, speed_percent: float) -> float:
        """The time the slowest joint needs at *speed_percent* of its top speed."""
        start = self.arm.state.view.joints
        seconds = max(
            abs(math.degrees(b - a)) / (top * speed_percent / 100)
            for a, b, top in zip(start, target, self.max_joint_speed_deg_s, strict=True)
        )
        return max(seconds, MIN_MOVE_SECONDS)


@dataclass(slots=True)
class EthernetServer:
    """The Ethernet server's text commands, read and answered for a :class:`YaskawaController`."""

    controller: YaskawaController

    def command(self, name: str) -> Command | None:
        """The command called *name*, or None for one the controller does not know.

        A command takes the command data and returns the answer's data, or
        None for an answer without data. It raises :class:`CommandRefusedError` when the
        controller refuses it.
        """
        commands: dict[str, Command] = {
            "RSTATS": self._rstats,
            "RPOSJ": self._rposj,
            "RPOSC": self._rposc,
            "RALARM": self._ralarm,
            "SVON": self._svon,
            "HOLD": self._hold,
            "RESET": self._reset,
            "CANCEL": self._cancel,
            "PMOVJ": self._pmovj,
            "PMOVL": self._pmovl,
            "MOVJ": self._movj,
            "MOVL": self._movl,
            "IMOV": self._imov,
            "IOREAD": self._ioread,
            "IOWRITE": self._iowrite,
        }
        return commands.get(name)

    def _rstats(self, data: str) -> str:
        data1, data2 = self.controller.status()
        return f"{int(data1)},{int(data2)}"

    def _rposj(self, data: str) -> str:
        return ",".join(str(p) for p in _pad(self.controller.pulses(), AXES_ON_THE_WIRE))

    def _rposc(self, data: str) -> str:
        coordinate, external = _parse_ints(data, count=2)
        self._check_coordinate(coordinate)
        if external not in (0, 1):
            raise CommandRefusedError(Refusal.BAD_DATA, f"external axis flag {external}")
        decimals = self.controller.profile.rotation_decimals
        x, y, z, rx, ry, rz = self.controller.arm.state.view.pose
        fields = [
            *(f"{m * 1000:.3f}" for m in (x, y, z)),
            *(f"{math.degrees(r):.{decimals}f}" for r in (rx, ry, rz)),
            str(self.controller.form),
            str(self.controller.tool),
        ]
        if external:
            fields += ["0"] * EXTERNAL_AXES
        return ",".join(fields)

    def _ralarm(self, data: str) -> str:
        alarms = [v for alarm in self.controller.alarms()[:ALARM_SLOTS] for v in alarm]
        return ",".join(str(v) for v in [0, 0, *_pad(alarms, ALARM_SLOTS * 2)])

    def _svon(self, data: str) -> None:
        self.controller.set_servo(_parse_switch(data))

    def _hold(self, data: str) -> None:
        self.controller.set_hold(_parse_switch(data))

    def _reset(self, data: str) -> None:
        self.controller.reset_alarm()

    def _cancel(self, data: str) -> None:
        self.controller.cancel_error()

    def _pmovj(self, data: str) -> None:
        """Speed (%), six pulses, tool, six external axes."""
        values = _parse_numbers(data, count=14)
        speed = _joint_speed(values[0])
        target = self.controller.joints_at(values[1:7])
        self.controller.tool = self.controller.check_tool(_integer(values[7]))
        self.controller.move_joints(target, speed)

    def _pmovl(self, data: str) -> None:
        """Speed selection, speed, six pulses, tool, six external axes."""
        values = _parse_numbers(data, count=15)
        selection, speed = _linear_speed(values[0], values[1])
        target = self.controller.arm.forward(self.controller.joints_at(values[2:8]))
        self.controller.tool = self.controller.check_tool(_integer(values[8]))
        self.controller.move_linear(target, selection, speed)

    def _movj(self, data: str) -> None:
        """Speed (%), coordinate, X Y Z Rx Ry Rz, type, tool, six external axes."""
        values = _parse_numbers(data, count=16)
        speed = _joint_speed(values[0])
        self._check_coordinate(_integer(values[1]))
        target = _pose(values[2:8])
        self.controller.form = _integer(values[8])
        self.controller.tool = self.controller.check_tool(_integer(values[9]))
        self.controller.move_joints(self.controller.arm.inverse(target), speed)

    def _movl(self, data: str) -> None:
        """Speed selection, speed, coordinate, X Y Z Rx Ry Rz, type, tool, six external axes."""
        values = _parse_numbers(data, count=17)
        selection, speed = _linear_speed(values[0], values[1])
        self._check_coordinate(_integer(values[2]))
        target = _pose(values[3:9])
        self.controller.form = _integer(values[9])
        self.controller.tool = self.controller.check_tool(_integer(values[10]))
        self.controller.move_linear(target, selection, speed)

    def _imov(self, data: str) -> None:
        """Speed selection, speed, coordinate, the step in X Y Z Rx Ry Rz, reserved, tool,
        six external axes.

        In the base or robot coordinate the step moves the tool along and turns it
        about the base axes; in the tool coordinate, along and about its own.
        """
        values = _parse_numbers(data, count=17)
        selection, speed = _linear_speed(values[0], values[1])
        coordinate = _integer(values[2])
        in_tool = coordinate == self.controller.profile.tool_coordinate
        if not in_tool:
            self._check_coordinate(coordinate)
        step = pose_to_mat(_pose(values[3:9]))
        self.controller.tool = self.controller.check_tool(_integer(values[10]))
        here = self.controller.arm.state.view.pose
        self.controller.move_linear(_stepped(here, step, in_tool=in_tool), selection, speed)

    def _ioread(self, data: str) -> str:
        """First signal and signal count → one value per byte."""
        start, count = _parse_ints(data, count=2)
        return ",".join(str(v) for v in self.controller.read_io(_bytes(start, count)))

    def _iowrite(self, data: str) -> None:
        """First signal, signal count, then one value per byte. Network inputs only."""
        values = _parse_ints(data, count=None)
        if len(values) < 3:
            raise CommandRefusedError(Refusal.BAD_DATA, f"{len(values)} fields")
        start, count, *bytes_ = values
        signals = _bytes(start, count)
        if len(bytes_) != len(signals):
            raise CommandRefusedError(Refusal.BAD_DATA, f"{len(bytes_)} bytes for {count} signals")
        self.controller.write_network_inputs(signals, bytes_)

    def _check_coordinate(self, coordinate: int) -> None:
        """Base and robot coordinates are one frame here (no base axis); user frames follow."""
        if coordinate in (BASE_COORDINATE, ROBOT_COORDINATE):
            return
        self.controller.check_user_frame(coordinate - 1)


def _stepped(pose: Pose, step: NDArray[np.float64], *, in_tool: bool) -> Pose:
    """*pose* moved by *step*: in the tool's own frame, or along and about the base axes."""
    here = pose_to_mat(pose)
    if in_tool:
        return _mat_to_pose(here @ step)
    there = np.eye(4)
    there[:3, :3] = step[:3, :3] @ here[:3, :3]
    there[:3, 3] = here[:3, 3] + step[:3, 3]
    return _mat_to_pose(there)


class SpeedSelection(IntEnum):
    """What a linear move's speed limits: V, the tool's travel in mm/s, or VR,
    its rotation in degrees/s."""

    V = 0
    VR = 1


def _joint_speed(value: float) -> float:
    if not MIN_JOINT_SPEED <= value <= MAX_JOINT_SPEED_PERCENT:
        raise CommandRefusedError(Refusal.BAD_DATA, f"joint speed {value} %")
    return value


def _linear_speed(selection: float, speed: float) -> tuple[SpeedSelection, float]:
    if selection not in (SpeedSelection.V, SpeedSelection.VR):
        raise CommandRefusedError(Refusal.BAD_DATA, f"speed selection {selection}")
    if speed < MIN_LINEAR_SPEED:
        raise CommandRefusedError(Refusal.BAD_DATA, f"linear speed {speed}")
    return SpeedSelection(int(selection)), speed


def _bytes(start: int, count: int) -> list[int]:
    """The first signal of each byte from *start*, for *count* signals.

    Both must fall on byte boundaries: a start that ends in 0 and a count in
    multiples of eight.
    """
    if start % 10 or count <= 0 or count % SIGNALS_PER_BYTE:
        raise CommandRefusedError(Refusal.BAD_DATA, f"signals #{start}, count {count}")
    return [start + 10 * i for i in range(count // SIGNALS_PER_BYTE)]


def _pose(values: list[float]) -> Pose:
    """X, Y, Z in mm and Rx, Ry, Rz in degrees → metres and radians."""
    x, y, z, rx, ry, rz = values
    return (
        Meters(x / 1000), Meters(y / 1000), Meters(z / 1000),
        Radians(math.radians(rx)), Radians(math.radians(ry)), Radians(math.radians(rz)),
    )


def _integer(value: float) -> int:
    if value != int(value):
        raise CommandRefusedError(Refusal.BAD_DATA, f"not an integer: {value}")
    return int(value)


def _parse_numbers(data: str, *, count: int) -> list[float]:
    """Exactly *count* comma-separated numbers."""
    try:
        values = [float(part) for part in data.split(",")]
    except ValueError as exc:
        raise CommandRefusedError(Refusal.BAD_DATA, f"not numbers: {data!r}") from exc
    if len(values) != count:
        raise CommandRefusedError(Refusal.BAD_DATA, f"{len(values)} fields, expected {count}")
    return values


def _parse_switch(data: str) -> bool:
    """``1`` for on, ``0`` for off."""
    (value,) = _parse_ints(data, count=1)
    if value not in (0, 1):
        raise CommandRefusedError(Refusal.BAD_DATA, f"switch {value}")
    return value == 1


def _parse_ints(data: str, *, count: int | None) -> list[int]:
    """Comma-separated integers: exactly *count* of them, or any number for None."""
    try:
        values = [int(part) for part in data.split(",")]
    except ValueError as exc:
        raise CommandRefusedError(Refusal.BAD_DATA, f"not integers: {data!r}") from exc
    if count is not None and len(values) != count:
        raise CommandRefusedError(Refusal.BAD_DATA, f"{len(values)} fields, expected {count}")
    return values


def _pad(values: list[int], count: int) -> list[int]:
    """*values* followed by zeros up to *count* (the external axes a cell lacks)."""
    return values + [0] * (count - len(values))


@dataclass(frozen=True, slots=True)
class _Request:
    name: str
    size: int
    run: Command


@dataclass(slots=True)
class _Session:
    """One connection: the START request, then up to Keep-Alive commands."""

    server: EthernetServer
    #: Commands this connection may run; None until the START request.
    keep_alive: int | None = None
    answered: int = 0
    pending: _Request | None = None

    def handle(self, message: str) -> Reply:
        if self.keep_alive is None:
            return self._start(message)
        if self.pending is not None:
            request, self.pending = self.pending, None
            return self._answer(request, message)
        return self._request(message)

    # --- START request ------------------------------------------------

    def _start(self, message: str) -> Reply:
        words = message.split()
        if words[:2] != ["CONNECT", "Robot_access"] or len(words) > 3:
            return Hangup([START_ERROR + "\r\n"])
        server_name = self.server.controller.profile.server_name
        if len(words) == 2:
            self.keep_alive = 1
            return server_name + ".\r\n"
        keep_alive = _parse_keep_alive(words[2])
        if keep_alive is None:
            return Hangup([START_ERROR + "\r\n"])
        self.keep_alive = keep_alive
        return f"{server_name} Keep-Alive:{keep_alive}.\r\n"

    # --- command ------------------------------------------------------

    def _request(self, message: str) -> Reply:
        words = message.split()
        if len(words) != 3 or words[0] != "HOSTCTRL_REQUEST" or not words[2].isdigit():
            return Hangup([f"NG: bad request {message!r}\r\n"])
        name, size = words[1], int(words[2])
        run = self.server.command(name)
        if run is None:
            return Hangup([f"NG: unknown command {name}\r\n"])
        request = _Request(name=name, size=size, run=run)
        ok = f"OK: {name}\r\n"
        if request.size > 0:
            self.pending = request
            return ok
        answer = self._answer(request, "")
        lines = [ok, *answer]
        return Hangup(lines) if isinstance(answer, Hangup) else lines

    def _answer(self, request: _Request, data: str) -> list[str] | Hangup:
        sent = len(data.encode()) + 1  # the size counts the closing CR
        if request.size > 0 and sent != request.size:
            return Hangup([f"NG: size {request.size} for {sent} bytes\r\n"])
        try:
            answer = request.run(data)
        except CommandRefusedError as exc:
            code = INTERPRETER_MESSAGES[exc.refusal]
            return Hangup([f"ERROR:{request.name} is not successful ({code:04d}).\r\n"])
        reply = NORMAL_ANSWER + "\r\n" if answer is None else answer + "\r"
        self.answered += 1
        if self.keep_alive != -1 and self.answered >= (self.keep_alive or 1):
            return Hangup([reply])
        return [reply]


def _parse_keep_alive(word: str) -> int | None:
    """``Keep-Alive:<n>`` (the manual's example ends it with a dot) → n, or None."""
    name, _, value = word.removesuffix(".").partition(":")
    if name != "Keep-Alive":
        return None
    try:
        n = int(value)
    except ValueError:
        return None
    return n if n == -1 or 2 <= n <= MAX_KEEP_ALIVE else None


class YaskawaRobot(Device, HasArm):
    """A Yaskawa arm and controller, serving the protocols its options name."""

    kind = "yaskawa"
    DEFAULT_PORT = MOTOMAN_PORT

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: YaskawaOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options, name=name, publish=self.publish)
        self.add_service(self.arm)
        self.protocols = options.protocols
        self.hses_port = options.hses_port
        self.controller = YaskawaController(
            arm=self.arm,
            profile=CONTROLLERS[options.controller],
            pulses_per_degree=options.pulses_per_degree,
            max_joint_speed_deg_s=options.max_joint_speed_deg_s,
            ladder_copies=dict(options.ladder_copies),
        )
        if "ethernet_server" in self.protocols:
            self.add_service(LineServer(
                endpoint.host, endpoint.port, session_factory=self.make_session, framer=CR,
            ))

    def make_session(self) -> SessionHandler:
        return _TracingSession(self, _Session(server=EthernetServer(self.controller)))

    def _serve(self, stop: threading.Event) -> None:
        if "hses" in self.protocols:
            self.emit(
                "warning",
                message=f"HSES is not implemented yet; nothing listens on UDP {self.hses_port}",
            )
        stop.wait()


@dataclass(slots=True)
class _TracingSession:
    """Thin wrapper that surfaces rx/tx events on the bus."""

    device: Device
    inner: SessionHandler

    def handle(self, message: str) -> Reply:
        self.device.emit("rx", line=message)
        reply = self.inner.handle(message)
        if reply is not None:
            self.device.emit("tx", reply=reply if isinstance(reply, str) else list(reply))
        return reply


@register("yaskawa", default_port=MOTOMAN_PORT, options=YaskawaOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: YaskawaOptions) -> Device:
    return YaskawaRobot(name, endpoint, bus, options)


@register("motoman_nx100", default_port=MOTOMAN_PORT, options=MotomanNX100Options)
def _nx100_factory(
    name: str, endpoint: Endpoint, bus: EventBus, options: MotomanNX100Options
) -> Device:
    return YaskawaRobot(name, endpoint, bus, options)
