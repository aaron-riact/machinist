"""A Yaskawa controller's state and what it does, whichever protocol asks."""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from datetime import datetime
from enum import Enum, IntEnum, IntFlag, auto
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from ....kinematics.api import Joints, Pose
from ....kinematics.dh_backend import _mat_to_pose, pose_to_mat
from ....kinematics.units import Radians
from ..arm import NoLinearPath, RobotArm

#: The alarm a faulted arm reports. A real controller names the cause; the
#: arm only knows that it faulted, so this is a stand-in.
FAULT_ALARM = 4000
#: A move takes at least this long, so a move to where the arm is still runs.
MIN_MOVE_SECONDS = 0.02


#: An alarm's time as the controller shows it, e.g. 2011/10/10 15:49.
ALARM_TIME_FORMAT = "%Y/%m/%d %H:%M"
#: How many alarms the history keeps (HSES reads 1 to 100).
ALARM_HISTORY = 100


@dataclass(frozen=True, slots=True)
class Alarm:
    code: int
    #: The sub code that says more, 0 when there is none.
    data: int
    time: str


@dataclass(frozen=True, slots=True)
class YaskawaArm:
    """What a scene gets by naming a Yaskawa arm."""

    #: The arm's kinematic chain, rooted at the controller's robot base frame.
    urdf: Path
    #: Encoder pulses per degree of each axis (S, L, U, R, B, T).
    pulses_per_degree: tuple[float, ...]
    #: Each axis's top speed in degrees per second.
    max_joint_speed_deg_s: tuple[float, ...]
    #: The model name the controller reports (HSES system information).
    model: str


_URDF_DIR = Path(__file__).parent.parent / "urdf"

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
        model="YR-UP50N-A00",
    ),
}


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


class Refusal(Enum):
    """Why the controller refuses a command, whichever protocol carried it."""

    BAD_DATA = auto()
    UNDEFINED_USER_FRAME = auto()
    EMERGENCY_STOP = auto()
    SERVO_OFF = auto()
    HELD = auto()
    NO_LINEAR_PATH = auto()
    NOT_WRITABLE = auto()
    BAD_TOOL = auto()
    NO_SUCH_JOB = auto()
    NO_EXECUTING_JOB = auto()


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


class SpeedSelection(IntEnum):
    """What a linear move's speed limits: V, the tool's travel in mm/s, or VR,
    its rotation in degrees/s."""

    V = 0
    VR = 1


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
    #: HLOCK: on a real controller it locks out the pendant and IO operation
    #: signals. The emulator has neither, so it is only remembered.
    hlock: bool = False
    ladder_copies: dict[int, int] = field(default_factory=dict)
    #: The arm's model name, empty when the scene names no arm from the catalog.
    model: str = ""
    #: Each IO byte by the number of its first signal; an unset byte reads 0.
    io: dict[int, int] = field(default_factory=dict)
    #: Registers and variables by kind ("register", "B", "I", ...), then number;
    #: an unset one reads 0.
    variables: dict[str, dict[int, float]] = field(default_factory=dict)
    #: The alarm showing, and every alarm so far, newest first.
    active_alarm: Alarm | None = None
    alarm_history: list[Alarm] = field(default_factory=list)
    #: Position variables by kind ("P", "BP", "EX"), then number, each its
    #: data type, form and axes as the protocol carries them; unset ones read 0.
    positions: dict[str, dict[int, tuple[int, ...]]] = field(default_factory=dict)
    #: S variables by number, as their bytes; an unset one is all NULs.
    strings: dict[int, bytes] = field(default_factory=dict)
    #: The jobs the controller holds, the one it executes and at which line,
    #: and each task's master job.
    jobs: frozenset[str] = frozenset()
    executing_job: str = ""
    executing_line: int = 0
    master_jobs: dict[int, str] = field(default_factory=dict)
    #: The message a host last put on the pendant's remote display.
    pendant_message: str = ""
    #: When the controller's power came on: when the device was made.
    powered_on: datetime = field(default_factory=datetime.now)

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

    def alarms(self) -> list[Alarm]:
        """The alarms showing, newest first.

        The arm does not tell the controller when it faults, so a fault becomes
        an alarm, and enters the history, when the controller first looks; a
        fault cleared before anyone looked leaves no trace.
        """
        faulted = self.arm.state.view.faulted
        if faulted and self.active_alarm is None:
            self.active_alarm = Alarm(
                code=FAULT_ALARM, data=0, time=datetime.now().strftime(ALARM_TIME_FORMAT)
            )
            self.alarm_history.insert(0, self.active_alarm)
            del self.alarm_history[ALARM_HISTORY:]
        if not faulted:
            self.active_alarm = None
        return [] if self.active_alarm is None else [self.active_alarm]

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
        self.active_alarm = None

    def cancel_error(self) -> None:
        """Cancels an error. The emulator raises none, so there is nothing to cancel."""

    def select_job(self, name: str, line: int = 0, *, master_of_task: int | None = None) -> None:
        """Set the executing job (and its line), or a task's master job."""
        if name not in self.jobs:
            raise CommandRefusedError(Refusal.NO_SUCH_JOB, f"no job {name!r}")
        if master_of_task is not None:
            self.master_jobs[master_of_task] = name
            return
        self.executing_job, self.executing_line = name, line

    def start_job(self) -> None:
        """Start the executing job. The emulator cannot run INFORM, so it finishes at once."""
        if not self.executing_job:
            raise CommandRefusedError(Refusal.NO_EXECUTING_JOB, "no job is set to execute")
        self._check_can_move()

    def check_tool(self, tool: int) -> int:
        if not 0 <= tool <= self.profile.max_tool:
            raise CommandRefusedError(Refusal.BAD_TOOL, f"tool {tool}")
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


def stepped(pose: Pose, step: NDArray[np.float64], *, in_tool: bool) -> Pose:
    """*pose* moved by *step*: in the tool's own frame, or along and about the base axes."""
    here = pose_to_mat(pose)
    if in_tool:
        return _mat_to_pose(here @ step)
    there = np.eye(4)
    there[:3, :3] = step[:3, :3] @ here[:3, :3]
    there[:3, 3] = here[:3, 3] + step[:3, 3]
    return _mat_to_pose(there)
