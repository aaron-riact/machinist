"""Yaskawa Motoman NX100 Ethernet-server emulator.

Reference: *NX100 Options Instructions for Ethernet Server Function*
(HW0483202), in the repository as ``NX100_http_network_command.pdf``.

The NX100 serves on TCP port 80 but the protocol is **not** HTTP. A session
runs::

    CONNECT Robot_access[ Keep-Alive:<n>]<CR><LF>
    ← OK: NX Information Server (3.00)[ Keep-Alive:<n>].<CR><LF>
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
from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum, IntFlag

from pydantic import model_validator

from ...core.device import Device
from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import CR
from ...transport.line_server import Hangup, Reply, SessionHandler
from .arm import ArmOptions, HasArm, RobotArm, arm_from_options

MOTOMAN_PORT = 80
SERVER_NAME = "OK: NX Information Server (3.00)"
#: The reply to a START request the controller cannot accept, before it closes.
START_ERROR = "NG: HTTP Error Response"
NORMAL_ANSWER = "0000"
MAX_KEEP_ALIVE = 32767
#: Position answers carry this many axes: the robot's six and six external ones.
AXES_ON_THE_WIRE = 12
EXTERNAL_AXES = 6
#: Coordinate numbers: base, robot, then user frames 1 to 24 as 2 to 25.
BASE_COORDINATE, ROBOT_COORDINATE, LAST_USER_COORDINATE = 0, 1, 25
#: The alarm RALARM reports while the arm is faulted. A real controller names
#: the cause; the arm only knows that it faulted, so this is a stand-in.
FAULT_ALARM = 4000
#: RALARM answers an error and four alarms, each a code and its data.
ALARM_SLOTS = 4

#: Encoder pulses per degree of each YR-UP50N axis (S, L, U, R, B, T): 4096
#: pulses per motor turn through each gear ratio, except L, whose ratio did
#: not match the arm and is measured from its pulse limits. The riact Motoman
#: driver converts with the same values.
UP50N_PULSES_PER_DEGREE = (1416.552, 1945.413, 1994.362, 895.052, 814.519, 418.627)


class MotomanOptions(ArmOptions):
    """The ``motoman_nx100`` block: an arm, and how its joints count pulses."""

    #: Encoder pulses per degree of each joint, in joint order.
    pulses_per_degree: tuple[float, ...] = UP50N_PULSES_PER_DEGREE

    @model_validator(mode="after")
    def _one_ratio_per_joint(self) -> MotomanOptions:
        if len(self.pulses_per_degree) != self.joint_count:
            raise ValueError(
                f"{len(self.pulses_per_degree)} pulses_per_degree for {self.joint_count} joints"
            )
        return self


class InterpreterMessage(IntEnum):
    """The code in an ``ERROR:<Command> is not successful (<code>).`` answer.

    The manual leaves the list to the Data Transmission manual (RE-CKI-A445),
    which is not at hand, so these codes are stand-ins that only name why a
    command failed.
    """

    BAD_DATA = 9001
    UNDEFINED_USER_FRAME = 9002
    EMERGENCY_STOP = 9003


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


class CommandError(Exception):
    """A command the controller refuses with an ERROR answer."""

    def __init__(self, code: InterpreterMessage, reason: str) -> None:
        super().__init__(reason)
        self.code = code


Command = Callable[[str], str | None]


@dataclass(slots=True)
class NX100Controller:
    """The controller state every session shares, and the commands that act on it."""

    arm: RobotArm
    pulses_per_degree: tuple[float, ...]
    cycle: Status1 = Status1.ONE_CYCLE
    #: The form (posture) type and tool number of the last move, which RPOSC
    #: reports. The arm has no tool files, so the tool only travels along.
    form: int = 0
    tool: int = 0
    #: HOLD from a command: the arm stops and takes no move until HOLD 0.
    hold: bool = False

    def command(self, name: str) -> Command | None:
        """The command called *name*, or None for one the controller does not know.

        A command takes the command data and returns the answer's data, or
        None for an answer without data. It raises :class:`CommandError` when
        the controller refuses it.
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
        }
        return commands.get(name)

    def _rstats(self, data: str) -> str:
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
        return f"{int(data1)},{int(data2)}"


    def _rposj(self, data: str) -> str:
        pulses = [
            round(math.degrees(joint) * ratio)
            for joint, ratio in zip(self.arm.state.view.joints, self.pulses_per_degree, strict=True)
        ]
        return ",".join(str(p) for p in _pad(pulses, AXES_ON_THE_WIRE))


    def _rposc(self, data: str) -> str:
        coordinate, external = _parse_ints(data, count=2)
        _check_coordinate(coordinate)
        if external not in (0, 1):
            raise CommandError(InterpreterMessage.BAD_DATA, f"external axis flag {external}")
        x, y, z, rx, ry, rz = self.arm.state.view.pose
        fields = [
            *(f"{m * 1000:.3f}" for m in (x, y, z)),
            *(f"{math.degrees(r):.2f}" for r in (rx, ry, rz)),
            str(self.form),
            str(self.tool),
        ]
        if external:
            fields += ["0"] * EXTERNAL_AXES
        return ",".join(fields)


    def _ralarm(self, data: str) -> str:
        alarms = [FAULT_ALARM, 0] if self.arm.state.view.faulted else []
        return ",".join(str(v) for v in [0, 0, *_pad(alarms, ALARM_SLOTS * 2)])

    def _svon(self, data: str) -> None:
        if _parse_switch(data):
            if self.arm.state.view.estopped:
                raise CommandError(InterpreterMessage.EMERGENCY_STOP, "emergency stop is engaged")
            self.arm.set_servo(True)
        else:
            self.arm.stop()
            self.arm.set_servo(False)

    def _hold(self, data: str) -> None:
        self.hold = _parse_switch(data)
        if self.hold:
            self.arm.stop()

    def _reset(self, data: str) -> None:
        self.arm.clear_fault()

    def _cancel(self, data: str) -> None:
        """Cancels an error. The emulator raises none, so there is nothing to cancel."""


def _parse_switch(data: str) -> bool:
    """``1`` for on, ``0`` for off."""
    (value,) = _parse_ints(data, count=1)
    if value not in (0, 1):
        raise CommandError(InterpreterMessage.BAD_DATA, f"switch {value}")
    return value == 1


def _check_coordinate(coordinate: int) -> None:
    """Base and robot coordinates are one frame here (no base axis); user frames are unset."""
    if coordinate in (BASE_COORDINATE, ROBOT_COORDINATE):
        return
    if ROBOT_COORDINATE < coordinate <= LAST_USER_COORDINATE:
        raise CommandError(
            InterpreterMessage.UNDEFINED_USER_FRAME, f"user frame {coordinate - 1} is not defined"
        )
    raise CommandError(InterpreterMessage.BAD_DATA, f"coordinate {coordinate}")


def _parse_ints(data: str, *, count: int) -> list[int]:
    """Exactly *count* comma-separated integers."""
    try:
        values = [int(part) for part in data.split(",")]
    except ValueError as exc:
        raise CommandError(InterpreterMessage.BAD_DATA, f"not integers: {data!r}") from exc
    if len(values) != count:
        raise CommandError(InterpreterMessage.BAD_DATA, f"{len(values)} fields, expected {count}")
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

    controller: NX100Controller
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
        if len(words) == 2:
            self.keep_alive = 1
            return SERVER_NAME + ".\r\n"
        keep_alive = _parse_keep_alive(words[2])
        if keep_alive is None:
            return Hangup([START_ERROR + "\r\n"])
        self.keep_alive = keep_alive
        return f"{SERVER_NAME} Keep-Alive:{keep_alive}.\r\n"

    # --- command ------------------------------------------------------

    def _request(self, message: str) -> Reply:
        words = message.split()
        if len(words) != 3 or words[0] != "HOSTCTRL_REQUEST" or not words[2].isdigit():
            return Hangup([f"NG: bad request {message!r}\r\n"])
        name, size = words[1], int(words[2])
        run = self.controller.command(name)
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
        except CommandError as exc:
            return Hangup([f"ERROR:{request.name} is not successful ({exc.code:04d}).\r\n"])
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


class MotomanNX100(LineServerDevice, HasArm):
    kind = "motoman_nx100"
    DEFAULT_PORT = MOTOMAN_PORT
    FRAMER = CR

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: MotomanOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options, name=name, publish=self.publish)
        self.add_service(self.arm)
        self.controller = NX100Controller(
            arm=self.arm, pulses_per_degree=options.pulses_per_degree
        )

    def make_session(self) -> SessionHandler:
        return _TracingSession(self, _Session(controller=self.controller))


@dataclass(slots=True)
class _TracingSession:
    """Thin wrapper that surfaces rx/tx events on the bus."""

    device: LineServerDevice
    inner: SessionHandler

    def handle(self, message: str) -> Reply:
        self.device.emit("rx", line=message)
        reply = self.inner.handle(message)
        if reply is not None:
            self.device.emit("tx", reply=reply if isinstance(reply, str) else list(reply))
        return reply


@register("motoman_nx100", default_port=MOTOMAN_PORT, options=MotomanOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: MotomanOptions) -> Device:
    return MotomanNX100(name, endpoint, bus, options)
