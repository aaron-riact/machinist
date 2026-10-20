"""The Yaskawa Ethernet server (NX100, DX200, YRC1000).

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
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from ....kinematics.api import Pose
from ....kinematics.dh_backend import _mat_to_pose, pose_to_mat
from ....kinematics.units import Meters, Radians
from ....transport.line_server import Hangup, Reply
from .controller import CommandRefusedError, Refusal, SpeedSelection, YaskawaController

#: The reply to a START request the controller cannot accept, before it closes.
START_ERROR = "NG: HTTP Error Response"
NORMAL_ANSWER = "0000"
MAX_KEEP_ALIVE = 32767
#: Position answers carry this many axes: the robot's six and six external ones.
AXES_ON_THE_WIRE = 12
EXTERNAL_AXES = 6
#: Coordinate numbers: base, robot, then user frames from 2 on.
BASE_COORDINATE, ROBOT_COORDINATE = 0, 1
#: RALARM answers an error and four alarms, each a code and its data.
ALARM_SLOTS = 4


#: Joint speed in percent of the maximum, and linear speed in mm/s or degrees/s.
MIN_JOINT_SPEED, MAX_JOINT_SPEED_PERCENT, MIN_LINEAR_SPEED = 0.01, 100.0, 0.1
#: Signal numbers count eight to a byte: #22010 to #22017 are byte 2201's bits.
SIGNALS_PER_BYTE = 8


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


Command = Callable[[str], str | None]


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


