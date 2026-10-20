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

from ....kinematics.api import Pose
from ....kinematics.dh_backend import pose_to_mat
from ....kinematics.units import Meters, Radians
from ....transport.line_server import Hangup, Reply
from .controller import (
    CYCLES,
    MODES,
    CommandRefusedError,
    Refusal,
    SpeedSelection,
    YaskawaController,
    stepped,
)

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


#: MDSP shows up to 30 bytes on the pendant's remote display.
PENDANT_MESSAGE_BYTES = 30
#: Joint speed in percent of the maximum, and linear speed in mm/s or degrees/s.
MIN_JOINT_SPEED, MAX_JOINT_SPEED_PERCENT, MIN_LINEAR_SPEED = 0.01, 100.0, 0.1
#: Signal numbers count eight to a byte: #22010 to #22017 are byte 2201's bits.
SIGNALS_PER_BYTE = 8


#: The code in an Ethernet server ``ERROR:<Command> is not successful (<code>).``
#: answer: the controller's interpreter message. The YRC1000 manual names some
#: (11.1.1: 2010 manipulator operating, 2020-2050 hold, 2060 alarm or error, 2080
#: incorrect mode, 2100 no command remote); the rest are the same numbers HSES
#: answers as added status (YMConnect's table: 2070 servo off, 3350 user
#: coordinate not taught, 4040 no such job, ...). A move with no straight path has
#: no code: a real controller takes it and stops with an alarm, so 9006 stands in.
INTERPRETER_MESSAGES: dict[Refusal, int] = {
    Refusal.BAD_DATA: 5200,  # data over range
    Refusal.UNDEFINED_USER_FRAME: 3350,
    Refusal.EMERGENCY_STOP: 3450,  # servo power cannot be turned on
    Refusal.SERVO_OFF: 2070,
    Refusal.HELD: 2050,  # command hold
    Refusal.NO_LINEAR_PATH: 9006,
    Refusal.NOT_WRITABLE: 2110,  # this data cannot be accessed
    Refusal.BAD_TOOL: 5200,
    Refusal.NO_SUCH_JOB: 4040,
    Refusal.NO_EXECUTING_JOB: 4060,
    Refusal.WRONG_MODE: 2080,
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
            "RJSEQ": self._rjseq,
            "SVON": self._svon,
            "HOLD": self._hold,
            "RESET": self._reset,
            "CANCEL": self._cancel,
            "HLOCK": self._hlock,
            "MODE": self._mode,
            "MDSP": self._mdsp,
            "CYCLE": self._cycle,
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
        showing = self.controller.alarms()[:ALARM_SLOTS]
        alarms = [v for alarm in showing for v in (alarm.code, alarm.data)]
        return ",".join(str(v) for v in [0, 0, *_pad(alarms, ALARM_SLOTS * 2)])

    def _rjseq(self, data: str) -> str:
        """The executing job's name, line and step. The emulator keeps no steps, so
        the step reads 0, as in HSES 0x73."""
        return f"{self.controller.executing_job},{self.controller.executing_line},0"

    def _svon(self, data: str) -> None:
        self.controller.set_servo(_parse_switch(data))

    def _hold(self, data: str) -> None:
        self.controller.set_hold(_parse_switch(data))

    def _reset(self, data: str) -> None:
        self.controller.reset_alarm()

    def _cancel(self, data: str) -> None:
        self.controller.cancel_error()

    def _hlock(self, data: str) -> None:
        """The interlock between the pendant and IO operation signals, 1 on, 0 off."""
        self.controller.hlock = _parse_switch(data)

    def _mdsp(self, data: str) -> None:
        """A message for the pendant's remote display."""
        if len(data.encode()) > PENDANT_MESSAGE_BYTES:
            raise CommandRefusedError(Refusal.BAD_DATA, f"{len(data.encode())}-byte message")
        self.controller.pendant_message = data

    def _mode(self, data: str) -> None:
        """1 teach, 2 play. A real controller takes MODE only when its external mode
        switch is enabled; the emulator always does."""
        (mode,) = _parse_ints(data, count=1)
        if mode not in MODES:
            raise CommandRefusedError(Refusal.BAD_DATA, f"mode {mode}")
        self.controller.mode = MODES[mode]

    def _cycle(self, data: str) -> None:
        """1 step, 2 one cycle, 3 auto."""
        (cycle,) = _parse_ints(data, count=1)
        if cycle not in CYCLES:
            raise CommandRefusedError(Refusal.BAD_DATA, f"cycle {cycle}")
        self.controller.cycle = CYCLES[cycle]

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
        self.controller.move_linear(stepped(here, step, in_tool=in_tool), selection, speed)

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
        # The size counts the data and its closing CR, or CR LF: the EPICS
        # StreamDevice support for the NX100 ends command data with CR LF and
        # counts both. The LF arrives at the start of the next message, where
        # the framer drops it.
        data_size = len(data.encode())
        if request.size > 0 and request.size - data_size not in (1, 2):
            return Hangup([f"NG: size {request.size} for {data_size + 1} bytes\r\n"])
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


