"""The Yaskawa Ethernet server (NX100, DX200, YRC1000).

References: *NX100 Options Instructions for Ethernet Server Function*
(HW0483202), in the repository as ``NX100_http_network_command.pdf``, the
DX200's (HW1481823) and the YRC1000's Ethernet function manual (HW1483358,
chapter 8). All three describe the same 35 commands, every one of which the
emulator answers but JWAIT, which they list only to say the Ethernet server
does not support it. What differs between controllers is in
:data:`CONTROLLERS`. Which command works in which state (teach or play mode,
moving, alarm) follows the YRC1000 manual's interlock table (11.1.1).

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
import struct
from collections.abc import Callable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from ....kinematics.api import Pose
from ....kinematics.dh_backend import pose_to_mat
from ....kinematics.units import Meters, Radians
from ....transport.line_server import Hangup, Reply
from .controller import (
    CYCLES,
    MODES,
    POSITION_NUMBERS,
    STANDARD_VARIABLES,
    STRING_SIZE,
    BufferedMove,
    CommandRefusedError,
    Refusal,
    SpeedSelection,
    UserFrame,
    YaskawaController,
    frame_step,
    from_frame,
    to_frame,
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


#: SAVEV and LOADV variable types 0 to 3: B, I, D and R, by the kind the
#: controller keeps them under; 7 is S. The manual gives no format for a real
#: value; the emulator answers it to float precision, 7 significant digits.
NUMBER_VARIABLES = {0: "B", 1: "I", 2: "D", 3: "R"}
#: The integers B, I and D variables hold.
INTEGER_RANGES = {"B": range(256), "I": range(-(2**15), 2**15), "D": range(-(2**31), 2**31)}
REAL_VARIABLE = "R"
STRING_VARIABLE = 7
#: Variable types 4 to 6: P, BP and EX, kept as HSES carries them (3.3.16-3.3.18):
#: P as data type, form, tool, user coordinate, extended form and eight axes;
#: BP and EX as data type and eight axes. A data type is 0 for pulses, or 16
#: base, 17 robot, 18 user or 19 tool coordinates, Cartesian axes in µm and
#: 0.0001 degrees. The Ethernet server shows six axes, and P alone has a tool.
POSITION_VARIABLES = {4: "P", 5: "BP", 6: "EX"}
ROBOT_POSITION = "P"
POSITION_FIELDS = {"P": 13, "BP": 9, "EX": 9}
PULSE_DATA, BASE_DATA, ROBOT_DATA, USER_DATA, TOOL_DATA = 0, 16, 17, 18, 19
#: RJDIR starts a new line after every 20 job names. The manual does not say
#: which line feed; the emulator uses CR, the end of every answer line.
JOB_NAMES_PER_LINE = 20
ALL_JOBS = "*"
#: The task SETMJ sets the master job of: CTASK selects none other here.
MASTER_TASK = 0
#: The control group commands act on, as RGROUP and CGROUP give it: a bit per
#: robot (R1 is bit 0) and per station. The emulated cell has R1 alone.
R1_GROUP, NO_STATIONS = 0b1, 0
#: The last line JSEQ can set.
MAX_JOB_LINE = 9999
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
    Refusal.MOVING: 2010,  # manipulator operating
    Refusal.BAD_GROUP: 3370,  # incorrect control group
    Refusal.NOT_PERMITTED: 4490,  # relative job / independent control not permitted
    Refusal.FRAME_NOT_SET: 3460,  # coordinate system cannot be set
    Refusal.ALARM: 2060,  # alarm or error occurring
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
            "RJDIR": self._rjdir,
            "RGROUP": self._rgroup,
            "RUFRAME": self._ruframe,
            "SAVEV": self._savev,
            "LOADV": self._loadv,
            "SVON": self._svon,
            "HOLD": self._hold,
            "RESET": self._reset,
            "CANCEL": self._cancel,
            "HLOCK": self._hlock,
            "CGROUP": self._cgroup,
            "CTASK": self._ctask,
            "MODE": self._mode,
            "MDSP": self._mdsp,
            "CYCLE": self._cycle,
            "PMOVJ": self._pmovj,
            "PMOVL": self._pmovl,
            "MOVJ": self._movj,
            "MOVL": self._movl,
            "IMOV": self._imov,
            "START": self._start,
            "DELETE": self._delete,
            "CVTRJ": self._convert_job,
            "CVTSJ": self._convert_job,
            "WUFRAME": self._wuframe,
            "SETMJ": self._setmj,
            "JSEQ": self._jseq,
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
        frame = self._frame(coordinate)
        if external not in (0, 1):
            raise CommandRefusedError(Refusal.BAD_DATA, f"external axis flag {external}")
        fields = [
            *self._pose_fields(to_frame(frame, self.controller.arm.state.view.pose)),
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

    def _rjdir(self, data: str) -> str:
        """Every job's name (``*``), or the names of the jobs a parent job calls."""
        parent = data.strip()
        names = sorted(self.controller.jobs) if parent == ALL_JOBS else (
            self.controller.related_jobs(parent)
        )
        per_line = JOB_NAMES_PER_LINE
        lines = [names[i : i + per_line] for i in range(0, len(names), per_line)]
        return "\r".join(",".join(line) for line in lines)

    def _rgroup(self, data: str) -> str:
        """The robots and stations commands act on, and the task: always R1 and the
        master task, as the cell has nothing else."""
        return f"{R1_GROUP},{NO_STATIONS},{MASTER_TASK}"

    def _ruframe(self, data: str) -> str:
        """A taught user frame, by coordinate No.: ORG, XX and XY in base coordinates
        (X Y Z in mm, Rx Ry Rz in degrees, the type), the tool and six external axes."""
        (coordinate,) = _parse_ints(data, count=1)
        frame = self.controller.taught_user_frame(coordinate - 1)
        points = (frame.org, frame.xx, frame.xy)
        fields = [
            field for point, form in zip(points, frame.types, strict=True)
            for field in (*self._pose_fields(point), str(form))
        ]
        return ",".join([*fields, str(frame.tool), *(str(p) for p in frame.external)])

    def _pose_fields(self, pose: Pose) -> list[str]:
        """X Y Z in mm (3 decimals), Rx Ry Rz in degrees (the controller's decimals)."""
        decimals = self.controller.profile.rotation_decimals
        x, y, z, rx, ry, rz = pose
        return [
            *(f"{m * 1000:.3f}" for m in (x, y, z)),
            *(f"{math.degrees(r):.{decimals}f}" for r in (rx, ry, rz)),
        ]

    def _savev(self, data: str) -> str:
        """A variable's value, by its type and number."""
        variable_type, number = _parse_ints(data, count=2)
        if variable_type == STRING_VARIABLE:
            return self.controller.string(_variable_number(number)).decode("ascii", "replace")
        if variable_type in POSITION_VARIABLES:
            return ",".join(self._position_fields(POSITION_VARIABLES[variable_type], number))
        if variable_type not in NUMBER_VARIABLES:
            raise CommandRefusedError(Refusal.BAD_DATA, f"variable type {variable_type}")
        kind = NUMBER_VARIABLES[variable_type]
        value = self.controller.variables.get(kind, {}).get(_variable_number(number), 0)
        return f"{value:.7g}" if kind == REAL_VARIABLE else str(int(value))

    def _loadv(self, data: str) -> None:
        """A variable's type and number, then its value."""
        self.controller.check_no_alarm()
        variable_type, number, value = _parse_variable_write(data)
        if variable_type == STRING_VARIABLE:
            text = value.encode("ascii", errors="replace")
            if len(text) > STRING_SIZE:
                raise CommandRefusedError(Refusal.BAD_DATA, f"{len(text)}-byte string")
            self.controller.set_string(_variable_number(number), text)
            return
        if variable_type in POSITION_VARIABLES:
            kind = POSITION_VARIABLES[variable_type]
            if number not in POSITION_NUMBERS:
                raise CommandRefusedError(Refusal.BAD_DATA, f"position variable {number}")
            record = self._position_record(kind, _parse_numbers(value, count=None))
            self.controller.positions.setdefault(kind, {})[number] = record
            return
        if variable_type not in NUMBER_VARIABLES:
            raise CommandRefusedError(Refusal.BAD_DATA, f"variable type {variable_type}")
        kind = NUMBER_VARIABLES[variable_type]
        number = _variable_number(number)
        self.controller.variables.setdefault(kind, {})[number] = _variable_value(kind, value)

    def _position_record(self, kind: str, values: list[float]) -> tuple[int, ...]:
        """A position variable from LOADV's fields, as the controller keeps it:
        0 and six pulses, or 1, the coordinate, X Y Z (mm) Rx Ry Rz (degrees) and the
        type; then the tool. EX takes pulses only, BP Cartesian in base coordinates."""
        if not values or values[0] not in (0, 1):
            raise CommandRefusedError(Refusal.BAD_DATA, "position data type")
        cartesian = values[0] == 1
        if len(values) != (10 if cartesian else 8):
            raise CommandRefusedError(Refusal.BAD_DATA, f"{len(values)} position fields")
        tool = self.controller.check_tool(_integer(values[-1]))
        if not cartesian:
            pulses = [_integer(v) for v in values[1:7]]
            fields = [0, 0, tool, 0, 0, *pulses] if kind == ROBOT_POSITION else [0, *pulses]
        else:
            data_type, user = self._position_data_type(kind, _integer(values[1]))
            x, y, z, rx, ry, rz = values[2:8]
            axes = [round(v * 1000) for v in (x, y, z)] + [round(v * 10000) for v in (rx, ry, rz)]
            fields = [data_type, *axes]
            if kind == ROBOT_POSITION:
                fields = [data_type, _integer(values[8]), tool, user, 0, *axes]
        return tuple(_pad(fields, POSITION_FIELDS[kind]))

    def _position_data_type(self, kind: str, coordinate: int) -> tuple[int, int]:
        """The data type and user frame a coordinate keeps a Cartesian position in.
        The master tool coordinate has no HSES data type, so it is not taken."""
        if kind == ROBOT_POSITION:
            if coordinate == BASE_COORDINATE:
                return BASE_DATA, 0
            if coordinate == ROBOT_COORDINATE:
                return ROBOT_DATA, 0
            if 1 <= coordinate - 1 <= self.controller.profile.user_frames:
                return USER_DATA, coordinate - 1
            if coordinate == self.controller.profile.tool_coordinate:
                return TOOL_DATA, 0
        elif kind != "EX" and coordinate == BASE_COORDINATE:
            return BASE_DATA, 0
        raise CommandRefusedError(Refusal.BAD_DATA, f"{kind} coordinate {coordinate}")

    def _position_fields(self, kind: str, number: int) -> list[str]:
        """A position variable as SAVEV answers it: 0 and six pulses, or 1, the
        coordinate, X Y Z (mm) Rx Ry Rz (degrees) and the type; then the tool."""
        if number not in POSITION_NUMBERS:
            raise CommandRefusedError(Refusal.BAD_DATA, f"position variable {number}")
        record = self.controller.positions.get(kind, {}).get(
            number, (0,) * POSITION_FIELDS[kind]
        )
        if kind == ROBOT_POSITION:
            data_type, form, tool, user, _, *axes = record
        else:
            (data_type, *axes), form, tool, user = record, 0, 0, 0
        if data_type == PULSE_DATA:
            return ["0", *(str(a) for a in axes[:6]), str(tool)]
        coordinate = {
            BASE_DATA: BASE_COORDINATE, ROBOT_DATA: ROBOT_COORDINATE, USER_DATA: user + 1,
            TOOL_DATA: self.controller.profile.tool_coordinate,
        }
        decimals = self.controller.profile.rotation_decimals
        return [
            "1", str(coordinate[data_type]),
            *(f"{a / 1000:.3f}" for a in axes[:3]),
            *(f"{a / 10000:.{decimals}f}" for a in axes[3:6]),
            str(form), str(tool),
        ]

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

    def _cgroup(self, data: str) -> None:
        """The robot and station for commands to act on, a bit for each. A group the
        cell lacks cannot be chosen, so R1 and no station is the only choice."""
        robots, stations = _parse_ints(data, count=2)
        if (robots, stations) != (R1_GROUP, NO_STATIONS):
            raise CommandRefusedError(Refusal.BAD_GROUP, f"robots {robots}, stations {stations}")

    def _ctask(self, data: str) -> None:
        """The task for commands to act on. CTASK is an option that needs independent
        control, which the emulated controller lacks, so it is always refused."""
        raise CommandRefusedError(Refusal.NOT_PERMITTED, "no independent control")

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
        self._answer_when_started(self.controller.move_joints(target, speed))

    def _pmovl(self, data: str) -> None:
        """Speed selection, speed, six pulses, tool, six external axes."""
        values = _parse_numbers(data, count=15)
        selection, speed = _linear_speed(values[0], values[1])
        target = self.controller.arm.forward(self.controller.joints_at(values[2:8]))
        self.controller.tool = self.controller.check_tool(_integer(values[8]))
        self._answer_when_started(self.controller.move_linear(target, selection, speed))

    def _movj(self, data: str) -> None:
        """Speed (%), coordinate, X Y Z Rx Ry Rz, type, tool, six external axes."""
        values = _parse_numbers(data, count=16)
        speed = _joint_speed(values[0])
        target = from_frame(self._frame(_integer(values[1])), _pose(values[2:8]))
        self.controller.form = _integer(values[8])
        self.controller.tool = self.controller.check_tool(_integer(values[9]))
        self._answer_when_started(self.controller.move_joints_to_pose(target, speed))

    def _movl(self, data: str) -> None:
        """Speed selection, speed, coordinate, X Y Z Rx Ry Rz, type, tool, six external axes."""
        values = _parse_numbers(data, count=17)
        selection, speed = _linear_speed(values[0], values[1])
        target = from_frame(self._frame(_integer(values[2])), _pose(values[3:9]))
        self.controller.form = _integer(values[9])
        self.controller.tool = self.controller.check_tool(_integer(values[10]))
        self._answer_when_started(self.controller.move_linear(target, selection, speed))

    def _imov(self, data: str) -> None:
        """Speed selection, speed, coordinate, the step in X Y Z Rx Ry Rz, reserved, tool,
        six external axes.

        In the base or robot coordinate the step moves the tool along and turns it
        about the base axes; in a user frame, about that frame's axes; in the tool
        coordinate, along and about its own.
        """
        values = _parse_numbers(data, count=17)
        selection, speed = _linear_speed(values[0], values[1])
        coordinate = _integer(values[2])
        in_tool = coordinate == self.controller.profile.tool_coordinate
        step = pose_to_mat(_pose(values[3:9]))
        if not in_tool:
            step = frame_step(self._frame(coordinate), step)
        self.controller.tool = self.controller.check_tool(_integer(values[10]))
        self._answer_when_started(self.controller.step_linear(
            step, in_tool=in_tool, selection=selection, speed=speed
        ))

    def _answer_when_started(self, move: BufferedMove | None) -> None:
        """A move that waits in the buffer is answered when it starts, on a
        controller that holds the answer (the YRC1000 manual, 11.1.3)."""
        if move is not None and self.controller.profile.holds_buffered_answer:
            move.wait_started()

    def _start(self, data: str) -> None:
        """Start the executing job from its line, or the job named from its start."""
        self.controller.start_job(data.strip() or None)

    def _delete(self, data: str) -> None:
        """Delete the job named, or every job (``*``)."""
        name = data.strip()
        self.controller.delete_jobs(None if name == ALL_JOBS else name)

    def _convert_job(self, data: str) -> None:
        """CVTRJ (to a relative job) and CVTSJ (to a standard job) need the relative
        job function, an option the emulated controller lacks, so both are refused."""
        raise CommandRefusedError(Refusal.NOT_PERMITTED, "no relative job function")

    def _wuframe(self, data: str) -> None:
        """A user frame's coordinate No., then ORG, XX and XY in base coordinates (X Y Z
        in mm, Rx Ry Rz in degrees, the type), the tool and six external axes."""
        values = _parse_numbers(data, count=29)
        coordinate = _integer(values[0])
        points = [values[1 + 7 * i : 8 + 7 * i] for i in range(3)]
        org, xx, xy = (_pose(p[:6]) for p in points)
        frame = UserFrame(
            org=org, xx=xx, xy=xy, types=(_integer(points[0][6]), _integer(points[1][6]),
                                          _integer(points[2][6])),
            tool=_integer(values[22]), external=tuple(_integer(v) for v in values[23:29]),
        )
        self.controller.teach_user_frame(coordinate - 1, frame)

    def _setmj(self, data: str) -> None:
        """A job to set as the master job, and at the same time as the executing job."""
        name = data.strip()
        self.controller.select_job(name, master_of_task=MASTER_TASK)
        self.controller.select_job(name)

    def _jseq(self, data: str) -> None:
        """A job name and line No. to set as the executing job."""
        name, line = _parse_job_and_number(data)
        if not 0 <= line <= MAX_JOB_LINE:
            raise CommandRefusedError(Refusal.BAD_DATA, f"line {line}")
        self.controller.select_job(name, line)

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

    def _frame(self, coordinate: int) -> NDArray[np.float64]:
        """The coordinate's frame in base coordinates. Base and robot coordinates are
        one frame here (no base axis); user frames follow, refused until taught."""
        if coordinate in (BASE_COORDINATE, ROBOT_COORDINATE):
            return np.eye(4)
        return self.controller.user_frame_transform(coordinate - 1)


def _parse_variable_write(data: str) -> tuple[int, int, str]:
    """LOADV's type and number, then the rest: a value, which may hold commas."""
    parts = data.split(",", 2)
    if len(parts) != 3:
        raise CommandRefusedError(Refusal.BAD_DATA, f"{len(parts)} fields")
    variable_type, number = _parse_ints(",".join(parts[:2]), count=2)
    return variable_type, number, parts[2]


def _variable_value(kind: str, text: str) -> float:
    """A B, I, D or R variable's value; an R value is rounded to a 32-bit float."""
    value: float
    try:
        if kind == REAL_VARIABLE:
            value = float(struct.unpack("<f", struct.pack("<f", float(text)))[0])
        else:
            value = int(text)
    except (ValueError, OverflowError) as exc:
        raise CommandRefusedError(Refusal.BAD_DATA, f"{kind} value {text!r}") from exc
    if not math.isfinite(value) or (kind != REAL_VARIABLE and value not in INTEGER_RANGES[kind]):
        raise CommandRefusedError(Refusal.BAD_DATA, f"{kind} value {value}")
    return value


def _variable_number(number: int) -> int:
    if number not in STANDARD_VARIABLES:
        raise CommandRefusedError(Refusal.BAD_DATA, f"variable {number}")
    return number


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


def _parse_numbers(data: str, *, count: int | None) -> list[float]:
    """Comma-separated numbers: exactly *count* of them, or any number for None."""
    try:
        values = [float(part) for part in data.split(",")]
    except ValueError as exc:
        raise CommandRefusedError(Refusal.BAD_DATA, f"not numbers: {data!r}") from exc
    if count is not None and len(values) != count:
        raise CommandRefusedError(Refusal.BAD_DATA, f"{len(values)} fields, expected {count}")
    return values


def _parse_job_and_number(data: str) -> tuple[str, int]:
    """A job name, then an integer."""
    name, _, number = data.partition(",")
    (value,) = _parse_ints(number, count=1)
    return name.strip(), value


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


