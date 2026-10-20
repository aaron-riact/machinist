"""The Yaskawa High-Speed Ethernet Server (HSES), answered for a controller.

Reference: *DX200 Options Instructions for High-Speed Ethernet Server
Function* (HW1481977), chapter 3.3. Each robot control command is one
request datagram and one answer datagram; the packets themselves are in
:mod:`machinist.transport.yaskawa_hses`.

A command's data are 32-bit little-endian integers, one per element. A
read either takes one element (Get_Attribute_Single, its attribute the
element's number) or all of them (Get_Attribute_All, attribute 0).
"""

from __future__ import annotations

import math
import struct
from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime, timedelta
from enum import IntEnum

from ....kinematics.api import Pose
from ....kinematics.dh_backend import pose_to_mat
from ....kinematics.units import Meters, Radians
from ....transport.yaskawa_hses import (
    ANSWER_SERVICE,
    AddedStatus,
    Answer,
    Division,
    FrameError,
    Request,
    Status,
    encode_answer,
    parse_request,
    request_id_of,
)
from .controller import (
    ALARM_TIME_FORMAT,
    CYCLES,
    Alarm,
    CommandRefusedError,
    Refusal,
    SpeedSelection,
    YaskawaController,
    stepped,
)


class Service(IntEnum):
    """How a request reaches its data (the CIP services the manual names)."""

    GET_ALL = 0x01
    SET_ALL = 0x02
    GET_SINGLE = 0x0E
    SET_SINGLE = 0x10
    READ_PLURAL = 0x33
    WRITE_PLURAL = 0x34


#: The added status (chapter 5) that answers each refusal. NO_LINEAR_PATH
#: has no code of its own; a real controller stops such a move with an alarm.
ADDED_STATUS: dict[Refusal, AddedStatus] = {
    Refusal.BAD_DATA: AddedStatus.OPERAND_RANGE,
    Refusal.UNDEFINED_USER_FRAME: AddedStatus.USER_COORDINATE_NOT_TAUGHT,
    Refusal.EMERGENCY_STOP: AddedStatus.SERVO_CANNOT_TURN_ON,
    Refusal.SERVO_OFF: AddedStatus.SERVO_OFF,
    Refusal.HELD: AddedStatus.COMMAND_HOLD,
    Refusal.NO_LINEAR_PATH: AddedStatus.OUTSIDE_THE_DATA,
    Refusal.NOT_WRITABLE: AddedStatus.OUTSIDE_THE_DATA,
    Refusal.BAD_TOOL: AddedStatus.TOOL_NUMBER_ERROR,
    Refusal.NO_SUCH_JOB: AddedStatus.NO_SUCH_JOB,
    Refusal.NO_EXECUTING_JOB: AddedStatus.NO_EXECUTING_JOB,
    Refusal.WRONG_MODE: AddedStatus.INCORRECT_MODE,
    Refusal.MOVING: AddedStatus.MANIPULATOR_OPERATING,
    Refusal.BAD_GROUP: AddedStatus.CONTROL_GROUP_ERROR,
    Refusal.NOT_PERMITTED: AddedStatus.FUNCTION_NOT_PERMITTED,
}


class HsesError(Exception):
    """A request the server answers with *status* (and *added_status*, if any)."""

    def __init__(self, status: Status, added_status: AddedStatus | None = None) -> None:
        super().__init__(f"{status.name} {added_status.name if added_status else ''}")
        self.status = status
        self.added_status = added_status


def _added(added_status: AddedStatus) -> HsesError:
    return HsesError(Status.VENDOR, added_status)


#: A robot control command: the request in, the answer's data out.
Handler = Callable[[Request], bytes]


#: The control group instances for robot 1: its axes in pulses, or Cartesian.
#: The emulated cell has one robot and no base or station axes.
R1_PULSE, R1_CARTESIAN = 1, 101
#: Position answers carry eight axes; a six-axis arm's last two are 0.
AXIS_ELEMENTS = 8
PULSE_AXES = ("S", "L", "U", "R", "B", "T")
CARTESIAN_AXES = ("X", "Y", "Z", "Rx", "Ry", "Rz")
#: The Cartesian position read (0x33D) as YMConnect sends and reads it: the group
#: kind (0 robots, 1 bases, 2 stations), the frame (base, robot, or user frame n
#: as n + 1), and the coordinate type the answer names (2 base, 3 robot, 4 user).
ROBOT_GROUPS = 0
BASE_FRAME, ROBOT_FRAME = 0, 1
UAPI_BASE, UAPI_ROBOT = 2, 3
#: A position's data type (3.3.6): pulses, or base coordinates.
PULSE_DATA, BASE_COORDINATE_DATA = 0, 16
#: The IO instances (3.3.9): a signal's number / 10, by the kind of signal.
IO_INSTANCES = (
    range(1, 513),  # robot user inputs
    range(1001, 1513),  # robot user outputs
    range(2001, 2513),  # external inputs
    range(2701, 2957),  # network inputs, the only writable ones
    range(3001, 3513),  # external outputs
    range(3701, 3957),  # network outputs
    range(4001, 4161),  # robot system inputs
    range(5001, 5301),  # robot system outputs
    range(6001, 6065),  # interface panel inputs
    range(7001, 8000),  # auxiliary relays
    range(8001, 8129),  # robot control status signals
    range(8201, 8221),  # pseudo inputs
)
#: An IO write carries its byte alone or followed by zeros: the manual draws
#: the data as a 32-bit row; moto-hses, verified on a YRC1000micro, sends four
#: bytes, YMConnect, Yaskawa's own client, eight, and others one.
IO_WRITE_SIZES = (1, 4, 8)
#: A single register, variable or IO value is addressed with attribute 1, as
#: the manual says, or 0, as moto-hses sends for variables on a real
#: YRC1000micro.
SINGLE_VALUE_ATTRIBUTES = (0, 1)
#: The most IO bytes one plural command carries (3.3.27).
MAX_PLURAL_IO = 474
class MoveKind(IntEnum):
    """What a move instruction does, by its instance (3.3.39, 3.3.40)."""

    JOINT = 1
    STRAIGHT = 2
    STRAIGHT_INCREMENT = 3


#: A move's speed class (3.3.39): percent for a joint move, mm/s or degrees/s
#: for a straight one, each sent in hundredths or tenths.
PERCENT, V, VR = 0, 1, 2
#: A move's operation coordinate (3.3.39).
BASE_COORDINATE, ROBOT_COORDINATE, USER_COORDINATE, TOOL_COORDINATE = 16, 17, 18, 19
#: System information instances (3.3.26): robots, stations, applications.
R1_SYSTEM = 11
APPLICATIONS = range(101, 109)
SYSTEM_INSTANCES = (*range(11, 19), *range(21, 45), *APPLICATIONS)
#: The name application 1 reports; the emulator has no application of its own.
APPLICATION = "HANDLING"


@dataclass(frozen=True, slots=True)
class Numbered:
    """A kind of register or variable: how its numbers run and its values travel."""

    name: str
    #: The struct format of one value.
    value_format: str
    numbers: range
    #: The numbers a host may write; all of them when None.
    writable: range | None
    #: The most values one plural command carries, and the multiple it counts in.
    plural_max: int
    plural_step: int = 1

    def pack(self, value: float) -> bytes:
        return struct.pack("<" + self.value_format, value)

    def unpack(self, data: bytes, *, count: int) -> tuple[float, ...]:
        if len(data) != count * struct.calcsize(self.value_format):
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        return struct.unpack(f"<{count}{self.value_format}", data)


#: Registers (3.3.10, 3.3.28): 0 to 999, of which 0 to 559 are writable.
REGISTERS = Numbered("register", "H", range(1000), range(560), plural_max=237)
#: Variables B, I, D and R (3.3.11-3.3.14, 3.3.29-3.3.32), 0 to 99 as the
#: standard setting has them; extended variables are an option not emulated.
#: A plural B command counts in twos.
STANDARD_VARIABLES = range(100)
BYTE_VARIABLES = Numbered("B", "B", STANDARD_VARIABLES, None, plural_max=474, plural_step=2)
INTEGER_VARIABLES = Numbered("I", "h", STANDARD_VARIABLES, None, plural_max=237)
DOUBLE_VARIABLES = Numbered("D", "i", STANDARD_VARIABLES, None, plural_max=118)
REAL_VARIABLES = Numbered("R", "f", STANDARD_VARIABLES, None, plural_max=118)
@dataclass(frozen=True, slots=True)
class PositionKind:
    """A kind of position variable: its fields, the data types it takes, and how many
    one plural command carries."""

    name: str
    #: 32-bit fields per variable, its data type first.
    elements: int
    data_types: frozenset[int]
    plural_max: int
    #: Whether one field can be read or written on its own.
    elementwise: bool

    @property
    def empty(self) -> tuple[int, ...]:
        return (0,) * self.elements

    def parse(self, data: bytes, *, count: int) -> list[tuple[int, ...]]:
        if len(data) != 4 * self.elements * count:
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        values = struct.unpack(f"<{self.elements * count}i", data)
        return [
            self.check(values[i * self.elements : (i + 1) * self.elements])
            for i in range(count)
        ]

    def check(self, record: tuple[int, ...]) -> tuple[int, ...]:
        if record[0] not in self.data_types:
            raise _added(AddedStatus.OPERAND_RANGE)
        return record


#: Position variables 0 to 127, the standard setting.
POSITION_NUMBERS = range(128)
#: P variables (3.3.16, 3.3.34): data type (pulses, or base, robot, user or tool
#: coordinates), form, tool, user coordinate, extended form, eight axes.
ROBOT_POSITIONS = PositionKind(
    "P", elements=13, data_types=frozenset({0, 16, 17, 18, 19}), plural_max=9,
    elementwise=False,
)
#: BP variables (3.3.17, 3.3.35): data type (pulses or base coordinates) and
#: eight axes. A single field can be written on its own; the rest keep theirs.
BASE_POSITIONS = PositionKind(
    "BP", elements=9, data_types=frozenset({0, 16}), plural_max=13, elementwise=True,
)
#: EX variables (3.3.18, 3.3.36): data type (pulses only) and eight axes.
EXTERNAL_POSITIONS = PositionKind(
    "EX", elements=9, data_types=frozenset({0}), plural_max=13, elementwise=True,
)
#: S variables (3.3.15, 3.3.41): 0 to 99, each 32 bytes on a DX200, of which
#: the 16-byte commands carry the first 16. A plural command carries up to 29
#: of 16 bytes or 14 of 32.
STRING_SIZE = 32
PLURAL_STRINGS = {16: 29, 32: 14}
#: Alarm history instances (3.3.2): the nth newest major failure (1 to 100),
#: monitor alarm (1001-), system user alarm (2001-), user alarm (3001-) and
#: off-line alarm (4001-).
MAJOR_FAILURES = range(1, 101)
ALARM_HISTORY_INSTANCES = tuple(
    n for start in (1, 1001, 2001, 3001, 4001) for n in range(start, start + 100)
)
#: Tasks (3.3.4): the master task is instance 1, sub tasks 1 to 15 follow.
MASTER_TASK = 1
TASKS = range(1, 17)
#: The speed override a job runs at, in hundredths of a percent: YMConnect, Yaskawa's
#: own client, divides it by 100. The emulator has no override, so it is 100 %.
SPEED_OVERRIDE = 10_000
#: Job select instances (3.3.24): the executing job, or task 0 to 15's master job.
EXECUTING_JOB = 1
MASTER_JOB_INSTANCES = range(10, 26)
JOB_NAME_FIELD = 32
MAX_JOB_LINE = 9999
#: The pendant's remote display takes 30 characters in a 32-byte field (3.3.22).
PENDANT_CHARACTERS, PENDANT_FIELD = 30, 32
#: Management time instances (3.3.25): control power, then servo power, playback
#: and motion time (total, per robot, per station), then operation time per
#: application.
CONTROL_POWER_TIME = 1
MANAGEMENT_TIMES = (
    CONTROL_POWER_TIME,
    *(n for base in (0, 100, 200) for n in (base + 10, *range(base + 11, base + 19),
                                            *range(base + 21, base + 45))),
    *range(301, 309),
)
#: The data of an on/off switch (3.3.20).
ON, OFF = 1, 2
#: An alarm's data shown as a decimal number (3.3.1, alarm type 1).
ALARM_TYPE_DECIMAL = 1
#: The name a faulted arm's alarm shows; the arm does not know the cause.
FAULT_ALARM_NAME = "MACHINIST FAULT"


def int32s(*values: int) -> bytes:
    return struct.pack(f"<{len(values)}i", *values)


def text(size: int, value: str = "") -> bytes:
    """A character string field of *size* bytes, padded with NULs."""
    return value.encode("ascii")[:size].ljust(size, b"\0")


@dataclass(slots=True)
class HighSpeedEthernetServer:
    """HSES robot control commands, read and answered for a :class:`YaskawaController`."""

    controller: YaskawaController

    def handle(self, datagram: bytes) -> bytes:
        """One request datagram in, one answer datagram out."""
        try:
            request = parse_request(datagram)
        except FrameError as exc:
            return encode_answer(Answer(
                division=Division.ROBOT, request_id=request_id_of(datagram), service=0,
                status=Status.VENDOR, added_status=exc.added_status,
            ))
        answer = Answer(
            division=request.division, request_id=request.request_id,
            service=request.service + ANSWER_SERVICE,
        )
        try:
            data = self._run(request)
        except HsesError as exc:
            return encode_answer(Answer(
                division=answer.division, request_id=answer.request_id, service=answer.service,
                status=exc.status, added_status=exc.added_status,
            ))
        except CommandRefusedError as exc:
            return encode_answer(Answer(
                division=answer.division, request_id=answer.request_id, service=answer.service,
                status=Status.VENDOR, added_status=ADDED_STATUS[exc.refusal],
            ))
        return encode_answer(Answer(
            division=answer.division, request_id=answer.request_id, service=answer.service,
            data=data,
        ))

    def _run(self, request: Request) -> bytes:
        if request.ack:
            raise _added(AddedStatus.ACK_ERROR)
        if request.division is not Division.ROBOT:
            raise HsesError(Status.UNDEFINED_COMMAND)  # file control is on its own port
        commands = self._commands()
        if self.controller.profile.yrc1000_commands:
            commands |= self._yrc1000_commands()
        handler = commands.get(request.command)
        if handler is None:
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handler(request)

    def _commands(self) -> dict[int, Handler]:
        return {
            0x70: self._alarm,
            0x71: self._alarm_history,
            0x72: self._status,
            0x73: self._executing_job,
            0x74: self._axis_configuration,
            0x75: self._position,
            0x76: self._zero_per_axis,
            0x77: self._zero_per_axis,
            0x78: self._io,
            0x79: self._single(REGISTERS),
            0x7A: self._single(BYTE_VARIABLES),
            0x7B: self._single(INTEGER_VARIABLES),
            0x7C: self._single(DOUBLE_VARIABLES),
            0x7D: self._single(REAL_VARIABLES),
            0x7E: self._string_variable(16),
            0x7F: self._position_variable(ROBOT_POSITIONS),
            0x80: self._position_variable(BASE_POSITIONS),
            0x81: self._position_variable(EXTERNAL_POSITIONS),
            0x82: self._reset_or_cancel,
            0x83: self._hold_servo_hlock,
            0x84: self._cycle,
            0x85: self._pendant_message,
            0x86: self._start_job,
            0x87: self._select_job,
            0x88: self._management_time,
            0x89: self._system_information,
            0x8A: self._move_cartesian,
            0x8B: self._move_pulse,
            0x8C: self._string_variable(32),
            0x300: self._plural_io,
            0x301: self._plural(REGISTERS),
            0x302: self._plural(BYTE_VARIABLES),
            0x303: self._plural(INTEGER_VARIABLES),
            0x304: self._plural(DOUBLE_VARIABLES),
            0x305: self._plural(REAL_VARIABLES),
            0x306: self._plural_strings(16),
            0x307: self._plural_positions(ROBOT_POSITIONS),
            0x308: self._plural_positions(BASE_POSITIONS),
            0x309: self._plural_positions(EXTERNAL_POSITIONS),
            0x30A: self._alarm_with_sub_codes,
            0x30B: self._alarm_history_with_sub_codes,
            0x30C: self._plural_strings(32),
        }

    def _yrc1000_commands(self) -> dict[int, Handler]:
        """Commands YRC1000 and newer controllers add, which the DX200 manual does not
        have; their formats are as YMConnect, Yaskawa's own client, sends and reads them."""
        return {
            0x33D: self._cartesian_position,
            0x39C: self._system_parameter,
        }

    # --- Cartesian position reading (0x33D, YRC1000 and newer) -------------------

    def _cartesian_position(self, request: Request) -> bytes:
        """A control group's position in the base, robot or a user frame, with a
        tool. In: group kind, index, frame and tool. Out: coordinate type, user
        frame, tool, form, a reserved word and eight axes in µm and 0.0001 degrees.
        The emulated arm has no tool files, so every tool reads the same position."""
        if request.service != Service.GET_ALL:
            raise HsesError(Status.UNDEFINED_COMMAND)
        if len(request.data) != 16:
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        kind, index, frame, tool, _ = struct.unpack("<2i2Hi", request.data)
        if (kind, index) != (ROBOT_GROUPS, 0):
            raise _added(AddedStatus.CONTROL_GROUP_ERROR)
        self.controller.check_tool(tool)
        if frame not in (BASE_FRAME, ROBOT_FRAME):
            self.controller.check_user_frame(frame - 1)
        coordinate = UAPI_BASE if frame == BASE_FRAME else UAPI_ROBOT
        axes = _micrometres_and_ten_thousandths(self.controller.arm.state.view.pose)
        return struct.pack("<i2h2I8i", coordinate, 0, tool, self.controller.form, 0,
                           *_pad(axes, AXIS_ELEMENTS, 0))

    # --- System parameter reading (0x39C, YRC1000 and newer) ---------------------

    def _system_parameter(self, request: Request) -> bytes:
        """The parameter type, group and number, as 32-bit integers, in; its value out."""
        if request.service != Service.GET_ALL:
            raise HsesError(Status.UNDEFINED_COMMAND)
        if len(request.data) != 12:
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        key = struct.unpack("<3i", request.data)
        return int32s(self.controller.parameters.get(key, 0))

    # --- 3.3.1 Alarm data reading (0x70) ---------------------------------------

    def _alarm(self, request: Request) -> bytes:
        """The newest alarm (instance 1) to the fourth newest (4): code, data, type,
        time and name."""
        newest = _instance(request, 1, 2, 3, 4)
        alarms = self.controller.alarms()
        return _read(request, _alarm_fields(alarms[newest - 1] if newest <= len(alarms) else None))

    # --- 3.3.2 Alarm history reading (0x71) ------------------------------------

    def _alarm_history(self, request: Request) -> bytes:
        """The alarm history, newest first: major failures (instances 1 to 100), then
        monitor, system user, user and off-line alarms, which the emulator never
        raises. Each as code, data, type, time and name."""
        instance = _instance(request, *ALARM_HISTORY_INSTANCES)
        self.controller.alarms()  # notice a fault that has not been looked at yet
        history = self.controller.alarm_history if instance in MAJOR_FAILURES else []
        nth = instance % 1000
        return _read(request, _alarm_fields(history[nth - 1] if nth <= len(history) else None))

    # --- 3.3.37, 3.3.38 Alarms with sub code strings (0x30A, 0x30B) --------------

    def _alarm_with_sub_codes(self, request: Request) -> bytes:
        """As the alarm read, then the sub code's additional information (16
        characters), its text (96) and which of those characters show reversed (96)."""
        newest = _instance(request, 1, 2, 3, 4)
        alarms = self.controller.alarms()
        alarm = alarms[newest - 1] if newest <= len(alarms) else None
        return _read(request, [*_alarm_fields(alarm), *_SUB_CODE_FIELDS])

    def _alarm_history_with_sub_codes(self, request: Request) -> bytes:
        instance = _instance(request, *ALARM_HISTORY_INSTANCES)
        self.controller.alarms()
        history = self.controller.alarm_history if instance in MAJOR_FAILURES else []
        nth = instance % 1000
        alarm = history[nth - 1] if nth <= len(history) else None
        return _read(request, [*_alarm_fields(alarm), *_SUB_CODE_FIELDS])

    # --- 3.3.5 Axis configuration information reading (0x74) ---------------------

    def _axis_configuration(self, request: Request) -> bytes:
        """The names of R1's axes, in pulses (instance 1) or Cartesian (101)."""
        names = PULSE_AXES if _instance(request, R1_PULSE, R1_CARTESIAN) == R1_PULSE else (
            CARTESIAN_AXES
        )
        return _read(request, [text(4, name) for name in _pad(list(names), AXIS_ELEMENTS, "")])

    # --- 3.3.6 Robot position data reading (0x75) ------------------------------

    def _position(self, request: Request) -> bytes:
        """R1's position in pulses (instance 1) or base coordinates (101): data type,
        form, tool, user coordinate, extended form, then eight axes."""
        if _instance(request, R1_PULSE, R1_CARTESIAN) == R1_PULSE:
            data_type, axes = PULSE_DATA, self.controller.pulses()
        else:
            data_type, axes = BASE_COORDINATE_DATA, _micrometres_and_ten_thousandths(
                self.controller.arm.state.view.pose
            )
        values = [data_type, self.controller.form, self.controller.tool, 0, 0,
                  *_pad(axes, AXIS_ELEMENTS, 0)]
        return _read(request, [int32s(v) for v in values])

    # --- 3.3.7 Position error (0x76), 3.3.8 Torque (0x77) ----------------------

    def _zero_per_axis(self, request: Request) -> bytes:
        """Robot 1's position error or torque per axis. The emulated arm follows its
        path exactly and has no dynamics, so both are 0."""
        _instance(request, R1_PULSE)
        return _read(request, [int32s(0)] * AXIS_ELEMENTS)

    # --- 3.3.9 I/O data reading / writing (0x78) --------------------------------

    def _io(self, request: Request) -> bytes:
        """One IO byte, the instance its first signal's number / 10. Network inputs
        are the only ones a host may write."""
        signal = _io_signal(request.instance)
        if request.attribute not in SINGLE_VALUE_ATTRIBUTES:
            raise _added(AddedStatus.ATTRIBUTE_ERROR)
        if request.service == Service.GET_SINGLE:
            return bytes(self.controller.read_io([signal]))
        if request.service == Service.SET_SINGLE:
            if len(request.data) not in IO_WRITE_SIZES:
                raise _added(AddedStatus.DATA_SIZE_ERROR)
            self.controller.write_network_inputs([signal], [request.data[0]])
            return b""
        raise HsesError(Status.UNDEFINED_COMMAND)

    # --- 3.3.27 Plural I/O data reading / writing (0x300) -----------------------

    def _plural_io(self, request: Request) -> bytes:
        """A count of IO bytes (even, at most 474) from the instance's byte on."""
        start = _io_signal(request.instance)
        if request.attribute != 0:
            raise _added(AddedStatus.ATTRIBUTE_ERROR)
        if len(request.data) < 4:
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        (count,) = struct.unpack_from("<i", request.data)
        if count <= 0 or count % 2 or count > MAX_PLURAL_IO:
            raise _added(AddedStatus.OPERAND_RANGE)
        signals = [start + 10 * i for i in range(count)]
        if not all(_known_io_signal(signal) for signal in signals):
            raise _added(AddedStatus.OUTSIDE_THE_DATA)
        if request.service == Service.READ_PLURAL:
            return int32s(count) + bytes(self.controller.read_io(signals))
        if request.service == Service.WRITE_PLURAL:
            values = request.data[4:]
            if len(values) != count:
                raise _added(AddedStatus.DATA_SIZE_ERROR)
            self.controller.write_network_inputs(signals, list(values))
            return b""
        raise HsesError(Status.UNDEFINED_COMMAND)

    # --- Registers and variables: one (3.3.10-) or many (3.3.28-) -------------

    def _single(self, kind: Numbered) -> Handler:
        def handle(request: Request) -> bytes:
            number = _numbered(kind, request.instance)
            if request.attribute not in SINGLE_VALUE_ATTRIBUTES:
                raise _added(AddedStatus.ATTRIBUTE_ERROR)
            store = self.controller.variables.setdefault(kind.name, {})
            if request.service in (Service.GET_SINGLE, Service.GET_ALL):
                return kind.pack(store.get(number, 0))
            if request.service in (Service.SET_SINGLE, Service.SET_ALL):
                (value,) = kind.unpack(request.data, count=1)
                _check_writable(kind, number)
                store[number] = value
                return b""
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handle

    def _plural(self, kind: Numbered) -> Handler:
        def handle(request: Request) -> bytes:
            first = _numbered(kind, request.instance)
            if request.attribute != 0:
                raise _added(AddedStatus.ATTRIBUTE_ERROR)
            if len(request.data) < 4:
                raise _added(AddedStatus.DATA_SIZE_ERROR)
            (count,) = struct.unpack_from("<i", request.data)
            if not 1 <= count <= kind.plural_max or count % kind.plural_step:
                raise _added(AddedStatus.OPERAND_RANGE)
            numbers = range(first, first + count)
            if numbers[-1] not in kind.numbers:
                raise _added(AddedStatus.OUTSIDE_THE_DATA)
            store = self.controller.variables.setdefault(kind.name, {})
            if request.service == Service.READ_PLURAL:
                return int32s(count) + b"".join(kind.pack(store.get(n, 0)) for n in numbers)
            if request.service == Service.WRITE_PLURAL:
                values = kind.unpack(request.data[4:], count=count)
                for number in numbers:
                    _check_writable(kind, number)
                store.update(zip(numbers, values, strict=True))
                return b""
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handle

    # --- Position variables: one (3.3.16-) or many (3.3.34-) ------------------

    def _position_variable(self, kind: PositionKind) -> Handler:
        def handle(request: Request) -> bytes:
            number = _position_number(request.instance)
            store = self.controller.positions.setdefault(kind.name, {})
            record = store.get(number, kind.empty)
            if request.service == Service.GET_ALL:
                return _read(request, [int32s(v) for v in record])
            if request.service == Service.GET_SINGLE and kind.elementwise:
                return _read(request, [int32s(v) for v in record])
            if request.service == Service.SET_ALL:
                if request.attribute != 0:
                    raise _added(AddedStatus.ATTRIBUTE_ERROR)
                store[number] = kind.parse(request.data, count=1)[0]
                return b""
            if request.service == Service.SET_SINGLE and kind.elementwise:
                if not 1 <= request.attribute <= kind.elements:
                    raise _added(AddedStatus.ATTRIBUTE_ERROR)
                if len(request.data) != 4:
                    raise _added(AddedStatus.DATA_SIZE_ERROR)
                updated = list(record)
                updated[request.attribute - 1] = struct.unpack("<i", request.data)[0]
                store[number] = kind.check(tuple(updated))
                return b""
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handle

    def _plural_positions(self, kind: PositionKind) -> Handler:
        def handle(request: Request) -> bytes:
            first = _position_number(request.instance)
            if request.attribute != 0:
                raise _added(AddedStatus.ATTRIBUTE_ERROR)
            if len(request.data) < 4:
                raise _added(AddedStatus.DATA_SIZE_ERROR)
            (count,) = struct.unpack_from("<i", request.data)
            if not 1 <= count <= kind.plural_max:
                raise _added(AddedStatus.OPERAND_RANGE)
            numbers = range(first, first + count)
            if numbers[-1] not in POSITION_NUMBERS:
                raise _added(AddedStatus.OUTSIDE_THE_DATA)
            store = self.controller.positions.setdefault(kind.name, {})
            if request.service == Service.READ_PLURAL:
                return int32s(count) + b"".join(
                    int32s(*store.get(n, kind.empty)) for n in numbers
                )
            if request.service == Service.WRITE_PLURAL:
                store.update(zip(numbers, kind.parse(request.data[4:], count=count), strict=True))
                return b""
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handle

    # --- S variables: 16 or 32 bytes, one or many (3.3.15, 3.3.41, ...) ---------

    def _string_variable(self, size: int) -> Handler:
        def handle(request: Request) -> bytes:
            number = _string_number(request.instance)
            if request.attribute not in SINGLE_VALUE_ATTRIBUTES:
                raise _added(AddedStatus.ATTRIBUTE_ERROR)
            if request.service in (Service.GET_SINGLE, Service.GET_ALL):
                return self._string(number)[:size]
            if request.service in (Service.SET_SINGLE, Service.SET_ALL):
                # YMConnect sends the text unpadded, so a write may be shorter.
                if not 0 < len(request.data) <= size:
                    raise _added(AddedStatus.DATA_SIZE_ERROR)
                self._set_string(number, request.data)
                return b""
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handle

    def _plural_strings(self, size: int) -> Handler:
        plural_max = PLURAL_STRINGS[size]

        def handle(request: Request) -> bytes:
            first = _string_number(request.instance)
            if request.attribute != 0:
                raise _added(AddedStatus.ATTRIBUTE_ERROR)
            if len(request.data) < 4:
                raise _added(AddedStatus.DATA_SIZE_ERROR)
            (count,) = struct.unpack_from("<i", request.data)
            if not 1 <= count <= plural_max:
                raise _added(AddedStatus.OPERAND_RANGE)
            numbers = range(first, first + count)
            if numbers[-1] not in STANDARD_VARIABLES:
                raise _added(AddedStatus.OUTSIDE_THE_DATA)
            if request.service == Service.READ_PLURAL:
                return int32s(count) + b"".join(self._string(n)[:size] for n in numbers)
            if request.service == Service.WRITE_PLURAL:
                values = request.data[4:]
                if len(values) != size * count:
                    raise _added(AddedStatus.DATA_SIZE_ERROR)
                for i, number in enumerate(numbers):
                    self._set_string(number, values[i * size : (i + 1) * size])
                return b""
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handle

    def _string(self, number: int) -> bytes:
        return self.controller.strings.get(number, bytes(STRING_SIZE))

    def _set_string(self, number: int, value: bytes) -> None:
        """A 16-byte write replaces the whole string, so its last 16 bytes clear."""
        self.controller.strings[number] = value.ljust(STRING_SIZE, b"\0")

    # --- 3.3.20 Hold / servo on/off (0x83) -------------------------------------

    def _hold_servo_hlock(self, request: Request) -> bytes:
        """HOLD (instance 1), servo power (2) or HLOCK (3), each 1 for on and 2 for off."""
        which = _instance(request, 1, 2, 3)
        (switch,) = _write(request, count=1)
        if switch not in (ON, OFF):
            raise _added(AddedStatus.OPERAND_RANGE)
        on = switch == ON
        if which == 1:
            self.controller.set_hold(on)
        elif which == 2:
            self.controller.set_servo(on)
        else:
            self.controller.hlock = on
        return b""

    # --- 3.3.21 Step / cycle / continuous switching (0x84) ----------------------

    def _cycle(self, request: Request) -> bytes:
        _instance(request, 2)
        (cycle,) = _write(request, count=1)
        if cycle not in CYCLES:
            raise _added(AddedStatus.OPERAND_RANGE)
        self.controller.cycle = CYCLES[cycle]
        return b""

    # --- 3.3.4 Executing job information reading (0x73) ------------------------

    def _executing_job(self, request: Request) -> bytes:
        """The master task's (instance 1) or a sub task's (2-16) job name (32
        characters), line, step and speed override. Only the master task runs."""
        task = _instance(request, *TASKS)
        name, line = "", 0
        if task == MASTER_TASK:
            name, line = self.controller.executing_job, self.controller.executing_line
        return _read(request, [
            text(32, name), int32s(line), int32s(0), int32s(SPEED_OVERRIDE),
        ])

    # --- 3.3.23 Start-up (job start) (0x86) ----------------------------------

    def _start_job(self, request: Request) -> bytes:
        _instance(request, 1)
        _write(request, count=1)
        self.controller.start_job()
        return b""

    # --- 3.3.24 Job select (0x87) --------------------------------------------

    def _select_job(self, request: Request) -> bytes:
        """The executing job (instance 1), name and line, or a task's master job
        (10 for task 0 to 25 for task 15), name only."""
        instance = _instance(request, EXECUTING_JOB, *MASTER_JOB_INSTANCES)
        if request.service != Service.SET_ALL:
            raise HsesError(Status.UNDEFINED_COMMAND)
        if len(request.data) != JOB_NAME_FIELD + 4:
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        name = request.data[:JOB_NAME_FIELD].split(b"\0", 1)[0].decode("ascii", "replace")
        (line,) = struct.unpack_from("<i", request.data, JOB_NAME_FIELD)
        if instance == EXECUTING_JOB:
            if not 0 <= line <= MAX_JOB_LINE:
                raise _added(AddedStatus.OPERAND_RANGE)
            self.controller.select_job(name, line)
        else:
            self.controller.select_job(name, master_of_task=instance - MASTER_JOB_INSTANCES.start)
        return b""

    # --- 3.3.22 Character string display to the pendant (0x85) -----------------

    def _pendant_message(self, request: Request) -> bytes:
        """Up to 30 characters for the pendant's remote display, in a 32-byte field."""
        _instance(request, 1)
        if request.service != Service.SET_SINGLE:
            raise HsesError(Status.UNDEFINED_COMMAND)
        if request.attribute != 1:
            raise _added(AddedStatus.ATTRIBUTE_ERROR)
        # YMConnect sends the text unpadded, so the field may be shorter.
        if not 0 < len(request.data) <= PENDANT_FIELD:
            raise _added(AddedStatus.DATA_SIZE_ERROR)
        message = request.data.split(b"\0", 1)[0]
        if len(message) > PENDANT_CHARACTERS:
            raise _added(AddedStatus.OPERAND_RANGE)
        self.controller.pendant_message = message.decode("ascii", errors="replace")
        return b""

    # --- 3.3.25 Management time acquiring (0x88) -------------------------------

    def _management_time(self, request: Request) -> bytes:
        """When a counter started (16 characters) and how long it has run (12, as
        hhhhhh:mm'ss). Only the control power time is counted; the emulator keeps
        no servo, playback, motion or operation times, so those show 0 run."""
        instance = _instance(request, *MANAGEMENT_TIMES)
        started = self.controller.powered_on
        elapsed = datetime.now() - started if instance == CONTROL_POWER_TIME else timedelta(0)
        return _read(request, [
            text(16, started.strftime(ALARM_TIME_FORMAT)), text(12, _elapsed(elapsed)),
        ])

    # --- 3.3.26 System information acquiring (0x89) ---------------------------

    def _system_information(self, request: Request) -> bytes:
        """Software version (24 characters), model or application (16) and parameter
        version (8) of a robot (instances 11-18), station (21-44) or application
        (101-108). Groups the cell lacks answer empty strings."""
        instance = _instance(request, *SYSTEM_INSTANCES)
        model = ""
        if instance == R1_SYSTEM:
            model = self.controller.model
        elif instance == APPLICATIONS.start:
            model = APPLICATION
        version = self.controller.profile.software_version
        return _read(request, [text(24, version), text(16, model), text(8)])

    # --- 3.3.39 Move instruction, Cartesian (0x8A) -----------------------------

    def _move_cartesian(self, request: Request) -> bytes:
        """Joint move (instance 1), straight move (2) or straight increment (3) to a
        pose: groups, speed class and speed, coordinate, X Y Z (µm), Rx Ry Rz
        (0.0001 degrees), two reserved, form, extended form, tool, user coordinate,
        then base and station axes."""
        kind = MoveKind(_instance(request, *MoveKind))
        values = _move_data(request, count=26)
        _robot_1_only(values[0], values[1])
        speed_class, speed = _speed(kind, values[2], values[3])
        coordinate = values[4]
        tool = self.controller.check_tool(values[15])
        self._check_move_coordinate(kind, coordinate, user_frame=values[16])
        target = _pose_from(values[5:11])
        if kind is MoveKind.STRAIGHT_INCREMENT:
            here = self.controller.arm.state.view.pose
            target = stepped(here, pose_to_mat(target), in_tool=coordinate == TOOL_COORDINATE)
        self.controller.form, self.controller.tool = values[13], tool
        if kind is MoveKind.JOINT:
            self.controller.move_joints(self.controller.arm.inverse(target), speed)
        else:
            self.controller.move_linear(target, speed_class, speed)
        return b""

    # --- 3.3.40 Move instruction, pulse (0x8B) ---------------------------------

    def _move_pulse(self, request: Request) -> bytes:
        """Joint move (instance 1) or straight move (2) to pulses: groups, speed class
        and speed, eight robot axes, tool, then base and station axes."""
        kind = MoveKind(_instance(request, MoveKind.JOINT, MoveKind.STRAIGHT))
        values = _move_data(request, count=22)
        _robot_1_only(values[0], values[1])
        speed_class, speed = _speed(kind, values[2], values[3])
        target = self.controller.joints_at([float(v) for v in values[4:12]])
        self.controller.tool = self.controller.check_tool(values[12])
        if kind is MoveKind.JOINT:
            self.controller.move_joints(target, speed)
        else:
            self.controller.move_linear(self.controller.arm.forward(target), speed_class, speed)
        return b""

    def _check_move_coordinate(self, kind: MoveKind, coordinate: int, *, user_frame: int) -> None:
        """Base and robot coordinates are one frame; no user frame is taught; the tool
        coordinate only makes sense for a step."""
        if coordinate in (BASE_COORDINATE, ROBOT_COORDINATE):
            return
        if coordinate == USER_COORDINATE:
            if not 1 <= user_frame <= self.controller.profile.user_frames:
                raise _added(AddedStatus.USER_NUMBER_ERROR)
            raise _added(AddedStatus.USER_COORDINATE_NOT_TAUGHT)
        if coordinate == TOOL_COORDINATE and kind is MoveKind.STRAIGHT_INCREMENT:
            return
        raise _added(AddedStatus.COORDINATE_SYSTEM_ERROR)

    # --- 3.3.19 Alarm reset / error cancel (0x82) ------------------------------

    def _reset_or_cancel(self, request: Request) -> bytes:
        kind = _instance(request, 1, 2)
        _write(request, count=1)
        if kind == 1:
            self.controller.reset_alarm()
        else:
            self.controller.cancel_error()
        return b""

    # --- 3.3.3 Status information reading (0x72) ------------------------------

    def _status(self, request: Request) -> bytes:
        _instance(request, 1)
        data1, data2 = self.controller.status()
        return _read(request, [int32s(data1), int32s(data2)])


def _instance(request: Request, *allowed: int) -> int:
    if request.instance not in allowed:
        raise _added(AddedStatus.INSTANCE_ERROR)
    return request.instance


def _pad[T](values: list[T], count: int, fill: T) -> list[T]:
    """*values* followed by *fill* up to *count*."""
    return values + [fill] * (count - len(values))


def _micrometres_and_ten_thousandths(pose: Pose) -> list[int]:
    """A pose in µm and 0.0001 degrees, the units of the move commands (3.3.39).

    The manual gives no units for position reads; these are the move
    commands' and need checking on a controller.
    """
    x, y, z, rx, ry, rz = pose
    position = [round(m * 1e6) for m in (x, y, z)]
    return position + [round(math.degrees(r) * 1e4) for r in (rx, ry, rz)]


def _move_data(request: Request, *, count: int) -> tuple[int, ...]:
    if request.service != Service.SET_ALL:
        raise HsesError(Status.UNDEFINED_COMMAND)
    if request.attribute != 1:
        raise _added(AddedStatus.ATTRIBUTE_ERROR)
    if len(request.data) != 4 * count:
        raise _added(AddedStatus.DATA_SIZE_ERROR)
    return struct.unpack(f"<{count}i", request.data)


def _robot_1_only(robot: int, station: int) -> None:
    """The emulated cell has robot 1 and no stations."""
    if robot != 1 or station != 0:
        raise _added(AddedStatus.CONTROL_GROUP_ERROR)


def _speed(kind: MoveKind, speed_class: int, value: int) -> tuple[SpeedSelection, float]:
    """A joint move's speed in percent, or a straight move's selection and speed."""
    if value <= 0:
        raise _added(AddedStatus.SPEED_ERROR)
    if kind is MoveKind.JOINT:
        if speed_class != PERCENT or value > 10_000:
            raise _added(AddedStatus.SPEED_ERROR)
        return SpeedSelection.V, value / 100
    if speed_class == V:
        return SpeedSelection.V, value / 10
    if speed_class == VR:
        return SpeedSelection.VR, value / 10
    raise _added(AddedStatus.SPEED_ERROR)


def _pose_from(values: tuple[int, ...]) -> Pose:
    """X Y Z in µm and Rx Ry Rz in 0.0001 degrees → metres and radians."""
    x, y, z, rx, ry, rz = values
    return (
        Meters(x / 1e6), Meters(y / 1e6), Meters(z / 1e6),
        Radians(math.radians(rx / 1e4)), Radians(math.radians(ry / 1e4)),
        Radians(math.radians(rz / 1e4)),
    )


def _position_number(instance: int) -> int:
    if instance not in POSITION_NUMBERS:
        raise _added(AddedStatus.INSTANCE_ERROR)
    return instance


def _string_number(instance: int) -> int:
    if instance not in STANDARD_VARIABLES:
        raise _added(AddedStatus.INSTANCE_ERROR)
    return instance


def _numbered(kind: Numbered, instance: int) -> int:
    if instance not in kind.numbers:
        raise _added(AddedStatus.INSTANCE_ERROR)
    return instance


def _check_writable(kind: Numbered, number: int) -> None:
    if kind.writable is not None and number not in kind.writable:
        raise _added(AddedStatus.OUTSIDE_THE_DATA)


def _known_io_signal(signal: int) -> bool:
    return any(signal // 10 in instances for instances in IO_INSTANCES)


def _io_signal(instance: int) -> int:
    """The number of the first signal of the IO byte an instance names."""
    if not any(instance in instances for instances in IO_INSTANCES):
        raise _added(AddedStatus.INSTANCE_ERROR)
    return instance * 10


def _alarm_fields(alarm: Alarm | None) -> list[bytes]:
    """An alarm's code, data, type, time (16 characters) and name (32); all zero for none."""
    if alarm is None:
        return [int32s(0), int32s(0), int32s(0), text(16), text(32)]
    return [
        int32s(alarm.code), int32s(alarm.data), int32s(ALARM_TYPE_DECIMAL),
        text(16, alarm.time), text(32, FAULT_ALARM_NAME),
    ]


#: A stand-in alarm has no sub code, so its sub code strings are empty.
_SUB_CODE_FIELDS = (text(16), text(96), text(96))


def _elapsed(elapsed: timedelta) -> str:
    """A run time as the controller shows it: hours, minutes and seconds, 000123:45'06."""
    seconds = int(elapsed.total_seconds())
    return f"{seconds // 3600:06d}:{seconds // 60 % 60:02d}'{seconds % 60:02d}"


def _write(request: Request, *, count: int) -> tuple[int, ...]:
    """The *count* 32-bit integers a Set_Attribute request carries."""
    if request.service not in (Service.SET_SINGLE, Service.SET_ALL):
        raise HsesError(Status.UNDEFINED_COMMAND)
    if request.attribute != 1:
        raise _added(AddedStatus.ATTRIBUTE_ERROR)
    if len(request.data) != 4 * count:
        raise _added(AddedStatus.DATA_SIZE_ERROR)
    return struct.unpack(f"<{count}i", request.data)


def _read(request: Request, elements: list[bytes]) -> bytes:
    """One element (Get_Attribute_Single) or all of them (Get_Attribute_All)."""
    if request.service == Service.GET_ALL:
        if request.attribute != 0:
            raise _added(AddedStatus.ATTRIBUTE_ERROR)
        return b"".join(elements)
    if request.service == Service.GET_SINGLE:
        if not 1 <= request.attribute <= len(elements):
            raise _added(AddedStatus.ATTRIBUTE_ERROR)
        return elements[request.attribute - 1]
    raise HsesError(Status.UNDEFINED_COMMAND)
