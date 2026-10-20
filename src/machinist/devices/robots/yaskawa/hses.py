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
    CommandRefusedError,
    Refusal,
    SpeedSelection,
    Status1,
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
#: What the emulator reports as its software version and application. A real
#: controller reports its own, such as "DS2.07.00A. (JP/US) -00".
SOFTWARE_VERSION = "MACHINIST"
APPLICATION = "HANDLING"
#: The data of an on/off switch (3.3.20).
ON, OFF = 1, 2
#: The cycles 0x84 switches to (3.3.21).
CYCLES = {1: Status1.STEP, 2: Status1.ONE_CYCLE, 3: Status1.AUTO}
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
        if request.division is not Division.ROBOT:
            raise HsesError(Status.UNDEFINED_COMMAND)  # file control is not emulated
        handler = self._commands().get(request.command)
        if handler is None:
            raise HsesError(Status.UNDEFINED_COMMAND)
        return handler(request)

    def _commands(self) -> dict[int, Handler]:
        return {
            0x70: self._alarm,
            0x72: self._status,
            0x74: self._axis_configuration,
            0x75: self._position,
            0x76: self._zero_per_axis,
            0x77: self._zero_per_axis,
            0x78: self._io,
            0x82: self._reset_or_cancel,
            0x83: self._hold_servo_hlock,
            0x84: self._cycle,
            0x89: self._system_information,
            0x8A: self._move_cartesian,
            0x8B: self._move_pulse,
            0x300: self._plural_io,
        }

    # --- 3.3.1 Alarm data reading (0x70) ---------------------------------------

    def _alarm(self, request: Request) -> bytes:
        """The newest alarm (instance 1) to the fourth newest (4): code, data, type,
        time and name."""
        newest = _instance(request, 1, 2, 3, 4)
        alarms = self.controller.alarms()
        if newest > len(alarms):
            return _read(request, [int32s(0), int32s(0), int32s(0), text(16), text(32)])
        code, data = alarms[newest - 1]
        return _read(request, [
            int32s(code), int32s(data), int32s(ALARM_TYPE_DECIMAL), text(16),
            text(32, FAULT_ALARM_NAME),
        ])

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
        if request.attribute != 1:
            raise _added(AddedStatus.ATTRIBUTE_ERROR)
        if request.service == Service.GET_SINGLE:
            return bytes(self.controller.read_io([signal]))
        if request.service == Service.SET_SINGLE:
            if len(request.data) != 1:
                raise _added(AddedStatus.DATA_SIZE_ERROR)
            self.controller.write_network_inputs([signal], list(request.data))
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
        return _read(request, [text(24, SOFTWARE_VERSION), text(16, model), text(8)])

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


def _known_io_signal(signal: int) -> bool:
    return any(signal // 10 in instances for instances in IO_INSTANCES)


def _io_signal(instance: int) -> int:
    """The number of the first signal of the IO byte an instance names."""
    if not any(instance in instances for instances in IO_INSTANCES):
        raise _added(AddedStatus.INSTANCE_ERROR)
    return instance * 10


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
