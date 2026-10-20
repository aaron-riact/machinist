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
from .controller import CommandRefusedError, Refusal, Status1, YaskawaController


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
            0x82: self._reset_or_cancel,
            0x83: self._hold_servo_hlock,
            0x84: self._cycle,
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
