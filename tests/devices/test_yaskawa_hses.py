"""The HSES server against the DX200 manual (HW1481977), chapter 3.3."""

from __future__ import annotations

import socket
import struct
import time
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.transport.message import UdpMessageTransport
from machinist.transport.yaskawa_hses import (
    AddedStatus,
    Answer,
    Division,
    Request,
    Status,
    encode_request,
    parse_answer,
)

from ..conftest import free_port, wait_running

GET_ALL, GET_SINGLE = 0x01, 0x0E


@pytest.fixture
def yrc() -> Iterator[YaskawaRobot]:
    options = YaskawaOptions(
        controller="yrc1000", robot="up50n", protocols=("ethernet_server", "hses"),
        hses_port=free_port(), start_joints_deg=(0, 0, 0, 0, -90, 0), jobs=("MAIN", "PICK"),
    )
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


def _ask(robot: YaskawaRobot, command: int, instance: int, attribute: int, service: int,
         data: bytes = b"", *, request_id: int = 1) -> Answer:
    client = UdpMessageTransport("127.0.0.1", robot.hses_port, timeout=2)
    try:
        request = Request(division=Division.ROBOT, request_id=request_id, command=command,
                          instance=instance, attribute=attribute, service=service, data=data)
        return parse_answer(client.request(encode_request(request)))
    finally:
        client.close()


def _ints(answer: Answer) -> tuple[int, ...]:
    return struct.unpack(f"<{len(answer.data) // 4}i", answer.data)


def test_status_read_answers_both_status_words(yrc: YaskawaRobot) -> None:
    """3.2.1 / 3.3.3: command remote, play, one cycle; servo on."""
    answer = _ask(yrc, 0x72, 1, 0, GET_ALL, request_id=42)
    assert answer.status is Status.OK
    assert answer.request_id == 42
    assert answer.service == 0x81
    assert _ints(answer) == (0xC2, 0x40)


def test_status_read_of_one_element(yrc: YaskawaRobot) -> None:
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x40,)


@pytest.mark.parametrize(
    ("instance", "attribute", "added_status"),
    [(2, 0, AddedStatus.INSTANCE_ERROR), (1, 3, AddedStatus.ATTRIBUTE_ERROR)],
)
def test_status_read_refuses_what_does_not_exist(
    yrc: YaskawaRobot, instance: int, attribute: int, added_status: AddedStatus
) -> None:
    service = GET_ALL if attribute == 0 else GET_SINGLE
    answer = _ask(yrc, 0x72, instance, attribute, service)
    assert (answer.status, answer.added_status) == (Status.VENDOR, added_status)


def test_an_undefined_command_is_answered_so(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x6F, 1, 0, GET_ALL).status is Status.UNDEFINED_COMMAND


def test_a_malformed_datagram_gets_the_manuals_added_status(yrc: YaskawaRobot) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(2)
        s.sendto(b"XERC" + bytes(28), ("127.0.0.1", yrc.hses_port))
        answer = parse_answer(s.recv(1024))
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.IDENTIFIER_ERROR)


def test_both_servers_share_one_controller(yrc: YaskawaRobot) -> None:
    """A HOLD over the Ethernet server shows in the status HSES reads."""
    with socket.create_connection(("127.0.0.1", yrc.endpoint.port), timeout=2) as s:
        s.sendall(b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST HOLD 2\r\n1\r")
        while s.recv(1024):
            pass
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x48,)


SET_SINGLE = 0x10


def test_alarm_read_without_an_alarm_is_all_zero(yrc: YaskawaRobot) -> None:
    """3.3.1: code, data, type (4 bytes each), time (16), name (32)."""
    assert _ask(yrc, 0x70, 1, 0, GET_ALL).data == bytes(60)


def test_alarm_read_reports_a_fault(yrc: YaskawaRobot) -> None:
    yrc.arm.fault()
    answer = _ask(yrc, 0x70, 1, 0, GET_ALL)
    assert struct.unpack("<3i", answer.data[:12]) == (4000, 0, 1)
    assert answer.data[28:].rstrip(b"\0") == b"MACHINIST FAULT"
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x50,)  # alarm, servo on


def test_alarm_read_of_one_element(yrc: YaskawaRobot) -> None:
    yrc.arm.fault()
    assert _ints(_ask(yrc, 0x70, 1, 1, GET_SINGLE)) == (4000,)
    assert _ask(yrc, 0x70, 2, 1, GET_SINGLE).data == bytes(4)  # no second alarm


def test_alarm_reset_clears_the_fault(yrc: YaskawaRobot) -> None:
    yrc.arm.fault()
    assert _ask(yrc, 0x82, 1, 1, SET_SINGLE, struct.pack("<i", 1)).status is Status.OK
    assert _ask(yrc, 0x70, 1, 0, GET_ALL).data == bytes(60)


def test_error_cancel_answers_normally(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x82, 2, 1, SET_SINGLE, struct.pack("<i", 1)).status is Status.OK


def test_a_write_without_its_data_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x82, 1, 1, SET_SINGLE)
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.DATA_SIZE_ERROR)


def _set(robot: YaskawaRobot, command: int, instance: int, value: int) -> Answer:
    return _ask(robot, command, instance, 1, SET_SINGLE, struct.pack("<i", value))


def test_hold_on_and_off(yrc: YaskawaRobot) -> None:
    assert _set(yrc, 0x83, 1, 1).status is Status.OK
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x48,)
    assert _set(yrc, 0x83, 1, 2).status is Status.OK
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x40,)


def test_servo_off_and_on(yrc: YaskawaRobot) -> None:
    assert _set(yrc, 0x83, 2, 2).status is Status.OK
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x00,)
    assert _set(yrc, 0x83, 2, 1).status is Status.OK
    assert _ints(_ask(yrc, 0x72, 1, 2, GET_SINGLE)) == (0x40,)


def test_servo_on_during_an_emergency_stop_is_refused(yrc: YaskawaRobot) -> None:
    yrc.arm.estop()
    answer = _set(yrc, 0x83, 2, 1)
    assert (answer.status, answer.added_status) == (Status.VENDOR,
                                                    AddedStatus.SERVO_CANNOT_TURN_ON)


def test_hlock_is_remembered(yrc: YaskawaRobot) -> None:
    assert _set(yrc, 0x83, 3, 1).status is Status.OK
    assert yrc.controller.hlock


def test_a_switch_takes_1_or_2(yrc: YaskawaRobot) -> None:
    answer = _set(yrc, 0x83, 1, 0)
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.OPERAND_RANGE)


@pytest.mark.parametrize(("cycle", "bit"), [(1, 0x01), (2, 0x02), (3, 0x04)])
def test_cycle_switching_shows_in_the_status(yrc: YaskawaRobot, cycle: int, bit: int) -> None:
    assert _set(yrc, 0x84, 2, cycle).status is Status.OK
    assert _ints(_ask(yrc, 0x72, 1, 1, GET_SINGLE)) == (0xC0 | bit,)


def test_axis_configuration_names_r1s_axes(yrc: YaskawaRobot) -> None:
    pulse = _ask(yrc, 0x74, 1, 0, GET_ALL).data
    assert [pulse[i:i + 4].rstrip(b"\0") for i in range(0, 32, 4)] == [
        b"S", b"L", b"U", b"R", b"B", b"T", b"", b""]
    cartesian = _ask(yrc, 0x74, 101, 0, GET_ALL).data
    assert cartesian[12:24] == b"Rx\0\0Ry\0\0Rz\0\0"


def test_position_in_pulses(yrc: YaskawaRobot) -> None:
    """Data type 0, form, tool, user coordinate, extended form, eight axes."""
    assert _ints(_ask(yrc, 0x75, 1, 0, GET_ALL)) == (
        0, 0, 0, 0, 0, 0, 0, 0, 0, -73307, 0, 0, 0)


def test_position_in_base_coordinates(yrc: YaskawaRobot) -> None:
    """The UP50N's flange at B -90: 1170 mm out, 805 mm up, pointing down."""
    values = _ints(_ask(yrc, 0x75, 101, 0, GET_ALL))
    assert values[0] == 16
    x, y, z, rx, ry, rz = values[5:11]
    assert (x, y, z) == (1_170_000, 0, 805_000)
    assert (abs(rx), ry, abs(rz)) == (1_800_000, 0, 1_800_000)


def test_position_of_one_axis(yrc: YaskawaRobot) -> None:
    assert _ints(_ask(yrc, 0x75, 1, 10, GET_SINGLE)) == (-73307,)


def test_position_of_a_group_the_cell_lacks_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x75, 2, 0, GET_ALL)
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.INSTANCE_ERROR)


def test_io_write_and_read_one_network_input_byte(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x78, 2701, 1, SET_SINGLE, b"\x05").status is Status.OK
    assert _ask(yrc, 0x78, 2701, 1, GET_SINGLE).data == b"\x05"


def test_io_write_outside_the_network_inputs_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x78, 3004, 1, SET_SINGLE, b"\x01")
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.OUTSIDE_THE_DATA)


def test_io_of_an_unknown_signal_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x78, 9001, 1, GET_SINGLE)
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.INSTANCE_ERROR)


def test_plural_io_writes_and_reads_bytes(yrc: YaskawaRobot) -> None:
    write = _ask(yrc, 0x300, 2701, 0, 0x34, struct.pack("<i", 4) + b"\x3f\x00\x19\x01")
    assert write.status is Status.OK
    read = _ask(yrc, 0x300, 2701, 0, 0x33, struct.pack("<i", 4))
    assert read.data == struct.pack("<i", 4) + b"\x3f\x00\x19\x01"


@pytest.mark.parametrize("count", [3, 0, 476])
def test_plural_io_counts_in_twos_up_to_474(yrc: YaskawaRobot, count: int) -> None:
    answer = _ask(yrc, 0x300, 2701, 0, 0x33, struct.pack("<i", count))
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.OPERAND_RANGE)


def test_hses_io_and_the_ethernet_servers_io_are_the_same_signals(yrc: YaskawaRobot) -> None:
    _ask(yrc, 0x78, 2701, 1, SET_SINGLE, b"\x2a")
    with socket.create_connection(("127.0.0.1", yrc.endpoint.port), timeout=2) as s:
        s.sendall(b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST IOREAD 8\r\n27010,8\r")
        received = b""
        while chunk := s.recv(1024):
            received += chunk
    assert received.endswith(b"OK: IOREAD\r\n42\r")


SET_ALL = 0x02
#: The fixture's pose: the flange 1170 mm out and 805 mm up, pointing down.
HOME = (1_170_000, 0, 805_000, 1_800_000, 0, 1_800_000)


def _cartesian(kind: int, xyz_rxryrz: tuple[int, ...], *, speed_class: int = 1,
               speed: int = 5000, coordinate: int = 16, tool: int = 0, user: int = 0,
               robot: int = 1, station: int = 0) -> bytes:
    return struct.pack("<26i", robot, station, speed_class, speed, coordinate, *xyz_rxryrz,
                       0, 0, 0, 0, tool, user, *([0] * 9))


def _pulses(pulses: tuple[int, ...], *, speed_class: int = 0, speed: int = 10_000,
            tool: int = 0) -> bytes:
    return struct.pack("<22i", 1, 0, speed_class, speed, *pulses, tool, *([0] * 9))


def _settle(robot: YaskawaRobot) -> None:
    deadline = time.monotonic() + 5
    while robot.arm.state.view.moving:
        assert time.monotonic() < deadline, "the move did not finish"
        time.sleep(0.02)


def _cartesian_position(robot: YaskawaRobot) -> tuple[int, ...]:
    return _ints(_ask(robot, 0x75, 101, 0, GET_ALL))[5:11]


def test_pulse_joint_move(yrc: YaskawaRobot) -> None:
    move = _pulses((0, 0, 0, 0, -36654, 0, 0, 0))
    assert _ask(yrc, 0x8B, 1, 1, SET_ALL, move).status is Status.OK
    _settle(yrc)
    assert _ints(_ask(yrc, 0x75, 1, 10, GET_SINGLE)) == (-36654,)


def test_pulse_straight_move(yrc: YaskawaRobot) -> None:
    move = _pulses((0, 0, 0, 0, -36654, 0, 0, 0), speed_class=2, speed=900)
    assert _ask(yrc, 0x8B, 2, 1, SET_ALL, move).status is Status.OK
    _settle(yrc)
    assert _ints(_ask(yrc, 0x75, 1, 10, GET_SINGLE)) == pytest.approx((-36654,), abs=50)


def test_cartesian_straight_move(yrc: YaskawaRobot) -> None:
    up = (HOME[0], HOME[1], HOME[2] + 100_000, *HOME[3:])
    assert _ask(yrc, 0x8A, 2, 1, SET_ALL, _cartesian(2, up)).status is Status.OK
    _settle(yrc)
    x, y, z, *_ = _cartesian_position(yrc)
    assert (x, y, z) == pytest.approx((1_170_000, 0, 905_000), abs=100)


def test_cartesian_joint_move(yrc: YaskawaRobot) -> None:
    out = (HOME[0] + 50_000, *HOME[1:])
    move = _cartesian(1, out, speed_class=0, speed=5000)
    assert _ask(yrc, 0x8A, 1, 1, SET_ALL, move).status is Status.OK
    _settle(yrc)
    assert _cartesian_position(yrc)[0] == pytest.approx(1_220_000, abs=100)


@pytest.mark.parametrize(("coordinate", "dz"), [(16, 50_000), (19, -50_000)])
def test_straight_increment_in_base_or_tool_coordinates(
    yrc: YaskawaRobot, coordinate: int, dz: int
) -> None:
    """The tool points down, so a step along its Z goes down in the base."""
    step = (0, 0, 50_000, 0, 0, 0)
    move = _cartesian(3, step, coordinate=coordinate)
    assert _ask(yrc, 0x8A, 3, 1, SET_ALL, move).status is Status.OK
    _settle(yrc)
    assert _cartesian_position(yrc)[2] == pytest.approx(805_000 + dz, abs=100)


@pytest.mark.parametrize(
    ("instance", "data", "added_status"),
    [
        (2, _cartesian(2, HOME, robot=2), AddedStatus.CONTROL_GROUP_ERROR),
        (2, _cartesian(2, HOME, station=1), AddedStatus.CONTROL_GROUP_ERROR),
        (1, _cartesian(1, HOME, speed_class=1), AddedStatus.SPEED_ERROR),
        (2, _cartesian(2, HOME, speed_class=0), AddedStatus.SPEED_ERROR),
        (2, _cartesian(2, HOME, speed=0), AddedStatus.SPEED_ERROR),
        (2, _cartesian(2, HOME, tool=64), AddedStatus.TOOL_NUMBER_ERROR),
        (2, _cartesian(2, HOME, coordinate=18, user=1), AddedStatus.USER_COORDINATE_NOT_TAUGHT),
        (2, _cartesian(2, HOME, coordinate=18, user=65), AddedStatus.USER_NUMBER_ERROR),
        (2, _cartesian(2, HOME, coordinate=19), AddedStatus.COORDINATE_SYSTEM_ERROR),
        (2, _cartesian(2, HOME)[:-4], AddedStatus.DATA_SIZE_ERROR),
    ],
)
def test_a_cartesian_move_is_refused_with_the_manuals_code(
    yrc: YaskawaRobot, instance: int, data: bytes, added_status: AddedStatus
) -> None:
    answer = _ask(yrc, 0x8A, instance, 1, SET_ALL, data)
    assert (answer.status, answer.added_status) == (Status.VENDOR, added_status)


@pytest.mark.parametrize(
    ("setup", "added_status"),
    [("servo_off", AddedStatus.SERVO_OFF), ("hold", AddedStatus.COMMAND_HOLD)],
)
def test_no_move_while_servo_is_off_or_held(
    yrc: YaskawaRobot, setup: str, added_status: AddedStatus
) -> None:
    if setup == "servo_off":
        _set(yrc, 0x83, 2, 2)
    else:
        _set(yrc, 0x83, 1, 1)
    answer = _ask(yrc, 0x8B, 1, 1, SET_ALL, _pulses((0,) * 8))
    assert (answer.status, answer.added_status) == (Status.VENDOR, added_status)


@pytest.mark.parametrize("command", [0x76, 0x77])
def test_position_error_and_torque_are_zero(yrc: YaskawaRobot, command: int) -> None:
    assert _ints(_ask(yrc, command, 1, 0, GET_ALL)) == (0,) * 8
    assert _ints(_ask(yrc, command, 1, 3, GET_SINGLE)) == (0,)
    assert _ask(yrc, command, 11, 0, GET_ALL).added_status == AddedStatus.INSTANCE_ERROR


def test_system_information_names_the_arm(yrc: YaskawaRobot) -> None:
    """Software version (24), model (16), parameter version (8)."""
    data = _ask(yrc, 0x89, 11, 0, GET_ALL).data
    assert len(data) == 48
    assert data[24:40].rstrip(b"\0") == b"YR-UP50N-A00"


def test_system_information_of_a_missing_group_is_empty(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x89, 12, 2, GET_SINGLE).data == bytes(16)
    assert _ask(yrc, 0x89, 101, 2, GET_SINGLE).data.rstrip(b"\0") == b"HANDLING"


def test_register_write_and_read(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x79, 7, 1, SET_SINGLE, struct.pack("<H", 1234)).status is Status.OK
    assert _ask(yrc, 0x79, 7, 1, GET_SINGLE).data == struct.pack("<H", 1234)
    assert _ask(yrc, 0x79, 8, 1, GET_SINGLE).data == struct.pack("<H", 0)


def test_registers_from_560_are_read_only(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x79, 560, 1, SET_SINGLE, struct.pack("<H", 1))
    assert (answer.status, answer.added_status) == (Status.VENDOR, AddedStatus.OUTSIDE_THE_DATA)
    assert _ask(yrc, 0x79, 999, 1, GET_SINGLE).status is Status.OK
    assert _ask(yrc, 0x79, 1000, 1, GET_SINGLE).added_status == AddedStatus.INSTANCE_ERROR


def test_plural_registers(yrc: YaskawaRobot) -> None:
    values = struct.pack("<3H", 1, 2, 3)
    assert _ask(yrc, 0x301, 10, 0, 0x34, struct.pack("<i", 3) + values).status is Status.OK
    read = _ask(yrc, 0x301, 9, 0, 0x33, struct.pack("<i", 5))
    assert read.data == struct.pack("<i5H", 5, 0, 1, 2, 3, 0)


def test_plural_registers_stop_at_237(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x301, 0, 0, 0x33, struct.pack("<i", 238))
    assert answer.added_status == AddedStatus.OPERAND_RANGE


def test_a_register_value_of_the_wrong_size_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x79, 7, 1, SET_SINGLE, struct.pack("<i", 1))
    assert answer.added_status == AddedStatus.DATA_SIZE_ERROR


@pytest.mark.parametrize(
    ("command", "fmt", "value"),
    [(0x7A, "<B", 200), (0x7B, "<h", -1234), (0x7C, "<i", -123_456_789), (0x7D, "<f", 1.5)],
)
def test_variable_write_and_read(yrc: YaskawaRobot, command: int, fmt: str,
                                 value: float) -> None:
    assert _ask(yrc, command, 99, 1, SET_SINGLE, struct.pack(fmt, value)).status is Status.OK
    assert struct.unpack(fmt, _ask(yrc, command, 99, 1, GET_SINGLE).data) == (value,)
    assert _ask(yrc, command, 100, 1, GET_SINGLE).added_status == AddedStatus.INSTANCE_ERROR


def test_variables_of_each_kind_are_their_own(yrc: YaskawaRobot) -> None:
    _ask(yrc, 0x7B, 5, 1, SET_SINGLE, struct.pack("<h", 7))
    assert _ask(yrc, 0x7C, 5, 1, GET_SINGLE).data == struct.pack("<i", 0)


@pytest.mark.parametrize(
    ("command", "fmt"), [(0x302, "B"), (0x303, "h"), (0x304, "i"), (0x305, "f")]
)
def test_plural_variables(yrc: YaskawaRobot, command: int, fmt: str) -> None:
    values = struct.pack(f"<2{fmt}", 1, 2)
    assert _ask(yrc, command, 3, 0, 0x34, struct.pack("<i", 2) + values).status is Status.OK
    assert _ask(yrc, command, 3, 0, 0x33, struct.pack("<i", 2)).data == (
        struct.pack("<i", 2) + values
    )


def test_plural_b_variables_count_in_twos(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x302, 0, 0, 0x33, struct.pack("<i", 3))
    assert answer.added_status == AddedStatus.OPERAND_RANGE


def test_plural_variables_stop_at_the_last_number(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x303, 98, 0, 0x33, struct.pack("<i", 3))
    assert answer.added_status == AddedStatus.OUTSIDE_THE_DATA


#: A P variable in base coordinates: type 16, form 4, tool 2, user 0, extended form 0,
#: X Y Z Rx Ry Rz and two unused axes.
P_BASE = (16, 4, 2, 0, 0, 1_000_000, 0, 500_000, 1_800_000, 0, 0, 0, 0)


def test_p_variable_write_and_read(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x7F, 127, 0, SET_ALL, struct.pack("<13i", *P_BASE)).status is Status.OK
    assert _ints(_ask(yrc, 0x7F, 127, 0, GET_ALL)) == P_BASE
    assert _ints(_ask(yrc, 0x7F, 3, 0, GET_ALL)) == (0,) * 13


def test_p_variables_are_written_whole(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x7F, 1, 1, SET_SINGLE, struct.pack("<i", 16))
    assert answer.status is Status.UNDEFINED_COMMAND


@pytest.mark.parametrize(
    ("instance", "record", "added_status"),
    [
        (128, P_BASE, AddedStatus.INSTANCE_ERROR),
        (1, (20, *P_BASE[1:]), AddedStatus.OPERAND_RANGE),
        (1, P_BASE[:12], AddedStatus.DATA_SIZE_ERROR),
    ],
)
def test_a_bad_p_variable_is_refused(yrc: YaskawaRobot, instance: int,
                                     record: tuple[int, ...], added_status: AddedStatus) -> None:
    data = struct.pack(f"<{len(record)}i", *record)
    assert _ask(yrc, 0x7F, instance, 0, SET_ALL, data).added_status == added_status


def test_plural_p_variables(yrc: YaskawaRobot) -> None:
    pulse = (0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 0, 0)
    data = struct.pack("<i26i", 2, *P_BASE, *pulse)
    assert _ask(yrc, 0x307, 10, 0, 0x34, data).status is Status.OK
    assert _ints(_ask(yrc, 0x307, 10, 0, 0x33, struct.pack("<i", 2))) == (2, *P_BASE, *pulse)


def test_plural_p_variables_stop_at_9(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x307, 0, 0, 0x33, struct.pack("<i", 10))
    assert answer.added_status == AddedStatus.OPERAND_RANGE


@pytest.mark.parametrize(("command", "data_type"), [(0x80, 16), (0x81, 0)])
def test_bp_and_ex_variables_whole_and_by_field(yrc: YaskawaRobot, command: int,
                                                data_type: int) -> None:
    record = (data_type, 1, 2, 3, 4, 5, 6, 7, 8)
    assert _ask(yrc, command, 5, 0, SET_ALL, struct.pack("<9i", *record)).status is Status.OK
    assert _ints(_ask(yrc, command, 5, 0, GET_ALL)) == record
    assert _ask(yrc, command, 5, 3, SET_SINGLE, struct.pack("<i", 99)).status is Status.OK
    assert _ints(_ask(yrc, command, 5, 3, GET_SINGLE)) == (99,)
    assert _ints(_ask(yrc, command, 5, 0, GET_ALL)) == (data_type, 1, 99, 3, 4, 5, 6, 7, 8)


def test_ex_variables_are_pulses_only(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x81, 5, 0, SET_ALL, struct.pack("<9i", 16, *([0] * 8)))
    assert answer.added_status == AddedStatus.OPERAND_RANGE


@pytest.mark.parametrize("command", [0x308, 0x309])
def test_plural_bp_and_ex_variables_stop_at_13(yrc: YaskawaRobot, command: int) -> None:
    assert _ask(yrc, command, 0, 0, 0x33, struct.pack("<i", 13)).status is Status.OK
    answer = _ask(yrc, command, 0, 0, 0x33, struct.pack("<i", 14))
    assert answer.added_status == AddedStatus.OPERAND_RANGE


def test_string_variable_32_bytes(yrc: YaskawaRobot) -> None:
    value = b"PICK FROM PALLET 3".ljust(32, b"\0")
    assert _ask(yrc, 0x8C, 4, 1, SET_SINGLE, value).status is Status.OK
    assert _ask(yrc, 0x8C, 4, 1, GET_SINGLE).data == value


def test_the_16_byte_command_carries_the_first_16(yrc: YaskawaRobot) -> None:
    _ask(yrc, 0x8C, 4, 1, SET_SINGLE, b"A" * 32)
    assert _ask(yrc, 0x7E, 4, 1, GET_SINGLE).data == b"A" * 16
    assert _ask(yrc, 0x7E, 4, 1, SET_SINGLE, b"B" * 16).status is Status.OK
    assert _ask(yrc, 0x8C, 4, 1, GET_SINGLE).data == b"B" * 16 + bytes(16)


def test_a_string_may_come_unpadded(yrc: YaskawaRobot) -> None:
    """YMConnect writes S variables without padding them."""
    assert _ask(yrc, 0x8C, 4, 1, SET_SINGLE, b"HELLO").status is Status.OK
    assert _ask(yrc, 0x8C, 4, 1, GET_SINGLE).data == b"HELLO".ljust(32, b"\0")


def test_a_string_longer_than_its_field_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x8C, 4, 1, SET_SINGLE, b"X" * 33)
    assert answer.added_status == AddedStatus.DATA_SIZE_ERROR


@pytest.mark.parametrize(("command", "size", "most"), [(0x306, 16, 29), (0x30C, 32, 14)])
def test_plural_strings(yrc: YaskawaRobot, command: int, size: int, most: int) -> None:
    values = b"X" * size + b"Y" * size
    assert _ask(yrc, command, 0, 0, 0x34, struct.pack("<i", 2) + values).status is Status.OK
    assert _ask(yrc, command, 0, 0, 0x33, struct.pack("<i", 2)).data == (
        struct.pack("<i", 2) + values
    )
    answer = _ask(yrc, command, 0, 0, 0x33, struct.pack("<i", most + 1))
    assert answer.added_status == AddedStatus.OPERAND_RANGE


def test_an_alarm_carries_the_time_it_was_noticed(yrc: YaskawaRobot) -> None:
    yrc.arm.fault()
    when = _ask(yrc, 0x70, 1, 4, GET_SINGLE).data.decode()
    assert time.strptime(when, "%Y/%m/%d %H:%M")


def test_alarm_history_keeps_alarms_after_their_reset(yrc: YaskawaRobot) -> None:
    yrc.arm.fault()
    _ask(yrc, 0x70, 1, 0, GET_ALL)
    _ask(yrc, 0x82, 1, 1, SET_SINGLE, struct.pack("<i", 1))
    yrc.arm.fault()
    assert _ints(_ask(yrc, 0x71, 1, 1, GET_SINGLE)) == (4000,)
    assert _ints(_ask(yrc, 0x71, 2, 1, GET_SINGLE)) == (4000,)
    assert _ask(yrc, 0x71, 3, 0, GET_ALL).data == bytes(60)


def test_alarm_history_of_other_kinds_is_empty(yrc: YaskawaRobot) -> None:
    yrc.arm.fault()
    assert _ask(yrc, 0x71, 1001, 0, GET_ALL).data == bytes(60)
    assert _ask(yrc, 0x71, 101, 0, GET_ALL).added_status == AddedStatus.INSTANCE_ERROR


@pytest.mark.parametrize("command", [0x30A, 0x30B])
def test_alarms_with_sub_codes_add_three_strings(yrc: YaskawaRobot, command: int) -> None:
    """3.3.37: the alarm read's 60 bytes, then 16 + 96 + 96 bytes of sub code text."""
    yrc.arm.fault()
    answer = _ask(yrc, command, 1, 0, GET_ALL)
    assert len(answer.data) == 268
    assert struct.unpack("<i", answer.data[:4]) == (4000,)
    assert answer.data[60:] == bytes(208)
    assert _ask(yrc, command, 1, 7, GET_SINGLE).data == bytes(96)


def test_pendant_message(yrc: YaskawaRobot) -> None:
    message = b"PALLET 3 DONE".ljust(32, b"\0")
    assert _ask(yrc, 0x85, 1, 1, SET_SINGLE, message).status is Status.OK
    assert yrc.controller.pendant_message == "PALLET 3 DONE"


def test_a_pendant_message_past_30_characters_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x85, 1, 1, SET_SINGLE, b"X" * 31 + b"\0")
    assert answer.added_status == AddedStatus.OPERAND_RANGE


def test_control_power_time_counts_from_the_start(yrc: YaskawaRobot) -> None:
    data = _ask(yrc, 0x88, 1, 0, GET_ALL).data
    assert time.strptime(data[:16].decode(), "%Y/%m/%d %H:%M")
    assert data[16:].decode().startswith("000000:00'")


def test_other_management_times_show_zero_run(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x88, 211, 2, GET_SINGLE).data == b"000000:00'00"
    assert _ask(yrc, 0x88, 2, 0, GET_ALL).added_status == AddedStatus.INSTANCE_ERROR


def _job(name: str, line: int = 0) -> bytes:
    return name.encode().ljust(32, b"\0") + struct.pack("<i", line)


def test_select_a_job_and_read_it_back(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x87, 1, 0, SET_ALL, _job("PICK", 12)).status is Status.OK
    data = _ask(yrc, 0x73, 1, 0, GET_ALL).data
    assert data[:32].rstrip(b"\0") == b"PICK"
    assert struct.unpack("<3i", data[32:]) == (12, 0, 10_000)  # 100.00 %


def test_selecting_a_job_the_controller_lacks_is_refused(yrc: YaskawaRobot) -> None:
    answer = _ask(yrc, 0x87, 1, 0, SET_ALL, _job("NOPE"))
    assert answer.added_status == AddedStatus.NO_SUCH_JOB


def test_a_master_job_does_not_change_the_executing_job(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x87, 10, 0, SET_ALL, _job("MAIN")).status is Status.OK
    assert yrc.controller.master_jobs == {0: "MAIN"}
    assert _ask(yrc, 0x73, 1, 1, GET_SINGLE).data == bytes(32)


def test_a_sub_task_runs_no_job(yrc: YaskawaRobot) -> None:
    _ask(yrc, 0x87, 1, 0, SET_ALL, _job("PICK"))
    assert _ask(yrc, 0x73, 2, 1, GET_SINGLE).data == bytes(32)


def test_starting_without_an_executing_job_is_refused(yrc: YaskawaRobot) -> None:
    answer = _set(yrc, 0x86, 1, 1)
    assert answer.added_status == AddedStatus.NO_EXECUTING_JOB


def test_starting_a_job_while_held_is_refused(yrc: YaskawaRobot) -> None:
    _ask(yrc, 0x87, 1, 0, SET_ALL, _job("PICK"))
    _set(yrc, 0x83, 1, 1)
    assert _set(yrc, 0x86, 1, 1).added_status == AddedStatus.COMMAND_HOLD


def test_a_started_job_finishes_at_once(yrc: YaskawaRobot) -> None:
    _ask(yrc, 0x87, 1, 0, SET_ALL, _job("PICK"))
    assert _set(yrc, 0x86, 1, 1).status is Status.OK
    assert _ints(_ask(yrc, 0x72, 1, 1, GET_SINGLE))[0] & 0x08 == 0  # not running


def test_robot_control_takes_only_requests(yrc: YaskawaRobot) -> None:
    client = UdpMessageTransport("127.0.0.1", yrc.hses_port, timeout=2)
    try:
        ack = Request(division=Division.ROBOT, request_id=1, command=0x72, instance=1,
                      attribute=0, service=GET_ALL, ack=True)
        answer = parse_answer(client.request(encode_request(ack)))
    finally:
        client.close()
    assert answer.added_status == AddedStatus.ACK_ERROR


def test_a_named_job_is_a_jbi_file(yrc: YaskawaRobot) -> None:
    assert yrc.controller.jobs == {"MAIN", "PICK"}
    assert yrc.controller.files["PICK.JBI"].startswith(b"/JOB\r\n//NAME PICK\r\n")


def test_an_io_write_may_carry_its_byte_in_a_32_bit_word(yrc: YaskawaRobot) -> None:
    """moto-hses sends the IO byte and three zeros."""
    assert _ask(yrc, 0x78, 2701, 1, SET_SINGLE, b"\x07\x00\x00\x00").status is Status.OK
    assert _ask(yrc, 0x78, 2701, 1, GET_SINGLE).data == b"\x07"


@pytest.mark.parametrize("attribute", [0, 1])
def test_a_single_variable_takes_attribute_0_or_1(yrc: YaskawaRobot, attribute: int) -> None:
    """moto-hses addresses single variables with attribute 0."""
    write = _ask(yrc, 0x7B, 1, attribute, SET_SINGLE, struct.pack("<h", 5))
    assert write.status is Status.OK
    assert _ask(yrc, 0x7B, 1, attribute, GET_SINGLE).data == struct.pack("<h", 5)


def test_the_software_version_tells_the_generation(yrc: YaskawaRobot) -> None:
    """YMConnect reads the controller generation from the version's first two letters."""
    assert _ask(yrc, 0x89, 11, 1, GET_SINGLE).data.rstrip(b"\0") == b"YAS2.80.00-00"


def test_a_yrc1000_reads_system_parameters(yrc: YaskawaRobot) -> None:
    """0x39C, as YMConnect reads RS022 when it connects: type 4 (RS), group 0, number 22."""
    assert _ints(_ask(yrc, 0x39C, 0, 0, GET_ALL, struct.pack("<3i", 4, 0, 22))) == (1,)
    assert _ints(_ask(yrc, 0x39C, 0, 0, GET_ALL, struct.pack("<3i", 1, 0, 541))) == (0,)


def test_r1_is_in_the_first_motion_slot(yrc: YaskawaRobot) -> None:
    """YMConnect maps its motion slots to control groups from type 11, numbers 33 to 36."""
    slots = [_ints(_ask(yrc, 0x39C, 0, 0, GET_ALL, struct.pack("<3i", 11, 0, n)))
             for n in range(33, 37)]
    assert slots == [(1,), (0,), (0,), (0,)]


def test_a_dx200_has_no_system_parameter_read() -> None:
    options = YaskawaOptions(controller="dx200", robot="up50n", protocols=("hses",),
                             hses_port=free_port(), hses_file_port=free_port())
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        answer = _ask(m, 0x39C, 0, 0, GET_ALL, struct.pack("<3i", 4, 0, 22))
        assert answer.status is Status.UNDEFINED_COMMAND
    finally:
        m.stop()


def test_an_io_write_may_carry_its_byte_in_8_bytes(yrc: YaskawaRobot) -> None:
    """YMConnect sends the IO byte and seven zeros."""
    assert _ask(yrc, 0x78, 2701, 1, SET_SINGLE, b"\x05" + bytes(7)).status is Status.OK
    assert _ask(yrc, 0x78, 2701, 1, GET_SINGLE).data == b"\x05"


def test_a_pendant_message_may_come_unpadded(yrc: YaskawaRobot) -> None:
    assert _ask(yrc, 0x85, 1, 1, SET_SINGLE, b"YMCONNECT").status is Status.OK
    assert yrc.controller.pendant_message == "YMCONNECT"
