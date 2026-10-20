"""The HSES server against the DX200 manual (HW1481977), chapter 3.3."""

from __future__ import annotations

import socket
import struct
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
        hses_port=free_port(), start_joints_deg=(0, 0, 0, 0, -90, 0),
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
