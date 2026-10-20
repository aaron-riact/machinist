"""HSES packets against the DX200 manual (HW1481977), chapter 3.1 and 3.2."""

from __future__ import annotations

import pytest

from machinist.transport.yaskawa_hses import (
    AddedStatus,
    Answer,
    Division,
    FrameError,
    Request,
    Status,
    encode_answer,
    encode_request,
    parse_answer,
    parse_request,
    request_id_of,
)

#: 3.2.1: the status read request, as the manual draws it.
STATUS_READ = (
    b"YERC" + b"\x20\x00" + b"\x00\x00" + b"\x03\x01\x00\x00" + b"\x00\x00\x00\x00"
    + b"99999999" + b"\x72\x00" + b"\x01\x00" + b"\x00\x01" + b"\x00\x00"
)


def test_a_request_is_a_32_byte_header_and_its_data() -> None:
    request = Request(
        division=Division.ROBOT, request_id=0, command=0x72, instance=1, attribute=0, service=1
    )
    assert encode_request(request) == STATUS_READ
    assert parse_request(STATUS_READ) == request


def test_an_answer_echoes_the_request_id_and_marks_its_last_block() -> None:
    """3.2.1: ACK 1, block 0x8000_0000, service 0x81, status 0."""
    answer = encode_answer(Answer(
        division=Division.ROBOT, request_id=7, service=0x81, data=b"\x01\x02\x03\x04",
    ))
    assert answer[:24] == (
        b"YERC\x20\x00\x04\x00\x03\x01\x01\x07\x00\x00\x00\x80" + b"99999999"
    )
    assert answer[24:32] == b"\x81\x00\x00\x00\x00\x00\x00\x00"
    assert answer[32:] == b"\x01\x02\x03\x04"


def test_an_answer_with_an_added_status_says_its_size() -> None:
    """3.1: status 0x1f, added status size 1, then the code."""
    answer = encode_answer(Answer(
        division=Division.ROBOT, request_id=1, service=0x90,
        status=Status.VENDOR, added_status=AddedStatus.SERVO_OFF,
    ))
    assert answer[24:32] == b"\x90\x1f\x01\x00\x70\x20\x00\x00"
    assert parse_answer(answer).added_status == AddedStatus.SERVO_OFF


def test_a_request_carries_its_data() -> None:
    request = Request(division=Division.ROBOT, request_id=2, command=0x79, instance=3,
                      attribute=1, service=0x10, data=b"\x2a\x00")
    datagram = encode_request(request)
    assert datagram[6:8] == b"\x02\x00"
    assert parse_request(datagram).data == b"\x2a\x00"


@pytest.mark.parametrize(
    ("datagram", "added_status"),
    [
        (b"YERC" + bytes(10), AddedStatus.HEADER_SIZE_ERROR),
        (b"XERC" + STATUS_READ[4:], AddedStatus.IDENTIFIER_ERROR),
        (STATUS_READ[:4] + b"\x10\x00" + STATUS_READ[6:], AddedStatus.HEADER_SIZE_ERROR),
        (STATUS_READ[:6] + b"\x04\x00" + STATUS_READ[8:], AddedStatus.FRAME_SIZE_MISMATCH),
        (STATUS_READ[:10] + b"\x02" + STATUS_READ[11:], AddedStatus.ACK_ERROR),
        (STATUS_READ[:9] + b"\x05" + STATUS_READ[10:], AddedStatus.DIVISION_ERROR),
    ],
)
def test_a_malformed_request_is_refused(datagram: bytes, added_status: AddedStatus) -> None:
    with pytest.raises(FrameError) as caught:
        parse_request(datagram)
    assert caught.value.added_status == added_status


def test_the_request_id_survives_a_malformed_request() -> None:
    assert request_id_of(b"XERC" + STATUS_READ[4:11] + b"\x09" + STATUS_READ[12:]) == 9
    assert request_id_of(b"YE") == 0


def test_a_hosts_ack_carries_its_block() -> None:
    """3.2.3: in a file transfer the host's ACKs have ACK 1 and a block number."""
    ack = Request(division=Division.FILE, request_id=2, command=0, instance=0, attribute=0,
                  service=0x16, ack=True, block=0x8000_0003)
    datagram = encode_request(ack)
    assert datagram[8:16] == b"\x03\x02\x01\x02\x03\x00\x00\x80"
    assert parse_request(datagram) == ack
