"""Packets of Yaskawa's High-Speed Ethernet Server (HSES).

Reference: *DX200 Options Instructions for High-Speed Ethernet Server
Function* (HW1481977), chapter 3.1. Every packet is a 32-byte header and a
data part of up to 479 bytes, in one UDP datagram::

    0   "YERC"                     identifier
    4   header size (0x20)         2 bytes
    6   data part size             2 bytes
    8   reserve 1 (3)              1 byte
    9   processing division        1 byte: 1 robot control, 2 file control
    10  ACK                        1 byte: 0 request, 1 anything else
    11  request ID                 1 byte, echoed in the answer
    12  block number               4 bytes: 0 in a request; the last answer
                                   block has 0x8000_0000 added
    16  reserve 2 ("99999999")     8 bytes
    24  sub-header                 8 bytes, request and answer differ:
        request:  command 2, instance 2, attribute 1, service 1, padding 2
        answer:   service 1 (the request's + 0x80), status 1,
                  added status size 1, padding 1, added status 2, padding 2

The manual does not say which byte order the multi-byte fields use. HSES
follows CIP (EtherNet/IP), which is little-endian, and so does this module;
check it against a real controller.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum

IDENTIFIER = b"YERC"
HEADER_SIZE = 0x20
RESERVE_1 = 3
RESERVE_2 = b"99999999"
MAX_DATA = 479
LAST_BLOCK = 0x8000_0000
#: The answer's service is the request's with this added.
ANSWER_SERVICE = 0x80

_HEADER = struct.Struct("<4sHHBBBBI8s")
_REQUEST_SUB = struct.Struct("<HHBBH")
_ANSWER_SUB = struct.Struct("<BBBBHH")


class Division(IntEnum):
    """The processing division: what kind of command a packet carries."""

    ROBOT = 1
    FILE = 2


class Status(IntEnum):
    """An answer's status (chapter 4)."""

    OK = 0x00
    UNDEFINED_COMMAND = 0x08
    INVALID_ELEMENT = 0x09
    #: The added status (chapter 5) says what went wrong.
    VENDOR = 0x1F
    NO_SUCH_ARRAY_NUMBER = 0x28


class AddedStatus(IntEnum):
    """Added status codes (chapter 5) the emulator answers with."""

    OPERAND_RANGE = 0x1012
    COMMAND_HOLD = 0x2050
    ALARM = 0x2060
    SERVO_OFF = 0x2070
    USER_COORDINATE_NOT_TAUGHT = 0x3350
    NO_SUCH_JOB = 0x4040
    NO_EXECUTING_JOB = 0x4060
    SERVO_CANNOT_TURN_ON = 0x3450
    UNDEFINED_COMMAND = 0xA000
    INSTANCE_ERROR = 0xA001
    ATTRIBUTE_ERROR = 0xA002
    DATA_SIZE_ERROR = 0xB003
    OUTSIDE_THE_DATA = 0xB004
    CONTROL_GROUP_ERROR = 0xB008
    SPEED_ERROR = 0xB009
    COORDINATE_SYSTEM_ERROR = 0xB00B
    TOOL_NUMBER_ERROR = 0xB00D
    USER_NUMBER_ERROR = 0xB00E
    FILE_NOT_FOUND = 0xE2B3
    BLOCK_NUMBER_ERROR = 0xE4A1
    ACK_ERROR = 0xE4A2
    DIVISION_ERROR = 0xE4A3
    HEADER_SIZE_ERROR = 0xE4A5
    IDENTIFIER_ERROR = 0xE4A6
    FRAME_SIZE_MISMATCH = 0xE4A7


@dataclass(frozen=True, slots=True)
class Request:
    """A packet from the host. Its sub-header is a request's; in a file transfer the
    host's later packets (its data blocks and its ACKs) carry *ack* and a *block*."""

    division: Division
    request_id: int
    command: int
    instance: int
    attribute: int
    service: int
    data: bytes = b""
    ack: bool = False
    block: int = 0


@dataclass(frozen=True, slots=True)
class Answer:
    division: Division
    request_id: int
    service: int
    status: Status = Status.OK
    added_status: int | None = None
    data: bytes = b""
    block: int = LAST_BLOCK


class FrameError(Exception):
    """A datagram that is not a well-formed request; answered with *added_status*."""

    def __init__(self, added_status: AddedStatus, reason: str) -> None:
        super().__init__(reason)
        self.added_status = added_status


def parse_request(datagram: bytes) -> Request:
    """The host's packet a datagram carries. Raises :class:`FrameError` for anything else."""
    if len(datagram) < HEADER_SIZE:
        raise FrameError(AddedStatus.HEADER_SIZE_ERROR, f"{len(datagram)} bytes")
    identifier, header_size, data_size, _reserve, division, ack, request_id, block, _r2 = (
        _HEADER.unpack_from(datagram)
    )
    if identifier != IDENTIFIER:
        raise FrameError(AddedStatus.IDENTIFIER_ERROR, f"identifier {identifier!r}")
    if header_size != HEADER_SIZE:
        raise FrameError(AddedStatus.HEADER_SIZE_ERROR, f"header size {header_size}")
    if data_size != len(datagram) - HEADER_SIZE:
        raise FrameError(
            AddedStatus.FRAME_SIZE_MISMATCH,
            f"data size {data_size} for {len(datagram) - HEADER_SIZE} bytes",
        )
    if ack not in (0, 1):
        raise FrameError(AddedStatus.ACK_ERROR, f"ACK {ack}")
    if division not in tuple(Division):
        raise FrameError(AddedStatus.DIVISION_ERROR, f"processing division {division}")
    command, instance, attribute, service, _pad = _REQUEST_SUB.unpack_from(
        datagram, _HEADER.size
    )
    return Request(
        division=Division(division),
        request_id=request_id,
        command=command,
        instance=instance,
        attribute=attribute,
        service=service,
        data=bytes(datagram[HEADER_SIZE:]),
        ack=ack == 1,
        block=block,
    )


def request_id_of(datagram: bytes) -> int:
    """The request ID of a datagram, as far as it gets; 0 if it is too short."""
    return datagram[11] if len(datagram) > 11 else 0


def encode_request(request: Request) -> bytes:
    """A request's datagram (what a host sends)."""
    header = _HEADER.pack(
        IDENTIFIER, HEADER_SIZE, len(request.data), RESERVE_1, request.division,
        int(request.ack), request.request_id, request.block, RESERVE_2,
    )
    sub = _REQUEST_SUB.pack(
        request.command, request.instance, request.attribute, request.service, 0
    )
    return header + sub + request.data


def encode_answer(answer: Answer) -> bytes:
    """An answer's datagram (what the controller sends)."""
    if len(answer.data) > MAX_DATA:
        raise ValueError(f"{len(answer.data)} bytes of data; at most {MAX_DATA} fit")
    header = _HEADER.pack(
        IDENTIFIER, HEADER_SIZE, len(answer.data), RESERVE_1, answer.division,
        1, answer.request_id, answer.block, RESERVE_2,
    )
    added_size = 0 if answer.added_status is None else 1
    sub = _ANSWER_SUB.pack(
        answer.service, answer.status, added_size, 0, answer.added_status or 0, 0
    )
    return header + sub + answer.data


def parse_answer(datagram: bytes) -> Answer:
    """The answer a datagram carries (what a host reads)."""
    _id, _hs, data_size, _r, division, _ack, request_id, block, _r2 = _HEADER.unpack_from(
        datagram
    )
    service, status, added_size, _p, added, _p2 = _ANSWER_SUB.unpack_from(datagram, _HEADER.size)
    return Answer(
        division=Division(division),
        request_id=request_id,
        service=service,
        status=Status(status),
        added_status=added if added_size else None,
        data=bytes(datagram[HEADER_SIZE : HEADER_SIZE + data_size]),
        block=block,
    )
