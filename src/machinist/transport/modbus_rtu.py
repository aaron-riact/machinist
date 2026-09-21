"""Modbus RTU framing, for a line that happens to be carried over TCP.

Modbus/TCP prefixes every PDU with an MBAP header that says how long the
PDU is, so a reader always knows where a frame ends. RTU has no such
header: the slave id leads, a CRC-16 trails, and one frame is told from
the next by 3.5 character times of silence on the wire. Carry that line
over a TCP stream and the silence is gone, so a frame's extent has to
come from the only other thing that knows it -- the function code.

That is what this module is: the length rule per function code, the CRC,
and a parse of the bytes into one request object per function. Nothing
here touches a socket; :class:`RtuStream` accumulates whatever arrived
and hands back the requests that are complete.

Only the holding-register functions are framed, because a holding
register is all a :class:`~machinist.transport.registers.RegisterPort`
models. A function code we cannot measure is not just unsupported, it
leaves the stream unreadable -- see :class:`UnframeableError`.
"""

from __future__ import annotations

import struct
from collections.abc import Sequence
from dataclasses import dataclass
from typing import ClassVar

__all__ = [
    "EX_ILLEGAL_DATA_ADDRESS",
    "EX_ILLEGAL_FUNCTION",
    "FUNC_READ_HOLDING",
    "FUNC_WRITE_MULTIPLE",
    "FUNC_WRITE_SINGLE",
    "ReadHolding",
    "RtuRequest",
    "RtuStream",
    "UnframeableError",
    "WriteMultiple",
    "WriteSingle",
    "crc16",
    "exception_reply",
    "frame_length",
    "framed",
]

FUNC_READ_HOLDING = 0x03
FUNC_WRITE_SINGLE = 0x06
FUNC_WRITE_MULTIPLE = 0x10

EX_ILLEGAL_FUNCTION = 0x01
EX_ILLEGAL_DATA_ADDRESS = 0x02

#: slave id, function, address, count/value, CRC -- the fixed-size requests.
_FIXED_LENGTH = 8
#: Bytes of a write-multiple request that precede its variable-length data.
_WRITE_MULTIPLE_HEADER = 7


class UnframeableError(ValueError):
    """Raised for a function code whose frame length we cannot work out.

    This is worse than an unsupported function. Without the length there
    is no way to find where the next frame starts, so a reader that meets
    one has lost the stream and can only answer and hang up.
    """


def crc16(data: bytes) -> int:
    """The CRC-16/MODBUS of *data*, transmitted least-significant byte first."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def framed(payload: bytes) -> bytes:
    """Append the CRC that makes *payload* a frame on the line."""
    return payload + struct.pack("<H", crc16(payload))


def exception_reply(slave_id: int, function: int, code: int) -> bytes:
    """The frame a slave sends when it refuses a request."""
    return framed(bytes([slave_id, function | 0x80, code]))


def frame_length(buffer: bytes) -> int | None:
    """How many bytes the request starting at *buffer* occupies.

    ``None`` means not enough has arrived to tell yet.
    """
    if len(buffer) < 2:
        return None
    function = buffer[1]
    if function in (FUNC_READ_HOLDING, FUNC_WRITE_SINGLE):
        return _FIXED_LENGTH
    if function == FUNC_WRITE_MULTIPLE:
        if len(buffer) < _WRITE_MULTIPLE_HEADER:
            return None
        return _WRITE_MULTIPLE_HEADER + buffer[6] + 2
    raise UnframeableError(f"cannot measure a frame for function 0x{function:02X}")


@dataclass(frozen=True, slots=True)
class _Request:
    """What every request carries: who it is addressed to."""

    FUNCTION: ClassVar[int]

    slave_id: int

    def exception(self, code: int) -> bytes:
        """The refusal frame for this request."""
        return exception_reply(self.slave_id, self.FUNCTION, code)


@dataclass(frozen=True, slots=True)
class ReadHolding(_Request):
    """Read *count* holding registers from *address*."""

    FUNCTION: ClassVar[int] = FUNC_READ_HOLDING

    address: int
    count: int

    def reply(self, values: Sequence[int]) -> bytes:
        payload = b"".join(struct.pack(">H", value & 0xFFFF) for value in values)
        return framed(bytes([self.slave_id, self.FUNCTION, len(payload)]) + payload)


@dataclass(frozen=True, slots=True)
class WriteSingle(_Request):
    """Write one holding register, which the slave echoes back verbatim."""

    FUNCTION: ClassVar[int] = FUNC_WRITE_SINGLE

    address: int
    value: int

    @property
    def values(self) -> tuple[int, ...]:
        return (self.value,)

    def reply(self) -> bytes:
        return framed(
            bytes([self.slave_id, self.FUNCTION]) + struct.pack(">HH", self.address, self.value)
        )


@dataclass(frozen=True, slots=True)
class WriteMultiple(_Request):
    """Write consecutive holding registers; the slave echoes address and count."""

    FUNCTION: ClassVar[int] = FUNC_WRITE_MULTIPLE

    address: int
    values: tuple[int, ...]

    def reply(self) -> bytes:
        return framed(
            bytes([self.slave_id, self.FUNCTION])
            + struct.pack(">HH", self.address, len(self.values))
        )


RtuRequest = ReadHolding | WriteSingle | WriteMultiple


class RtuStream:
    """The bytes arriving from one master, handed back as whole requests.

    A frame whose CRC does not check out is dropped the way a real slave
    drops it -- silently, and without losing the reader's place, because
    the length came from the function code rather than from the CRC.
    """

    def __init__(self) -> None:
        self._buffer = bytearray()

    def feed(self, data: bytes) -> list[RtuRequest]:
        """Add *data* to the line and return every request now complete."""
        self._buffer.extend(data)
        requests: list[RtuRequest] = []
        while True:
            length = frame_length(bytes(self._buffer))
            if length is None or len(self._buffer) < length:
                return requests
            frame = bytes(self._buffer[:length])
            del self._buffer[:length]
            request = _parse(frame)
            if request is not None:
                requests.append(request)


def _parse(frame: bytes) -> RtuRequest | None:
    """Turn one complete frame into a request, or ``None`` if its CRC is wrong."""
    if crc16(frame) != 0:
        return None
    slave_id, function = frame[0], frame[1]
    if function == FUNC_READ_HOLDING:
        address, count = struct.unpack(">HH", frame[2:6])
        return ReadHolding(slave_id=slave_id, address=address, count=count)
    if function == FUNC_WRITE_SINGLE:
        address, value = struct.unpack(">HH", frame[2:6])
        return WriteSingle(slave_id=slave_id, address=address, value=value)
    address, count = struct.unpack(">HH", frame[2:6])
    if frame[6] != count * 2:
        return None
    values = struct.unpack(f">{count}H", frame[7 : 7 + frame[6]])
    return WriteMultiple(slave_id=slave_id, address=address, values=values)
