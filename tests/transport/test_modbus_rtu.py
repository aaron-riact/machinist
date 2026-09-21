"""RTU framing: where a frame ends, and whether its CRC stands up."""

from __future__ import annotations

import pytest

from machinist.transport.modbus_rtu import (
    EX_ILLEGAL_DATA_ADDRESS,
    ReadHolding,
    RtuStream,
    UnframeableError,
    WriteMultiple,
    WriteSingle,
    crc16,
    frame_length,
    framed,
)


def test_crc_matches_the_published_check_value() -> None:
    """CRC-16/MODBUS is defined by its check value over b"123456789"."""
    assert crc16(b"123456789") == 0x4B37


def test_a_framed_payload_checks_out_to_zero() -> None:
    """Running the CRC over a frame including its own CRC leaves nothing."""
    assert crc16(framed(b"\x41\x03\x01\x0b\x00\x02")) == 0


def test_read_and_write_single_are_eight_bytes() -> None:
    assert frame_length(b"\x41\x03") == 8
    assert frame_length(b"\x41\x06") == 8


def test_write_multiple_is_measured_by_its_byte_count() -> None:
    header = b"\x41\x10\x00\x00\x00\x02\x04"
    assert frame_length(header) == 13


def test_a_frame_too_short_to_measure_is_not_measured_yet() -> None:
    assert frame_length(b"") is None
    assert frame_length(b"\x41") is None
    assert frame_length(b"\x41\x10\x00\x00\x00\x02") is None


def test_an_unknown_function_leaves_the_stream_unreadable() -> None:
    with pytest.raises(UnframeableError, match="0x04") as caught:
        frame_length(b"\x41\x04\x00\x00")

    assert (caught.value.slave_id, caught.value.function) == (0x41, 0x04)


def test_a_read_request_parses() -> None:
    stream = RtuStream()

    requests = stream.feed(framed(b"\x41\x03\x01\x0b\x00\x02"))

    assert requests == [ReadHolding(slave_id=0x41, address=0x010B, count=2)]


def test_a_write_single_request_parses() -> None:
    stream = RtuStream()

    requests = stream.feed(framed(b"\x41\x06\x00\x02\x00\x01"))

    assert requests == [WriteSingle(slave_id=0x41, address=2, value=1)]
    assert requests[0].values == (1,)  # type: ignore[union-attr]


def test_a_write_multiple_request_parses() -> None:
    stream = RtuStream()

    requests = stream.feed(framed(b"\x41\x10\x00\x00\x00\x02\x04\x01\x90\x04\x4c"))

    assert requests == [WriteMultiple(slave_id=0x41, address=0, values=(400, 1100))]


def test_a_request_split_across_packets_waits_for_the_rest() -> None:
    """Nothing guarantees a master writes a whole frame in one send."""
    stream = RtuStream()
    frame = framed(b"\x41\x03\x01\x0b\x00\x02")

    for byte in frame[:-1]:
        assert stream.feed(bytes([byte])) == []

    assert stream.feed(frame[-1:]) == [ReadHolding(slave_id=0x41, address=0x010B, count=2)]


def test_two_requests_in_one_packet_both_come_back() -> None:
    stream = RtuStream()
    frame = framed(b"\x41\x03\x01\x0b\x00\x02")

    assert stream.feed(frame + frame) == [
        ReadHolding(slave_id=0x41, address=0x010B, count=2),
        ReadHolding(slave_id=0x41, address=0x010B, count=2),
    ]


def test_a_bad_crc_is_dropped_without_losing_the_next_frame() -> None:
    """A slave ignores a corrupt frame; the length came from the function code."""
    stream = RtuStream()
    corrupt = framed(b"\x41\x03\x01\x0b\x00\x02")[:-1] + b"\x00"
    good = framed(b"\x41\x03\x01\x0b\x00\x01")

    assert stream.feed(corrupt) == []
    assert stream.feed(good) == [ReadHolding(slave_id=0x41, address=0x010B, count=1)]


def test_a_write_multiple_whose_byte_count_lies_is_dropped() -> None:
    stream = RtuStream()

    assert stream.feed(framed(b"\x41\x10\x00\x00\x00\x01\x04\x01\x90\x04\x4c")) == []


def test_a_read_reply_carries_the_values_big_endian() -> None:
    request = ReadHolding(slave_id=0x41, address=0x010B, count=2)

    assert request.reply([500, 1]) == framed(b"\x41\x03\x04\x01\xf4\x00\x01")


def test_a_write_single_reply_echoes_the_request() -> None:
    request = WriteSingle(slave_id=0x41, address=2, value=1)

    assert request.reply() == framed(b"\x41\x06\x00\x02\x00\x01")


def test_a_write_multiple_reply_echoes_address_and_count() -> None:
    request = WriteMultiple(slave_id=0x41, address=0, values=(400, 1100))

    assert request.reply() == framed(b"\x41\x10\x00\x00\x00\x02")


def test_a_refusal_sets_the_high_bit_of_the_function() -> None:
    request = ReadHolding(slave_id=0x41, address=0x9999, count=1)

    assert request.exception(EX_ILLEGAL_DATA_ADDRESS) == framed(b"\x41\x83\x02")
