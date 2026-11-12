"""The cell's ladder: rungs parsed from the controller's instruction list, and run
after the writes that can change what they read."""

from __future__ import annotations

import struct
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.ladder import (
    BitCopy,
    ByteCopy,
    Ladder,
    RegisterMove,
    parse_rung,
)
from machinist.transport.message import UdpMessageTransport
from machinist.transport.yaskawa_hses import Answer, Division, Request, encode_request, parse_answer

from ..conftest import free_port, wait_running

GET_SINGLE, SET_SINGLE = 0x0E, 0x10


@pytest.mark.parametrize(("text", "rung"), [
    ("GSTR #10010 GOUT #30010", ByteCopy(10010, 30010)),
    ("STR #27012 OUT #30134", BitCopy(27012, 30134, inverted=False)),
    ("STR-NOT  #27012   OUT #30134", BitCopy(27012, 30134, inverted=True)),
    ("MOV M020,#00030", RegisterMove(20, 30)),
    ("MOV M560 #30140", RegisterMove(560, 30140)),
])
def test_a_rung_parses_from_its_instructions(text: str, rung: object) -> None:
    assert parse_rung(text) == rung


@pytest.mark.parametrize("text", [
    "GSTR #10011 GOUT #30010",   # not the first signal of a byte
    "STR #27018 OUT #30134",     # a byte has bits 0 to 7
    "AND #27012 OUT #30134",     # not a rung the ladder knows
    "GSTR #10010",               # no output
])
def test_a_rung_it_cannot_run_is_refused(text: str) -> None:
    with pytest.raises(ValueError, match=r"rung|byte"):
        parse_rung(text)


def test_rungs_run_in_order_on_bytes_bits_and_registers() -> None:
    ladder = Ladder.parse([
        "GSTR #10010 GOUT #30010",
        "STR #27012 OUT #30147",
        "STR-NOT #27012 OUT #30146",
        "MOV M560 #30150",
    ])
    io = {10010: 0b1001, 27010: 0b100, 30140: 0b0100_0000}
    ladder.scan(io, {560: 0x1234})
    assert io[30010] == 0b1001
    assert io[30140] == 0b1000_0000      # bit 7 copied on, bit 6 inverted off
    assert (io[30150], io[30160]) == (0x34, 0x12)


def test_a_bad_rung_fails_when_the_scene_loads() -> None:
    with pytest.raises(ValueError, match="rung"):
        YaskawaOptions(controller="dx200", robot="up50n", ladder=("OUT #30010",))


@pytest.fixture
def cell() -> Iterator[YaskawaRobot]:
    options = YaskawaOptions(
        controller="dx200", robot="up50n", protocols=("hses",), hses_port=free_port(),
        ladder_copies={27010: 30040},
        ladder=("STR-NOT #27020 OUT #30050", "MOV M12 #30060"),
    )
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


def _ask(robot: YaskawaRobot, command: int, instance: int, service: int,
         data: bytes = b"") -> Answer:
    client = UdpMessageTransport("127.0.0.1", robot.hses_port, timeout=2)
    try:
        return parse_answer(client.request(encode_request(Request(
            division=Division.ROBOT, request_id=1, command=command, instance=instance,
            attribute=1, service=service, data=data))))
    finally:
        client.close()


def _byte(robot: YaskawaRobot, signal: int) -> int:
    return _ask(robot, 0x78, signal // 10, GET_SINGLE).data[0]


def test_a_network_input_write_reaches_the_outputs_its_rungs_name(cell: YaskawaRobot) -> None:
    assert _byte(cell, 30050) == 0b1        # inverted: #27020 off shows on, once scanned
    _ask(cell, 0x78, 2701, SET_SINGLE, bytes([0b101]))
    _ask(cell, 0x78, 2702, SET_SINGLE, bytes([0b1]))
    assert _byte(cell, 30040) == 0b101
    assert _byte(cell, 30050) == 0b0


def test_a_register_write_reaches_the_bytes_it_moves_to(cell: YaskawaRobot) -> None:
    _ask(cell, 0x79, 12, SET_SINGLE, struct.pack("<H", 0x0203))
    assert (_byte(cell, 30060), _byte(cell, 30070)) == (0x03, 0x02)
