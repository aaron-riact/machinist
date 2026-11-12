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
    BlockMove,
    Contact,
    Ladder,
    LoadByte,
    Out,
    RegisterMove,
    StoreByte,
    parse_instructions,
    parse_rung,
    split_rungs,
)
from machinist.transport.message import UdpMessageTransport
from machinist.transport.yaskawa_hses import Answer, Division, Request, encode_request, parse_answer

from ..conftest import free_port, wait_running

GET_SINGLE, SET_SINGLE = 0x0E, 0x10


@pytest.mark.parametrize(("text", "rung"), [
    ("GSTR #10010 GOUT #30010", (LoadByte(10010), StoreByte(30010))),
    ("STR #27012 OUT #30134", (Contact("STR", 27012), Out(30134))),
    ("STR-NOT  #27012   OUT #30134", (Contact("STR", 27012, inverted=True), Out(30134))),
    ("MOV M020,#00030", (RegisterMove(20, 30),)),
    ("MOV M560 #30140", (RegisterMove(560, 30140),)),
    ("STR #70010 BMOV #20030,4,#00050", (Contact("STR", 70010), BlockMove(20030, 4, 50))),
    ("STR #70020 AND #82010 OR-NOT #70020 OUT #70030",
     (Contact("STR", 70020), Contact("AND", 82010), Contact("OR", 70020, inverted=True),
      Out(70030))),
])
def test_a_rung_parses_from_its_instructions(text: str, rung: object) -> None:
    assert parse_rung(text) == rung


@pytest.mark.parametrize(("text", "complaint"), [
    ("GSTR #10011 GOUT #30010", "not the first signal of a byte"),
    ("STR #27018 OUT #30134", "a byte holds bits 0 to 7"),
    ("STR #27012 XOR #30134", "is not one the emulator runs"),
    ("GSTR #10010", "ends without an output"),
])
def test_a_rung_it_cannot_run_is_refused(text: str, complaint: str) -> None:
    with pytest.raises(ValueError, match=complaint):
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


#: Signals #70010-#70017 as eight inputs a to h, bit 0 to 7.
A, B, C, D, E, F, G, H = (70010 + bit for bit in range(8))


def _result(rung: str, inputs: int) -> bool:
    io = {70010: inputs}
    Ladder.parse([f"{rung} OUT #30010"]).scan(io, {})
    return bool(io[30010] & 1)


@pytest.mark.parametrize(("inputs", "on"), [
    (0b0000_0011, True),     # a and b
    (0b0000_0001, False),
    (0b0000_0100, True),     # c alone
    (0b0000_1100, False),    # c, but d inverted is off
])
def test_and_and_or_combine_contacts_in_order(inputs: int, on: bool) -> None:
    # (a AND b) OR (c AND NOT d), written without branches: a AND b, then OR-STR a branch.
    rung = f"STR #{A} AND #{B} STR #{C} AND-NOT #{D} OR-STR"
    assert _result(rung, inputs) is on


def test_or_str_joins_every_branch_then_the_rest_applies_to_all() -> None:
    """Four branches, three OR-STR, then AND-NOT."""
    rung = (f"STR #{A} STR #{B} STR #{C} STR #{D} OR-STR OR-STR OR-STR AND-NOT #{E}")
    assert _result(rung, 0b0000_1000) is True
    assert _result(rung, 0b0001_1000) is False
    assert _result(rung, 0) is False


def test_and_str_needs_both_branches() -> None:
    rung = f"STR #{A} OR #{B} STR #{C} OR #{D} AND-STR"
    assert _result(rung, 0b0000_0101) is True
    assert _result(rung, 0b0000_0011) is False


def test_a_rung_with_several_outputs_gives_each_the_result() -> None:
    io = {70010: 1}
    Ladder.parse([f"STR #{A} OUT #30010 OUT #30011"]).scan(io, {})
    assert io[30010] == 0b11


def test_a_self_inverting_relay_turns_over_on_each_scan() -> None:
    """A ladder's own clock: STR-NOT #70110 OUT #70110."""
    ladder = Ladder.parse(["STR-NOT #70110 OUT #70110"])
    io: dict[int, int] = {}
    ladder.scan(io, {})
    assert io[70110] == 0b0000_0001
    ladder.scan(io, {})
    assert io[70110] == 0


def test_bmov_and_mov_run_only_while_their_contact_is_on() -> None:
    ladder = Ladder.parse([f"STR #{A} BMOV #20030,2,#00050", f"STR #{B} MOV M020,#00030"])
    io = {20030: 7, 20040: 8}
    ladder.scan(io, {20: 0x0102})
    assert 50 not in io
    assert 30 not in io
    io[70010] = 0b11
    ladder.scan(io, {20: 0x0102})
    assert (io[50], io[60]) == (7, 8)
    assert (io[30], io[40]) == (0x02, 0x01)


def test_an_instruction_list_splits_into_rungs_at_each_new_start() -> None:
    instructions = parse_instructions(
        f"STR #{A} OUT #30010 OUT #30011 STR #{B} OR #{C} OUT #30012 GSTR #10010 GOUT #30010")
    assert [len(rung) for rung in split_rungs(instructions)] == [3, 3, 2]


def test_a_bad_rung_fails_when_the_scene_loads() -> None:
    with pytest.raises(ValueError, match="ladder instruction"):
        YaskawaOptions(controller="dx200", robot="up50n", ladder=("SET #30010",))


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
