"""The cell's ladder: rungs parsed from the controller's instruction list, and run
after the writes that can change what they read."""

from __future__ import annotations

import struct
import time
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.ladder import (
    BlockMove,
    Contact,
    Ladder,
    LadderMemory,
    LoadByte,
    Out,
    RegisterMove,
    StoreByte,
    parse_instructions,
    parse_ladder_file,
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


#: A CIOPRG.LST in the layout a DX200 saves it in; the rungs are made up.
CIOPRG = "\r\n".join([
    "/CIOPRG DN", "//NAME STANDARD", "//ATTR SC", "//CIOAP GENERAL", "//$M",
    "1,2,0,0,0,0,0,0,0,0", *["0,0,0,0,0,0,0,0,0,0"] * 6, "0,0,0,0,0,0,0,0,0,9",
    "//KEY 0",
    "PART 1", "STR-NOT #70110", "OUT #70110",
    "PART 2", "STR #20020", "OUT #00020", "GSTR #10020", "GOUT #30020",
    "STR #70110", "MOV M920,#00030",
    "END",
    "//ALARM1", "", "Example alarm",
])


def test_a_ladder_file_gives_its_header_registers_and_its_rungs() -> None:
    saved = parse_ladder_file(CIOPRG)
    assert saved.registers[920] == 1
    assert saved.registers[921] == 2
    assert saved.registers[999] == 9
    assert len(saved.registers) == 80
    assert saved.ladder.rungs == (
        (Contact("STR", 70110, inverted=True), Out(70110)),
        (Contact("STR", 20020), Out(20)),
        (LoadByte(10020), StoreByte(30020)),
        (Contact("STR", 70110), RegisterMove(920, 30)),
    )


def test_a_ladder_file_with_an_instruction_it_cannot_run_is_refused() -> None:
    with pytest.raises(ValueError, match="'XOR #20020' is not one the emulator runs"):
        parse_ladder_file(CIOPRG.replace("STR #20020", "XOR #20020"))


# --- Timers and counters, as the DX200 Concurrent I/O manual's examples run them ----


def _timer_scan(ladder: Ladder, io: dict[int, int], registers: dict[int, float],
                memory: LadderMemory, now: float, *, start: bool) -> bool:
    io[70010] = int(start)
    ladder.scan(io, registers, memory, now)
    return bool(io.get(70100, 0) & 1)


def test_a_timer_turns_its_output_on_once_its_input_has_been_on_for_its_time() -> None:
    """11.2.12: STR #70010, TMR M010,100, OUT #70100: 10.0 s after the input."""
    ladder = Ladder.parse(["STR #70010 TMR M010,100 OUT #70100"])
    io: dict[int, int] = {}
    registers: dict[int, float] = {}
    memory = LadderMemory()
    assert not _timer_scan(ladder, io, registers, memory, 0.0, start=False)
    assert registers[10] == 100
    assert not _timer_scan(ladder, io, registers, memory, 1.0, start=True)
    assert not _timer_scan(ladder, io, registers, memory, 3.5, start=True)
    assert registers[10] == 75                    # counted down every 0.1 s
    assert not _timer_scan(ladder, io, registers, memory, 10.95, start=True)
    assert _timer_scan(ladder, io, registers, memory, 11.0, start=True)
    assert registers[10] == 0
    assert not _timer_scan(ladder, io, registers, memory, 12.0, start=False)
    assert registers[10] == 100                   # back at its set value


def test_a_timer_takes_its_set_value_from_a_register() -> None:
    ladder = Ladder.parse(["STR #70010 TMR M010,M020 OUT #70100"])
    io: dict[int, int] = {}
    registers: dict[int, float] = {20: 5}
    memory = LadderMemory()
    _timer_scan(ladder, io, registers, memory, 0.0, start=True)
    assert _timer_scan(ladder, io, registers, memory, 0.5, start=True)


COUNTER = "STR #70010 STR #70011 CNT M010,3 OUT #70100"


def _count(ladder: Ladder, io: dict[int, int], registers: dict[int, float],
           memory: LadderMemory, *, counting: bool, reset: bool = False) -> bool:
    io[70010] = int(counting) | int(reset) << 1
    ladder.scan(io, registers, memory)
    return bool(io.get(70100, 0) & 1)


def test_a_counter_turns_its_output_on_at_its_third_count() -> None:
    """11.2.13: counter input #70010, reset input #70011, CNT M010,3."""
    ladder = Ladder.parse([COUNTER])
    io: dict[int, int] = {}
    registers: dict[int, float] = {}
    memory = LadderMemory()
    assert not _count(ladder, io, registers, memory, counting=False)
    assert registers[10] == 3                    # from its set value at power-on
    for expected in (2, 1):
        assert not _count(ladder, io, registers, memory, counting=True)
        assert not _count(ladder, io, registers, memory, counting=False)
        assert registers[10] == expected
    assert _count(ladder, io, registers, memory, counting=True)
    assert registers[10] == 0
    assert _count(ladder, io, registers, memory, counting=False)
    assert _count(ladder, io, registers, memory, counting=True)     # counted up: ignored
    assert not _count(ladder, io, registers, memory, counting=False, reset=True)
    assert registers[10] == 3


def test_a_counter_on_at_power_on_does_not_count_until_it_turns_on_again() -> None:
    ladder = Ladder.parse([COUNTER])
    io: dict[int, int] = {}
    registers: dict[int, float] = {10: 0}
    memory = LadderMemory()
    _count(ladder, io, registers, memory, counting=True)
    assert registers[10] == 3


def test_reset_wins_when_both_inputs_turn_on_together() -> None:
    ladder = Ladder.parse([COUNTER])
    io: dict[int, int] = {}
    registers: dict[int, float] = {}
    memory = LadderMemory()
    _count(ladder, io, registers, memory, counting=False)
    assert not _count(ladder, io, registers, memory, counting=True, reset=True)
    assert registers[10] == 3


def test_a_counter_on_a_turning_relay_pulses_once_every_100_scans() -> None:
    """#70110 turns over each scan; CNT counts it and resets on its own output, #70111."""
    ladder = Ladder.parse(["STR-NOT #70110 OUT #70110",
                           "STR #70110 STR #70111 CNT M011,50 OUT #70111"])
    io: dict[int, int] = {}
    registers: dict[int, float] = {}
    memory = LadderMemory()
    pulses = []
    for scan in range(1, 320):
        ladder.scan(io, registers, memory)
        if io[70110] >> 1 & 1:
            pulses.append(scan)
    # #70110 is on at the first scan, which does not count; it turns on again every
    # second scan after: 50 counts take 100 scans, 0.4 s at the controller's 4 ms.
    assert pulses == [101, 201, 301]


@pytest.mark.parametrize(("text", "complaint"), [
    ("STR #70010 TMR M600,10 OUT #70100", "kept in M000 to M559"),
    ("STR #70010 CNT M010,3 OUT #70100", "CNT without its counter input"),
])
def test_a_timer_or_counter_that_cannot_run_is_refused(text: str, complaint: str) -> None:
    with pytest.raises(ValueError, match=complaint):
        Ladder.parse([text]).scan({}, {})


def _scanning(ladder: tuple[str, ...], **options: object) -> YaskawaRobot:
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), YaskawaOptions(
        controller="dx200", robot="up50n", ladder=ladder, **options))
    robot.start()
    wait_running(robot)
    return robot


def test_a_started_controllers_ladder_scans_on_its_own_so_a_timer_runs_out() -> None:
    robot = _scanning(("STR #27010 TMR M010,2 OUT #30010",))
    try:
        controller = robot.controller
        controller.write_network_inputs([27010], [1])
        assert not controller.signal(30010)
        # Nothing is written after this: only the ladder's own scans can turn it on.
        controller.wait_until(lambda: controller.signal(30010), 2)
        assert controller.signal(30010)
    finally:
        robot.stop()


def test_without_its_own_scans_the_ladder_runs_only_after_writes() -> None:
    robot = _scanning(("STR-NOT #70110 OUT #70110",), ladder_scan_ms=None)
    try:
        controller = robot.controller
        before = controller.signal(70110)
        time.sleep(0.05)
        assert controller.signal(70110) == before
    finally:
        robot.stop()


def test_a_scan_time_of_zero_is_refused() -> None:
    with pytest.raises(ValueError, match="ladder_scan_ms"):
        YaskawaOptions(controller="dx200", robot="up50n", ladder_scan_ms=0)


@pytest.mark.parametrize(("text", "complaint"), [
    ("STR #70010 OR-STR OUT #70100", "OR-STR without a branch before it"),
    ("STR #70010 AND-STR OUT #70100", "AND-STR without a branch before it"),
])
def test_a_rung_that_cannot_run_is_refused_when_the_ladder_is_made(text: str,
                                                                    complaint: str) -> None:
    with pytest.raises(ValueError, match=complaint):
        Ladder.parse([text])
