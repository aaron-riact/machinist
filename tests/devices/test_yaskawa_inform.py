"""Running a job's I/O part: what starting a cell's gripper or interface job switches."""

from __future__ import annotations

import struct
from collections.abc import Iterator
from dataclasses import dataclass, field

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.inform import (
    CallJob,
    Comparison,
    Job,
    JobStoppedError,
    Jump,
    Literal,
    SetOutput,
    SignalTest,
    Unsupported,
    Variable,
    parse_instruction,
    run_job,
)
from machinist.transport.message import UdpMessageTransport
from machinist.transport.yaskawa_hses import Answer, Division, Request, encode_request, parse_answer

from ..conftest import free_port, wait_running


def _job(*lines: str) -> str:
    return "\r\n".join(["/JOB", "//NAME X", "//INST", "NOP", *lines, "END"]) + "\r\n"


@dataclass
class _Host:
    jobs: dict[str, str]
    io: dict[int, int] = field(default_factory=dict)
    variables: dict[str, dict[int, float]] = field(default_factory=dict)
    scans: int = 0

    def job_text(self, name: str) -> str | None:
        return self.jobs.get(name)

    def run_ladder(self) -> None:
        self.scans += 1


@pytest.mark.parametrize(("text", "instruction"), [
    ("DOUT OT#(D001) ON", SetOutput(Variable("D", 1), on=True)),
    ("DOUT OT#(5) OFF", SetOutput(Literal(5), on=False)),
    ("JUMP *OVER IF IN#(D021)=ON",
     Jump("OVER", SignalTest(output=False, ref=Variable("D", 21), on=True))),
    ("CALL JOB:GRIP_CLOSE IF I005=2",
     CallJob("GRIP_CLOSE", Comparison(Variable("I", 5), "=", Literal(2)))),
    ("MOVL P010 V=50.0 PL=0", Unsupported("MOVL P010 V=50.0 PL=0")),
    ("SET I204 EXPRESS I203 / 3", Unsupported("SET I204 EXPRESS I203 / 3")),
    ("'OPEN GRIPPER A", None),
    ("TIMER T=I020", None),
])
def test_a_line_parses_to_what_it_does(text: str, instruction: object) -> None:
    assert parse_instruction(text) == instruction


def test_an_output_named_by_a_variable_is_switched_and_the_ladder_runs() -> None:
    host = _Host({"OPEN": _job("DOUT OT#(D002) OFF", "DOUT OT#(D001) ON")},
                 variables={"D": {1: 1, 2: 2}})
    run_job(host, "OPEN")
    assert host.io[10010] == 0b01            # OT#1 on, OT#2 off: byte #10010
    assert host.scans == 2


def test_a_jump_on_an_input_skips_what_lies_between() -> None:
    host = _Host({"J": _job("JUMP *OVER IF IN#(9)=ON", "DOUT OT#(1) ON", "*OVER",
                            "DOUT OT#(2) ON")}, io={20: 0b1})
    run_job(host, "J")
    assert host.io[10010] == 0b10


def test_a_called_job_runs_and_the_caller_goes_on() -> None:
    host = _Host({"MAIN": _job("SET B000 3", "CALL JOB:SUB IF B000=3", "DOUT OT#(2) ON"),
                  "SUB": _job("DOUT OT#(1) ON", "RET", "DOUT OT#(8) ON")})
    run_job(host, "MAIN")
    assert host.io[10010] == 0b11


def test_a_pulse_leaves_its_output_off() -> None:
    host = _Host({"P": _job("DOUT OT#(3) ON", "PULSE OT#(3) T=0.50")})
    run_job(host, "P")
    assert host.io[10010] == 0


def test_a_group_output_sets_its_whole_byte() -> None:
    host = _Host({"G": _job("DOUT OG#(2) 2")})
    run_job(host, "G")
    assert host.io[10020] == 2


@pytest.mark.parametrize(("lines", "reason"), [
    (("MOVJ C00000 VJ=10.00",), "not emulated"),
    (("WAIT IN#(7)=ON",), "WAIT not met"),
    (("JUMP *NOWHERE",), "no label"),
    (("CALL JOB:MISSING",), "no such job"),
    (("*L", "JUMP *L"), "steps"),
])
def test_a_job_that_cannot_run_stops_with_the_reason(lines: tuple[str, ...], reason: str) -> None:
    with pytest.raises(JobStoppedError, match=reason):
        run_job(_Host({"J": _job(*lines)}), "J")


def test_what_is_jumped_past_need_not_be_emulated() -> None:
    host = _Host({"J": _job("JUMP *ON", "MOVL P001 V=50.0", "*ON", "DOUT OT#(1) ON")})
    run_job(host, "J")
    assert host.io[10010] == 1


def test_a_job_without_nop_does_nothing() -> None:
    assert Job.parse("X", "/JOB\r\n//NAME X\r\n").lines == ()


# --- Through HSES: select and start a job, read what it switched ------------


@pytest.fixture
def cell() -> Iterator[YaskawaRobot]:
    options = YaskawaOptions(
        controller="dx200", robot="up50n", protocols=("hses",), hses_port=free_port(),
        ladder=("GSTR #10010 GOUT #30010",),
    )
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.controller.files["GRIP_OPEN.JBI"] = _job("DOUT OT#(D002) OFF", "DOUT OT#(D001) ON").encode()
    m.controller.files["MOVE.JBI"] = _job("MOVJ C00000 VJ=10.00").encode()
    m.controller.variables["D"] = {1: 1, 2: 2}
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


def _ask(robot: YaskawaRobot, command: int, instance: int, attribute: int, service: int,
         data: bytes = b"") -> Answer:
    client = UdpMessageTransport("127.0.0.1", robot.hses_port, timeout=2)
    try:
        return parse_answer(client.request(encode_request(Request(
            division=Division.ROBOT, request_id=1, command=command, instance=instance,
            attribute=attribute, service=service, data=data))))
    finally:
        client.close()


def _start(robot: YaskawaRobot, job: str) -> None:
    _ask(robot, 0x87, 1, 0, 0x02, job.encode().ljust(32, b"\0") + struct.pack("<i", 0))
    _ask(robot, 0x86, 1, 1, 0x10, struct.pack("<i", 1))


def test_a_started_job_switches_its_outputs_through_the_ladder(cell: YaskawaRobot) -> None:
    _start(cell, "GRIP_OPEN")
    assert _ask(cell, 0x78, 3001, 1, 0x0E).data[0] == 0b01


def test_a_job_that_stops_is_noted(cell: YaskawaRobot) -> None:
    events: list[tuple[str, dict[str, object]]] = []
    cell.controller.note = lambda kind, **fields: events.append((kind, fields))
    _start(cell, "MOVE")
    jobs = [event for event in events if event[0].startswith("job_")]
    assert jobs == [("job_stopped", {"job": "MOVE", "line": 1,
                                       "reason": "'MOVJ C00000 VJ=10.00' is not emulated"})]
