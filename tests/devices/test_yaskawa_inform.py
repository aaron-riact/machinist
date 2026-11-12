"""Running a job's I/O part: what starting a cell's gripper or interface job switches."""

from __future__ import annotations

import struct
import time
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.controller import (
    CommandRefusedError,
    Status1,
    YaskawaController,
)
from machinist.devices.robots.yaskawa.inform import (
    CallJob,
    Comparison,
    Job,
    JobInterruptedError,
    JobStoppedError,
    Jump,
    Literal,
    Move,
    PositionRef,
    SetOutput,
    SignalTest,
    Timer,
    Unsupported,
    Variable,
    Wait,
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
    positions: dict[str, dict[int, tuple[int, ...]]] = field(default_factory=dict)
    scans: int = 0
    #: Seconds passed, on the host's own clock: nothing really waits.
    clock: float = 0.0
    #: What the job left to do later, and when.
    later: list[tuple[float, Callable[[], None]]] = field(default_factory=list)

    def job_text(self, name: str) -> str | None:
        return self.jobs.get(name)

    def run_ladder(self) -> None:
        self.scans += 1

    def checkpoint(self) -> None:
        pass

    def sleep(self, seconds: float) -> None:
        self.clock += seconds

    def wait_until(self, test: Callable[[], bool], seconds: float | None) -> None:
        if test():
            return
        if seconds is None:
            raise JobInterruptedError("nothing here would ever switch it")
        self.clock += seconds

    def after(self, seconds: float, action: Callable[[], None]) -> None:
        self.later.append((self.clock + seconds, action))


@pytest.mark.parametrize(("text", "instruction"), [
    ("DOUT OT#(D001) ON", SetOutput(Variable("D", 1), on=True)),
    ("DOUT OT#(5) OFF", SetOutput(Literal(5), on=False)),
    ("JUMP *OVER IF IN#(D021)=ON",
     Jump("OVER", SignalTest(output=False, ref=Variable("D", 21), on=True))),
    ("CALL JOB:GRIP_CLOSE IF I005=2",
     CallJob("GRIP_CLOSE", Comparison(Variable("I", 5), "=", Literal(2)))),
    ("MOVL P010 V=50.0 PL=0", Move(True, PositionRef(own=False, number=10), Literal(50))),
    ("ARCON ASF#(1)", Unsupported("ARCON ASF#(1)")),
    ("'OPEN GRIPPER A", None),
    ("ADVSTOP", None),
    ("ADVINIT", None),
    ("SHCKSET SSL#(5)", None),
    ("SHCKRST", None),
    ("TIMER T=I020", Timer(Variable("I", 20))),
    ("WAIT IN#(D022)=ON", Wait(SignalTest(output=False, ref=Variable("D", 22), on=True))),
    ("WAIT IN#(7)=OFF T=2.5",
     Wait(SignalTest(output=False, ref=Literal(7), on=False), Literal(2.5))),
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


def test_a_pulse_switches_its_output_on_and_off_again_after_its_time() -> None:
    host = _Host({"P": _job("PULSE OT#(3) T=0.50", "PULSE OT#(4)")})
    run_job(host, "P")
    assert host.io[10010] == 0b1100
    assert [when for when, _ in host.later] == [0.5, 0.3]
    for _, action in host.later:
        action()
    assert host.io[10010] == 0


def test_a_timer_lets_its_time_pass() -> None:
    host = _Host({"T": _job("TIMER T=0.20", "SET I000 3", "TIMER T=I000")})
    run_job(host, "T")
    assert host.clock == pytest.approx(3.2)


def test_a_wait_already_met_goes_straight_on() -> None:
    host = _Host({"W": _job("WAIT IN#(9)=ON", "DOUT OT#(1) ON")}, io={20: 0b1})
    run_job(host, "W")
    assert host.clock == 0
    assert host.io[10010] == 1


def test_a_wait_with_a_time_goes_on_once_that_time_has_passed() -> None:
    host = _Host({"W": _job("WAIT IN#(9)=ON T=1.5", "DOUT OT#(1) ON")})
    run_job(host, "W")
    assert host.clock == 1.5
    assert host.io[10010] == 1


def test_a_group_output_sets_its_whole_byte() -> None:
    host = _Host({"G": _job("DOUT OG#(2) 2")})
    run_job(host, "G")
    assert host.io[10020] == 2


@pytest.mark.parametrize(("lines", "reason"), [
    (("ARCON ASF#(1)",), "not emulated"),
    (("WAIT IN#(7)=ON",), "nothing here would ever switch it"),
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
    m.controller.files["ARC.JBI"] = _job("ARCON ASF#(1)").encode()
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
    assert robot.controller.wait_job(2)


def test_a_started_job_switches_its_outputs_through_the_ladder(cell: YaskawaRobot) -> None:
    _start(cell, "GRIP_OPEN")
    assert _ask(cell, 0x78, 3001, 1, 0x0E).data[0] == 0b01


def test_a_job_that_stops_is_noted(cell: YaskawaRobot) -> None:
    events: list[tuple[str, dict[str, object]]] = []
    cell.controller.note = lambda kind, **fields: events.append((kind, fields))
    _start(cell, "ARC")
    jobs = [event for event in events if event[0].startswith("job_")]
    assert jobs == [("job_stopped", {"job": "ARC", "line": 1,
                                       "reason": "'ARCON ASF#(1)' is not emulated"})]


# --- On the controller: a started job runs on its own thread --------------------------


@pytest.fixture
def robot() -> YaskawaRobot:
    options = YaskawaOptions(controller="dx200", robot="up50n")
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.controller.files["WAITING.JBI"] = _job("WAIT IN#(9)=ON", "DOUT OT#(1) ON").encode()
    m.controller.files["TIMED.JBI"] = _job("TIMER T=0.30", "DOUT OT#(1) ON").encode()
    return m


def test_a_started_job_runs_on_while_the_start_is_answered(robot: YaskawaRobot) -> None:
    controller = robot.controller
    controller.start_job("WAITING")
    assert controller.job_running
    assert Status1.RUNNING in controller.status()[0]
    controller.set_input(9, True)
    assert controller.wait_job(2)
    assert controller.output(1)
    assert Status1.RUNNING not in controller.status()[0]


def test_a_timer_takes_its_time(robot: YaskawaRobot) -> None:
    started = time.monotonic()
    robot.controller.start_job("TIMED")
    assert robot.controller.wait_job(2)
    assert time.monotonic() - started >= 0.3
    assert robot.controller.output(1)


def test_a_second_start_while_a_job_runs_is_refused(robot: YaskawaRobot) -> None:
    robot.controller.start_job("WAITING")
    with pytest.raises(CommandRefusedError, match="a job is running"):
        robot.controller.start_job("TIMED")
    robot.controller.set_input(9, True)
    assert robot.controller.wait_job(2)


@pytest.mark.parametrize(("stop", "reason"), [
    (lambda c: c.set_hold(True), "hold"),
    (lambda c: c.set_servo(False), "servo off"),
])
def test_a_hold_or_servo_off_stops_the_running_job(
        robot: YaskawaRobot, stop: Callable[[YaskawaController], None], reason: str) -> None:
    events: list[tuple[str, dict[str, object]]] = []
    robot.controller.note = lambda kind, **fields: events.append((kind, fields))
    robot.controller.start_job("WAITING")
    stop(robot.controller)
    assert robot.controller.wait_job(2)
    assert events == [("job_stopped", {"job": "WAITING", "line": 1, "reason": reason})]
    assert not robot.controller.output(1)


def test_msg_shows_on_the_pendant(robot: YaskawaRobot) -> None:
    robot.controller.files["M.JBI"] = _job('MSG "OUT OF PARTS"').encode()
    robot.controller.start_job("M")
    assert robot.controller.wait_job(2)
    assert robot.controller.pendant_message == "OUT OF PARTS"


def test_pause_stops_the_job_there(robot: YaskawaRobot) -> None:
    events: list[tuple[str, dict[str, object]]] = []
    robot.controller.note = lambda kind, **fields: events.append((kind, fields))
    robot.controller.files["P.JBI"] = _job("PAUSE", "DOUT OT#(1) ON").encode()
    robot.controller.start_job("P")
    assert robot.controller.wait_job(2)
    assert events == [("job_stopped", {"job": "P", "line": 1, "reason": "PAUSE"})]
    assert not robot.controller.output(1)


@pytest.mark.parametrize(("line", "io", "value"), [
    ("DIN B000 OT#(2)", {10010: 0b10}, 1),
    ("DIN B000 OT#(1)", {10010: 0b10}, 0),
    ("DIN B000 IN#(10)", {20: 0b10}, 1),
    ("DIN B000 SOUT#(9)", {50020: 0b1}, 1),
    ("DIN B000 IG#(2)", {20: 0xA5}, 0xA5),
    ("DIN B000 OG#(2)", {10020: 9}, 9),
])
def test_din_reads_a_signal_or_a_group(line: str, io: dict[int, int], value: int) -> None:
    host = _Host({"D": _job("SET B000 7", line)}, io=dict(io))
    run_job(host, "D")
    assert host.variables["B"][0] == value


def test_setreg_sets_a_register_and_the_ladder_runs() -> None:
    host = _Host({"R": _job("SETREG MREG#(200) 100")})
    run_job(host, "R")
    assert host.variables["register"][200] == 100
    assert host.scans == 1
