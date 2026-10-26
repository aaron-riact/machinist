"""The NX100 Ethernet server against the manual (HW0483202).

The byte strings follow the manual's transmission examples (section 3.3).
"""

from __future__ import annotations

import math
import socket
import threading
import time
from collections.abc import Iterator

import numpy as np
import pytest

from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import CONTROLLERS, ROBOTS, YaskawaOptions, YaskawaRobot
from machinist.kinematics.dh_backend import pose_to_mat

from ..conftest import free_port, wait_running

#: The UP50N's pulse ratios and speeds, without its URDF: with no kinematics the
#: arm's pose is its joint values, which keeps the expected answers readable.
UP50N_AXES = {
    "pulses_per_degree": ROBOTS["up50n"].pulses_per_degree,
    "max_joint_speed_deg_s": ROBOTS["up50n"].max_joint_speed_deg_s,
}
START = (10, 20, -30, 0, 45, -90)


def _running(options: YaskawaOptions) -> Iterator[YaskawaRobot]:
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


@pytest.fixture
def motoman() -> Iterator[YaskawaRobot]:
    yield from _running(YaskawaOptions(controller="nx100", start_joints_deg=START, **UP50N_AXES))


@pytest.fixture
def with_jobs() -> Iterator[YaskawaRobot]:
    yield from _running(YaskawaOptions(controller="nx100", start_joints_deg=START,
                                       jobs=("MAIN", "PICK"), **UP50N_AXES))


@pytest.fixture
def up50n() -> Iterator[YaskawaRobot]:
    """A DX200 with the UP50N's kinematics, at X 1170, Z 805 mm, the tool pointing down."""
    yield from _running(YaskawaOptions(controller="dx200", robot="up50n",
                                       start_joints_deg=(0, 0, 0, 0, -90, 0)))


@pytest.fixture
def dx200() -> Iterator[YaskawaRobot]:
    yield from _running(YaskawaOptions(controller="dx200", start_joints_deg=START, **UP50N_AXES))


def _connect(motoman: YaskawaRobot) -> socket.socket:
    return socket.create_connection((motoman.endpoint.host, motoman.endpoint.port), timeout=2)


def _session(motoman: YaskawaRobot, sent: bytes) -> bytes:
    """Send *sent*, then read everything until the controller closes the socket."""
    with _connect(motoman) as s:
        s.sendall(sent)
        received = b""
        while chunk := s.recv(4096):
            received += chunk
        return received


def _read(s: socket.socket, expected: bytes) -> bytes:
    """Read as many bytes as *expected* holds."""
    received = b""
    while len(received) < len(expected):
        received += s.recv(4096)
    return received


def test_single_command_answers_then_closes(motoman: YaskawaRobot) -> None:
    """3.3.1: one RSTATS without Keep-Alive, then the controller closes."""
    received = _session(motoman, b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST RSTATS 0\r\n")
    assert received == (
        b"OK: NX Information Server (3.00).\r\n"
        b"OK: RSTATS\r\n"
        b"194,64\r"
    )


def test_keep_alive_answers_n_commands_then_closes(motoman: YaskawaRobot) -> None:
    """3.3.4: Keep-Alive:2 runs two commands, then the controller closes."""
    received = _session(
        motoman,
        b"CONNECT Robot_access Keep-Alive:2.\r\n"
        b"HOSTCTRL_REQUEST RSTATS 0\r\n"
        b"HOSTCTRL_REQUEST RSTATS 0\r\n"
        b"HOSTCTRL_REQUEST RSTATS 0\r\n",
    )
    assert received == (
        b"OK: NX Information Server (3.00) Keep-Alive:2.\r\n"
        b"OK: RSTATS\r\n194,64\r"
        b"OK: RSTATS\r\n194,64\r"
    )


def test_keep_alive_minus_one_keeps_the_socket_open(motoman: YaskawaRobot) -> None:
    with _connect(motoman) as s:
        s.sendall(b"CONNECT Robot_access Keep-Alive:-1\r\n")
        banner = b"OK: NX Information Server (3.00) Keep-Alive:-1.\r\n"
        assert _read(s, banner) == banner
        for _ in range(3):
            s.sendall(b"HOSTCTRL_REQUEST RSTATS 0\r\n")
            answer = b"OK: RSTATS\r\n194,64\r"
            assert _read(s, answer) == answer


@pytest.mark.parametrize(
    "start",
    [
        b"HOSTCTRL_REQUEST RSTATS 0\r\n",
        b"CONNECT Robot\r\n",
        b"CONNECT Robot_access Keep-Alive:1\r\n",
    ],
)
def test_a_bad_start_request_is_refused_and_closed(motoman: YaskawaRobot, start: bytes) -> None:
    assert _session(motoman, start) == b"NG: HTTP Error Response\r\n"


def test_an_unknown_command_is_refused_and_closed(motoman: YaskawaRobot) -> None:
    received = _session(
        motoman, b"CONNECT Robot_access Keep-Alive:-1\r\nHOSTCTRL_REQUEST NOPE 0\r\n"
    )
    assert received.endswith(b"Keep-Alive:-1.\r\nNG: unknown command NOPE\r\n")


def test_jwait_is_not_an_ethernet_server_command(motoman: YaskawaRobot) -> None:
    """The manual lists JWAIT only to say the Ethernet server does not support it."""
    received = _session(motoman, b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST JWAIT 0\r\n")
    assert received.endswith(b"NG: unknown command JWAIT\r\n")


def test_a_size_that_does_not_count_the_data_is_refused(motoman: YaskawaRobot) -> None:
    """The size counts the data and its closing CR: ``ab<CR>`` is 3 bytes."""
    received = _session(
        motoman, b"CONNECT Robot_access Keep-Alive:-1\r\nHOSTCTRL_REQUEST RSTATS 2\r\nab\r"
    )
    assert received.endswith(b"OK: RSTATS\r\nNG: size 2 for 3 bytes\r\n")


def _answer(motoman: YaskawaRobot, command: str, data: str = "") -> bytes:
    """The answer to one *command*, after its OK line."""
    size = len(data) + 1 if data else 0
    sent = f"CONNECT Robot_access\r\nHOSTCTRL_REQUEST {command} {size}\r\n"
    if data:
        sent += data + "\r"
    received = _session(motoman, sent.encode())
    return received.split(f"OK: {command}\r\n".encode(), 1)[1]


def _noting(robot: YaskawaRobot) -> list[tuple[str, dict[str, object]]]:
    notes: list[tuple[str, dict[str, object]]] = []
    robot.controller.note = lambda kind, **details: notes.append((kind, details))
    return notes


def test_a_command_that_changes_something_is_noted(motoman: YaskawaRobot) -> None:
    notes = _noting(motoman)
    _answer(motoman, "HOLD", "1")
    assert ("command", {"command": "HOLD", "data": "1", "answer": "0000"}) in notes


def test_a_poll_is_not_noted(motoman: YaskawaRobot) -> None:
    notes = _noting(motoman)
    _answer(motoman, "RSTATS")
    assert notes == []


def test_a_session_for_several_commands_is_noted(motoman: YaskawaRobot) -> None:
    notes = _noting(motoman)
    _session(motoman, b"CONNECT Robot_access Keep-Alive:2\r\nHOSTCTRL_REQUEST RSTATS 0\r\n"
                      b"HOSTCTRL_REQUEST RSTATS 0\r\n")
    assert notes == [("session", {"keep_alive": 2})]


def test_a_refusal_is_noted_with_its_reason(motoman: YaskawaRobot) -> None:
    notes = _noting(motoman)
    _answer(motoman, "SVON", "2")
    kind, details = notes[-1]
    assert kind == "refused"
    assert details["answer"] == "ERROR:SVON is not successful (5200)."
    assert details["reason"] == "switch 2"


def test_an_unknown_command_is_noted_as_refused(motoman: YaskawaRobot) -> None:
    notes = _noting(motoman)
    _session(motoman, b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST NOPE 0\r\n")
    assert notes[-1] == ("refused", {"answer": "NG: unknown command NOPE", "command": "NOPE"})


def test_rposj_answers_twelve_axes_in_pulses(motoman: YaskawaRobot) -> None:
    """S, L, U, R, B, T in encoder pulses, then six external axes."""
    assert _answer(motoman, "RPOSJ") == b"14166,38908,-59831,0,36653,-37676,0,0,0,0,0,0\r"


def test_pulses_per_degree_needs_one_ratio_per_joint() -> None:
    with pytest.raises(ValueError, match="2 pulses_per_degree for 6 joints"):
        YaskawaOptions(
            controller="nx100", pulses_per_degree=(1.0, 2.0), max_joint_speed_deg_s=(1.0,) * 6
        )


def test_rposc_answers_mm_degrees_form_and_tool(motoman: YaskawaRobot) -> None:
    """X, Y, Z in mm (3 decimals), Rx, Ry, Rz in degrees (2 decimals), type, tool.

    The fixture's arm has no kinematics, so its pose is its joint values.
    """
    assert _answer(motoman, "RPOSC", "0, 0") == (
        b"174.533,349.066,-523.599,0.00,45.00,-90.00,0,0\r"
    )


def test_rposc_with_external_axes_adds_six(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "RPOSC", "1,1").endswith(b",0,0,0,0,0,0,0,0\r")


def test_rposc_in_an_undefined_user_frame_is_an_error(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "RPOSC", "2,0") == b"ERROR:RPOSC is not successful (3350).\r\n"


@pytest.mark.parametrize("data", ["26,0", "0,2", "0", "a,b"])
def test_rposc_with_bad_data_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    assert _answer(motoman, "RPOSC", data) == b"ERROR:RPOSC is not successful (5200).\r\n"


def test_ralarm_answers_no_error_and_no_alarm(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "RALARM") == b"0,0,0,0,0,0,0,0,0,0\r"


def test_ralarm_reports_a_fault_as_an_alarm(motoman: YaskawaRobot) -> None:
    motoman.arm.fault()
    assert _answer(motoman, "RALARM") == b"0,0,4000,0,0,0,0,0,0,0\r"
    assert _answer(motoman, "RSTATS") == b"194,80\r"


def test_reset_clears_the_alarm(motoman: YaskawaRobot) -> None:
    motoman.arm.fault()
    assert _answer(motoman, "RESET") == b"0000\r\n"
    assert _answer(motoman, "RALARM") == b"0,0,0,0,0,0,0,0,0,0\r"


@pytest.mark.parametrize(
    ("command", "data"),
    [("PMOVJ", "10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"), ("SVON", "1"),
     ("LOADV", "0,1,5"), ("JSEQ", "MAIN,0"), ("START", "MAIN")],
)
def test_an_alarm_refuses_moves_and_writes(
    with_jobs: YaskawaRobot, command: str, data: str
) -> None:
    """2060: alarm or error occurring (the interlock table, 11.1.1)."""
    with_jobs.arm.fault()
    error = f"ERROR:{command} is not successful (2060).\r\n".encode()
    assert _answer(with_jobs, command, data) == error


def test_an_alarm_still_lets_hold_and_reads_through(motoman: YaskawaRobot) -> None:
    motoman.arm.fault()
    assert _answer(motoman, "HOLD", "1") == b"0000\r\n"
    assert _answer(motoman, "SAVEV", "0,1") == b"0\r"


def test_svon_switches_the_servo(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "SVON", "0") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,0\r"
    assert _answer(motoman, "SVON", "1") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,64\r"


def test_svon_during_an_emergency_stop_is_an_error(motoman: YaskawaRobot) -> None:
    motoman.arm.estop()
    assert _answer(motoman, "SVON", "1") == b"ERROR:SVON is not successful (3450).\r\n"


def test_hold_shows_in_rstats_until_released(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "HOLD", "1") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,72\r"
    assert _answer(motoman, "HOLD", "0") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,64\r"


@pytest.mark.parametrize("command", ["SVON", "HOLD", "HLOCK"])
def test_a_switch_takes_only_0_or_1(motoman: YaskawaRobot, command: str) -> None:
    error = f"ERROR:{command} is not successful (5200).\r\n"
    assert _answer(motoman, command, "2") == error.encode()


def test_cancel_answers_normally(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "CANCEL") == b"0000\r\n"


@pytest.mark.parametrize(("cycle", "rstats"), [("1", b"193,64\r"), ("2", b"194,64\r"),
                                               ("3", b"196,64\r")])
def test_cycle_selects_step_one_cycle_or_auto(motoman: YaskawaRobot, cycle: str,
                                              rstats: bytes) -> None:
    assert _answer(motoman, "CYCLE", cycle) == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == rstats


def test_cycle_takes_only_1_to_3(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "CYCLE", "4") == b"ERROR:CYCLE is not successful (5200).\r\n"


def test_hlock_sets_the_interlock(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "HLOCK", "1") == b"0000\r\n"
    assert motoman.controller.hlock
    assert _answer(motoman, "HLOCK", "0") == b"0000\r\n"
    assert not motoman.controller.hlock


def test_mdsp_shows_a_message_on_the_pendant(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "MDSP", "PART 12, DONE") == b"0000\r\n"
    assert motoman.controller.pendant_message == "PART 12, DONE"


def test_mdsp_takes_at_most_30_bytes(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "MDSP", "X" * 31) == b"ERROR:MDSP is not successful (5200).\r\n"


def test_mode_selects_teach_or_play(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "MODE", "1") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"162,64\r"
    assert _answer(motoman, "MODE", "2") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,64\r"


def test_mode_takes_only_1_or_2(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "MODE", "3") == b"ERROR:MODE is not successful (5200).\r\n"


def test_a_move_in_teach_mode_is_an_error(motoman: YaskawaRobot) -> None:
    """2080: incorrect mode (the interlock table, 11.1.1)."""
    _answer(motoman, "MODE", "1")
    move = "10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", move) == b"ERROR:PMOVJ is not successful (2080).\r\n"


def test_rjseq_reads_the_executing_job_line_and_step(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.select_job("PICK", 12)
    assert _answer(with_jobs, "RJSEQ") == b"PICK,12,0\r"


def test_rjseq_without_an_executing_job_reads_no_name(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "RJSEQ") == b",0,0\r"


def test_rjdir_lists_every_job(with_jobs: YaskawaRobot) -> None:
    assert _answer(with_jobs, "RJDIR", "*") == b"MAIN,PICK\r"


def test_rjdir_lists_the_jobs_a_parent_calls(with_jobs: YaskawaRobot) -> None:
    files = with_jobs.controller.files
    files["MAIN.JBI"] = b"/JOB\r\nNOP\r\nCALL JOB:PICK\r\nJUMP JOB:PLACE\r\nEND\r\n"
    files["PLACE.JBI"] = b"/JOB\r\nNOP\r\nCALL JOB:HOME\r\nEND\r\n"
    files["HOME.JBI"] = b"/JOB\r\nNOP\r\nEND\r\n"
    assert _answer(with_jobs, "RJDIR", "MAIN") == b"PICK,PLACE,HOME\r"


def test_rjdir_of_a_job_that_calls_none_is_empty(with_jobs: YaskawaRobot) -> None:
    assert _answer(with_jobs, "RJDIR", "PICK") == b"\r"


def test_rjdir_with_a_missing_child_is_an_error(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.files["MAIN.JBI"] = b"/JOB\r\nNOP\r\nCALL JOB:GONE\r\nEND\r\n"
    assert _answer(with_jobs, "RJDIR", "MAIN") == b"ERROR:RJDIR is not successful (4040).\r\n"


def test_rjdir_starts_a_line_every_20_names(with_jobs: YaskawaRobot) -> None:
    for n in range(19):
        with_jobs.controller.files[f"J{n:02d}.JBI"] = b""
    lines = _answer(with_jobs, "RJDIR", "*").split(b"\r")
    assert [len(line.split(b",")) for line in lines] == [20, 1, 1]


def test_a_yrc1000_starts_an_rjdir_line_every_5_names() -> None:
    """As a real one (an HC10DTP's) listed its 52 jobs: ten lines of 5, then 2."""
    options = YaskawaOptions(controller="yrc1000", start_joints_deg=START,
                             jobs=tuple(f"J{n:02d}" for n in range(7)), **UP50N_AXES)
    for robot in _running(options):
        lines = _answer(robot, "RJDIR", "*").split(b"\r")
        assert [len(line.split(b",")) for line in lines] == [5, 2, 1]


def test_start_runs_the_executing_job(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.select_job("PICK", 3)
    assert _answer(with_jobs, "START") == b"0000\r\n"


def test_start_without_an_executing_job_is_an_error(with_jobs: YaskawaRobot) -> None:
    assert _answer(with_jobs, "START") == b"ERROR:START is not successful (4060).\r\n"


def test_start_with_a_name_runs_that_job_from_its_start(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.select_job("PICK", 3)
    assert _answer(with_jobs, "START", "MAIN") == b"0000\r\n"
    assert _answer(with_jobs, "RJSEQ") == b"MAIN,0,0\r"


def test_start_in_teach_mode_keeps_the_executing_job(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.select_job("PICK", 3)
    _answer(with_jobs, "MODE", "1")
    assert _answer(with_jobs, "START", "MAIN") == b"ERROR:START is not successful (2080).\r\n"
    assert _answer(with_jobs, "RJSEQ") == b"PICK,3,0\r"


def test_delete_removes_a_job_in_teach_mode(with_jobs: YaskawaRobot) -> None:
    _answer(with_jobs, "MODE", "1")
    assert _answer(with_jobs, "DELETE", "PICK") == b"0000\r\n"
    assert _answer(with_jobs, "RJDIR", "*") == b"MAIN\r"


def test_delete_star_removes_every_job(with_jobs: YaskawaRobot) -> None:
    _answer(with_jobs, "MODE", "1")
    assert _answer(with_jobs, "DELETE", "*") == b"0000\r\n"
    assert _answer(with_jobs, "RJDIR", "*") == b"\r"
    assert "SYSTEM.SYS" in with_jobs.controller.files


def test_delete_in_play_mode_is_an_error(with_jobs: YaskawaRobot) -> None:
    """2080: the interlock table (11.1.1) allows editing in teach mode only."""
    assert _answer(with_jobs, "DELETE", "PICK") == b"ERROR:DELETE is not successful (2080).\r\n"


def test_delete_of_a_job_the_controller_lacks_is_an_error(with_jobs: YaskawaRobot) -> None:
    _answer(with_jobs, "MODE", "1")
    assert _answer(with_jobs, "DELETE", "NOPE") == b"ERROR:DELETE is not successful (4040).\r\n"


def test_a_deleted_job_no_longer_executes(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.select_job("PICK", 3)
    _answer(with_jobs, "MODE", "1")
    _answer(with_jobs, "DELETE", "PICK")
    assert _answer(with_jobs, "RJSEQ") == b",0,0\r"


@pytest.mark.parametrize(("command", "data"), [("CVTRJ", "PICK,0"), ("CVTSJ", "PICK,0,0")])
def test_job_conversion_needs_the_relative_job_function(
    with_jobs: YaskawaRobot, command: str, data: str
) -> None:
    _answer(with_jobs, "MODE", "1")
    error = f"ERROR:{command} is not successful (4490).\r\n".encode()
    assert _answer(with_jobs, command, data) == error


#: User frame 1 (coordinate 2): its origin at X 1000 mm, its X axis along base Y
#: and its Y axis along base -X.
FRAME_1 = ("2, 1000,0,0,0,0,0,0, 1000,100,0,0,0,0,0, 900,0,0,0,0,0,0, 3, 0,0,0,0,0,0")


def test_wuframe_teaches_a_user_frame_in_teach_mode(motoman: YaskawaRobot) -> None:
    _answer(motoman, "MODE", "1")
    assert _answer(motoman, "WUFRAME", FRAME_1) == b"0000\r\n"
    frame = motoman.controller.user_frames[1]
    assert frame.tool == 3
    assert frame.transform()[:3, :].round(6).tolist() == [
        [0, -1, 0, 1], [1, 0, 0, 0], [0, 0, 1, 0],
    ]


def test_ruframe_reads_a_taught_frame_back(motoman: YaskawaRobot) -> None:
    _answer(motoman, "MODE", "1")
    _answer(motoman, "WUFRAME", FRAME_1)
    assert _answer(motoman, "RUFRAME", "2") == (
        b"1000.000,0.000,0.000,0.00,0.00,0.00,0,"
        b"1000.000,100.000,0.000,0.00,0.00,0.00,0,"
        b"900.000,0.000,0.000,0.00,0.00,0.00,0,3,0,0,0,0,0,0\r"
    )


def test_ruframe_of_a_frame_not_taught_is_an_error(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "RUFRAME", "2") == b"ERROR:RUFRAME is not successful (3350).\r\n"


@pytest.mark.parametrize("data", ["1", "26", "x"])
def test_ruframe_with_bad_data_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    assert _answer(motoman, "RUFRAME", data) == b"ERROR:RUFRAME is not successful (5200).\r\n"


def test_wuframe_in_play_mode_is_an_error(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "WUFRAME", FRAME_1) == b"ERROR:WUFRAME is not successful (2080).\r\n"


def test_wuframe_with_points_on_a_line_is_an_error(motoman: YaskawaRobot) -> None:
    _answer(motoman, "MODE", "1")
    on_a_line = FRAME_1.replace("900,0,0", "1000,200,0")
    error = b"ERROR:WUFRAME is not successful (3460).\r\n"
    assert _answer(motoman, "WUFRAME", on_a_line) == error


@pytest.mark.parametrize("data", [FRAME_1.replace("2,", "26,", 1),  # user frame 25 of 24
                                  FRAME_1.replace(" 3,", " 24,"),  # tool 24 of 0-23
                                  "2, 1000"])
def test_wuframe_with_bad_data_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    _answer(motoman, "MODE", "1")
    assert _answer(motoman, "WUFRAME", data) == b"ERROR:WUFRAME is not successful (5200).\r\n"


def test_setmj_sets_the_master_and_executing_job(with_jobs: YaskawaRobot) -> None:
    with_jobs.controller.select_job("PICK", 7)
    assert _answer(with_jobs, "SETMJ", "MAIN") == b"0000\r\n"
    assert with_jobs.controller.master_jobs[0] == "MAIN"
    assert _answer(with_jobs, "RJSEQ") == b"MAIN,0,0\r"


def test_setmj_of_a_job_the_controller_lacks_is_an_error(with_jobs: YaskawaRobot) -> None:
    assert _answer(with_jobs, "SETMJ", "NOPE") == b"ERROR:SETMJ is not successful (4040).\r\n"


def test_jseq_sets_the_executing_job_and_line(with_jobs: YaskawaRobot) -> None:
    assert _answer(with_jobs, "JSEQ", "PICK,5") == b"0000\r\n"
    assert _answer(with_jobs, "RJSEQ") == b"PICK,5,0\r"


def test_jseq_of_a_job_the_controller_lacks_is_an_error(with_jobs: YaskawaRobot) -> None:
    assert _answer(with_jobs, "JSEQ", "NOPE,0") == b"ERROR:JSEQ is not successful (4040).\r\n"


@pytest.mark.parametrize("data", ["PICK,10000", "PICK", "PICK,x"])
def test_jseq_with_bad_data_is_an_error(with_jobs: YaskawaRobot, data: str) -> None:
    assert _answer(with_jobs, "JSEQ", data) == b"ERROR:JSEQ is not successful (5200).\r\n"


def test_jseq_while_the_arm_moves_is_an_error(with_jobs: YaskawaRobot) -> None:
    """2010: manipulator operating (the interlock table, 11.1.1)."""
    _answer(with_jobs, "PMOVJ", "1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0")
    assert _answer(with_jobs, "JSEQ", "PICK,0") == b"ERROR:JSEQ is not successful (2010).\r\n"


def test_rgroup_reads_r1_and_the_master_task(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "RGROUP") == b"1,0,0\r"


def test_cgroup_takes_r1(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "CGROUP", "1,0") == b"0000\r\n"


@pytest.mark.parametrize("data", ["2,0", "3,0", "1,1"])
def test_cgroup_of_a_group_the_cell_lacks_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    assert _answer(motoman, "CGROUP", data) == b"ERROR:CGROUP is not successful (3370).\r\n"


def test_ctask_needs_independent_control(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "CTASK", "0") == b"ERROR:CTASK is not successful (4490).\r\n"


@pytest.mark.parametrize(
    ("variable_type", "kind", "value", "answer"),
    [(0, "B", 200, b"200"), (1, "I", -1234, b"-1234"), (2, "D", -123456789, b"-123456789"),
     (3, "R", 1.5, b"1.500000E+00")],
)
def test_savev_reads_a_number_variable(
    motoman: YaskawaRobot, variable_type: int, kind: str, value: float, answer: bytes
) -> None:
    motoman.controller.variables[kind] = {7: value}
    assert _answer(motoman, "SAVEV", f"{variable_type},7") == answer + b"\r"


def test_savev_reads_a_p_variable_in_pulses(motoman: YaskawaRobot) -> None:
    motoman.controller.positions["P"] = {5: (0, 0, 3, 0, 0, 1, 2, 3, 4, 5, 6, 0, 0)}
    assert _answer(motoman, "SAVEV", "4,5") == b"0,1,2,3,4,5,6,3\r"


def test_savev_reads_a_p_variable_in_base_coordinates(motoman: YaskawaRobot) -> None:
    """Form 4 as HSES keeps it is type 1 here, as a real YRC1000's P000 read."""
    record = (16, 4, 2, 0, 0, 1000500, -2000, 300000, 1800000, 0, -900000, 0, 0)
    motoman.controller.positions["P"] = {5: record}
    assert _answer(motoman, "SAVEV", "4,5") == (
        b"1,0,1000.500,-2.000,300.000,180.00,0.00,-90.00,1,2\r"
    )


def test_savev_reads_a_p_variable_in_a_user_frame(motoman: YaskawaRobot) -> None:
    """HSES keeps user frame 3 as data type 18, user 3; the Ethernet server calls it 4."""
    motoman.controller.positions["P"] = {5: (18, 0, 0, 3, *([0] * 9))}
    assert _answer(motoman, "SAVEV", "4,5").startswith(b"1,4,")


def test_savev_reads_a_p_variable_in_the_tool_frame(motoman: YaskawaRobot) -> None:
    """The NX100 numbers the tool frame 26, after its 24 user frames."""
    motoman.controller.positions["P"] = {5: (19, *([0] * 12))}
    assert _answer(motoman, "SAVEV", "4,5").startswith(b"1,26,")


def test_savev_reads_a_bp_variable_with_tool_0(motoman: YaskawaRobot) -> None:
    motoman.controller.positions["BP"] = {1: (0, 10, 20, 0, 0, 0, 0, 0, 0)}
    assert _answer(motoman, "SAVEV", "5,1") == b"0,10,20,0,0,0,0,0\r"


def test_savev_reads_an_unset_position_variable_as_pulses(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "SAVEV", "6,127") == b"0,0,0,0,0,0,0,0\r"


def test_savev_reads_an_unset_variable_as_0(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "SAVEV", "1,99") == b"0\r"


def test_savev_reads_a_string_variable(motoman: YaskawaRobot) -> None:
    motoman.controller.strings[3] = b"HELLO".ljust(32, b"\0")
    assert _answer(motoman, "SAVEV", "7,3") == b"HELLO\r"


@pytest.mark.parametrize("data", ["0,100", "4,128", "8,0", "0", "x,1"])
def test_savev_with_bad_data_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    assert _answer(motoman, "SAVEV", data) == b"ERROR:SAVEV is not successful (5200).\r\n"


@pytest.mark.parametrize(
    ("data", "kind", "value"),
    [("0,7,200", "B", 200), ("1,7,-1234", "I", -1234), ("2,7,-123456789", "D", -123456789),
     ("3,7,1.5", "R", 1.5)],
)
def test_loadv_writes_a_number_variable(
    motoman: YaskawaRobot, data: str, kind: str, value: float
) -> None:
    assert _answer(motoman, "LOADV", data) == b"0000\r\n"
    assert motoman.controller.variables[kind][7] == value


def test_loadv_then_savev_reads_back(motoman: YaskawaRobot) -> None:
    """A real number as a real YRC1000 (an HC10DTP's) gave R000 = 9.5: 9.500000E+00."""
    _answer(motoman, "LOADV", "3,1,0.1")
    assert _answer(motoman, "SAVEV", "3,1") == b"1.000000E-01\r"


def test_savev_reads_a_real_zero_as_0(motoman: YaskawaRobot) -> None:
    """The same controller gave R001, zero, as a bare 0."""
    assert _answer(motoman, "SAVEV", "3,2") == b"0\r"


def test_loadv_writes_a_string_with_commas(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "LOADV", "7,3,A,B C") == b"0000\r\n"
    assert _answer(motoman, "SAVEV", "7,3") == b"A,B C\r"


def test_loadv_writes_a_p_variable_in_pulses(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "LOADV", "4,5,0,1,2,3,4,5,6,3") == b"0000\r\n"
    assert motoman.controller.positions["P"][5] == (0, 0, 3, 0, 0, 1, 2, 3, 4, 5, 6, 0, 0)


def test_loadv_writes_a_p_variable_in_a_user_frame(motoman: YaskawaRobot) -> None:
    """User frame 3 is coordinate 4; HSES keeps it as data type 18, user 3, and type 4
    as HSES numbers the form, 1."""
    data = "4,5,1,4,1000.5,-2,300,180,0,-90,4,2"
    assert _answer(motoman, "LOADV", data) == b"0000\r\n"
    record = (18, 1, 2, 3, 0, 1000500, -2000, 300000, 1800000, 0, -900000, 0, 0)
    assert motoman.controller.positions["P"][5] == record


def test_the_ethernet_server_numbers_the_form_apart_from_hses(motoman: YaskawaRobot) -> None:
    """A real YRC1000 (an HC10DTP's) read one posture as form 4 over HSES (0x75,
    0x33D) and as type 1 in RPOSC: HSES's bit 2 is the Ethernet server's bit 0."""
    motoman.controller.form = 0b000100
    assert _answer(motoman, "RPOSC", "0,0").endswith(b",1,0\r")


def test_a_moves_type_is_kept_as_hses_numbers_the_form(motoman: YaskawaRobot) -> None:
    move = "50, 0, 100, 200, 300, 10, 20, 30, 1, 5, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVJ", move) == b"0000\r\n"
    assert motoman.controller.form == 0b000100


def test_loadv_then_savev_reads_a_position_back(motoman: YaskawaRobot) -> None:
    _answer(motoman, "LOADV", "4,9,1,0,1000.500,-2.000,300.000,180.00,0.00,-90.00,4,2")
    assert _answer(motoman, "SAVEV", "4,9") == (
        b"1,0,1000.500,-2.000,300.000,180.00,0.00,-90.00,4,2\r"
    )


def test_loadv_writes_a_bp_variable_in_base_coordinates(motoman: YaskawaRobot) -> None:
    assert _answer(motoman, "LOADV", "5,1,1,0,100,0,0,0,0,0,0,0") == b"0000\r\n"
    assert motoman.controller.positions["BP"][1] == (16, 100000, 0, 0, 0, 0, 0, 0, 0)


@pytest.mark.parametrize(
    "data",
    ["6,1,1,0,0,0,0,0,0,0,0,0",  # EX takes pulses only
     "5,1,1,1,0,0,0,0,0,0,0,0",  # BP takes base coordinates only
     "4,1,1,27,0,0,0,0,0,0,0,0",  # the master tool coordinate
     "4,1,0,1,2,3,4,5,6,24",  # tool 24 on an NX100
     "4,1,0,1,2,3",  # too few fields
     "4,128,0,0,0,0,0,0,0,0"],
)
def test_loadv_with_a_bad_position_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    assert _answer(motoman, "LOADV", data) == b"ERROR:LOADV is not successful (5200).\r\n"


@pytest.mark.parametrize(
    "data", ["0,7,256", "1,7,32768", "2,7,2147483648", "3,7,x", "3,7,inf", "3,7,nan",
             "0,100,1", "8,0,1", "0,7",
             "7,0," + "X" * 33],
)
def test_loadv_with_bad_data_is_an_error(motoman: YaskawaRobot, data: str) -> None:
    assert _answer(motoman, "LOADV", data) == b"ERROR:LOADV is not successful (5200).\r\n"


def _wait_idle(motoman: YaskawaRobot) -> None:
    deadline = time.monotonic() + 5
    while motoman.controller.moving:
        assert time.monotonic() < deadline, "the move did not finish"
        time.sleep(0.02)


def test_pmovj_moves_to_pulses(motoman: YaskawaRobot) -> None:
    target = "100, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", target) == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"202,64\r"  # running
    _wait_idle(motoman)
    assert _answer(motoman, "RPOSJ") == b"0,0,0,0,0,0,0,0,0,0,0,0\r"
    assert _answer(motoman, "RPOSC", "0,0").endswith(b",0,3\r")  # the move's tool


#: Two joint moves, each about half a second at 50 %: to zero, then S to 10 degrees.
TO_ZERO = "50, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
S_TO_10 = "50, 14166, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"


def _wait_all_moves(robot: YaskawaRobot) -> None:
    deadline = time.monotonic() + 5
    while robot.controller.moving or robot.controller.buffered_moves:
        assert time.monotonic() < deadline, "the moves did not finish"
        time.sleep(0.02)


class _Background:
    """A command sent on its own connection, whose answer arrives later."""

    def __init__(self, robot: YaskawaRobot, command: str, data: str) -> None:
        self.answer: bytes | None = None
        self._thread = threading.Thread(target=self._send, args=(robot, command, data))
        self._thread.start()

    def _send(self, robot: YaskawaRobot, command: str, data: str) -> None:
        self.answer = _answer(robot, command, data)

    def result(self) -> bytes | None:
        self._thread.join(5)
        return self.answer


def _until_buffered(robot: YaskawaRobot, count: int) -> None:
    deadline = time.monotonic() + 2
    while robot.controller.buffered_moves < count:
        assert time.monotonic() < deadline, "the move was not buffered"
        time.sleep(0.005)


def test_a_move_sent_while_moving_runs_after(dx200: YaskawaRobot) -> None:
    assert _answer(dx200, "PMOVJ", TO_ZERO) == b"0000\r\n"
    assert _answer(dx200, "PMOVJ", S_TO_10) == b"0000\r\n"
    _wait_all_moves(dx200)
    assert _answer(dx200, "RPOSJ") == b"14166,0,0,0,0,0,0,0,0,0,0,0\r"


def test_a_dx200_answers_a_buffered_move_when_it_starts(dx200: YaskawaRobot) -> None:
    """The YRC1000 manual (11.1.3) holds the response until the move starts."""
    _answer(dx200, "PMOVJ", TO_ZERO)
    sent = time.monotonic()
    assert _answer(dx200, "PMOVJ", S_TO_10) == b"0000\r\n"
    assert time.monotonic() - sent > 0.3
    assert dx200.controller.buffered_moves == 0


def test_an_nx100_answers_a_buffered_move_at_once(motoman: YaskawaRobot) -> None:
    _answer(motoman, "PMOVJ", TO_ZERO)
    assert _answer(motoman, "PMOVJ", S_TO_10) == b"0000\r\n"
    assert motoman.controller.buffered_moves == 1


def test_rstats_shows_running_until_the_buffered_move_ends(dx200: YaskawaRobot) -> None:
    _answer(dx200, "PMOVJ", TO_ZERO)
    second = _Background(dx200, "PMOVJ", S_TO_10)
    _until_buffered(dx200, 1)
    while dx200.controller.buffered_moves:
        assert _answer(dx200, "RSTATS").startswith(b"202,")  # running
    assert second.result() == b"0000\r\n"
    _wait_all_moves(dx200)
    assert _answer(dx200, "RSTATS").startswith(b"194,")


def test_a_dx200_takes_one_move_behind_the_running_one(dx200: YaskawaRobot) -> None:
    """The YRC1000 manual (11.1.3) accepts a single command while moving."""
    _answer(dx200, "PMOVJ", TO_ZERO)
    second = _Background(dx200, "PMOVJ", S_TO_10)
    _until_buffered(dx200, 1)
    assert _answer(dx200, "PMOVJ", TO_ZERO) == b"ERROR:PMOVJ is not successful (2010).\r\n"
    assert second.result() == b"0000\r\n"


def test_an_nx100_takes_many_moves_behind_the_running_one(motoman: YaskawaRobot) -> None:
    for move in (TO_ZERO, S_TO_10, TO_ZERO, S_TO_10):
        assert _answer(motoman, "PMOVJ", move) == b"0000\r\n"
    assert motoman.controller.buffered_moves == 3
    _wait_all_moves(motoman)
    assert _answer(motoman, "RPOSJ") == b"14166,0,0,0,0,0,0,0,0,0,0,0\r"


def test_the_move_buffer_is_configurable() -> None:
    options = YaskawaOptions(controller="nx100", start_joints_deg=START, move_buffer=0,
                             **UP50N_AXES)
    for robot in _running(options):
        _answer(robot, "PMOVJ", TO_ZERO)
        assert _answer(robot, "PMOVJ", S_TO_10) == b"ERROR:PMOVJ is not successful (2010).\r\n"


#: S to 10 degrees at 5 %, about a second.
S_TO_10_SLOW = "5, 14166, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"


def _first_move_done(robot: YaskawaRobot) -> None:
    deadline = time.monotonic() + 5
    while robot.controller.buffered_moves:
        assert time.monotonic() < deadline, "the first move did not finish"
        time.sleep(0.02)


def test_an_nx100_hides_a_buffered_move_from_rstats(motoman: YaskawaRobot) -> None:
    """The running bit quirk: the bit drops when the first move ends, though the
    move sent after it still runs."""
    notes: list[str] = []
    motoman.controller.note = lambda kind, **details: notes.append(kind)
    _answer(motoman, "PMOVJ", TO_ZERO)
    _answer(motoman, "PMOVJ", S_TO_10_SLOW)
    assert _answer(motoman, "RSTATS").startswith(b"202,")  # the first move shows
    _first_move_done(motoman)
    assert motoman.arm.state.view.moving
    assert _answer(motoman, "RSTATS").startswith(b"194,")  # the second does not
    assert "quirk" in notes


def test_the_running_bit_quirk_can_be_turned_off() -> None:
    options = YaskawaOptions(controller="nx100", start_joints_deg=START,
                             running_bit_quirk=False, **UP50N_AXES)
    for robot in _running(options):
        _answer(robot, "PMOVJ", TO_ZERO)
        _answer(robot, "PMOVJ", S_TO_10_SLOW)
        _first_move_done(robot)
        assert _answer(robot, "RSTATS").startswith(b"202,")


def test_a_dx200_shows_a_buffered_move_in_rstats(dx200: YaskawaRobot) -> None:
    _answer(dx200, "PMOVJ", TO_ZERO)
    _answer(dx200, "PMOVJ", S_TO_10_SLOW)
    _first_move_done(dx200)
    assert _answer(dx200, "RSTATS").startswith(b"202,")


def test_hold_drops_the_buffered_moves(dx200: YaskawaRobot) -> None:
    """A held answer to a dropped move is an error: 2050, command hold."""
    _answer(dx200, "PMOVJ", TO_ZERO)
    second = _Background(dx200, "PMOVJ", S_TO_10)
    _until_buffered(dx200, 1)
    _answer(dx200, "HOLD", "1")
    assert second.result() == b"ERROR:PMOVJ is not successful (2050).\r\n"
    assert dx200.controller.buffered_moves == 0
    assert not dx200.arm.state.view.moving


def test_a_buffered_step_starts_where_the_move_before_ends(up50n: YaskawaRobot) -> None:
    """Two IMOV steps of 10 mm along base Y, the second sent while the first runs."""
    step = "0, 50, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(up50n, "IMOV", step) == b"0000\r\n"
    assert _answer(up50n, "IMOV", step) == b"0000\r\n"
    _wait_all_moves(up50n)
    assert _same_pose(_answer(up50n, "RPOSC", "0,0"), (1170, 20, 805, 180, 0, 180))


def test_pmovj_takes_as_long_as_the_slowest_joint(motoman: YaskawaRobot) -> None:
    """T turns 90 degrees at 350 degrees/s; at 100 % that is the longest joint."""
    joints = "14166, 38908, -59831, 0, 36653, 0"
    started = time.monotonic()
    _answer(motoman, "PMOVJ", f"100, {joints}, 0, 0, 0, 0, 0, 0, 0")
    _wait_idle(motoman)
    assert time.monotonic() - started == pytest.approx(90 / 350, abs=0.1)


@pytest.mark.parametrize(
    ("data", "code"),
    [
        ("0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0", 5200),  # speed below 0.01 %
        ("10, 0, 0, 0, 0, 0, 0, 24, 0, 0, 0, 0, 0, 0", 5200),  # tool 24
        ("10, 0, 0, 0, 0, 0, 0, 0", 5200),  # too few fields
    ],
)
def test_pmovj_with_bad_data_is_an_error(motoman: YaskawaRobot, data: str, code: int) -> None:
    error = f"ERROR:PMOVJ is not successful ({code}).\r\n"
    assert _answer(motoman, "PMOVJ", data) == error.encode()


def test_a_move_while_servo_is_off_is_an_error(motoman: YaskawaRobot) -> None:
    motoman.arm.set_servo(False)
    move = "10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", move) == b"ERROR:PMOVJ is not successful (2070).\r\n"


def test_a_move_while_held_is_an_error(motoman: YaskawaRobot) -> None:
    motoman.controller.hold = True
    move = "10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", move) == b"ERROR:PMOVJ is not successful (2050).\r\n"


def test_pmovl_takes_speed_selection_and_speed(motoman: YaskawaRobot) -> None:
    move = "0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVL", move) == b"0000\r\n"
    bad = "2, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVL", bad) == b"ERROR:PMOVL is not successful (5200).\r\n"


def test_movj_moves_to_a_pose_and_records_its_form_and_tool(motoman: YaskawaRobot) -> None:
    move = "50, 0, 100, 200, 300, 10, 20, 30, 4, 5, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVJ", move) == b"0000\r\n"
    assert _answer(motoman, "RPOSC", "0,0").endswith(b",4,5\r")


def test_movl_moves_to_a_pose(motoman: YaskawaRobot) -> None:
    move = "0, 250, 0, 100, 200, 300, 10, 20, 30, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVL", move) == b"0000\r\n"


def test_movl_in_a_user_frame_is_an_error(motoman: YaskawaRobot) -> None:
    move = "0, 250, 2, 100, 200, 300, 10, 20, 30, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVL", move) == b"ERROR:MOVL is not successful (3350).\r\n"


@pytest.mark.parametrize("coordinate", [0, 26])
def test_imov_steps_in_base_or_tool_coordinates(motoman: YaskawaRobot, coordinate: int) -> None:
    step = f"0, 100, {coordinate}, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "IMOV", step) == b"0000\r\n"


def _teach(motoman: YaskawaRobot, frame: str) -> None:
    _answer(motoman, "MODE", "1")
    assert _answer(motoman, "WUFRAME", frame) == b"0000\r\n"
    _answer(motoman, "MODE", "2")


def test_rposc_reads_the_pose_in_a_taught_user_frame(motoman: YaskawaRobot) -> None:
    """A frame 100 mm along base X, not turned: the pose shifts by 100 mm."""
    _teach(motoman, "2, 100,0,0,0,0,0,0, 200,0,0,0,0,0,0, 100,100,0,0,0,0,0, 0, 0,0,0,0,0,0")
    assert _answer(motoman, "RPOSC", "2,0") == (
        b"74.533,349.066,-523.599,0.00,45.00,-90.00,0,0\r"
    )


def _same_pose(answer: bytes, expected: tuple[float, ...]) -> bool:
    """Whether RPOSC's X Y Z (mm) and Rx Ry Rz (degrees) are *expected*: to 0.1 mm, as
    the arm's inverse kinematics lands within hundredths, and the angles compared as
    the rotation they make."""
    got = [float(v) for v in answer.split(b",")[:6]]
    a, b = (pose_to_mat((*(v / 1000 for v in p[:3]), *(math.radians(v) for v in p[3:])))
            for p in (got, expected))
    return bool(np.allclose(a[:3, 3], b[:3, 3], atol=1e-4)
                and np.allclose(a[:3, :3], b[:3, :3], atol=1e-4))


def test_movl_in_a_taught_user_frame_moves_in_that_frame(up50n: YaskawaRobot) -> None:
    """FRAME_1 is turned 90 degrees about Z: its X is base Y, its Y base -X. The arm
    starts at X 1170, Z 805, tool down."""
    _teach(up50n, FRAME_1)
    move = "0, 500, 2, 50, -100, 805, 180, 0, 90, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(up50n, "MOVL", move) == b"0000\r\n"
    _wait_idle(up50n)
    assert _same_pose(_answer(up50n, "RPOSC", "0,0"), (1100, 50, 805, 180, 0, 180))
    assert _same_pose(_answer(up50n, "RPOSC", "2,0"), (50, -100, 805, 180, 0, 90))


def test_imov_in_a_taught_user_frame_steps_along_its_axes(up50n: YaskawaRobot) -> None:
    _teach(up50n, FRAME_1)
    step = "0, 100, 2, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(up50n, "IMOV", step) == b"0000\r\n"
    _wait_idle(up50n)
    assert _same_pose(_answer(up50n, "RPOSC", "0,0"), (1170, 10, 805, 180, 0, 180))


def test_iowrite_then_ioread_as_in_the_manual(motoman: YaskawaRobot) -> None:
    """3.3.3: write three bytes from #22010, then read them back."""
    assert _answer(motoman, "IOWRITE", "22010,24,63,0,25") == b"0000\r\n"
    assert _answer(motoman, "IOREAD", "22010,24") == b"63,0,25\r"


def test_ioread_of_unset_signals_reads_zero(motoman: YaskawaRobot) -> None:
    """3.3.2 reads three bytes from #50010."""
    assert _answer(motoman, "IOREAD", "50010, 24") == b"0,0,0\r"


def test_iowrite_outside_the_network_inputs_is_an_error(motoman: YaskawaRobot) -> None:
    error = b"ERROR:IOWRITE is not successful (2110).\r\n"
    assert _answer(motoman, "IOWRITE", "30040,8,1") == error


@pytest.mark.parametrize(
    ("command", "data"),
    [
        ("IOREAD", "22011,8"),  # not the first signal of a byte
        ("IOREAD", "22010,4"),  # not a multiple of eight
        ("IOWRITE", "22010,16,1"),  # one byte for two
        ("IOWRITE", "22010,8,256"),  # not a byte
    ],
)
def test_io_with_bad_data_is_an_error(motoman: YaskawaRobot, command: str, data: str) -> None:
    error = f"ERROR:{command} is not successful (5200).\r\n"
    assert _answer(motoman, command, data) == error.encode()


def test_the_ladder_copies_a_network_input_byte() -> None:
    options = YaskawaOptions(controller="nx100", ladder_copies={22010: 30040}, **UP50N_AXES)
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        assert _answer(m, "IOWRITE", "22010,8,5") == b"0000\r\n"
        assert _answer(m, "IOREAD", "30040,8") == b"5\r"
    finally:
        m.stop()


@pytest.mark.parametrize("copies", [{20010: 30040}, {22011: 30040}, {22010: 30041}])
def test_ladder_copies_name_network_input_bytes(copies: dict[int, int]) -> None:
    with pytest.raises(ValueError, match=r"ladder copies|first signal"):
        YaskawaOptions(controller="nx100", ladder_copies=copies, **UP50N_AXES)


# --- DX200 (HW1481823): what differs from the NX100 -------------------------


def test_the_up50n_35_reaches_as_riacts_model_of_it() -> None:
    """The long-reach UP50N: 1.150 m L to U and 1.225 m U to the wrist, so with the
    flange down at zero it is 1.370 m out and 1.085 m up, where riflex draws it."""
    options = YaskawaOptions(controller="nx100", robot="up50n_35",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"1370.000,0.000,1085.000,")


def test_the_mh50_reaches_as_its_data_sheet() -> None:
    """0.870 m L to U, 0.210 m up and 1.025 m out to the wrist, 0.175 m to the flange:
    with the flange down at zero it is 1.170 m out and 0.905 m up."""
    options = YaskawaOptions(controller="nx100", robot="mh50",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"1170.000,0.000,905.000,")


def test_the_mh50_35_reaches_as_its_data_sheet() -> None:
    """1.150 m L to U, 0.210 m up and 1.225 m out to the wrist, 0.175 m to the flange:
    with the flange down at zero it is 1.370 m out and 1.185 m up."""
    options = YaskawaOptions(controller="nx100", robot="mh50_35",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"1370.000,0.000,1185.000,")


def test_the_mh50_20_reaches_as_its_data_sheet() -> None:
    """1.150 m L to U, 0.200 m up and 1.800 m out to the wrist, 0.105 m to the flange:
    at zero the flange points out, 2.050 m out and 1.350 m up. Its B stops at -50
    degrees, so it cannot start flange down as the others do."""
    options = YaskawaOptions(controller="nx100", robot="mh50_20",
                             start_joints_deg=(0, 0, 0, 0, 0, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"2050.000,0.000,1350.000,")


def test_the_gp4_reaches_as_its_model() -> None:
    """0.260 m L to U, 0.015 m up and 0.290 m out to the wrist, 0.072 m to the flange:
    with the flange down at zero it is 0.290 m out and 0.203 m up."""
    options = YaskawaOptions(controller="yrc1000", robot="gp4",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"290.000,0.000,203.000,")


def test_the_gp7_reaches_as_its_model() -> None:
    """0.040 m S to L, 0.445 m L to U, 0.040 m up and 0.440 m out to the wrist, 0.080 m
    to the flange: with the flange down at zero it is 0.480 m out and 0.405 m up."""
    options = YaskawaOptions(controller="yrc1000", robot="gp7",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"480.000,0.000,405.000,")


def test_the_hc10dtp_reaches_as_its_model() -> None:
    """0.700 m L to U, 0.500 m U to R, B 0.162 m to the side, 0.170 m to the flange:
    with the flange down at zero it is 0.500 m out, 0.162 m aside and 0.530 m up."""
    options = YaskawaOptions(controller="yrc1000", robot="hc10dtp",
                             start_joints_deg=(0, 0, 0, 0, -90, 0))
    for robot in _running(options):
        assert _answer(robot, "RPOSC", "0,0").startswith(b"500.000,162.000,530.000,")


def test_the_hc10dtp_flange_turns_as_a_real_ones() -> None:
    """A real HC10DTP at these joints read its flange (tool 2, all zeros) at
    517.391, -61.828, 296.519 and Rx -180, Ry 0, Rz -180: its X along base -X, its Y
    along base +Y, its Z down."""
    options = YaskawaOptions(controller="yrc1000", robot="hc10dtp",
                             start_joints_deg=(-24.93, 4.27, -23.32, 0.0, -62.41, -155.07))
    for robot in _running(options):
        fields = [float(v) for v in _answer(robot, "RPOSC", "0,0").split(b",")[:6]]
        assert fields[:3] == pytest.approx([517.391, -61.828, 296.519], abs=0.5)
        rotation = pose_to_mat((0, 0, 0, *(math.radians(v) for v in fields[3:])))[:3, :3]
        assert rotation == pytest.approx(np.diag([-1.0, 1.0, -1.0]), abs=1e-3)


def test_the_hc10dtp_counts_pulses_as_a_real_one() -> None:
    """A real HC10DTP's ALL.PRM: its top speeds in pulses/s and its soft limits give
    2671.305, 2671.306, 2347.416 and 1619.146 pulses per degree."""
    options = YaskawaOptions(controller="yrc1000", robot="hc10dtp",
                             start_joints_deg=(10, 10, 10, 10, 10, 10))
    for robot in _running(options):
        assert _answer(robot, "RPOSJ").startswith(b"26713,26713,23474,16191,16191,16191,")


#: A real GP8's joints, and its flange (tool 3, all zeros) as its controller read it.
GP8_JOINTS = (-53.69, 49.298, 6.273, -117.022, -64.038, -128.842)
GP8_FLANGE = (421.904, -465.945, 22.228, 90.004, 1.793, 89.529)


def test_the_gp8_flange_is_where_a_real_ones_is() -> None:
    """Position to 0.001 mm on the real one; the frame turned as its controller reports it."""
    options = YaskawaOptions(controller="yrc1000", robot="gp8", start_joints_deg=GP8_JOINTS)
    for robot in _running(options):
        fields = [float(v) for v in _answer(robot, "RPOSC", "0,0").split(b",")[:6]]
        assert fields[:3] == pytest.approx(GP8_FLANGE[:3], abs=0.5)
        rotation = pose_to_mat((0, 0, 0, *(math.radians(v) for v in fields[3:])))[:3, :3]
        real = pose_to_mat((0, 0, 0, *(math.radians(v) for v in GP8_FLANGE[3:])))[:3, :3]
        assert rotation == pytest.approx(real, abs=1e-3)


def test_the_gp8_counts_pulses_and_names_itself_as_a_real_one() -> None:
    """S at exactly 1024 pulses per degree; the model field as its HSES gives it."""
    options = YaskawaOptions(controller="yrc1000", robot="gp8",
                             start_joints_deg=(10, 10, 10, 10, 10, 10))
    for robot in _running(options):
        assert _answer(robot, "RPOSJ").startswith(b"10240,12136,9102,8533,7282,4649,")
        assert robot.controller.model == "1-06VX8-A0*(GP8)"


#: A real GP4's joints, and its flange (tool 1, all zeros) as its YRC1000micro read it.
GP4_JOINTS = (12.42, 4.574, -41.782, -2.23, -46.87, -342.226)
GP4_FLANGE = (221.948, 50.973, -12.188, -177.5411, 2.6209, 28.6797)


def test_the_gp4_flange_is_where_a_real_ones_is() -> None:
    """Position to 0.001 mm on the real one; the frame turned as its controller reports it."""
    options = YaskawaOptions(controller="yrc1000", robot="gp4", start_joints_deg=GP4_JOINTS)
    for robot in _running(options):
        fields = [float(v) for v in _answer(robot, "RPOSC", "0,0").split(b",")[:6]]
        assert fields[:3] == pytest.approx(GP4_FLANGE[:3], abs=0.5)
        rotation = pose_to_mat((0, 0, 0, *(math.radians(v) for v in fields[3:])))[:3, :3]
        real = pose_to_mat((0, 0, 0, *(math.radians(v) for v in GP4_FLANGE[3:])))[:3, :3]
        assert rotation == pytest.approx(real, abs=1e-3)


def test_the_gp4_counts_pulses_and_names_itself_as_a_real_one() -> None:
    """A real GP4's ALL.PRM, on a YRC1000micro: S at 1028.6559 pulses per degree; the
    model field as its HSES gives it."""
    options = YaskawaOptions(controller="yrc1000", robot="gp4",
                             start_joints_deg=(10, 10, 10, 10, 10, 10))
    for robot in _running(options):
        assert _answer(robot, "RPOSJ").startswith(b"10287,9216,7964,8440,7737,4551,")
        assert robot.controller.model == "1-06VX4-A0*(GP4)"


def test_dx200_names_itself_dx(dx200: YaskawaRobot) -> None:
    received = _session(dx200, b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST RSTATS 0\r\n")
    assert received.startswith(b"OK: DX Information Server (1.00).\r\n")


def test_dx200_answers_rotation_to_4_decimals(dx200: YaskawaRobot) -> None:
    assert _answer(dx200, "RPOSC", "0,0") == (
        b"174.533,349.066,-523.599,0.0000,45.0000,-90.0000,0,0\r"
    )


@pytest.mark.parametrize(("tool", "answer"), [(63, b"0000\r\n"), (64, None)])
def test_dx200_has_64_tools(dx200: YaskawaRobot, tool: int, answer: bytes | None) -> None:
    move = f"10, 14166, 38908, -59831, 0, 36653, -37676, {tool}, 0, 0, 0, 0, 0, 0"
    expected = answer or b"ERROR:PMOVJ is not successful (5200).\r\n"
    assert _answer(dx200, "PMOVJ", move) == expected


@pytest.mark.parametrize(("coordinate", "code"), [(65, 3350), (66, 5200)])
def test_dx200_has_64_user_frames(dx200: YaskawaRobot, coordinate: int, code: int) -> None:
    error = f"ERROR:RPOSC is not successful ({code}).\r\n"
    assert _answer(dx200, "RPOSC", f"{coordinate},0") == error.encode()


def test_dx200_steps_in_the_tool_coordinate_66(dx200: YaskawaRobot) -> None:
    step = "0, 100, 66, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(dx200, "IMOV", step) == b"0000\r\n"


def test_dx200_network_inputs_start_at_27010(dx200: YaskawaRobot) -> None:
    assert _answer(dx200, "IOWRITE", "27010,24,63,0,25") == b"0000\r\n"
    assert _answer(dx200, "IOREAD", "27010,24") == b"63,0,25\r"
    error = b"ERROR:IOWRITE is not successful (2110).\r\n"
    assert _answer(dx200, "IOWRITE", "22010,8,1") == error


def test_ladder_copies_follow_the_controllers_network_inputs() -> None:
    YaskawaOptions(controller="dx200", ladder_copies={27010: 30040}, **UP50N_AXES)
    with pytest.raises(ValueError, match="not a network input"):
        YaskawaOptions(controller="dx200", ladder_copies={22010: 30040}, **UP50N_AXES)


def test_the_yrc1000_names_itself_yr_and_otherwise_takes_the_dx200s_values() -> None:
    """Its name as a real HC10DTP's YRC1000 gave it: no space before the bracket, one
    inside it."""
    yrc1000, dx200 = CONTROLLERS["yrc1000"], CONTROLLERS["dx200"]
    assert yrc1000.server_name == "OK: YR Information Server( 1.00)"
    assert (yrc1000.rotation_decimals, yrc1000.max_tool, yrc1000.user_frames,
            yrc1000.network_inputs) == (dx200.rotation_decimals, dx200.max_tool,
                                        dx200.user_frames, dx200.network_inputs)


# --- the robot catalog ---------------------------------------------------------


def test_naming_a_robot_fills_in_its_urdf_pulses_and_speeds() -> None:
    options = YaskawaOptions(controller="nx100", robot="up50n")
    assert options.urdf == str(ROBOTS["up50n"].urdf)
    assert options.pulses_per_degree == ROBOTS["up50n"].pulses_per_degree
    assert options.max_joint_speed_deg_s == ROBOTS["up50n"].max_joint_speed_deg_s


def test_a_scene_overrides_what_the_robot_fills_in() -> None:
    options = YaskawaOptions(controller="nx100", robot="up50n", max_joint_speed_deg_s=(1.0,) * 6)
    assert options.max_joint_speed_deg_s == (1.0,) * 6
    assert options.pulses_per_degree == ROBOTS["up50n"].pulses_per_degree


def test_an_unknown_robot_is_refused() -> None:
    with pytest.raises(ValueError, match=r"unknown robot 'up5o'; known: .*up50n"):
        YaskawaOptions(controller="nx100", robot="up5o")


def test_without_a_robot_the_scene_gives_the_axes() -> None:
    with pytest.raises(ValueError, match="pulses_per_degree"):
        YaskawaOptions(controller="nx100")


def test_the_up50n_reports_its_flange_where_the_urdf_puts_it() -> None:
    """At the zero position the flange is 1345 mm out and 980 mm up, Z along base X."""
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(),
                     YaskawaOptions(controller="nx100", robot="up50n"))
    x, y, z, rx, ry, rz = m.arm.state.view.pose
    assert (x, y, z) == pytest.approx((1.345, 0.0, 0.980))
    assert (rx, ry, rz) == pytest.approx((0.0, 1.5707963, 0.0), abs=1e-6)


# --- the yaskawa kind and its protocols -----------------------------------------


def test_the_yaskawa_kind_needs_a_controller() -> None:
    with pytest.raises(ValueError, match="controller"):
        default_registry.create("yaskawa", "r1", Endpoint("127.0.0.1", 0), EventBus(),
                                {"robot": "up50n"})


@pytest.mark.parametrize(
    ("protocols", "complaint"),
    [((), "protocols is empty"), (("ethernet_server", "ethernet_server"), "one twice")],
)
def test_protocols_must_name_each_server_once(protocols: tuple[str, ...], complaint: str) -> None:
    with pytest.raises(ValueError, match=complaint):
        YaskawaOptions(controller="yrc1000", robot="up50n", protocols=protocols)


def test_an_unknown_protocol_is_refused() -> None:
    with pytest.raises(ValueError, match="protocols"):
        YaskawaOptions(controller="yrc1000", robot="up50n", protocols=("modbus",))


def test_the_nx100_has_no_hses() -> None:
    with pytest.raises(ValueError, match="the NX100 has no HSES"):
        YaskawaOptions(controller="nx100", robot="up50n", protocols=("hses",))


def test_without_the_ethernet_server_nothing_listens_on_its_port() -> None:
    port = free_port()
    options = YaskawaOptions(controller="yrc1000", robot="up50n", protocols=("hses",),
                             hses_port=free_port())
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", port), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        with pytest.raises(ConnectionRefusedError):
            socket.create_connection(("127.0.0.1", port), timeout=1)
    finally:
        m.stop()


def test_motoman_nx100_is_a_yaskawa_nx100_with_a_up50n() -> None:
    m = default_registry.create("motoman_nx100", "r1", Endpoint("127.0.0.1", free_port()),
                                EventBus(), {})
    assert isinstance(m, YaskawaRobot)
    assert m.controller.profile == CONTROLLERS["nx100"]
    assert m.controller.pulses_per_degree == ROBOTS["up50n"].pulses_per_degree
    assert m.protocols == ("ethernet_server",)


def test_command_data_may_end_with_cr_lf_counted_in_the_size(motoman: YaskawaRobot) -> None:
    """EPICS StreamDevice sends "1, 0<CR><LF>" with size 6."""
    received = _session(
        motoman,
        b"CONNECT Robot_access Keep-Alive:2\r\n"
        b"HOSTCTRL_REQUEST RPOSC 6\r\n1, 0\r\n"
        b"HOSTCTRL_REQUEST RSTATS 0\r\n",
    )
    assert b"OK: RPOSC\r\n174.533," in received
    assert received.endswith(b"OK: RSTATS\r\n194,64\r")


def test_the_ok_line_and_the_answer_arrive_apart(motoman: YaskawaRobot) -> None:
    """MotomanConnector reads the OK line and the answer with one recv each."""
    with _connect(motoman) as s:
        s.sendall(b"CONNECT Robot_access Keep-Alive:-1\r\n")
        assert s.recv(1024).startswith(b"OK: NX Information Server")
        s.sendall(b"HOSTCTRL_REQUEST RSTATS 0\r\n")
        assert s.recv(1024) == b"OK: RSTATS\r\n"
        assert s.recv(1024) == b"194,64\r"


def _device(**options: object) -> YaskawaRobot:
    return YaskawaRobot("r1", Endpoint("127.0.0.1", 8000), EventBus(),
                        YaskawaOptions(start_joints_deg=START, **UP50N_AXES, **options))


def test_the_settings_name_the_controller_and_the_arm() -> None:
    settings = _device(controller="dx200", robot="up50n").settings()
    assert settings[0] == "controller DX200, arm up50n"


def test_the_settings_name_each_protocol_and_where_it_serves() -> None:
    settings = _device(controller="dx200", protocols=("ethernet_server", "hses"),
                       hses_port=9000, hses_file_port=9001).settings()
    assert "Ethernet server on TCP 127.0.0.1:8000" in settings
    assert "HSES on UDP 127.0.0.1:9000, files on :9001" in settings


def test_the_settings_leave_out_a_protocol_not_served() -> None:
    settings = _device(controller="dx200", protocols=("hses",)).settings()
    assert not any(line.startswith("Ethernet server") for line in settings)


def test_the_settings_give_the_move_buffer() -> None:
    assert "moves taken while moving: 1" in _device(controller="dx200").settings()
    assert "moves taken while moving: unlimited" in _device(controller="nx100").settings()


def test_the_settings_name_the_running_bit_quirk_only_when_on() -> None:
    assert any("running bit quirk" in line for line in _device(controller="nx100").settings())
    assert not any("running bit quirk" in line for line in _device(controller="dx200").settings())


def test_a_controller_set_to_start_with_its_servos_off_reports_them_off() -> None:
    for robot in _running(YaskawaOptions(controller="nx100", start_joints_deg=START,
                                         servo_on=False, **UP50N_AXES)):
        received = _session(robot, b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST RSTATS 0\r\n")
        assert received.endswith(b"OK: RSTATS\r\n194,0\r")


def test_the_settings_say_when_the_servos_start_off() -> None:
    assert "servos off" in _device(controller="dx200", servo_on=False).settings()
    assert "servos off" not in _device(controller="dx200").settings()
