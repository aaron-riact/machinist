"""The NX100 Ethernet server against the manual (HW0483202).

The byte strings follow the manual's transmission examples (section 3.3).
"""

from __future__ import annotations

import socket
import time
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import MotomanNX100, MotomanOptions

from ..conftest import free_port, wait_running


@pytest.fixture
def motoman() -> Iterator[MotomanNX100]:
    bus = EventBus()
    options = MotomanOptions(start_joints_deg=(10, 20, -30, 0, 45, -90))
    m = MotomanNX100("r1", Endpoint("127.0.0.1", free_port()), bus, options)
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


def _connect(motoman: MotomanNX100) -> socket.socket:
    return socket.create_connection((motoman.endpoint.host, motoman.endpoint.port), timeout=2)


def _session(motoman: MotomanNX100, sent: bytes) -> bytes:
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


def test_single_command_answers_then_closes(motoman: MotomanNX100) -> None:
    """3.3.1: one RSTATS without Keep-Alive, then the controller closes."""
    received = _session(motoman, b"CONNECT Robot_access\r\nHOSTCTRL_REQUEST RSTATS 0\r\n")
    assert received == (
        b"OK: NX Information Server (3.00).\r\n"
        b"OK: RSTATS\r\n"
        b"194,64\r"
    )


def test_keep_alive_answers_n_commands_then_closes(motoman: MotomanNX100) -> None:
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


def test_keep_alive_minus_one_keeps_the_socket_open(motoman: MotomanNX100) -> None:
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
def test_a_bad_start_request_is_refused_and_closed(motoman: MotomanNX100, start: bytes) -> None:
    assert _session(motoman, start) == b"NG: HTTP Error Response\r\n"


def test_an_unknown_command_is_refused_and_closed(motoman: MotomanNX100) -> None:
    received = _session(
        motoman, b"CONNECT Robot_access Keep-Alive:-1\r\nHOSTCTRL_REQUEST NOPE 0\r\n"
    )
    assert received.endswith(b"Keep-Alive:-1.\r\nNG: unknown command NOPE\r\n")


def test_a_size_that_does_not_count_the_data_is_refused(motoman: MotomanNX100) -> None:
    """The size counts the data and its closing CR: ``ab<CR>`` is 3 bytes."""
    received = _session(
        motoman, b"CONNECT Robot_access Keep-Alive:-1\r\nHOSTCTRL_REQUEST RSTATS 2\r\nab\r"
    )
    assert received.endswith(b"OK: RSTATS\r\nNG: size 2 for 3 bytes\r\n")


def _answer(motoman: MotomanNX100, command: str, data: str = "") -> bytes:
    """The answer to one *command*, after its OK line."""
    size = len(data) + 1 if data else 0
    sent = f"CONNECT Robot_access\r\nHOSTCTRL_REQUEST {command} {size}\r\n"
    if data:
        sent += data + "\r"
    received = _session(motoman, sent.encode())
    return received.split(f"OK: {command}\r\n".encode(), 1)[1]


def test_rposj_answers_twelve_axes_in_pulses(motoman: MotomanNX100) -> None:
    """S, L, U, R, B, T in encoder pulses, then six external axes."""
    assert _answer(motoman, "RPOSJ") == b"14166,38908,-59831,0,36653,-37676,0,0,0,0,0,0\r"


def test_pulses_per_degree_needs_one_ratio_per_joint() -> None:
    with pytest.raises(ValueError, match="2 pulses_per_degree for 6 joints"):
        MotomanOptions(pulses_per_degree=(1.0, 2.0))


def test_rposc_answers_mm_degrees_form_and_tool(motoman: MotomanNX100) -> None:
    """X, Y, Z in mm (3 decimals), Rx, Ry, Rz in degrees (2 decimals), type, tool.

    The fixture's arm has no kinematics, so its pose is its joint values.
    """
    assert _answer(motoman, "RPOSC", "0, 0") == (
        b"174.533,349.066,-523.599,0.00,45.00,-90.00,0,0\r"
    )


def test_rposc_with_external_axes_adds_six(motoman: MotomanNX100) -> None:
    assert _answer(motoman, "RPOSC", "1,1").endswith(b",0,0,0,0,0,0,0,0\r")


def test_rposc_in_an_undefined_user_frame_is_an_error(motoman: MotomanNX100) -> None:
    assert _answer(motoman, "RPOSC", "2,0") == b"ERROR:RPOSC is not successful (9002).\r\n"


@pytest.mark.parametrize("data", ["26,0", "0,2", "0", "a,b"])
def test_rposc_with_bad_data_is_an_error(motoman: MotomanNX100, data: str) -> None:
    assert _answer(motoman, "RPOSC", data) == b"ERROR:RPOSC is not successful (9001).\r\n"


def test_ralarm_answers_no_error_and_no_alarm(motoman: MotomanNX100) -> None:
    assert _answer(motoman, "RALARM") == b"0,0,0,0,0,0,0,0,0,0\r"


def test_ralarm_reports_a_fault_as_an_alarm(motoman: MotomanNX100) -> None:
    motoman.arm.fault()
    assert _answer(motoman, "RALARM") == b"0,0,4000,0,0,0,0,0,0,0\r"
    assert _answer(motoman, "RSTATS") == b"194,80\r"


def test_reset_clears_the_alarm(motoman: MotomanNX100) -> None:
    motoman.arm.fault()
    assert _answer(motoman, "RESET") == b"0000\r\n"
    assert _answer(motoman, "RALARM") == b"0,0,0,0,0,0,0,0,0,0\r"


def test_svon_switches_the_servo(motoman: MotomanNX100) -> None:
    assert _answer(motoman, "SVON", "0") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,0\r"
    assert _answer(motoman, "SVON", "1") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,64\r"


def test_svon_during_an_emergency_stop_is_an_error(motoman: MotomanNX100) -> None:
    motoman.arm.estop()
    assert _answer(motoman, "SVON", "1") == b"ERROR:SVON is not successful (9003).\r\n"


def test_hold_shows_in_rstats_until_released(motoman: MotomanNX100) -> None:
    assert _answer(motoman, "HOLD", "1") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,72\r"
    assert _answer(motoman, "HOLD", "0") == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"194,64\r"


@pytest.mark.parametrize("command", ["SVON", "HOLD"])
def test_a_switch_takes_only_0_or_1(motoman: MotomanNX100, command: str) -> None:
    error = f"ERROR:{command} is not successful (9001).\r\n"
    assert _answer(motoman, command, "2") == error.encode()


def test_cancel_answers_normally(motoman: MotomanNX100) -> None:
    assert _answer(motoman, "CANCEL") == b"0000\r\n"


def _wait_idle(motoman: MotomanNX100) -> None:
    deadline = time.monotonic() + 5
    while motoman.arm.state.view.moving:
        assert time.monotonic() < deadline, "the move did not finish"
        time.sleep(0.02)


def test_pmovj_moves_to_pulses(motoman: MotomanNX100) -> None:
    target = "100, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", target) == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"202,64\r"  # running
    _wait_idle(motoman)
    assert _answer(motoman, "RPOSJ") == b"0,0,0,0,0,0,0,0,0,0,0,0\r"
    assert _answer(motoman, "RPOSC", "0,0").endswith(b",0,3\r")  # the move's tool


def test_pmovj_takes_as_long_as_the_slowest_joint(motoman: MotomanNX100) -> None:
    """T turns 90 degrees at 350 degrees/s; at 100 % that is the longest joint."""
    joints = "14166, 38908, -59831, 0, 36653, 0"
    started = time.monotonic()
    _answer(motoman, "PMOVJ", f"100, {joints}, 0, 0, 0, 0, 0, 0, 0")
    _wait_idle(motoman)
    assert time.monotonic() - started == pytest.approx(90 / 350, abs=0.1)


@pytest.mark.parametrize(
    ("data", "code"),
    [
        ("0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0", 9001),  # speed below 0.01 %
        ("10, 0, 0, 0, 0, 0, 0, 24, 0, 0, 0, 0, 0, 0", 9001),  # tool 24
        ("10, 0, 0, 0, 0, 0, 0, 0", 9001),  # too few fields
    ],
)
def test_pmovj_with_bad_data_is_an_error(motoman: MotomanNX100, data: str, code: int) -> None:
    error = f"ERROR:PMOVJ is not successful ({code}).\r\n"
    assert _answer(motoman, "PMOVJ", data) == error.encode()


def test_a_move_while_servo_is_off_is_an_error(motoman: MotomanNX100) -> None:
    motoman.arm.set_servo(False)
    move = "10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", move) == b"ERROR:PMOVJ is not successful (9004).\r\n"


def test_a_move_while_held_is_an_error(motoman: MotomanNX100) -> None:
    motoman.controller.hold = True
    move = "10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", move) == b"ERROR:PMOVJ is not successful (9005).\r\n"


def test_pmovl_takes_speed_selection_and_speed(motoman: MotomanNX100) -> None:
    move = "0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVL", move) == b"0000\r\n"
    bad = "2, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVL", bad) == b"ERROR:PMOVL is not successful (9001).\r\n"


def test_movj_moves_to_a_pose_and_records_its_form_and_tool(motoman: MotomanNX100) -> None:
    move = "50, 0, 100, 200, 300, 10, 20, 30, 4, 5, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVJ", move) == b"0000\r\n"
    assert _answer(motoman, "RPOSC", "0,0").endswith(b",4,5\r")


def test_movl_moves_to_a_pose(motoman: MotomanNX100) -> None:
    move = "0, 250, 0, 100, 200, 300, 10, 20, 30, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVL", move) == b"0000\r\n"


def test_movl_in_a_user_frame_is_an_error(motoman: MotomanNX100) -> None:
    move = "0, 250, 2, 100, 200, 300, 10, 20, 30, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "MOVL", move) == b"ERROR:MOVL is not successful (9002).\r\n"


@pytest.mark.parametrize("coordinate", [0, 26])
def test_imov_steps_in_base_or_tool_coordinates(motoman: MotomanNX100, coordinate: int) -> None:
    step = f"0, 100, {coordinate}, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "IMOV", step) == b"0000\r\n"


def test_iowrite_then_ioread_as_in_the_manual(motoman: MotomanNX100) -> None:
    """3.3.3: write three bytes from #22010, then read them back."""
    assert _answer(motoman, "IOWRITE", "22010,24,63,0,25") == b"0000\r\n"
    assert _answer(motoman, "IOREAD", "22010,24") == b"63,0,25\r"


def test_ioread_of_unset_signals_reads_zero(motoman: MotomanNX100) -> None:
    """3.3.2 reads three bytes from #50010."""
    assert _answer(motoman, "IOREAD", "50010, 24") == b"0,0,0\r"


def test_iowrite_outside_the_network_inputs_is_an_error(motoman: MotomanNX100) -> None:
    error = b"ERROR:IOWRITE is not successful (9007).\r\n"
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
def test_io_with_bad_data_is_an_error(motoman: MotomanNX100, command: str, data: str) -> None:
    error = f"ERROR:{command} is not successful (9001).\r\n"
    assert _answer(motoman, command, data) == error.encode()


def test_the_ladder_copies_a_network_input_byte() -> None:
    options = MotomanOptions(ladder_copies={22010: 30040})
    m = MotomanNX100("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
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
        MotomanOptions(ladder_copies=copies)
