"""The NX100 Ethernet server against the manual (HW0483202).

The byte strings follow the manual's transmission examples (section 3.3).
"""

from __future__ import annotations

import socket
import time
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import CONTROLLERS, ROBOTS, YaskawaOptions, YaskawaRobot

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


@pytest.mark.parametrize("command", ["SVON", "HOLD"])
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


def _wait_idle(motoman: YaskawaRobot) -> None:
    deadline = time.monotonic() + 5
    while motoman.arm.state.view.moving:
        assert time.monotonic() < deadline, "the move did not finish"
        time.sleep(0.02)


def test_pmovj_moves_to_pulses(motoman: YaskawaRobot) -> None:
    target = "100, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0, 0, 0, 0"
    assert _answer(motoman, "PMOVJ", target) == b"0000\r\n"
    assert _answer(motoman, "RSTATS") == b"202,64\r"  # running
    _wait_idle(motoman)
    assert _answer(motoman, "RPOSJ") == b"0,0,0,0,0,0,0,0,0,0,0,0\r"
    assert _answer(motoman, "RPOSC", "0,0").endswith(b",0,3\r")  # the move's tool


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
    yrc1000, dx200 = CONTROLLERS["yrc1000"], CONTROLLERS["dx200"]
    assert yrc1000.server_name == "OK: YR Information Server (1.00)"
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
    with pytest.raises(ValueError, match="unknown robot 'up5o'; known: up50n"):
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
