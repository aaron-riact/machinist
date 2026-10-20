"""The NX100 Ethernet server against the manual (HW0483202).

The byte strings follow the manual's transmission examples (section 3.3).
"""

from __future__ import annotations

import socket
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.motoman import MotomanNX100, MotomanOptions

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
