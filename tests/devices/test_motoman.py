from __future__ import annotations

import socket

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions
from machinist.devices.robots.motoman import MotomanNX100

from ..conftest import free_port, wait_running


@pytest.fixture
def motoman() -> MotomanNX100:
    bus = EventBus()
    m = MotomanNX100("r1", Endpoint("127.0.0.1", free_port()), bus, options=ArmOptions())
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


def _talk(motoman: MotomanNX100, lines: list[str], *, expect: int) -> list[str]:
    """Send CRLF-terminated lines, read ``expect`` CRLF-terminated replies."""
    with socket.create_connection(
        (motoman.endpoint.host, motoman.endpoint.port), timeout=2
    ) as s:
        s.sendall("".join(f"{line}\r\n" for line in lines).encode())
        buf = b""
        while buf.count(b"\r\n") < expect:
            chunk = s.recv(4096)
            if not chunk:
                break
            buf += chunk
        return [p for p in buf.decode().split("\r\n") if p]


def test_connect_robot_access_succeeds(motoman: MotomanNX100) -> None:
    """Regression for observed ngrep trace where CONNECT produced E2010."""
    replies = _talk(motoman, ["CONNECT Robot_access Keep-Alive:-1"], expect=1)
    assert replies[0].startswith("OK: NX Information Server")
    assert "Keep-Alive:-1" in replies[0]


def test_commands_before_connect_are_rejected(motoman: MotomanNX100) -> None:
    replies = _talk(motoman, ["HOSTCTRL_REQUEST RSTATS 0"], expect=1)
    assert replies[0].startswith("NG:")


def test_rstats_after_connect(motoman: MotomanNX100) -> None:
    replies = _talk(
        motoman,
        ["CONNECT Robot_access", "HOSTCTRL_REQUEST RSTATS 0"],
        expect=3,
    )
    # 1: connect banner, 2: OK: RSTATS, 3: READY
    assert replies[0].startswith("OK: NX Information Server")
    assert replies[1] == "OK: RSTATS"
    assert replies[2] == "READY"
