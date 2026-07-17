"""Integration tests for the dual-protocol FOCAS + Karel robot device."""

import socket
import struct
import threading

import pytest

from src.machinist.core.events import EventBus
from src.machinist.core.io import SignalBank
from src.machinist.core.types import Endpoint
from src.machinist.devices.robots.arm import (
    ArmOptions, RobotArm, arm_from_options,
)
from src.machinist.devices.robots.fanuc import (
    FanucFocasRobot, FanucFocasRobotOptions,
)
from src.machinist.transport.focas import (
    FocasSubpacket, FocasFrame, CONNECT_REQ, CLOSE_REQ, VAR_REQ,
    CONNECT_RESP, VAR_RESP,
)
_next_focas = 19293
_next_karel = 19393
_port_lock = threading.Lock()


def _make_device(**kwargs) -> tuple[FanucFocasRobot, int, int]:
    global _next_focas, _next_karel
    with _port_lock:
        focas_port = _next_focas
        _next_focas += 1
        karel_port = _next_karel
        _next_karel += 1
    opts = FanucFocasRobotOptions(focas_port=focas_port, karel_port=karel_port, **kwargs)
    ep = Endpoint(host="127.0.0.1", port=focas_port)
    arm = arm_from_options(ArmOptions(joint_count=6))
    io = SignalBank(owner="test-robot")
    dev = FanucFocasRobot("test-robot", ep, EventBus(), opts, arm=arm, io=io)
    return dev, focas_port, karel_port


@pytest.fixture
def robot():
    dev, focas_port, karel_port = _make_device()
    ready = threading.Event()
    t = threading.Thread(target=dev._run, args=(threading.Event(),), daemon=True)
    t.start()
    import time as _t
    for _ in range(50):
        _t.sleep(0.02)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.1)
            s.connect(("127.0.0.1", focas_port))
            s.close()
            s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s2.settimeout(0.1)
            s2.connect(("127.0.0.1", karel_port))
            s2.close()
            yield dev, focas_port, karel_port
            dev._shutdown()
            return
        except (ConnectionRefusedError, OSError):
            continue
    pytest.fail("servers did not start")


def _connect(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect(("127.0.0.1", port))
    return sock


def _focas_send(sock: socket.socket, frame: bytes) -> FocasFrame:
    sock.sendall(frame)
    raw = sock.recv(4096)
    return FocasFrame.decode(raw)


# ----- FOCAS tests -----------------------------------------------------------

class TestFocasInterface:
    def test_connect(self, robot):
        _, focas_port, _ = robot
        sock = _connect(focas_port)
        resp = _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        assert resp.type == CONNECT_RESP
        sock.close()

    def test_sysinfo(self, robot):
        dev, focas_port, _ = robot
        sock = _connect(focas_port)
        _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x18)
        resp = _focas_send(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        body = resp.response_subpackets[0][2:]
        addinfo, max_axes = struct.unpack(">HH", body[14:18])
        assert max_axes == 6
        assert body[18:20] == b"R-"  # model prefix
        sock.close()

    def test_axes(self, robot):
        dev, focas_port, _ = robot
        sock = _connect(focas_port)
        _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x26)
        resp = _focas_send(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        sock.close()

    def test_get_time(self, robot):
        dev, focas_port, _ = robot
        sock = _connect(focas_port)
        _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x45, v1=0)
        resp = _focas_send(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        sock.close()

    def test_read_pmc_y(self, robot):
        """Read Y section → digital outputs."""
        dev, focas_port, _ = robot
        dev.io["do1"].set(True)
        dev.io["do2"].set(True)
        sock = _connect(focas_port)
        _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        # Read Y section (2), address 1, count 2, byte size
        sp = FocasSubpacket(c1=2, c2=1, c3=0x8001, v1=1, v2=1, v3=2, v4=2, v5=0)
        resp = _focas_send(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        body = resp.response_subpackets[0][2:]
        payload = body[14:]
        assert payload[0] == 1  # do1
        assert payload[1] == 1  # do2
        sock.close()

    def test_write_pmc_y(self, robot):
        """Write Y section → digital outputs."""
        dev, focas_port, _ = robot
        sock = _connect(focas_port)
        _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(
            c1=2, c2=1, c3=0x8002, v1=1, v2=1, v3=2, v4=1, v5=0,
            payload=b"\x01",
        )
        resp = _focas_send(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        assert dev.io["do1"].value is True
        sock.close()

    def test_unknown_function(self, robot):
        _, focas_port, _ = robot
        sock = _connect(focas_port)
        _focas_send(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=9, c2=9, c3=0xFF)
        resp = _focas_send(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        body = resp.response_subpackets[0][2:]
        err = struct.unpack(">HHHh", body[:8])[3]
        assert err == 1
        sock.close()


# ----- Karel tests -----------------------------------------------------------

def _karel_cmd(port: int, cmd: str) -> str:
    sock = _connect(port)
    sock.sendall((cmd + "\n").encode("ascii"))
    raw = sock.recv(4096)
    sock.close()
    return raw.decode("ascii").strip()


class TestKarelInterface:
    def test_getjpos(self, robot):
        _, _, karel_port = robot
        reply = _karel_cmd(karel_port, "getjpos")
        parts = reply.split(",")
        assert len(parts) == 6
        for p in parts:
            float(p)

    def test_getdi(self, robot):
        _, _, karel_port = robot
        reply = _karel_cmd(karel_port, "getdi 1")
        assert reply in ("0", "1")

    def test_setdo_getdi(self, robot):
        dev, _, karel_port = robot
        _karel_cmd(karel_port, "setdo 1,1")
        assert dev.io["do1"].value is True
        _karel_cmd(karel_port, "setdo 1,0")
        assert dev.io["do1"].value is False

    def test_stop_reset(self, robot):
        dev, _, karel_port = robot
        reply = _karel_cmd(karel_port, "stop")
        assert reply == "OK"
        reply = _karel_cmd(karel_port, "reset")
        assert reply == "OK"

    def test_unknown_verb(self, robot):
        _, _, karel_port = robot
        reply = _karel_cmd(karel_port, "blargh")
        assert reply.startswith("ERR")
