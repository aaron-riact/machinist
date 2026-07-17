"""Integration tests for the FANUC FOCAS CNC device."""

import socket
import struct
import threading

import pytest

from src.machinist.core.events import EventBus
from src.machinist.core.types import Endpoint
from src.machinist.devices.machines.fanuc_focas_cnc import (
    FanucFocasCnc, FanucFocasCncOptions,
)
from src.machinist.devices.machines.state import MachineState, CycleState
from src.machinist.transport.focas import (
    FocasSubpacket, FocasFrame, CONNECT_REQ, CLOSE_REQ, VAR_REQ,
    CONNECT_RESP, CLOSE_RESP, VAR_RESP,
)


_next_port = 19193
_port_lock = threading.Lock()


def _make_device(**kwargs) -> tuple[FanucFocasCnc, int]:
    global _next_port
    with _port_lock:
        port = _next_port
        _next_port += 1
    opts = FanucFocasCncOptions(**kwargs)
    ep = Endpoint(host="127.0.0.1", port=port)
    state = MachineState()
    for i in range(1, opts.door_count + 1):
        state.doors[str(i)] = state.door(str(i))
    dev = FanucFocasCnc("test-cnc", ep, EventBus(), opts, state=state)
    stop = threading.Event()
    t = threading.Thread(target=dev._run, args=(stop,), daemon=True)
    t.start()
    import time as _t
    for _ in range(50):
        _t.sleep(0.02)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.1)
            s.connect(("127.0.0.1", port))
            s.close()
            return dev, port
        except (ConnectionRefusedError, OSError):
            continue
    raise RuntimeError(f"server did not start on port {port}")


def _connect(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect(("127.0.0.1", port))
    return sock


def _send_recv(sock: socket.socket, frame: bytes) -> FocasFrame:
    sock.sendall(frame)
    raw = sock.recv(4096)
    return FocasFrame.decode(raw)


def _resp_payload(resp: FocasFrame) -> bytes:
    """Extract the payload from a single-subpacket VAR_RESP response."""
    body = resp.response_subpackets[0][2:]  # strip subpacket length
    return body[14:]  # c1(2)+c2(2)+c3(2)+filler(6)+plen(2)


@pytest.fixture
def cnc():
    dev, port = _make_device()
    yield dev, port
    dev._shutdown()


# ----- connect / close -------------------------------------------------------

class TestConnect:
    def test_connect_close(self, cnc):
        _, port = cnc
        sock = _connect(port)
        resp = _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        assert resp.type == CONNECT_RESP
        resp = _send_recv(sock, FocasFrame(type=CLOSE_REQ).encode())
        assert resp.type == CLOSE_RESP
        sock.close()

    def test_unknown_function(self, cnc):
        _, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=9, c2=9, c3=0xFF)
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        body = resp.response_subpackets[0][2:]
        err = struct.unpack(">HHHh", body[:8])[3]
        assert err == 1  # unknown function
        sock.close()


# ----- read functions --------------------------------------------------------

class TestReadFunctions:
    def test_sysinfo(self, cnc):
        _, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x18)
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        rsp = _resp_payload(resp)
        addinfo, max_axes = struct.unpack(">HH", rsp[:4])
        assert max_axes == 8
        assert rsp[4:6] == b"0i"
        sock.close()

    def test_status(self, cnc):
        _, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x19)
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        sock.close()

    def test_get_time(self, cnc):
        _, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x45, v1=0)  # date
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        sock.close()

    def test_read_pmc(self, cnc):
        _, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=2, c2=1, c3=0x8001, v1=100, v2=100, v3=9, v4=4, v5=1)
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        sock.close()

    def test_feedrate(self, cnc):
        dev, port = cnc
        dev._state.feed = 250.0
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sp = FocasSubpacket(c1=1, c2=1, c3=0x24)
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        sock.close()


# ----- control functions -----------------------------------------------------

class TestControlFunctions:
    def test_write_pmc_door(self, cnc):
        dev, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        payload = struct.pack(">H", 1)
        sp = FocasSubpacket(
            c1=2, c2=1, c3=0x8002, v1=0x100, v2=0x100, v3=9, v4=1, v5=1,
            payload=payload,
        )
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        assert dev._state.door("1").open
        sock.close()

    def test_write_pmc_cycle(self, cnc):
        dev, port = cnc
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        payload = struct.pack(">H", 1)
        sp = FocasSubpacket(
            c1=2, c2=1, c3=0x8002, v1=0x200, v2=0x200, v3=9, v4=1, v5=1,
            payload=payload,
        )
        resp = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert resp.type == VAR_RESP
        assert dev._state.cycle == CycleState.RUNNING
        sock.close()
