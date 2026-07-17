"""Tests for the FOCAS threaded server."""

import socket
import struct
import threading

import pytest

from src.machinist.transport.focas import (
    FocasSubpacket, FocasFrame, CONNECT_REQ, CLOSE_REQ, VAR_REQ,
    CONNECT_RESP, CLOSE_RESP, VAR_RESP,
)
from src.machinist.transport.focas_server import FocasServer


_next_port = 18194
_port_lock = threading.Lock()


def _server() -> tuple[FocasServer, dict[str, bool], int]:
    global _next_port
    called: dict[str, bool] = {"connect": False, "disconnect": False}

    def handler(sp: FocasSubpacket) -> bytes:
        if sp.c1 == 1 and sp.c2 == 1 and sp.c3 == 0x18:
            return sp.encode_response_ok(
                struct.pack(">HH2s2s4s4s2s", 0, 8, b"0i", b"TF", b"3000", b"1.00", b"  ")
            )
        if sp.c1 == 1 and sp.c2 == 1 and sp.c3 == 0x19:
            return sp.encode_response_ok(bytes(14))
        return sp.encode_response_error(1)

    def on_connect():
        called["connect"] = True

    def on_disconnect():
        called["disconnect"] = True

    with _port_lock:
        port = _next_port
        _next_port += 1

    sv = FocasServer(
        host="127.0.0.1", port=port, on_request=handler,
        on_connect=on_connect, on_disconnect=on_disconnect,
    )
    ready = threading.Event()
    t = threading.Thread(target=sv.serve_forever, args=(ready,), daemon=True)
    t.start()
    ready.wait(timeout=2)
    return sv, called, port


@pytest.fixture
def server():
    sv, called, port = _server()
    yield sv, called, port
    sv.shutdown()


# ----- helpers --------------------------------------------------------------

def _connect(port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(2)
    sock.connect(("127.0.0.1", port))
    return sock


def _send_recv(sock: socket.socket, frame: bytes) -> FocasFrame:
    sock.sendall(frame)
    raw = sock.recv(4096)
    return FocasFrame.decode(raw)


# ----- tests ----------------------------------------------------------------

class TestConnectDisconnect:
    def test_connect_handshake(self, server):
        sv, called, port = server
        sock = _connect(port)
        raw = _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        assert raw.type == CONNECT_RESP
        assert called["connect"]
        sock.close()

    def test_close_handshake(self, server):
        sv, called, port = server
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        raw = _send_recv(sock, FocasFrame(type=CLOSE_REQ).encode())
        assert raw.type == CLOSE_RESP
        sock.close()

    def test_disconnect_callback(self, server):
        sv, called, port = server
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())
        sock.close()
        import time
        time.sleep(0.1)
        assert called["disconnect"]


class TestVarRequest:
    def test_single_subpacket(self, server):
        sv, _, port = server
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())

        sp = FocasSubpacket(c1=1, c2=1, c3=0x18)
        raw = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert raw.type == VAR_RESP
        assert len(raw.response_subpackets) == 1
        sock.close()

    def test_multi_subpacket(self, server):
        sv, _, port = server
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())

        sps = (
            FocasSubpacket(c1=1, c2=1, c3=0x18),
            FocasSubpacket(c1=1, c2=1, c3=0x19),
        )
        raw = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=sps).encode())
        assert raw.type == VAR_RESP
        assert len(raw.response_subpackets) == 2
        sock.close()

    def test_real_sysinfo_response(self, server):
        sv, _, port = server
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())

        sp = FocasSubpacket(c1=1, c2=1, c3=0x18)
        raw = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        resp = raw.response_subpackets[0]
        rsp_body = resp[2:]  # strip length prefix
        c1, c2, c3 = struct.unpack(">HHH", rsp_body[:6])
        assert (c1, c2, c3) == (1, 1, 0x18)
        sock.close()


class TestConcurrent:
    def test_two_clients(self, server):
        sv, _, port = server
        sock1 = _connect(port)
        sock2 = _connect(port)
        _send_recv(sock1, FocasFrame(type=CONNECT_REQ).encode())
        _send_recv(sock2, FocasFrame(type=CONNECT_REQ).encode())
        assert sv.client_count == 2
        sock1.close()
        sock2.close()
        import time
        time.sleep(0.1)
        assert sv.client_count == 0


class TestErrorHandling:
    def test_unknown_function(self, server):
        sv, _, port = server
        sock = _connect(port)
        _send_recv(sock, FocasFrame(type=CONNECT_REQ).encode())

        sp = FocasSubpacket(c1=9, c2=9, c3=0xFF)
        raw = _send_recv(sock, FocasFrame(type=VAR_REQ, subpackets=(sp,)).encode())
        assert raw.type == VAR_RESP
        rsp_body = raw.response_subpackets[0][2:]
        err_code = struct.unpack(">HHHh", rsp_body[:8])[3]
        assert err_code == 1
        sock.close()
