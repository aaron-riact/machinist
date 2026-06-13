from __future__ import annotations

import socket
import urllib.request

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.machines.haas_ngc import HaasNGC, HaasNGCOptions, make_device
from machinist.transport.broadcast import BroadcastServer

from ..conftest import free_port, wait_running


def _make(tmp_path, **opts) -> HaasNGC:
    raw_dprint_port = opts.pop("dprint_port", None)
    raw_mtconnect_port = opts.pop("mtconnect_port", None)
    raw_smb = opts.pop("smb", None)
    raw_opcua = opts.pop("opcua", None)
    host = "127.0.0.1"
    port = free_port()
    dprint = BroadcastServer(host, raw_dprint_port) if raw_dprint_port is not None else None
    otp = HaasNGCOptions(
        doors=tuple(opts.pop("doors", ["main"])),
        program_folder=opts.pop("program_folder", str(tmp_path)),
        dprint_port=raw_dprint_port,
        mtconnect_port=raw_mtconnect_port,
        smb=raw_smb,
        opcua=raw_opcua,
    )
    bus = EventBus()
    d = make_device("haas1", Endpoint(host, port), bus, otp, dprint=dprint)
    d.start()
    wait_running(d)
    return d


def _readline(sock: socket.socket, terminator: bytes = b"\r\n") -> bytes:
    buf = b""
    while terminator not in buf:
        chunk = sock.recv(1024)
        if not chunk:
            break
        buf += chunk
    return buf


@pytest.fixture
def haas(tmp_path):
    d = _make(tmp_path)
    try:
        yield d
    finally:
        d.stop()


def test_mdc_q100_returns_serial(haas: HaasNGC) -> None:
    with socket.create_connection((haas.endpoint.host, haas.endpoint.port), timeout=2) as s:
        s.sendall(b"Q100\r\n")
        assert b"SERIAL NUMBER" in _readline(s)


def test_program_library_roundtrip(haas: HaasNGC) -> None:
    haas.programs.write("O0001.nc", "DPRINT[hello]\nM30\n")
    assert "O0001.nc" in haas.programs.list()
    assert "DPRINT" in haas.programs.read("O0001.nc")


def test_dprint_broadcast_to_connected_clients(tmp_path) -> None:
    import time
    dprint_port = free_port()
    d = _make(tmp_path, dprint_port=dprint_port)
    try:
        with socket.create_connection(("127.0.0.1", dprint_port), timeout=2) as s:
            s.settimeout(2.0)
            # Wait for the server's accept loop to register us.
            assert d._dprint is not None
            for _ in range(20):
                with d._dprint._clients_lock:
                    if d._dprint._clients:
                        break
                time.sleep(0.05)
            d.state.dprint("PART COMPLETE")
            line = _readline(s, terminator=b"\n")
            assert b"PART COMPLETE" in line
    finally:
        d.stop()


def test_mtconnect_probe_and_current(tmp_path) -> None:
    mtc_port = free_port()
    d = _make(tmp_path, mtconnect_port=mtc_port)
    try:
        d.programs.write("O1000.nc", "G1 X12.5 Y3 Z-7\nM30\n")
        d.run_program("O1000.nc")
        if d._runner is not None:
            d._runner.join(timeout=2)
        probe = urllib.request.urlopen(
            f"http://127.0.0.1:{mtc_port}/probe", timeout=2
        ).read().decode()
        assert "MTConnectDevices" in probe
        assert 'id="door_main"' in probe
        assert 'id="x"' in probe
        current = urllib.request.urlopen(
            f"http://127.0.0.1:{mtc_port}/current", timeout=2
        ).read().decode()
        assert "MTConnectStreams" in current
        assert "IDLE" in current
        assert '<Position dataItemId="x">12.5</Position>' in current
    finally:
        d.stop()


def test_run_program_executes_dprint(haas: HaasNGC) -> None:
    haas.programs.write("O0002.nc", "DPRINT[hi]\nM30\n")
    haas.run_program("O0002.nc")
    # wait for runner
    if haas._runner is not None:
        haas._runner.join(timeout=2)
    assert "hi" in haas.state.dprint_log


def test_mdc_reports_program_telemetry(haas: HaasNGC) -> None:
    haas.programs.write("O0003.nc", "T1 M06\nM03 S1200\nM30\n")
    haas.run_program("O0003.nc")
    if haas._runner is not None:
        haas._runner.join(timeout=2)
    with socket.create_connection((haas.endpoint.host, haas.endpoint.port), timeout=2) as s:
        s.sendall(b"Q200\r\n")
        assert b"TOOL CHANGES, 1" in _readline(s)
        s.sendall(b"Q201\r\n")
        assert b"USING TOOL, 1" in _readline(s)
        s.sendall(b"Q500\r\n")
        line = _readline(s)
        assert b"PARTS, 1" in line
        assert b"STATUS" not in line  # real HAAS has no literal STATUS field
        s.sendall(b"Q402\r\n")
        assert b"M30 #1, 1" in _readline(s)
