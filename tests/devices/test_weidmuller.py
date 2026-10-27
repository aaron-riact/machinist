from __future__ import annotations

import socket
import struct

from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import Endpoint
from machinist.devices.io_controllers.weidmuller_ur20 import WeidmullerUR20, WeidmullerUR20Options
from machinist.transport.modbus_server import HoldingRegisterServer

from ..conftest import free_port, wait_running


def _modbus_request(host: str, port: int, body: bytes) -> bytes:
    header = struct.pack(">HHHB", 1, 0, len(body) + 1, 1)
    with socket.create_connection((host, port), timeout=2) as s:
        s.sendall(header + body)
        return s.recv(256)


def test_write_then_read_outputs() -> None:
    port = free_port()
    device = WeidmullerUR20(
        "io1", Endpoint("127.0.0.1", port), EventBus(), WeidmullerUR20Options(inputs=8, outputs=8),
        io=SignalBank(owner="io1"),
    )
    device.add_service(
        HoldingRegisterServer(host="127.0.0.1", port=port, registers=device.register_port)
    )
    device.start()
    try:
        wait_running(device)
        # Write 0xFF to register 0x0800 (outputs 1..8 all on).
        write_body = struct.pack(">BHH", 0x06, 0x0800, 0x00FF)
        _modbus_request("127.0.0.1", port, write_body)

        # Read back register 0x0800.
        read_body = struct.pack(">BHH", 0x03, 0x0800, 1)
        resp = _modbus_request("127.0.0.1", port, read_body)
        # MBAP(7) + func(1) + count(1) + 2 bytes
        value = struct.unpack(">H", resp[9:11])[0]
        assert value == 0xFF
        # Signals are reflected.
        assert all(device.io[f"o{i}"].value for i in range(1, 9))
    finally:
        device.stop()


def _device(outputs: int = 8) -> WeidmullerUR20:
    return WeidmullerUR20(
        "io1", Endpoint("127.0.0.1", 0), EventBus(),
        WeidmullerUR20Options(inputs=8, outputs=outputs), io=SignalBank(owner="io1"),
    )


def test_coil_0x8000_is_the_first_output() -> None:
    device = _device()

    device.coil_port.write(0x8000, True)

    assert device.io["o1"].value


def test_a_coil_sets_only_its_own_output() -> None:
    device = _device()

    device.coil_port.write(0x8002, True)

    assert [device.io[f"o{i}"].value for i in range(1, 9)] == [
        False, False, True, False, False, False, False, False,
    ]


def test_a_coil_can_clear_an_output() -> None:
    device = _device()
    device.io["o3"].set(True)

    device.coil_port.write(0x8002, False)

    assert not device.io["o3"].value


def test_a_coil_output_reads_back_from_the_output_registers() -> None:
    device = _device()

    device.coil_port.write(0x8001, True)

    assert device.register_port.read(0x0800, 1) == [0b10]


def test_a_coil_past_the_last_output_is_ignored() -> None:
    device = _device(outputs=2)

    device.coil_port.write(0x8002, True)

    assert not any(device.io[f"o{i}"].value for i in (1, 2))
