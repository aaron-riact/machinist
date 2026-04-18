from __future__ import annotations

import socket
import struct

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.io_controllers.weidmuller_ur20 import WeidmullerUR20

from ..conftest import free_port, wait_running


def _modbus_request(host: str, port: int, body: bytes) -> bytes:
    header = struct.pack(">HHHB", 1, 0, len(body) + 1, 1)
    with socket.create_connection((host, port), timeout=2) as s:
        s.sendall(header + body)
        return s.recv(256)


def test_write_then_read_outputs() -> None:
    port = free_port()
    device = WeidmullerUR20(
        "io1", Endpoint("127.0.0.1", port), EventBus(), {"inputs": 8, "outputs": 8}
    )
    device.start()
    try:
        wait_running(device)
        # Write 0xFF to register 0x0100 (outputs 1..8 all on).
        write_body = struct.pack(">BHH", 0x06, 0x0100, 0x00FF)
        _modbus_request("127.0.0.1", port, write_body)

        # Read back register 0x0100.
        read_body = struct.pack(">BHH", 0x03, 0x0100, 1)
        resp = _modbus_request("127.0.0.1", port, read_body)
        # MBAP(7) + func(1) + count(1) + 2 bytes
        value = struct.unpack(">H", resp[9:11])[0]
        assert value == 0xFF
        # Signals are reflected.
        assert all(device.io[f"o{i}"].value for i in range(1, 9))
    finally:
        device.stop()
