"""Function 0x05, Write Single Coil, on the Modbus/TCP server."""

from __future__ import annotations

import struct

from machinist.transport.modbus_server import CoilPort, HoldingRegisterServer
from machinist.transport.registers import RegisterPort


def _server(*, with_coils: bool = True) -> tuple[HoldingRegisterServer, dict[int, bool]]:
    written: dict[int, bool] = {}
    registers = RegisterPort(on_read=lambda a: 0, on_write=lambda a, v: None)
    coils = CoilPort(on_write=written.__setitem__) if with_coils else None
    server = HoldingRegisterServer(host="127.0.0.1", port=0, registers=registers, coils=coils)
    return server, written


def _write_coil(address: int, value: int) -> bytes:
    return struct.pack(">BHH", 0x05, address, value)


def test_0xff00_sets_the_coil() -> None:
    server, written = _server()

    server._handle(_write_coil(0x8003, 0xFF00))

    assert written == {0x8003: True}


def test_0x0000_clears_the_coil() -> None:
    server, written = _server()

    server._handle(_write_coil(0x8003, 0x0000))

    assert written == {0x8003: False}


def test_the_reply_echoes_the_request() -> None:
    server, _ = _server()
    request = _write_coil(0x8003, 0xFF00)

    assert server._handle(request) == request


def test_any_other_value_is_refused() -> None:
    server, written = _server()

    reply = server._handle(_write_coil(0x8003, 0x0001))

    assert (reply, written) == (bytes([0x85, 0x03]), {})


def test_a_device_without_coils_does_not_know_the_function() -> None:
    server, _ = _server(with_coils=False)

    assert server._handle(_write_coil(0x8003, 0xFF00)) == bytes([0x85, 0x01])
