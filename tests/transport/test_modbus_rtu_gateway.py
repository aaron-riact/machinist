"""A TCP port onto a flange line, speaking RTU."""

from __future__ import annotations

import socket
import threading
from collections.abc import Iterator

import pytest

from machinist.transport.flange_bus import FlangeBus
from machinist.transport.modbus_rtu import framed
from machinist.transport.modbus_rtu_gateway import ModbusRtuGateway, parse_gateway_ports
from machinist.transport.registers import RegisterPort

from ..conftest import free_port

HOST = "127.0.0.1"


def _serve(gateway: ModbusRtuGateway) -> None:
    """Run the service on a thread, the way a device does, and wait for its ports."""
    ready = threading.Event()
    threading.Thread(target=gateway.serve_forever, args=(ready,), daemon=True).start()
    assert ready.wait(timeout=2.0), "gateway did not bind"


def _port(cells: dict[int, int]) -> RegisterPort:
    """A slave whose register model runs out, the way a list-backed one does."""

    def read(address: int) -> int:
        try:
            return cells[address]
        except KeyError:
            raise IndexError(address) from None

    return RegisterPort(on_read=read, on_write=cells.__setitem__)


@pytest.fixture
def cells() -> dict[int, int]:
    return dict.fromkeys(range(0x0200), 0)


@pytest.fixture
def line(cells: dict[int, int]) -> FlangeBus:
    bus = FlangeBus()
    bus.attach(0x41, _port(cells))
    return bus


@pytest.fixture
def gateway(line: FlangeBus) -> Iterator[ModbusRtuGateway]:
    served = ModbusRtuGateway(host=HOST, ports=[free_port()], line=line)
    _serve(served)
    try:
        yield served
    finally:
        served.shutdown()


def _exchange(port: int, request: bytes, *, expect: int = 64, timeout: float = 1.0) -> bytes:
    with socket.create_connection((HOST, port), timeout=timeout) as sock:
        sock.sendall(request)
        try:
            return sock.recv(expect)
        except TimeoutError:
            return b""


def test_a_read_reaches_the_slave_on_the_line(
    gateway: ModbusRtuGateway, cells: dict[int, int]
) -> None:
    cells[0x010B] = 500
    cells[0x010C] = 1

    reply = _exchange(gateway.ports[0], framed(b"\x41\x03\x01\x0b\x00\x02"))

    assert reply == framed(b"\x41\x03\x04\x01\xf4\x00\x01")


def test_a_write_reaches_the_slave_on_the_line(
    gateway: ModbusRtuGateway, cells: dict[int, int]
) -> None:
    reply = _exchange(gateway.ports[0], framed(b"\x41\x10\x00\x00\x00\x02\x04\x01\x90\x04\x4c"))

    assert reply == framed(b"\x41\x10\x00\x00\x00\x02")
    assert (cells[0], cells[1]) == (400, 1100)


def test_a_single_register_write_is_echoed(
    gateway: ModbusRtuGateway, cells: dict[int, int]
) -> None:
    reply = _exchange(gateway.ports[0], framed(b"\x41\x06\x00\x02\x00\x01"))

    assert reply == framed(b"\x41\x06\x00\x02\x00\x01")
    assert cells[2] == 1


def test_two_requests_on_one_connection_are_both_answered(gateway: ModbusRtuGateway) -> None:
    port = gateway.ports[0]
    with socket.create_connection((HOST, port), timeout=1.0) as sock:
        sock.sendall(framed(b"\x41\x06\x00\x02\x00\x01"))
        first = sock.recv(64)
        sock.sendall(framed(b"\x41\x03\x00\x02\x00\x01"))
        second = sock.recv(64)

    assert first == framed(b"\x41\x06\x00\x02\x00\x01")
    assert second == framed(b"\x41\x03\x02\x00\x01")


def test_a_frame_split_across_sends_is_answered_once_whole(gateway: ModbusRtuGateway) -> None:
    request = framed(b"\x41\x03\x00\x02\x00\x01")
    with socket.create_connection((HOST, gateway.ports[0]), timeout=1.0) as sock:
        sock.sendall(request[:3])
        sock.sendall(request[3:])

        assert sock.recv(64) == framed(b"\x41\x03\x02\x00\x00")


def test_an_absent_slave_leaves_the_line_quiet(gateway: ModbusRtuGateway) -> None:
    """Nothing answers to 0x42, so nothing comes back and the master times out."""
    reply = _exchange(gateway.ports[0], framed(b"\x42\x03\x01\x0b\x00\x02"), timeout=0.3)

    assert reply == b""


def test_an_unreadable_address_is_refused(gateway: ModbusRtuGateway) -> None:
    reply = _exchange(gateway.ports[0], framed(b"\x41\x03\x09\x99\x00\x01"))

    assert reply == framed(b"\x41\x83\x02")


def test_an_unsupported_function_is_refused(gateway: ModbusRtuGateway) -> None:
    """Coils cannot be framed, so the answer is all the master gets."""
    reply = _exchange(gateway.ports[0], framed(b"\x41\x01\x00\x00\x00\x01"))

    assert reply == framed(b"\x41\x81\x01")


def test_a_corrupt_frame_is_ignored(gateway: ModbusRtuGateway) -> None:
    corrupt = framed(b"\x41\x03\x01\x0b\x00\x02")[:-1] + b"\x00"

    assert _exchange(gateway.ports[0], corrupt, timeout=0.3) == b""


def test_every_port_opens_onto_the_same_line(line: FlangeBus, cells: dict[int, int]) -> None:
    """A controller's doors onto the flange are several ports, one line."""
    gateway = ModbusRtuGateway(host=HOST, ports=[free_port(), free_port()], line=line)
    _serve(gateway)
    try:
        _exchange(gateway.ports[0], framed(b"\x41\x06\x00\x02\x00\x07"))
        reply = _exchange(gateway.ports[1], framed(b"\x41\x03\x00\x02\x00\x01"))
    finally:
        gateway.shutdown()

    assert reply == framed(b"\x41\x03\x02\x00\x07")
    assert cells[2] == 7


def test_clients_are_counted_while_they_are_connected(line: FlangeBus) -> None:
    seen: list[int] = []
    gateway = ModbusRtuGateway(
        host=HOST, ports=[free_port()], line=line, on_connect_change=seen.append
    )
    _serve(gateway)
    try:
        _exchange(gateway.ports[0], framed(b"\x41\x03\x00\x02\x00\x01"))
    finally:
        gateway.shutdown()

    assert seen[0] == 1
    assert gateway.client_count == 0


def test_a_port_already_in_use_fails_the_caller(line: FlangeBus) -> None:
    """The bind happens before *ready*, so the device sees the failure."""
    taken = socket.socket()
    taken.bind((HOST, 0))
    taken.listen(1)
    try:
        gateway = ModbusRtuGateway(host=HOST, ports=[taken.getsockname()[1]], line=line)
        with pytest.raises(OSError, match="in use"):
            gateway.serve_forever(threading.Event())
    finally:
        taken.close()


# --- the option that names the ports ----------------------------------


def test_leaving_the_option_out_takes_the_default() -> None:
    assert parse_gateway_ports(None, default=(60000,)) == (60000,)


def test_asking_for_it_takes_the_default_too() -> None:
    assert parse_gateway_ports(True, default=(12345,)) == (12345,)


def test_a_device_whose_default_is_nothing_stays_shut() -> None:
    assert parse_gateway_ports(None, default=()) == ()


def test_false_shuts_the_passthrough() -> None:
    assert parse_gateway_ports(False, default=(60000,)) == ()


def test_one_port_can_be_named_on_its_own() -> None:
    assert parse_gateway_ports(1502, default=(60000,)) == (1502,)


def test_several_ports_can_be_named() -> None:
    assert parse_gateway_ports([1502, 1503], default=(60000,)) == (1502, 1503)


def test_something_that_is_not_a_port_is_refused() -> None:
    with pytest.raises(ValueError, match="flange_gateway_ports"):
        parse_gateway_ports("60000", default=(60000,))
