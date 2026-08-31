from __future__ import annotations

import time

from machinist.transport.ethernetip import (
    EtherNetIPAdapter,
    EtherNetIPAdapterConfig,
    EtherNetIPScanner,
    EtherNetIPScannerConfig,
)

from ..conftest import free_port


class _FakeEEIPClient:
    def __init__(self) -> None:
        self.registered: tuple[str, int] | None = None
        self.forward_open_called = False
        self.forward_close_called = False
        self.unregister_called = False
        self.last_received_implicit_message = None
        self.o_t_iodata = []
        self.t_o_iodata = [1, 2, 3]

    def register_session(self, host: str, port: int) -> int:
        self.registered = (host, port)
        return 1

    def forward_open(self) -> None:
        self.forward_open_called = True

    def forward_close(self) -> None:
        self.forward_close_called = True

    def unregister_session(self) -> None:
        self.unregister_called = True


def test_scanner_configures_client_and_transfers_blocks() -> None:
    created: list[_FakeEEIPClient] = []

    def _factory() -> _FakeEEIPClient:
        client = _FakeEEIPClient()
        created.append(client)
        return client

    scanner = EtherNetIPScanner(
        EtherNetIPScannerConfig(host="192.0.2.10", output_length=4, input_length=4),
        client_factory=_factory,
    )

    scanner.open()
    scanner.write_output_block(b"\x10\x20")
    assert scanner.connected is True

    payload = scanner.read_input_block()
    scanner.close()

    client = created[0]
    assert client.registered == ("192.0.2.10", 44818)
    assert client.forward_open_called is True
    assert client.forward_close_called is True
    assert client.unregister_called is True
    assert client.o_t_iodata == [0x10, 0x20, 0x00, 0x00]
    assert payload == b"\x01\x02\x03\x00"


def test_real_scanner_can_exchange_blocks_with_adapter() -> None:
    tcp_port = free_port()
    udp_port = free_port()
    originator_udp_port = free_port()
    adapter = EtherNetIPAdapter(
        EtherNetIPAdapterConfig(
            host="127.0.0.1",
            port=tcp_port,
            udp_port=udp_port,
            output_length=4,
            input_length=4,
            requested_packet_rate_ms=20,
        )
    )
    scanner = EtherNetIPScanner(
        EtherNetIPScannerConfig(
            host="127.0.0.1",
            port=tcp_port,
            originator_udp_port=originator_udp_port,
            target_udp_port=udp_port,
            output_length=4,
            input_length=4,
            requested_packet_rate_ms=20,
        )
    )
    adapter.open()
    try:
        scanner.open()
        scanner.write_output_block(b"\xAA\x55")
        for _ in range(20):
            if adapter.read_input_block().startswith(b"\xAA\x55"):
                break
            time.sleep(0.02)
        assert adapter.peer_connected is True
        assert adapter.read_input_block() == b"\xAA\x55\x00\x00"

        adapter.write_output_block(b"\x11\x22\x33")
        payload = b""
        for _ in range(20):
            payload = scanner.read_input_block()
            if payload.startswith(b"\x11\x22\x33"):
                break
            time.sleep(0.02)
        assert payload == b"\x11\x22\x33\x00"
    finally:
        scanner.close()
        adapter.close()


def test_forward_open_t_o_api_is_read_from_the_request() -> None:
    """The adapter paces T->O off the rate the scanner asked for (100ms in mazak6)."""
    from machinist.transport.ethernetip import _forward_open_t_o_api_us

    packet = bytearray(82)
    packet[74:78] = (100_000).to_bytes(4, "little")
    assert _forward_open_t_o_api_us(bytes(packet)) == 100_000

    packet[74:78] = (0).to_bytes(4, "little")
    assert _forward_open_t_o_api_us(bytes(packet)) is None

    packet[74:78] = (60_000_000).to_bytes(4, "little")
    assert _forward_open_t_o_api_us(bytes(packet)) is None

    assert _forward_open_t_o_api_us(b"\x00" * 40) is None
