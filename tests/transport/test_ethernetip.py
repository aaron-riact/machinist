from __future__ import annotations

from machinist.transport.ethernetip import EtherNetIPScanner, EtherNetIPScannerConfig


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
