from __future__ import annotations

from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import Endpoint
from machinist.devices.machines.mazak_840d import (
    MazakSinumerik840D,
    MazakSinumerik840DOptions,
)
from machinist.transport.s7_server import S7Server, S7Store


def test_door_command_sets_status_signals() -> None:
    store = S7Store()
    io = SignalBank(owner="m1")
    server = S7Server(host="127.0.0.1", port=0, store=store, backend="stub")
    device = MazakSinumerik840D(
        "m1", Endpoint("127.0.0.1", 0), EventBus(), MazakSinumerik840DOptions(),
        io=io, store=store, server=server,
    )
    # Don't actually start the server (port 0 cannot bind to S7 listener
    # in this stub). Exercise the IO wiring directly.
    device.io["door_open_cmd"].set(True)
    assert device.io["door_is_open"].value is True
    assert device.io["door_is_closed"].value is False
    assert device.state.door("main").open is True

    device.io["door_close_cmd"].set(True)
    assert device.io["door_is_closed"].value is True
    assert device.io["door_is_open"].value is False


def test_s7_store_round_trip() -> None:
    store = S7Store()
    io = SignalBank(owner="m2")
    server = S7Server(host="127.0.0.1", port=0, store=store, backend="stub")
    device = MazakSinumerik840D(
        "m2", Endpoint("127.0.0.1", 0), EventBus(), MazakSinumerik840DOptions(),
        io=io, store=store, server=server,
    )
    # Map default door_open_cmd is DB1, byte 0, bit 0.
    device._store.write_bit(1, 0, 0, True)  # noqa: SLF001 - intentional in test
    assert device.io["door_open_cmd"].value is True
