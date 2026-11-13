"""Every transport server implements the one Service lifecycle."""

from __future__ import annotations

import threading

import pytest

from machinist.transport.broadcast import BroadcastServer
from machinist.transport.focas_server import FocasServer
from machinist.transport.iolink_http_master import IOLinkHttpMaster
from machinist.transport.line_server import LineServer
from machinist.transport.modbus_server import HoldingRegisterServer
from machinist.transport.mtconnect import MTConnectAgent
from machinist.transport.opcua_server import OpcUaServer
from machinist.transport.s7_server import S7Server, S7Store
from machinist.transport.service import Service, Ticker


@pytest.mark.parametrize(
    "cls",
    [
        BroadcastServer,
        FocasServer,
        HoldingRegisterServer,
        IOLinkHttpMaster,
        LineServer,
        MTConnectAgent,
        OpcUaServer,
        S7Server,
    ],
    ids=lambda c: c.__name__,
)
def test_server_class_is_a_service(cls: type) -> None:
    assert issubclass(cls, Service)


def test_s7_stub_backend_is_served_as_a_service() -> None:
    server = S7Server(host="127.0.0.1", port=0, store=S7Store(), backend="stub")
    assert isinstance(server, Service)


def test_service_needs_both_halves_of_the_lifecycle() -> None:
    class ServeOnly(Service):
        def serve_forever(self, ready: threading.Event | None = None) -> None:
            pass

    with pytest.raises(TypeError):
        ServeOnly()  # type: ignore[abstract]


@pytest.mark.timeout(5)
def test_poller_calls_its_function_until_shut_down() -> None:
    from machinist.transport.service import Poller

    calls: list[float] = []
    poller = Poller(lambda: calls.append(1.0), interval=0.01)
    ready = threading.Event()
    thread = threading.Thread(target=poller.serve_forever, args=(ready,), daemon=True)
    thread.start()
    assert ready.wait(timeout=1.0)
    deadline = threading.Event()
    deadline.wait(0.05)
    poller.shutdown()
    thread.join(timeout=1.0)
    assert not thread.is_alive()
    assert len(calls) >= 2


def test_a_ticker_keeps_its_rate_however_long_each_tick_takes() -> None:
    """Each tick takes 3 ms of a 5 ms interval: a poller would call every 8 ms."""
    now = [0.0]
    calls: list[float] = []

    def tick() -> None:
        calls.append(now[0])
        now[0] += 0.003
        if len(calls) == 4:
            ticker.shutdown()

    def wait(seconds: float) -> None:
        now[0] += seconds

    ticker = Ticker(tick, interval=0.005, clock=lambda: now[0], wait=wait)
    ticker.serve_forever()
    assert calls == pytest.approx([0.0, 0.005, 0.010, 0.015])


def test_a_ticker_that_falls_behind_skips_rather_than_bursts() -> None:
    now = [0.0]
    calls: list[float] = []

    def tick() -> None:
        calls.append(now[0])
        now[0] += 0.05 if len(calls) == 1 else 0.001   # the first tick takes 10 intervals
        if len(calls) == 4:
            ticker.shutdown()

    def wait(seconds: float) -> None:
        now[0] += seconds

    ticker = Ticker(tick, interval=0.005, clock=lambda: now[0], wait=wait)
    ticker.serve_forever()
    # After the slow tick, the next is due at once and the rest one interval apart:
    # no run of calls to make up the nine it missed.
    assert calls == pytest.approx([0.0, 0.05, 0.055, 0.060])
