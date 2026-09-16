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
from machinist.transport.service import Service


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
