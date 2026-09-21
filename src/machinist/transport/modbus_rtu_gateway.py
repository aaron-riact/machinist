"""A TCP port that opens onto an RS485 line.

A robot controller with a Modbus master on its tool flange usually lets
the network at that line as well: connect to the controller, speak RTU
at it, and the frames come out on the flange addressed to whichever
slave id they name. Dobot puts that on port 60000, UR on 12345.

This is the front end for it. Where
:class:`~machinist.transport.modbus_server.HoldingRegisterServer` serves
one device's :class:`~machinist.transport.registers.RegisterPort` over
Modbus/TCP, this serves a whole
:class:`~machinist.transport.flange_bus.FlangeBus` over RTU -- the slave
id in each frame picks the device, so one port reaches every tool on the
line. Several ports can share one gateway, because on the controller
they are several doors onto the same line.

Addressing a slave that is not on the line produces no reply at all.
That is what the line does: the frame goes out, nothing answers, and the
master waits out its own timeout. Answering "gateway target device
failed to respond" would be kinder and would stop a driver ever
exercising the timeout path it will meet on real hardware.
"""

from __future__ import annotations

import contextlib
import socket
import threading
from collections.abc import Callable, Sequence
from typing import Any

from .flange_bus import FlangeBus, NoSlaveError
from .modbus_rtu import (
    EX_ILLEGAL_DATA_ADDRESS,
    EX_ILLEGAL_FUNCTION,
    ReadHolding,
    RtuRequest,
    RtuStream,
    UnframeableError,
    WriteMultiple,
    WriteSingle,
    exception_reply,
)
from .service import Service

__all__ = ["ModbusRtuGateway", "parse_gateway_ports"]

#: How long an accept or recv blocks before the stop flag is looked at again.
_POLL_SECONDS = 0.25


class ModbusRtuGateway(Service):
    """RTU-over-TCP doors onto one flange line, served as one :class:`Service`."""

    def __init__(
        self,
        *,
        host: str,
        ports: Sequence[int],
        line: FlangeBus,
        on_connect_change: Callable[[int], None] | None = None,
    ) -> None:
        self._host = host
        self._ports = tuple(ports)
        self._line = line
        self._on_connect_change = on_connect_change
        self._listeners: list[socket.socket] = []
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()
        self._client_count = 0
        self._lock = threading.Lock()

    @property
    def ports(self) -> tuple[int, ...]:
        return self._ports

    @property
    def client_count(self) -> int:
        return self._client_count

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        """Bind every port, accept on each one, and block until shut down.

        Binding happens before *ready* is set, so a port already in use
        faults the device that registered this rather than failing in a
        thread nobody is watching.
        """
        try:
            for port in self._ports:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                try:
                    sock.bind((self._host, port))
                    sock.listen(8)
                except OSError:
                    sock.close()
                    raise
                sock.settimeout(_POLL_SECONDS)
                self._listeners.append(sock)

            for sock in self._listeners:
                thread = threading.Thread(target=self._accept, args=(sock,), daemon=True)
                thread.start()
                self._threads.append(thread)
        except OSError:
            self._close_listeners()
            raise

        if ready is not None:
            ready.set()
        self._stop.wait()

    def shutdown(self) -> None:
        self._stop.set()
        self._close_listeners()
        for thread in self._threads:
            thread.join(timeout=2.0)
        self._threads.clear()

    def _close_listeners(self) -> None:
        for sock in self._listeners:
            sock.close()
        self._listeners.clear()

    # -----------------------------------------------------------------

    def _accept(self, sock: socket.socket) -> None:
        while not self._stop.is_set():
            try:
                client, _ = sock.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            self._count_client(+1)
            threading.Thread(target=self._serve, args=(client,), daemon=True).start()

    def _serve(self, client: socket.socket) -> None:
        client.settimeout(_POLL_SECONDS)
        stream = RtuStream()
        try:
            while not self._stop.is_set():
                try:
                    data = client.recv(512)
                except TimeoutError:
                    continue
                except OSError:
                    return
                if not data:
                    return
                try:
                    requests = stream.feed(data)
                except UnframeableError as exc:
                    self._refuse_unframeable(client, exc)
                    return
                for request in requests:
                    reply = self._answer(request)
                    if reply is not None:
                        client.sendall(reply)
        finally:
            self._count_client(-1)
            client.close()

    def _answer(self, request: RtuRequest) -> bytes | None:
        """The frame to send back, or ``None`` to leave the line quiet."""
        try:
            match request:
                case ReadHolding():
                    values = self._line.read_holding(
                        request.slave_id, request.address, request.count
                    )
                    return request.reply(values)
                case WriteSingle() | WriteMultiple():
                    self._line.write_holding(
                        request.slave_id, request.address, list(request.values)
                    )
                    return request.reply()
        except NoSlaveError:
            return None
        except IndexError:
            return request.exception(EX_ILLEGAL_DATA_ADDRESS)

    def _refuse_unframeable(self, client: socket.socket, exc: UnframeableError) -> None:
        """Answer a function we cannot frame, then let the caller hang up.

        The reply is a courtesy: the stream position is lost with the
        frame, so there is nothing sensible left to read afterwards.
        """
        if exc.slave_id not in self._line.slave_ids:
            return
        with contextlib.suppress(OSError):
            client.sendall(exception_reply(exc.slave_id, exc.function, EX_ILLEGAL_FUNCTION))

    def _count_client(self, delta: int) -> None:
        with self._lock:
            self._client_count += delta
            count = self._client_count
            callback = self._on_connect_change
        if callback is not None:
            callback(count)


def parse_gateway_ports(raw: Any, *, default: Sequence[int]) -> tuple[int, ...]:
    """Read a ``flange_gateway_ports`` option into the ports to listen on.

    Every device that has a flange spells the option the same way, and
    differs only in whether leaving it out means the port real hardware
    answers on or means nothing at all -- which is what *default* says.
    ``true`` asks for that same default, ``false`` shuts the passthrough,
    and a number or a list of them names the ports outright.
    """
    if raw is None or raw is True:
        return tuple(default)
    if raw is False:
        return ()
    if isinstance(raw, int):
        return (raw,)
    if isinstance(raw, list | tuple):
        return tuple(int(port) for port in raw)
    raise ValueError(f"flange_gateway_ports wants false, a port or a list of them, got {raw!r}")
