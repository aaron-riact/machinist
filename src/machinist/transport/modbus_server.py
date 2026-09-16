"""A tiny native Modbus/TCP server for emulating slave devices.

We deliberately don't depend on ``pymodbus`` for the framework itself —
that library is large, asyncio-only, and pulls in many transitive
dependencies. Industrial Modbus over TCP is a small, well-defined wire
format (MBAP header + PDU) that we implement here for the function
codes our emulators actually use:

* 0x03  Read Holding Registers
* 0x06  Write Single Register
* 0x10  Write Multiple Registers

The server serves a device's :class:`~machinist.transport.registers.RegisterPort`,
so a device maps register addresses to its own state without coupling to
the wire format -- and the same port can be served over another front end.
"""

from __future__ import annotations

import socket
import struct
import threading
from collections.abc import Callable

from .registers import ReadCallback, RegisterPort, WriteCallback
from .service import Service

__all__ = ["HoldingRegisterServer", "ReadCallback", "RegisterPort", "WriteCallback"]

_MBAP = struct.Struct(">HHHB")  # txn id, proto id, length, unit id
_HEADER_LEN = _MBAP.size

_FUNC_READ_HOLDING = 0x03
_FUNC_WRITE_SINGLE = 0x06
_FUNC_WRITE_MULTIPLE = 0x10

_EX_ILLEGAL_FUNCTION = 0x01
_EX_ILLEGAL_DATA_ADDRESS = 0x02


class HoldingRegisterServer(Service):
    """Threaded Modbus/TCP server exposing a holding-register callback."""

    def __init__(
        self,
        *,
        host: str,
        port: int,
        registers: RegisterPort,
        on_connect_change: Callable[[int], None] | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._registers = registers
        self._on_connect_change = on_connect_change
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._client_count = 0
        self._lock = threading.Lock()

    @property
    def client_count(self) -> int:
        return self._client_count

    @property
    def listening(self) -> bool:
        """True once :meth:`serve_forever` has bound its socket."""
        return self._sock is not None

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.listen(8)
        self._sock.settimeout(0.25)
        if ready is not None:
            ready.set()
        try:
            while not self._stop.is_set():
                try:
                    client, _ = self._sock.accept()
                except TimeoutError:
                    continue
                except OSError:
                    return
                with self._lock:
                    self._client_count += 1
                    cb = self._on_connect_change
                if cb is not None:
                    cb(self._client_count)
                threading.Thread(target=self._serve, args=(client,), daemon=True).start()
        finally:
            if self._sock is not None:
                self._sock.close()

    def shutdown(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    # -----------------------------------------------------------------

    def _serve(self, client: socket.socket) -> None:
        client.settimeout(0.25)
        try:
            while not self._stop.is_set():
                header = _recv_exact(client, _HEADER_LEN, self._stop)
                if header is None:
                    return
                txn, proto, length, unit = _MBAP.unpack(header)
                body = _recv_exact(client, length - 1, self._stop)
                if body is None:
                    return
                response = self._handle(body)
                if response is None:
                    return
                client.sendall(_MBAP.pack(txn, proto, len(response) + 1, unit) + response)
        finally:
            with self._lock:
                self._client_count -= 1
                cb = self._on_connect_change
                count = self._client_count
            if cb is not None:
                cb(count)
            client.close()

    def _handle(self, body: bytes) -> bytes | None:
        func = body[0]
        try:
            if func == _FUNC_READ_HOLDING:
                return self._read_holding(body)
            if func == _FUNC_WRITE_SINGLE:
                return self._write_single(body)
            if func == _FUNC_WRITE_MULTIPLE:
                return self._write_multiple(body)
        except IndexError:
            return _exception(func, _EX_ILLEGAL_DATA_ADDRESS)
        return _exception(func, _EX_ILLEGAL_FUNCTION)

    def _read_holding(self, body: bytes) -> bytes:
        address, count = struct.unpack(">HH", body[1:5])
        payload = b"".join(struct.pack(">H", v) for v in self._registers.read(address, count))
        return bytes([_FUNC_READ_HOLDING, len(payload)]) + payload

    def _write_single(self, body: bytes) -> bytes:
        address, value = struct.unpack(">HH", body[1:5])
        self._registers.write(address, [value])
        return bytes([_FUNC_WRITE_SINGLE]) + struct.pack(">HH", address, value)

    def _write_multiple(self, body: bytes) -> bytes:
        address, count = struct.unpack(">HH", body[1:5])
        values = [struct.unpack(">H", body[6 + i * 2 : 8 + i * 2])[0] for i in range(count)]
        self._registers.write(address, values)
        return bytes([_FUNC_WRITE_MULTIPLE]) + struct.pack(">HH", address, count)


def _recv_exact(sock: socket.socket, n: int, stop: threading.Event) -> bytes | None:
    buf = bytearray()
    while len(buf) < n:
        if stop.is_set():
            return None
        try:
            chunk = sock.recv(n - len(buf))
        except TimeoutError:
            continue
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _exception(func: int, code: int) -> bytes:
    return bytes([func | 0x80, code])
