"""Transport-agnostic request/response message channels.

Some protocols (notably SRCI) are not line-oriented text streams; they
exchange whole *binary telegrams* in a cyclic request/response pattern:
the controller sends a command frame, the device answers with a status
frame. The wire byte-layout of those frames is the protocol's concern;
*how the bytes travel* is the transport's concern.

This module owns that second concern and nothing else. It defines two
tiny Protocols — a client :class:`MessageTransport` and a
:class:`MessageServer` — plus concrete TCP and UDP implementations.
Protocols depend on these abstractions, never on sockets, so the same
SRCI codec can run over TCP today and Modbus or PROFINET tomorrow with
no code change.

* **TCP** frames are length-prefixed (4-byte big-endian) so message
  boundaries survive stream coalescing.
* **UDP** frames map one-to-one onto datagrams, which already carry
  their own boundaries.
"""

from __future__ import annotations

import contextlib
import socket
import struct
import threading
from collections.abc import Callable
from typing import Protocol

#: Server-side handler: a request frame in, a response frame out.
FrameHandler = Callable[[bytes], bytes]

_LENGTH = struct.Struct(">I")
_MAX_FRAME = 1 << 20  # 1 MiB guard against hostile/garbage length prefixes


class MessageTransport(Protocol):
    """Client side: send one frame, get one frame back."""

    def request(self, payload: bytes) -> bytes: ...

    def close(self) -> None: ...


class MessageServer(Protocol):
    """Server side: answer request frames until shut down."""

    def serve_forever(
        self, handler: FrameHandler, ready: threading.Event | None = None
    ) -> None: ...

    def shutdown(self) -> None: ...


# --- TCP ----------------------------------------------------------------


def _recv_exactly(sock: socket.socket, count: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < count:
        chunk = sock.recv(count - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _read_frame(sock: socket.socket) -> bytes | None:
    header = _recv_exactly(sock, _LENGTH.size)
    if header is None:
        return None
    (length,) = _LENGTH.unpack(header)
    if length > _MAX_FRAME:
        raise ValueError(f"frame too large: {length} bytes")
    return _recv_exactly(sock, length)


def _write_frame(sock: socket.socket, payload: bytes) -> None:
    sock.sendall(_LENGTH.pack(len(payload)) + payload)


class TcpMessageTransport:
    """Length-prefixed TCP client channel."""

    def __init__(self, host: str, port: int, *, timeout: float = 5.0) -> None:
        self._sock = socket.create_connection((host, port), timeout=timeout)

    def request(self, payload: bytes) -> bytes:
        _write_frame(self._sock, payload)
        reply = _read_frame(self._sock)
        if reply is None:
            raise ConnectionError("connection closed before reply")
        return reply

    def close(self) -> None:
        self._sock.close()


class TcpMessageServer:
    """Length-prefixed TCP server channel (one thread per client)."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._stop = threading.Event()

    def serve_forever(
        self, handler: FrameHandler, ready: threading.Event | None = None
    ) -> None:
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
                threading.Thread(
                    target=self._serve_client, args=(client, handler), daemon=True
                ).start()
        finally:
            if self._sock is not None:
                self._sock.close()

    def _serve_client(self, client: socket.socket, handler: FrameHandler) -> None:
        client.settimeout(0.25)
        try:
            while not self._stop.is_set():
                try:
                    frame = _read_frame(client)
                except TimeoutError:
                    continue
                except (OSError, ValueError):
                    return
                if frame is None:
                    return
                _write_frame(client, handler(frame))
        finally:
            client.close()

    def shutdown(self) -> None:
        self._stop.set()
        if self._sock is not None:
            with contextlib.suppress(OSError):
                self._sock.shutdown(socket.SHUT_RDWR)


# --- UDP ----------------------------------------------------------------


class UdpMessageTransport:
    """Datagram client channel; one datagram per frame."""

    def __init__(self, host: str, port: int, *, timeout: float = 5.0) -> None:
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(timeout)

    def request(self, payload: bytes) -> bytes:
        self._sock.sendto(payload, self._addr)
        reply, _ = self._sock.recvfrom(_MAX_FRAME)
        return reply

    def close(self) -> None:
        self._sock.close()


class UdpMessageServer:
    """Datagram server channel; answers each datagram from one socket."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._stop = threading.Event()

    def serve_forever(
        self, handler: FrameHandler, ready: threading.Event | None = None
    ) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.settimeout(0.25)
        if ready is not None:
            ready.set()
        try:
            while not self._stop.is_set():
                try:
                    payload, addr = self._sock.recvfrom(_MAX_FRAME)
                except TimeoutError:
                    continue
                except OSError:
                    return
                self._sock.sendto(handler(payload), addr)
        finally:
            if self._sock is not None:
                self._sock.close()

    def shutdown(self) -> None:
        self._stop.set()
        if self._sock is not None:
            self._sock.close()


# --- transport selection ------------------------------------------------

_CLIENTS: dict[str, Callable[..., MessageTransport]] = {
    "tcp": TcpMessageTransport,
    "udp": UdpMessageTransport,
}
_SERVERS: dict[str, Callable[[str, int], MessageServer]] = {
    "tcp": TcpMessageServer,
    "udp": UdpMessageServer,
}


def transports() -> tuple[str, ...]:
    """Names of the transports available for SRCI and friends."""
    return tuple(_SERVERS)


def open_transport(name: str, host: str, port: int, **kwargs: object) -> MessageTransport:
    """Construct a client transport by name (``tcp`` | ``udp``)."""
    try:
        factory = _CLIENTS[name]
    except KeyError:
        raise ValueError(f"unknown transport {name!r}; have {transports()}") from None
    return factory(host, port, **kwargs)


def open_server(name: str, host: str, port: int) -> MessageServer:
    """Construct a server transport by name (``tcp`` | ``udp``)."""
    try:
        factory = _SERVERS[name]
    except KeyError:
        raise ValueError(f"unknown transport {name!r}; have {transports()}") from None
    return factory(host, port)

