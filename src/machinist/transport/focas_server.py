"""Threaded FOCAS1/2 TCP server for FANUC CNC/robot emulation.

Follows the same shape as :class:`HoldingRegisterServer` — a self-contained
TCP server that speaks the FOCAS wire protocol and calls back into the
device for request handling.
"""

from __future__ import annotations

import socket
import struct
import threading
from collections.abc import Callable

from .focas import (
    FocasFrame,
    FocasSubpacket,
    VAR_REQ,
    CONNECT_REQ,
    CONNECT_RESP,
    CLOSE_REQ,
    CLOSE_RESP,
)

FocasHandler = Callable[[FocasSubpacket], bytes]
ConnectHandler = Callable[[], bytes | None]
DisconnectHandler = Callable[[], None]

_FRAME_HEADER = struct.Struct(">4sHHH")  # sync, version, type, length


class FocasServer:
    """Threaded TCP server speaking the FOCAS wire protocol.

    Usage — device wires callbacks once, then calls ``serve_forever``::

        server = FocasServer(
            host="0.0.0.0",
            port=8193,
            on_request=my_handler,
        )
        ready = threading.Event()
        thread = threading.Thread(target=server.serve_forever, args=(ready,), daemon=True)
        thread.start()
        ready.wait()
    """

    def __init__(
        self,
        *,
        host: str,
        port: int,
        on_request: FocasHandler,
        on_connect: ConnectHandler | None = None,
        on_disconnect: DisconnectHandler | None = None,
    ) -> None:
        self._host = host
        self._port = port
        self._on_request = on_request
        self._on_connect = on_connect
        self._on_disconnect = on_disconnect
        self._sock: socket.socket | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._client_count = 0

    @property
    def client_count(self) -> int:
        return self._client_count

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self._host, self._port))
        sock.listen()
        sock.settimeout(0.2)
        self._sock = sock
        if ready is not None:
            ready.set()
        try:
            while not self._stop.is_set():
                try:
                    client, _addr = sock.accept()
                except TimeoutError:
                    continue
                except OSError:
                    return
                with self._lock:
                    self._client_count += 1
                thread = threading.Thread(
                    target=self._client_loop, args=(client,), daemon=True
                )
                thread.start()
        finally:
            self._close_sock(sock)
            self._sock = None

    def shutdown(self) -> None:
        self._stop.set()
        self._close_sock(self._sock)
        self._sock = None

    def _client_loop(self, client: socket.socket) -> None:
        try:
            client.settimeout(1.0)
            while not self._stop.is_set():
                frame_data = _read_frame(client)
                if frame_data is None:
                    return
                frame = FocasFrame.decode(frame_data)
                reply = self._handle_frame(frame)
                if reply is not None:
                    client.sendall(reply)
        except (OSError, ConnectionError, ValueError):
            pass
        finally:
            with self._lock:
                self._client_count -= 1
            if self._on_disconnect is not None:
                self._on_disconnect()
            self._close_sock(client)

    @staticmethod
    def _close_sock(sock: socket.socket | None) -> None:
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _handle_frame(self, frame: FocasFrame) -> bytes | None:
        if frame.type == CONNECT_REQ:
            resp_payload = _connect_response_payload()
            if self._on_connect is not None:
                extra = self._on_connect()
                if extra is not None:
                    resp_payload = extra
            return FocasFrame(version=frame.version, type=CONNECT_RESP).encode()
        if frame.type == CLOSE_REQ:
            return FocasFrame(version=frame.version, type=CLOSE_RESP).encode()
        if frame.type == VAR_REQ:
            resp_sps = [self._on_request(sp) for sp in frame.subpackets]
            return frame.encode_var_response(resp_sps)
        return None


def _read_frame(sock: socket.socket) -> bytes | None:
    """Read one complete FOCAS frame or return None on disconnect."""
    header = _recv_exactly(sock, 10)
    if header is None:
        return None
    sync, version, ftype, length = _FRAME_HEADER.unpack(header)
    if sync != b"\xa0\xa0\xa0\xa0":
        return None
    payload = b""
    if length > 0:
        payload = _recv_exactly(sock, length)
        if payload is None:
            return None
    return header + payload


def _recv_exactly(sock: socket.socket, count: int) -> bytes | None:
    buf = bytearray()
    while len(buf) < count:
        try:
            chunk = sock.recv(count - len(buf))
        except TimeoutError:
            continue
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def _connect_response_payload() -> bytes:
    return struct.pack(">I", 0x00010000)  # protocol revision
