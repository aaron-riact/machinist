"""Thin EtherNet/IP scanner wrapper around the vendored ``eeip`` client."""

from __future__ import annotations

import socket
import threading
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from datetime import datetime
from threading import RLock
from typing import Any, Union

from eeip import ConnectionType, EEIPClient, RealTimeFormat

_EnumValue = Union[RealTimeFormat, ConnectionType]


@dataclass(frozen=True, slots=True)
class EtherNetIPScannerConfig:
    host: str
    port: int = 44818
    originator_udp_port: int = 2222
    target_udp_port: int = 2222
    assembly_object_class: int = 0x04
    configuration_assembly_instance_id: int = 0x01
    output_assembly_instance_id: int = 0x64
    input_assembly_instance_id: int = 0x65
    output_length: int = 100
    input_length: int = 100
    requested_packet_rate_ms: int = 20
    o_t_realtime_format: str = "modeless"
    t_o_realtime_format: str = "modeless"
    o_t_connection_type: str = "point_to_point"
    t_o_connection_type: str = "point_to_point"


@dataclass(frozen=True, slots=True)
class EtherNetIPAdapterConfig:
    host: str
    port: int = 44818
    udp_port: int = 2222
    output_length: int = 100
    input_length: int = 100
    requested_packet_rate_ms: int = 20
    o_t_realtime_format: str = "modeless"
    t_o_realtime_format: str = "modeless"


_REALTIME_FORMATS = {
    "header32bit": RealTimeFormat.HEADER32BIT,
    "heartbeat": RealTimeFormat.HEARTBEAT,
    "zerolength": RealTimeFormat.ZEROLENGTH,
    "modeless": RealTimeFormat.MODELESS,
}

_CONNECTION_TYPES = {
    "null": ConnectionType.NULL,
    "multicast": ConnectionType.MULTICAST,
    "point_to_point": ConnectionType.POINT_TO_POINT,
}


class EtherNetIPScanner:
    """Owns one outbound Class 1 connection to an EtherNet/IP adapter."""

    def __init__(
        self,
        config: EtherNetIPScannerConfig,
        *,
        client_factory: Callable[[], Any] = EEIPClient,
    ) -> None:
        self._config = config
        self._client_factory = client_factory
        self._client: Any | None = None
        self._lock = RLock()
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def last_received_at(self) -> datetime | None:
        with self._lock:
            client = self._client
            if client is None:
                return None
            last = client.last_received_implicit_message
        return last if isinstance(last, datetime) else None

    def open(self) -> None:
        with self._lock:
            if self._connected:
                return
            client = self._client_factory()
            self._configure(client)
            client.register_session(self._config.host, self._config.port)
            client.forward_open()
            self._client = client
            self._connected = True

    def close(self) -> None:
        with self._lock:
            client = self._client
            self._client = None
            self._connected = False
        if client is None:
            return
        with suppress(Exception):
            client.forward_close()
        with suppress(Exception):
            client.unregister_session()

    def write_output_block(self, data: bytes | bytearray) -> None:
        payload = list(bytes(data[: self._config.output_length]))
        if len(payload) < self._config.output_length:
            payload.extend([0] * (self._config.output_length - len(payload)))
        with self._lock:
            client = self._require_client()
            client.o_t_iodata = payload

    def read_input_block(self) -> bytes:
        with self._lock:
            client = self._require_client()
            payload = bytes(client.t_o_iodata[: self._config.input_length])
        if len(payload) < self._config.input_length:
            payload += b"\x00" * (self._config.input_length - len(payload))
        return payload

    def _configure(self, client: Any) -> None:
        cfg = self._config
        client.originator_udp_port = cfg.originator_udp_port
        client.target_udp_port = cfg.target_udp_port
        client.assembly_object_class = cfg.assembly_object_class
        client.configuration_assembly_instance_id = cfg.configuration_assembly_instance_id
        client.o_t_instance_id = cfg.output_assembly_instance_id
        client.t_o_instance_id = cfg.input_assembly_instance_id
        client.o_t_length = cfg.output_length
        client.t_o_length = cfg.input_length
        requested_packet_rate = cfg.requested_packet_rate_ms * 1000
        client.o_t_requested_packet_rate = requested_packet_rate
        client.t_o_requested_packet_rate = requested_packet_rate
        client.o_t_realtime_format = _enum_value(
            _REALTIME_FORMATS, cfg.o_t_realtime_format, "o_t_realtime_format"
        )
        client.t_o_realtime_format = _enum_value(
            _REALTIME_FORMATS, cfg.t_o_realtime_format, "t_o_realtime_format"
        )
        client.o_t_connection_type = _enum_value(
            _CONNECTION_TYPES, cfg.o_t_connection_type, "o_t_connection_type"
        )
        client.t_o_connection_type = _enum_value(
            _CONNECTION_TYPES, cfg.t_o_connection_type, "t_o_connection_type"
        )
        client.o_t_variable_length = False
        client.t_o_variable_length = False
        client.o_t_owner_redundant = False
        client.t_o_owner_redundant = False

    def _require_client(self) -> Any:
        if self._client is None:
            raise RuntimeError("EtherNet/IP scanner is not connected")
        return self._client


class EtherNetIPAdapter:
    """Minimal EtherNet/IP adapter/server for Class 1 I/O."""

    def __init__(self, config: EtherNetIPAdapterConfig) -> None:
        self._config = config
        self._lock = RLock()
        self._session_handle = 1
        self._listening = False
        self._peer_connected = False
        self._last_received_at: datetime | None = None
        self._input_block = bytearray(config.input_length)
        self._output_block = bytearray(config.output_length)
        self._tcp_socket: socket.socket | None = None
        self._udp_socket: socket.socket | None = None
        self._client_socket: socket.socket | None = None
        self._accept_thread: threading.Thread | None = None
        self._udp_thread: threading.Thread | None = None
        self._udp_send_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._peer_udp: tuple[str, int] | None = None
        self._peer_udp_port: int = 0
        self._connection_id_o_t = 0
        self._connection_id_t_o = 0
        self._udp_sequence = 0
        self._connection_generation = 0
        self._o_t_realtime_format: str = self._config.o_t_realtime_format
        self._t_o_realtime_format: str = self._config.t_o_realtime_format
        self._input_length: int = config.input_length
        self._output_length: int = config.output_length

    @property
    def connected(self) -> bool:
        return self._listening

    @property
    def peer_connected(self) -> bool:
        return self._peer_connected

    @property
    def connection_generation(self) -> int:
        return self._connection_generation

    @property
    def last_received_at(self) -> datetime | None:
        with self._lock:
            return self._last_received_at

    def open(self) -> None:
        with self._lock:
            if self._listening:
                return
            tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            tcp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            tcp_socket.bind((self._config.host, self._config.port))
            tcp_socket.listen()
            tcp_socket.settimeout(0.2)

            udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            udp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            udp_socket.bind((self._config.host, self._config.udp_port))
            udp_socket.settimeout(0.2)

            self._tcp_socket = tcp_socket
            self._udp_socket = udp_socket
            self._stop.clear()
            self._listening = True
            self._accept_thread = threading.Thread(target=self._accept_loop, daemon=True)
            self._accept_thread.start()
            self._udp_thread = threading.Thread(target=self._udp_loop, daemon=True)
            self._udp_thread.start()
            self._udp_send_thread = threading.Thread(target=self._udp_send_loop, daemon=True)
            self._udp_send_thread.start()

    def close(self) -> None:
        with self._lock:
            tcp_socket = self._tcp_socket
            udp_socket = self._udp_socket
            client_socket = self._client_socket
            accept_thread = self._accept_thread
            udp_thread = self._udp_thread
            udp_send_thread = self._udp_send_thread
            self._tcp_socket = None
            self._udp_socket = None
            self._client_socket = None
            self._accept_thread = None
            self._udp_thread = None
            self._udp_send_thread = None
            self._listening = False
            self._peer_connected = False
            self._peer_udp = None
            self._connection_id_o_t = 0
            self._connection_id_t_o = 0
            self._stop.set()
        for sock in (client_socket, tcp_socket, udp_socket):
            if sock is None:
                continue
            with suppress(OSError):
                sock.close()
        for thread in (accept_thread, udp_thread, udp_send_thread):
            if thread is not None:
                thread.join(timeout=1.0)

    def write_output_block(self, data: bytes | bytearray) -> None:
        payload = bytes(data[: self._output_length]).ljust(
            self._output_length, b"\x00"
        )
        with self._lock:
            self._output_block[:] = payload

    def read_input_block(self) -> bytes:
        with self._lock:
            return bytes(self._input_block)

    def drop_peer(self) -> None:
        with self._lock:
            client_socket = self._client_socket
            self._client_socket = None
            self._peer_connected = False
            self._peer_udp = None
            self._connection_id_o_t = 0
            self._connection_id_t_o = 0
        if client_socket is None:
            return
        with suppress(OSError):
            client_socket.close()

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            tcp_socket = self._tcp_socket
            if tcp_socket is None:
                return
            try:
                client, _addr = tcp_socket.accept()
            except TimeoutError:
                continue
            except OSError:
                return
            with self._lock:
                if self._client_socket is not None:
                    with suppress(OSError):
                        self._client_socket.close()
                self._client_socket = client
            thread = threading.Thread(target=self._client_loop, args=(client,), daemon=True)
            thread.start()

    def _client_loop(self, client: socket.socket) -> None:
        client.settimeout(0.2)
        try:
            while not self._stop.is_set():
                packet = _read_encapsulation_packet(client)
                if packet is None:
                    return
                command = int.from_bytes(packet[0:2], "little")
                if command == 0x0065:
                    reply = _register_session_reply(packet, self._session_handle)
                    client.sendall(reply)
                elif command == 0x0066:
                    with self._lock:
                        self._peer_connected = False
                    return
                elif command == 0x006F:
                    service = packet[40] if len(packet) > 40 else 0
                    if service in (0x54, 0x58):
                        self._handle_forward_open(client, packet)
                    elif service == 0x4E:
                        self._handle_forward_close(client, packet)
        except OSError:
            pass
        finally:
            with self._lock:
                if self._client_socket is client:
                    self._client_socket = None
                self._peer_connected = False

    def _handle_forward_open(self, client: socket.socket, packet: bytes) -> None:
        with self._lock:
            self._connection_id_o_t = int.from_bytes(packet[48:52], "little")
            self._connection_id_t_o = int.from_bytes(packet[52:56], "little")
            # Parse scanner's UDP port from CPF Item 2 (Socket Address Info 0x8001)
            item_count = int.from_bytes(packet[30:32], "little")
            if item_count >= 3 and len(packet) > 40:
                item1_len = int.from_bytes(packet[38:40], "little")
                item2_start = 40 + item1_len
                if (item2_start + 4 <= len(packet)
                        and int.from_bytes(packet[item2_start:item2_start + 2], "little") == 0x8001):
                    port_raw = packet[item2_start + 6:item2_start + 8]
                    self._peer_udp_port = port_raw[0] << 8 | port_raw[1]
            self._peer_connected = True
            self._connection_generation += 1
            import sys
            svc = packet[40]
            is_large = svc == 0x58
            print(f"[EIP] Forward Open: service=0x{svc:02X} is_large={is_large} len={len(packet)}", file=sys.stderr)
            print(f"[EIP]   40-81: {packet[40:82].hex()}", file=sys.stderr)
            cp_len = 4 if is_large else 2
            mask = 0xFFFF if is_large else 0x1FF
            o_sz = int.from_bytes(packet[72:72 + cp_len], "little") & mask
            t_sz = int.from_bytes(packet[78:78 + cp_len], "little") & mask
            print(f"[EIP]   o_t_conn_size={o_sz} t_o_conn_size={t_sz}", file=sys.stderr)
            # O→T = Originator→Target = adapter input; T→O = Target→Originator = adapter output
            # Infer closest format by matching data_length = conn_size - header_offset to config
            _MAP = {0: "heartbeat", 2: "modeless", 6: "header32bit"}
            def _best_fmt(conn_size: int, cfg_len: int) -> str:
                best = 2, "modeless"
                for off in _MAP:
                    if conn_size > off:
                        diff = abs((conn_size - off) - cfg_len)
                        if diff < abs((conn_size - best[0]) - cfg_len):
                            best = off, _MAP[off]
                return best[1]
            self._o_t_realtime_format = _best_fmt(o_sz, self._input_length)
            inferred_in = o_sz - {v: k for k, v in _MAP.items()}[self._o_t_realtime_format]
            # Pick T→O format whose data_length best matches O→T's inferred data_length.
            # Both assemblies are typically the same size, and the O→T reference is
            # more reliable than the config default when sizes differ.
            self._t_o_realtime_format = _best_fmt(t_sz, inferred_in)
            inferred_out = t_sz - {v: k for k, v in _MAP.items()}[self._t_o_realtime_format]
            print(f"[EIP]   O→T fmt={self._o_t_realtime_format} inferred_input_len={inferred_in} (cfg={self._input_length})", file=sys.stderr)
            print(f"[EIP]   T→O fmt={self._t_o_realtime_format} inferred_output_len={inferred_out} (cfg={self._output_length})", file=sys.stderr)
            if inferred_in != self._input_length:
                print(f"[EIP]   adapting input_length {self._input_length} → {inferred_in}", file=sys.stderr)
                self._input_length = inferred_in
                old = self._input_block
                self._input_block = bytearray(inferred_in)
                self._input_block[:len(old)] = old[:inferred_in]
            if inferred_out != self._output_length:
                print(f"[EIP]   adapting output_length {self._output_length} → {inferred_out}", file=sys.stderr)
                self._output_length = inferred_out
                old = self._output_block
                self._output_block = bytearray(inferred_out)
                self._output_block[:len(old)] = old[:inferred_out]
        path_size = packet[41] if len(packet) > 41 else 0
        payload = (
            packet[48:64]          # O→T CID + T→O CID + Serial + VendorID + SerialNum
            + packet[68:72]        # O→T API (= O→T Requested Packet Rate)
            + packet[74:78]        # T→O API (= T→O Requested Packet Rate)
            + (0).to_bytes(2, "little")  # Application Reply Size
        )
        socket_address = bytearray()
        socket_address += (0x8001).to_bytes(2, "little")
        socket_address += (16).to_bytes(2, "little")
        socket_address += (2).to_bytes(2, "big")          # sin_family = AF_INET
        socket_address += self._config.udp_port.to_bytes(2, "big")  # sin_port
        socket_address += (0).to_bytes(4, "big")           # sin_address = 0.0.0.0
        socket_address += (0).to_bytes(8, "big")           # sin_zero
        reply = _send_rrdata_reply(
            packet,
            service=packet[40] | 0x80,
            payload=payload,
            session_handle=self._session_handle,
            socket_address=bytes(socket_address),
        )
        client.sendall(reply)

    def _handle_forward_close(self, client: socket.socket, packet: bytes) -> None:
        with self._lock:
            self._peer_connected = False
            self._peer_udp = None
            self._connection_id_o_t = 0
            self._connection_id_t_o = 0
        client.sendall(
            _send_rrdata_reply(
                packet,
                service=0xCE,
                payload=b"",
                session_handle=self._session_handle,
            )
        )

    def _udp_loop(self) -> None:
        while not self._stop.is_set():
            udp_socket = self._udp_socket
            if udp_socket is None:
                return
            try:
                message, address = udp_socket.recvfrom(2048)
            except TimeoutError:
                continue
            except OSError:
                return
            if len(message) < 20:
                continue
            with self._lock:
                got_cid = int.from_bytes(message[6:10], "little")
                if got_cid != self._connection_id_o_t:
                    continue
                got_type = int.from_bytes(message[14:16], "little")
                if got_type != 0x00B1:
                    continue
                seq_addr_len = int.from_bytes(message[16:18], "little")
                next_item = 14 + 4 + seq_addr_len
                if (next_item + 2 <= len(message)
                        and int.from_bytes(message[next_item:next_item + 2], "little") == 0x00B2):
                    data_item_len = int.from_bytes(message[next_item + 2:next_item + 4], "little")
                    raw_payload = message[next_item + 4:next_item + 4 + data_item_len]
                    if _uses_header32bit(self._o_t_realtime_format):
                        raw_payload = raw_payload[4:]
                else:
                    hdr = 4 if _uses_header32bit(self._o_t_realtime_format) else 0
                    raw_payload = message[20 + hdr:]
                block = bytes(raw_payload[: self._input_length]).ljust(
                    self._input_length, b"\x00"
                )
                self._input_block[:] = block
                self._peer_udp = address
                self._last_received_at = datetime.utcnow()
                self._peer_connected = True

    def _udp_send_loop(self) -> None:
        interval = self._config.requested_packet_rate_ms / 1000.0
        while not self._stop.is_set():
            with self._lock:
                udp_socket = self._udp_socket
                peer_udp = self._peer_udp
                peer_port = self._peer_udp_port or (peer_udp[1] if peer_udp else 0)
                connection_id = self._connection_id_t_o
                payload = bytes(self._output_block)
                active = self._peer_connected and peer_udp is not None and connection_id != 0
                if active:
                    self._udp_sequence += 1
                    sequence = self._udp_sequence
                    target = (peer_udp[0], peer_port)
            if active and udp_socket is not None and peer_udp is not None:
                message = _build_udp_message(
                    connection_id=connection_id,
                    sequence=sequence,
                    payload=payload,
                    realtime_format=self._t_o_realtime_format,
                )
                with suppress(OSError):
                    udp_socket.sendto(message, target)
            self._stop.wait(interval)


def _enum_value(mapping: dict[str, _EnumValue], raw: str, field: str) -> _EnumValue:
    key = raw.strip().lower()
    try:
        return mapping[key]
    except KeyError as exc:
        allowed = ", ".join(sorted(mapping))
        raise ValueError(f"{field} must be one of: {allowed}") from exc


def _read_encapsulation_packet(client: socket.socket) -> bytes | None:
    header = _recv_exact(client, 24)
    if header is None:
        return None
    length = int.from_bytes(header[2:4], "little")
    body = _recv_exact(client, length)
    if body is None:
        return None
    return header + body


def _recv_exact(client: socket.socket, size: int) -> bytes | None:
    chunks = bytearray()
    while len(chunks) < size:
        try:
            chunk = client.recv(size - len(chunks))
        except TimeoutError:
            continue
        if not chunk:
            return None
        chunks.extend(chunk)
    return bytes(chunks)


def _register_session_reply(request: bytes, session_handle: int) -> bytes:
    body = b"\x01\x00\x00\x00"
    return _encapsulation_reply(request, command=0x0065, session_handle=session_handle, body=body)


def _send_rrdata_reply(
    request: bytes, *, service: int, payload: bytes, session_handle: int,
    socket_address: bytes | None = None,
) -> bytes:
    cip = bytes([service, 0x00, 0x00, 0x00]) + payload
    item_count = 3 if socket_address is not None else 2
    cpf = bytearray()
    cpf += item_count.to_bytes(2, "little")
    cpf += (0).to_bytes(2, "little")  # Null Address Item type
    cpf += (0).to_bytes(2, "little")  # Null Address Item length
    cpf += (0x00B2).to_bytes(2, "little")  # Unconnected Data type
    cpf += len(cip).to_bytes(2, "little")  # Unconnected Data length
    cpf += cip
    if socket_address is not None:
        cpf += socket_address
    body = b"\x00\x00\x00\x00\x00\x00" + bytes(cpf)
    return _encapsulation_reply(request, command=0x006F, session_handle=session_handle, body=body)


def _encapsulation_reply(
    request: bytes, *, command: int, session_handle: int, body: bytes,
) -> bytes:
    header = bytearray(24)
    header[0:2] = command.to_bytes(2, "little")
    header[2:4] = len(body).to_bytes(2, "little")
    header[4:8] = session_handle.to_bytes(4, "little")
    header[8:12] = (0).to_bytes(4, "little")
    header[12:20] = request[12:20]
    header[20:24] = (0).to_bytes(4, "little")
    return bytes(header) + body


def _uses_header32bit(raw: str) -> bool:
    return raw.strip().lower() == "header32bit"


def _build_udp_message(
    *, connection_id: int, sequence: int, payload: bytes, realtime_format: str,
) -> bytes:
    header32bit = _uses_header32bit(realtime_format)
    message = bytearray()
    message += (2).to_bytes(2, "little")            # item count
    message += (0x8002).to_bytes(2, "little")        # type + attr
    message += (8).to_bytes(2, "little")             # length (conn_id + seq)
    message += connection_id.to_bytes(4, "little")   # connection ID
    message += sequence.to_bytes(4, "little")        # sequence number
    message += (0x00B1).to_bytes(2, "little")        # Sequence Address Item
    data_length = len(payload) + 2 + (4 if header32bit else 0)
    message += data_length.to_bytes(2, "little")     # item length
    message += (sequence & 0xFFFF).to_bytes(2, "little")  # seq count
    if header32bit:
        hdr32 = (sequence & 0xFFFF) | (1 << 16)  # seq_count + Run/Idle=1
        message += hdr32.to_bytes(4, "little")
    message += payload
    return bytes(message)
