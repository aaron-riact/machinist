"""Thin EtherNet/IP scanner wrapper around the vendored ``eeip`` client."""

from __future__ import annotations

from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from datetime import datetime
from threading import RLock
from typing import Any

from eeip import ConnectionType, EEIPClient, RealTimeFormat


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


def _enum_value(mapping: dict[str, object], raw: str, field: str) -> object:
    key = raw.strip().lower()
    try:
        return mapping[key]
    except KeyError as exc:
        allowed = ", ".join(sorted(mapping))
        raise ValueError(f"{field} must be one of: {allowed}") from exc
