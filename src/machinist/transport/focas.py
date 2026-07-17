"""FOCAS1/2 frame codec — pure data structures and wire format.

``FocasSubpacket`` is a structured request subpacket (c1..c3, v1..v5, payload).
``FocasFrame`` holds parsed frames — for VAR_REQ the subpackets are decoded
into ``FocasSubpacket``; for VAR_RESP they are stored as raw ``bytes``
(since the response wire format differs from the request format).

Frame format::

    sync       4 bytes  0xa0a0a0a0
    version    2 bytes  big-endian uint16
    type       2 bytes  big-endian uint16
    length     2 bytes  big-endian uint16  (payload length)
    payload    N bytes

Types::

    0x0101  connect request       0x0102  connect response
    0x0201  close request         0x0202  close response
    0x2101  variable request      0x2102  variable response

Variable-request payload (type 0x2101)::

    count  2 bytes  uint16
    [for each:]
      len     2 bytes  uint16  (includes this field)
      c1-c3   6 bytes  uint16×3
      v1–v5  20 bytes  int32×5
      trail   N bytes

Variable-response payload (type 0x2102)::

    count  2 bytes  uint16
    [for each:]
      len     2 bytes  uint16
      resp    N bytes  (opaque — use ``encode_response_ok``/``encode_response_error``)
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import ClassVar


_SYNC = b"\xa0" * 4
_HEADER = struct.Struct(">HHH")
_SUBPACKET_LEN = struct.Struct(">H")
_SUBPACKET_ARGS = struct.Struct(">HHHiiiii")  # c1, c2, c3, v1..v5
_SUBPACKET_RESP_HEADER = struct.Struct(">HHH")
_SUBPACKET_RESP_OK = struct.Struct(">6sH")

CONNECT_REQ = 0x0101
CONNECT_RESP = 0x0102
CLOSE_REQ = 0x0201
CLOSE_RESP = 0x0202
VAR_REQ = 0x2101
VAR_RESP = 0x2102


@dataclass(frozen=True, slots=True)
class FocasSubpacket:
    c1: int
    c2: int
    c3: int
    v1: int = 0
    v2: int = 0
    v3: int = 0
    v4: int = 0
    v5: int = 0
    payload: bytes = b""

    def encode(self) -> bytes:
        head = _SUBPACKET_ARGS.pack(self.c1, self.c2, self.c3,
                                    self.v1, self.v2, self.v3, self.v4, self.v5)
        body = head + self.payload
        return _SUBPACKET_LEN.pack(len(body) + 2) + body

    def encode_response_ok(self, data: bytes = b"") -> bytes:
        head = _SUBPACKET_RESP_HEADER.pack(self.c1, self.c2, self.c3)
        filler_payload = _SUBPACKET_RESP_OK.pack(b"\x00" * 6, len(data))
        return _SUBPACKET_LEN.pack(len(head) + len(filler_payload) + len(data) + 2) + head + filler_payload + data

    def encode_response_error(self, error_code: int) -> bytes:
        head = _SUBPACKET_RESP_HEADER.pack(self.c1, self.c2, self.c3)
        err = struct.pack(">h", error_code)
        body = head + err + b"\x00" * 4
        return _SUBPACKET_LEN.pack(len(body) + 2) + body

    _SUB_SIZE = _SUBPACKET_ARGS.size

    @staticmethod
    def decode(data: bytes) -> FocasSubpacket:
        c1, c2, c3, v1, v2, v3, v4, v5 = _SUBPACKET_ARGS.unpack(data[:FocasSubpacket._SUB_SIZE])
        payload = data[FocasSubpacket._SUB_SIZE:]
        return FocasSubpacket(c1, c2, c3, v1, v2, v3, v4, v5, payload)


@dataclass(frozen=True, slots=True)
class FocasFrame:
    version: int = 1
    type: int = 0
    subpackets: tuple[FocasSubpacket, ...] = ()
    response_subpackets: tuple[bytes, ...] = ()

    SYNC: ClassVar[bytes] = _SYNC

    def encode(self) -> bytes:
        if self.type == VAR_REQ:
            sp_data = b"".join(sp.encode() for sp in self.subpackets)
            payload = _SUBPACKET_LEN.pack(len(self.subpackets)) + sp_data
        else:
            payload = b""
        header = _HEADER.pack(self.version, self.type, len(payload))
        return _SYNC + header + payload

    def encode_var_response(self, subpackets: list[bytes]) -> bytes:
        payload = _SUBPACKET_LEN.pack(len(subpackets)) + b"".join(subpackets)
        header = _HEADER.pack(self.version, VAR_RESP, len(payload))
        return _SYNC + header + payload

    @staticmethod
    def decode(data: bytes) -> FocasFrame:
        if len(data) < 10:
            raise ValueError("frame too short")
        if data[:4] != _SYNC:
            raise ValueError(f"bad sync: {data[:4].hex()}")
        version, ftype, length = _HEADER.unpack(data[4:10])
        if len(data) < 10 + length:
            raise ValueError(f"frame truncated: need {10 + length}, have {len(data)}")
        payload = data[10:10 + length]
        if ftype != VAR_REQ and ftype != VAR_RESP:
            return FocasFrame(version=version, type=ftype)
        if len(payload) < 2:
            raise ValueError("var frame missing subpacket count")
        count = _SUBPACKET_LEN.unpack(payload[:2])[0]
        offset = 2
        if ftype == VAR_REQ:
            sps: list[FocasSubpacket] = []
            for _ in range(count):
                if offset + 2 > len(payload):
                    raise ValueError("truncated subpacket length")
                sp_len = _SUBPACKET_LEN.unpack(payload[offset:offset + 2])[0]
                if sp_len < 2 or offset + sp_len > len(payload):
                    raise ValueError(f"bad subpacket length {sp_len}")
                sp_data = payload[offset + 2:offset + sp_len]
                sps.append(FocasSubpacket.decode(sp_data))
                offset += sp_len
            return FocasFrame(version=version, type=ftype, subpackets=tuple(sps))
        else:  # VAR_RESP
            rsp_sps: list[bytes] = []
            for _ in range(count):
                if offset + 2 > len(payload):
                    raise ValueError("truncated response subpacket length")
                sp_len = _SUBPACKET_LEN.unpack(payload[offset:offset + 2])[0]
                if sp_len < 2 or offset + sp_len > len(payload):
                    raise ValueError(f"bad response subpacket length {sp_len}")
                rsp_sps.append(payload[offset:offset + sp_len])
                offset += sp_len
            return FocasFrame(version=version, type=ftype, response_subpackets=tuple(rsp_sps))
