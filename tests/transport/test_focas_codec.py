"""Tests for the FOCAS frame codec."""

import struct
from src.machinist.transport.focas import (
    CONNECT_REQ, CONNECT_RESP, CLOSE_REQ, CLOSE_RESP, VAR_REQ, VAR_RESP,
    FocasSubpacket, FocasFrame,
)


class TestFocasSubpacket:
    def test_encode_decode_single(self):
        sp = FocasSubpacket(c1=1, c2=1, c3=0x18)
        raw = sp.encode()
        assert len(raw) > 2
        # decode needs just the subpacket body (after length prefix)
        sp_len = struct.unpack(">H", raw[:2])[0]
        body = raw[2:]
        assert len(body) + 2 == sp_len
        decoded = FocasSubpacket.decode(body)
        assert (decoded.c1, decoded.c2, decoded.c3) == (1, 1, 0x18)
        assert decoded.v1 == decoded.v2 == decoded.v3 == decoded.v4 == decoded.v5 == 0
        assert decoded.payload == b""

    def test_trailing_payload(self):
        sp = FocasSubpacket(c1=2, c2=1, c3=0x8001, v1=2204, v2=2205,
                            v3=9, v4=1, payload=b"\x00" * 256)
        raw = sp.encode()
        sp_len = struct.unpack(">H", raw[:2])[0]
        body = raw[2:]
        assert len(body) + 2 == sp_len
        decoded = FocasSubpacket.decode(body)
        assert (decoded.c1, decoded.c2, decoded.c3) == (2, 1, 0x8001)
        assert (decoded.v1, decoded.v2, decoded.v3, decoded.v4) == (2204, 2205, 9, 1)
        assert decoded.payload == b"\x00" * 256

    def test_response_ok_format(self):
        sp = FocasSubpacket(c1=1, c2=1, c3=0x45)
        payload = struct.pack(">HHH", 2020, 5, 14)
        resp = sp.encode_response_ok(payload)
        # wire format: len(2) + c1(2) + c2(2) + c3(2) + filler(6) + payload_len(2) + payload
        sp_len = struct.unpack(">H", resp[:2])[0]
        body = resp[2:]
        assert len(body) + 2 == sp_len
        # first 6 bytes: c1, c2, c3
        assert struct.unpack(">HHH", body[:6]) == (1, 1, 0x45)
        # next 6: filler (zeros)
        assert body[6:12] == b"\x00" * 6
        # payload length
        plen = struct.unpack(">H", body[12:14])[0]
        assert plen == len(payload)
        assert body[14:14 + plen] == payload

    def test_response_error_format(self):
        sp = FocasSubpacket(c1=1, c2=1, c3=0x30)
        resp = sp.encode_response_error(6)
        sp_len = struct.unpack(">H", resp[:2])[0]
        body = resp[2:]
        assert len(body) + 2 == sp_len
        assert struct.unpack(">HHHh", body[:8]) == (1, 1, 0x30, 6)

    def test_response_ok_empty(self):
        sp = FocasSubpacket(c1=1, c2=1, c3=0x19)
        resp = sp.encode_response_ok()
        assert len(resp) == 16  # 2 len + 6 head + 6 filler + 2 plen


class TestFocasFrame:
    def test_encode_decode_connect(self):
        raw = FocasFrame(type=CONNECT_REQ).encode()
        assert raw[:4] == b"\xa0" * 4
        decoded = FocasFrame.decode(raw)
        assert decoded.type == CONNECT_REQ
        assert decoded.version == 1
        assert len(decoded.subpackets) == 0

    def test_connect_response(self):
        raw = FocasFrame(type=CONNECT_RESP).encode()
        decoded = FocasFrame.decode(raw)
        assert decoded.type == CONNECT_RESP

    def test_close_roundtrip(self):
        raw = FocasFrame(type=CLOSE_REQ).encode()
        decoded = FocasFrame.decode(raw)
        assert decoded.type == CLOSE_REQ
        resp = FocasFrame(type=CLOSE_RESP).encode()
        decoded_resp = FocasFrame.decode(resp)
        assert decoded_resp.type == CLOSE_RESP

    def test_var_req_single_subpacket(self):
        sp = FocasSubpacket(c1=1, c2=1, c3=0x18)
        frame = FocasFrame(type=VAR_REQ, subpackets=(sp,))
        raw = frame.encode()
        decoded = FocasFrame.decode(raw)
        assert decoded.type == VAR_REQ
        assert len(decoded.subpackets) == 1
        s = decoded.subpackets[0]
        assert (s.c1, s.c2, s.c3) == (1, 1, 0x18)

    def test_var_req_multi_subpacket(self):
        sp1 = FocasSubpacket(c1=1, c2=1, c3=0x45, v1=0)
        sp2 = FocasSubpacket(c1=1, c2=1, c3=0x45, v1=1)
        frame = FocasFrame(type=VAR_REQ, subpackets=(sp1, sp2))
        raw = frame.encode()
        decoded = FocasFrame.decode(raw)
        assert decoded.type == VAR_REQ
        assert len(decoded.subpackets) == 2
        assert decoded.subpackets[0].v1 == 0
        assert decoded.subpackets[1].v1 == 1

    def test_var_resp_roundtrip(self):
        sp = FocasSubpacket(c1=1, c2=1, c3=0x45)
        resp_bytes = sp.encode_response_ok(struct.pack(">HHH", 2020, 5, 14))
        frame = FocasFrame()
        raw = frame.encode_var_response([resp_bytes])
        decoded = FocasFrame.decode(raw)
        assert decoded.type == VAR_RESP
        assert len(decoded.response_subpackets) == 1
        assert decoded.response_subpackets[0] == resp_bytes

    def test_var_resp_multi_roundtrip(self):
        sp1 = FocasSubpacket(c1=1, c2=1, c3=0x45, v1=0)
        sp2 = FocasSubpacket(c1=1, c2=1, c3=0x45, v1=1)
        r1 = sp1.encode_response_ok(struct.pack(">HHH", 2020, 5, 14))
        r2 = sp2.encode_response_ok(struct.pack(">HHH", 12, 15, 5))
        frame = FocasFrame()
        raw = frame.encode_var_response([r1, r2])
        decoded = FocasFrame.decode(raw)
        assert decoded.type == VAR_RESP
        assert len(decoded.response_subpackets) == 2

    def test_decode_invalid_sync(self):
        import pytest
        with pytest.raises(ValueError, match="bad sync"):
            FocasFrame.decode(b"\x00" * 10)

    def test_decode_truncated(self):
        import pytest
        with pytest.raises(ValueError, match="frame too short"):
            FocasFrame.decode(b"\xa0\xa0\xa0\xa0\x00\x01\x21\x01")

    def test_decode_missing_payload(self):
        import pytest
        with pytest.raises(ValueError, match="frame truncated"):
            FocasFrame.decode(b"\xa0\xa0\xa0\xa0\x00\x01\x21\x01\x00\x02")
