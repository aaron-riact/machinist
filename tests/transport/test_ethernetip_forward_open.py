from __future__ import annotations

from machinist.transport.ethernetip import (
    EtherNetIPAdapter,
    EtherNetIPAdapterConfig,
    _forward_open_connection_path,
    _parse_connection_points,
    _send_rrdata_reply,
)

# Connection path from the live Mazak captures: electronic key (10 bytes) +
# Assembly class, Instance 1, O->T Connection Point 100 (0x64),
# T->O Connection Point 101 (0x65).
_GOOD_PATH = bytes.fromhex("34000000000000000000" "20 04" "24 01" "2C 64" "2C 65".replace(" ", ""))
# Like the good path but the O->T point is 0x7B (123) -> invalid for Mazak.
_BAD_O2T_PATH = bytes.fromhex("34000000000000000000" "20 04" "24 01" "2C 7B" "2C 65".replace(" ", ""))
# 16-bit connection point variant: 0x2D 64 00 = connection point 100.
_WIDE_PATH = bytes.fromhex("34000000000000000000" "20 04" "24 01" "2D 64 00" "2D 65 00".replace(" ", ""))


def _build_adapter() -> EtherNetIPAdapter:
    # __init__ does not bind sockets, so this is safe without open().
    return EtherNetIPAdapter(EtherNetIPAdapterConfig(host="127.0.0.1", port=44818))


def _cip_slice(reply: bytes) -> bytes:
    # Encapsulation header (24) + 6 null body prefix + CPF prefix (10) = 40.
    return reply[40:]


def test_reply_success_keeps_legacy_prefix() -> None:
    reply = _send_rrdata_reply(
        bytes(24), service=0xD4, payload=b"PAYLOAD", session_handle=1,
    )
    cip = _cip_slice(reply)
    assert cip[0] == 0xD4          # service (response bit set by caller)
    assert cip[1] == 0x00          # general status
    assert cip[2] == 0x00          # reserved
    assert cip[3] == 0x00          # additional status size
    assert cip[4:4 + 7] == b"PAYLOAD"


def test_reply_encodes_status_without_extended_status() -> None:
    reply = _send_rrdata_reply(
        bytes(24), service=0xD4, payload=b"PAYLOAD", session_handle=1, status=0x05,
    )
    cip = _cip_slice(reply)
    assert cip[1] == 0x05
    assert cip[3] == 0x00
    assert cip[4:4 + 7] == b"PAYLOAD"


def test_reply_encodes_status_with_one_extended_status_word() -> None:
    # 0x0128 = Invalid T->O size (ODVA-defined additional status).
    ext = (0x0128).to_bytes(2, "little")
    reply = _send_rrdata_reply(
        bytes(24), service=0xD4, payload=b"PAYLOAD", session_handle=1,
        status=0x01, extended_status=ext,
    )
    cip = _cip_slice(reply)
    assert cip[1] == 0x01
    assert cip[3] == 0x01          # one additional-status word
    assert cip[4:6] == ext
    assert cip[6:6 + 7] == b"PAYLOAD"


def test_parse_connection_points_extracts_o_t_and_t_o() -> None:
    assert _parse_connection_points(_GOOD_PATH) == [0x64, 0x65]


def test_parse_connection_points_reports_bad_o_t_point() -> None:
    assert _parse_connection_points(_BAD_O2T_PATH) == [0x7B, 0x65]


def test_parse_connection_points_handles_16_bit_points() -> None:
    assert _parse_connection_points(_WIDE_PATH) == [0x64, 0x65]


def test_forward_open_connection_path_reads_path_at_fixed_offset() -> None:
    packet = bytearray(100)
    packet[81] = 9  # path size in 16-bit words -> 18 bytes
    packet[82:82 + len(_GOOD_PATH)] = _GOOD_PATH
    assert _forward_open_connection_path(bytes(packet)) == _GOOD_PATH


def test_forward_open_connection_path_handles_short_packet() -> None:
    assert _forward_open_connection_path(b"\x00" * 10) == b""
