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


class _RecordingClient:
    def __init__(self) -> None:
        self.sent: list[bytes] = []

    def sendall(self, data: bytes) -> None:
        self.sent.append(data)


def _build_forward_open(
    *, o_t_size: int, t_o_size: int, path: bytes = _GOOD_PATH,
    serial: int = 1, vendor: int = 1, originator: int = 0xBEEF00D, service: int = 0x54,
) -> bytes:
    """Craft a minimal Forward Open request that the handler can validate."""
    pkt = bytearray(82 + len(path))
    pkt[40] = service
    pkt[48:52] = (0x11111111).to_bytes(4, "little")  # O->T CID
    pkt[52:56] = (0x22222222).to_bytes(4, "little")  # T->O CID
    pkt[56:58] = serial.to_bytes(2, "little")
    pkt[58:60] = vendor.to_bytes(2, "little")
    pkt[60:64] = originator.to_bytes(4, "little")
    pkt[72:74] = o_t_size.to_bytes(2, "little")      # O->T connection size
    pkt[78:80] = t_o_size.to_bytes(2, "little")      # T->O connection size
    pkt[81] = len(path) // 2                          # path size in words
    pkt[82:82 + len(path)] = path
    return bytes(pkt)


def _reply_status(reply: bytes) -> int:
    # CIP General Status byte sits at offset 41 in the encapsulation reply.
    return reply[41]


def _reply_ext_size(reply: bytes) -> int:
    return reply[43]


def _status_of_forward_open(
    adapter: EtherNetIPAdapter, pkt: bytes, client: _RecordingClient | None = None,
) -> int:
    client = client or _RecordingClient()
    adapter._handle_forward_open(client, pkt)
    assert len(client.sent) == 1
    return _reply_status(client.sent[0])


# --- Base (spec-strict) EtherNetIPAdapter behaviour ---
# Default config: input_length = output_length = 100.
# Valid real-time headers are 0 (heartbeat), 2 (modeless), 6 (header32bit).


def test_generic_accepts_valid_header_sizes() -> None:
    adapter = _build_adapter()
    # 106 -> O->T header 6; 102 -> T->O header 2.
    pkt = _build_forward_open(o_t_size=106, t_o_size=102)
    assert _status_of_forward_open(adapter, pkt) == 0x00
    assert adapter.peer_connected is True


def test_generic_rejects_wrong_o_t_connection_point() -> None:
    adapter = _build_adapter()
    pkt = _build_forward_open(o_t_size=106, t_o_size=102, path=_BAD_O2T_PATH)
    assert _status_of_forward_open(adapter, pkt) == 0x05  # Path destination unknown
    assert adapter.peer_connected is False


def test_generic_rejects_wrong_t_o_connection_point() -> None:
    adapter = _build_adapter()
    bad_t2o = bytes.fromhex("34000000000000000000" "20 04" "24 01" "2C 64" "2C 7B".replace(" ", ""))
    pkt = _build_forward_open(o_t_size=106, t_o_size=102, path=bad_t2o)
    assert _status_of_forward_open(adapter, pkt) == 0x05
    assert adapter.peer_connected is False


def test_generic_rejects_invalid_o_t_header() -> None:
    adapter = _build_adapter()
    # 105 -> O->T header 5 (not in {0,2,6}).
    pkt = _build_forward_open(o_t_size=105, t_o_size=102)
    assert _status_of_forward_open_ext(adapter, pkt) == (0x01, 0x0127)


def test_generic_rejects_invalid_t_o_header() -> None:
    adapter = _build_adapter()
    # 105 -> T->O header 5 (not in {0,2,6}).
    pkt = _build_forward_open(o_t_size=106, t_o_size=105)
    assert _status_of_forward_open(adapter, pkt) == 0x01
    assert _status_of_forward_open_ext(adapter, pkt) == (0x01, 0x0128)


def test_generic_rejects_oversized_o_t() -> None:
    adapter = _build_adapter()
    # 120 -> O->T header 20 (too big).
    pkt = _build_forward_open(o_t_size=120, t_o_size=102)
    assert _status_of_forward_open_ext(adapter, pkt) == (0x01, 0x0127)


def test_generic_rejects_undersized_o_t() -> None:
    # Spec-strict: a header smaller than 0 is also invalid -> rejected.
    adapter = _build_adapter()
    pkt = _build_forward_open(o_t_size=80, t_o_size=102)  # header -20
    assert _status_of_forward_open_ext(adapter, pkt) == (0x01, 0x0127)


def test_generic_rejects_duplicate_forward_open() -> None:
    adapter = _build_adapter()
    client = _RecordingClient()
    pkt = _build_forward_open(o_t_size=106, t_o_size=102, serial=1)
    adapter._handle_forward_open(client, pkt)
    assert _reply_status(client.sent[0]) == 0x00
    # Same identity opened again while still connected -> duplicate.
    dup = _RecordingClient()
    adapter._handle_forward_open(dup, pkt)
    assert _status_of_forward_open_ext(adapter, pkt) == (0x01, 0x0100)


def _status_of_forward_open_ext(adapter: EtherNetIPAdapter, pkt: bytes) -> tuple[int, int]:
    client = _RecordingClient()
    adapter._handle_forward_open(client, pkt)
    reply = client.sent[0]
    status = _reply_status(reply)
    ext = int.from_bytes(reply[44:46], "little") if _reply_ext_size(reply) >= 1 else 0
    return (status, ext)
