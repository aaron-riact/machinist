from __future__ import annotations

from machinist.transport.ethernetip import (
    EtherNetIPAdapter,
    EtherNetIPAdapterConfig,
    _send_rrdata_reply,
)


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
