from __future__ import annotations

from machinist.transport.framing import CR


def test_cr_framer_ends_a_message_at_cr() -> None:
    buf = bytearray(b"50010,24\r")
    assert CR.decode(buf) == ["50010,24"]
    assert buf == bytearray()


def test_cr_framer_drops_the_lf_of_a_crlf() -> None:
    buf = bytearray(b"HOSTCTRL_REQUEST IOREAD 9\r\n50010,24\r")
    assert CR.decode(buf) == ["HOSTCTRL_REQUEST IOREAD 9", "50010,24"]


def test_cr_framer_keeps_a_partial_message() -> None:
    buf = bytearray(b"CONNECT Robot_ac")
    assert CR.decode(buf) == []
    assert buf == bytearray(b"CONNECT Robot_ac")


def test_cr_framer_sends_a_reply_as_given() -> None:
    assert CR.encode("162,0\r") == b"162,0\r"
