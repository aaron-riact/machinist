"""Message framing for line-oriented TCP protocols.

Industrial protocols vary in how they delimit messages on the wire:

* newline-terminated (UR, HAAS MDC)
* CRLF-terminated (Yaskawa NX100)
* open-paren / close-paren delimited (Dobot dashboard:
  ``Verb(args)`` — the close-paren is the terminator, responses end
  with ``;``)
* length-prefixed, binary, …

Rather than bake any of that into :class:`LineServer`, we use a small
``Framer`` protocol. Each framer owns *both* directions (decode bytes
→ messages, encode message → bytes). Concrete implementations are
trivial (a few lines each) and compose cleanly with the server.

Framers are **stateless** between messages — per-session state belongs
in :class:`SessionHandler` (see ``line_server``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol


class Framer(Protocol):
    """Byte ↔ message codec for a line-like TCP protocol."""

    def decode(self, buf: bytearray) -> list[str]:
        """Consume complete messages from ``buf`` in place."""
        ...

    def encode(self, message: str) -> bytes:
        """Encode a single outgoing message."""
        ...


@dataclass(frozen=True, slots=True)
class TerminatorFramer:
    """Incoming and outgoing messages each end with a fixed terminator.

    Typical industrial use: ``TerminatorFramer("\\r\\n")``.
    """

    rx_terminator: str = "\n"
    tx_terminator: str | None = None  # None ⇒ same as rx_terminator
    encoding: str = "ascii"

    def decode(self, buf: bytearray) -> list[str]:
        term = self.rx_terminator.encode(self.encoding)
        out: list[str] = []
        while term in buf:
            head, _, rest = buf.partition(term)
            buf[:] = rest
            out.append(head.decode(self.encoding, errors="replace"))
        return out

    def encode(self, message: str) -> bytes:
        term = (self.tx_terminator or self.rx_terminator)
        return f"{message}{term}".encode(self.encoding)


@dataclass(frozen=True, slots=True)
class ParenFramer:
    """Dobot-style framing: incoming ends at the first ``)``; reply
    terminates with ``;`` (no newline).

    This is the actual Dobot TCP/IP dashboard protocol per the V4.6.2
    interface guide — *not* newline-terminated as naïve reads suggest.
    """

    tx_terminator: str = ";"
    encoding: str = "ascii"

    def decode(self, buf: bytearray) -> list[str]:
        out: list[str] = []
        while (idx := buf.find(b")")) >= 0:
            head = bytes(buf[: idx + 1])
            del buf[: idx + 1]
            out.append(head.decode(self.encoding, errors="replace").strip())
        return out

    def encode(self, message: str) -> bytes:
        return f"{message}{self.tx_terminator}".encode(self.encoding)


# Convenience framer singletons for common cases.
NEWLINE = TerminatorFramer(rx_terminator="\n")
CRLF = TerminatorFramer(rx_terminator="\r\n")
PAREN = ParenFramer()
