"""SRCI — Standard Robot Command Interface.

SRCI is a vendor-neutral command interface between a controller (PLC)
and a robot. In the field it rides PROFINET as cyclic process data, but
nothing about the *command semantics* depends on that wire: a controller
issues a command telegram and the robot answers with a status telegram.

This package owns those telegrams and the role logic around them, and
stays deliberately transport-agnostic — it speaks
:mod:`machinist.transport.message`, so the very same codec drives the
emulator over TCP or UDP today and Modbus/PROFINET later.

Public surface:

* :class:`CommandTelegram` / :class:`StatusTelegram` — frozen, with
  ``encode`` / ``decode``.
* :class:`Function` / :class:`StatusFlag` — the command vocabulary.
* :class:`SrciServer` — drives a robot arm from command telegrams.
* :class:`SrciClient` — issues commands over a pluggable transport.
"""

from __future__ import annotations

from .client import SrciClient
from .codec import (
    CommandTelegram,
    Function,
    StatusFlag,
    StatusTelegram,
)
from .server import SrciServer

__all__ = [
    "CommandTelegram",
    "Function",
    "SrciClient",
    "SrciServer",
    "StatusFlag",
    "StatusTelegram",
]
