"""The cell's concurrent I/O ladder: the rungs that pass signals on, in the
controller's own instruction list (CIOPRG.LST), as far as a cell's wiring needs.

A rung is one line of instructions, parsed when the scene loads:

* ``GSTR #27010 GOUT #30040``: copy a byte (eight signals) to another.
* ``STR #27012 OUT #30134``: copy one signal to another.
* ``STR-NOT #27012 OUT #30134``: copy it inverted.
* ``MOV M560 #30140``: move a register's 16 bits to two bytes, low byte first,
  as ``MOV M020,#00030`` puts a register on a group of inputs.

Signal numbers are the controller's: a byte by its first signal (a multiple of
ten), a signal by its byte plus its bit (0-7). The ladder runs after every write
that can change what it reads; a real one scans all the time.
"""

from __future__ import annotations

import re
from collections.abc import Iterable, MutableMapping
from dataclasses import dataclass

_SIGNAL = r"#(\d{5})"
_BYTE_COPY = re.compile(rf"GSTR {_SIGNAL} GOUT {_SIGNAL}")
_BIT_COPY = re.compile(rf"(STR|STR-NOT) {_SIGNAL} OUT {_SIGNAL}")
_REGISTER_MOVE = re.compile(rf"MOV M(\d{{1,3}})(?:, ?| ){_SIGNAL}")


@dataclass(frozen=True, slots=True)
class ByteCopy:
    source: int
    target: int


@dataclass(frozen=True, slots=True)
class BitCopy:
    source: int
    target: int
    inverted: bool


@dataclass(frozen=True, slots=True)
class RegisterMove:
    register: int
    target: int


Rung = ByteCopy | BitCopy | RegisterMove


def parse_rung(text: str) -> Rung:
    """One rung from its instructions; anything else is refused."""
    words = " ".join(text.split())
    if match := _BYTE_COPY.fullmatch(words):
        source, target = (_byte(match[1]), _byte(match[2]))
        return ByteCopy(source, target)
    if match := _BIT_COPY.fullmatch(words):
        return BitCopy(_bit(match[2]), _bit(match[3]), inverted=match[1] == "STR-NOT")
    if match := _REGISTER_MOVE.fullmatch(words):
        return RegisterMove(int(match[1]), _byte(match[2]))
    raise ValueError(
        f"rung {text!r} is not GSTR #... GOUT #..., STR[-NOT] #... OUT #... or MOV M... #..."
    )


def _byte(digits: str) -> int:
    signal = int(digits)
    if signal % 10:
        raise ValueError(f"#{digits} is not the first signal of a byte")
    return signal


def _bit(digits: str) -> int:
    signal = int(digits)
    if signal % 10 > 7:
        raise ValueError(f"#{digits} is no signal: a byte holds bits 0 to 7")
    return signal


@dataclass(frozen=True, slots=True)
class Ladder:
    rungs: tuple[Rung, ...] = ()

    @classmethod
    def parse(cls, rungs: Iterable[str]) -> Ladder:
        return cls(tuple(parse_rung(rung) for rung in rungs))

    def scan(self, io: MutableMapping[int, int], registers: MutableMapping[int, float]) -> None:
        """Run every rung once, in order, on the controller's bytes and registers."""
        for rung in self.rungs:
            match rung:
                case ByteCopy(source, target):
                    io[target] = io.get(source, 0)
                case BitCopy(source, target, inverted):
                    on = bool(io.get(_byte_of(source), 0) >> (source % 10) & 1) != inverted
                    _set_bit(io, target, on)
                case RegisterMove(register, target):
                    value = int(registers.get(register, 0)) & 0xFFFF
                    io[target], io[target + 10] = value & 0xFF, value >> 8


def _byte_of(signal: int) -> int:
    return signal - signal % 10


def _set_bit(io: MutableMapping[int, int], signal: int, on: bool) -> None:
    byte, bit = _byte_of(signal), signal % 10
    value = io.get(byte, 0)
    io[byte] = value | 1 << bit if on else value & ~(1 << bit)
