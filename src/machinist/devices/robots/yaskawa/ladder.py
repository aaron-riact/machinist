"""The cell's concurrent I/O ladder, in the controller's own instruction list
(CIOPRG.LST): the rungs that pass signals on.

A rung starts with ``STR``, ``STR-NOT`` or ``GSTR`` and runs through its outputs.
Its contacts work out one result, on or off:

* ``STR #x`` starts it from signal x (``STR-NOT``: its inverse). A further ``STR``
  starts a new branch and keeps the result so far for ``OR-STR`` / ``AND-STR``.
* ``AND #x`` / ``OR #x`` (and their ``-NOT`` forms) combine a signal into it.
* ``OR-STR`` / ``AND-STR`` combine the branch with the one kept before it.

Its outputs:

* ``OUT #y``: signal y takes the result. A rung may have several.
* ``GSTR #a`` ... ``GOUT #b``: byte b takes byte a.
* ``BMOV #a,n,#b``: while the result is on, the n bytes from a go to those from b.
* ``MOV M020,#30010``: while the result is on, the register's 16 bits go to two
  bytes, low byte first.

Timers and counters (DX200 Concurrent I/O manual, E1102000155GB05, 11.2.12-13) keep
their current value in a register and pass their contact on as the result, for the
``OUT`` after them:

* ``TMR M010,20``: an on-delay timer on a 0.1 s clock. While the result before it is
  on, the current value counts down from the set value (20: 2.0 s); the contact is on
  at 0. Off, the value is back at the set value and the contact off.
* ``CNT M011,5``: a counter with two inputs, the counter input (the first ``STR``) and
  the reset input (the second). Each time the counter input turns on, the value counts
  down by one, unless reset is on, which puts it back at the set value; the contact is
  on at 0.

Both start from their set value, as a controller's do at power-on. The set value is a
number or a register (``TMR M010,M020``).

A rung given alone, as a scene's ``ladder`` option does, may start with ``MOV``: with
no contact before it, it always runs.

Signal numbers are the controller's: a byte by its first signal (a multiple of
ten), a signal by its byte plus its bit (0-7). The ladder runs after every write
that can change what it reads; a real one scans all the time.
"""

from __future__ import annotations

import re
from collections.abc import Iterable, MutableMapping
from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class Contact:
    """STR, AND or OR on a signal, inverted for the -NOT forms."""

    kind: str
    signal: int
    inverted: bool = False


@dataclass(frozen=True, slots=True)
class Combine:
    """OR-STR or AND-STR."""

    kind: str


@dataclass(frozen=True, slots=True)
class Out:
    signal: int


@dataclass(frozen=True, slots=True)
class LoadByte:
    """GSTR."""

    byte: int


@dataclass(frozen=True, slots=True)
class StoreByte:
    """GOUT."""

    byte: int


@dataclass(frozen=True, slots=True)
class BlockMove:
    source: int
    count: int
    target: int


@dataclass(frozen=True, slots=True)
class RegisterMove:
    register: int
    target: int


@dataclass(frozen=True, slots=True)
class Timer:
    """TMR: its current value's register, and its set value (a number or a register)."""

    register: int
    preset: int
    preset_is_register: bool = False


@dataclass(frozen=True, slots=True)
class Counter:
    """CNT: as Timer."""

    register: int
    preset: int
    preset_is_register: bool = False


Instruction = (Contact | Combine | Out | LoadByte | StoreByte | BlockMove | RegisterMove
               | Timer | Counter)
#: A timer's clock, in seconds.
TIMER_TICK = 0.1
#: The registers a timer or counter may keep its current value in.
CURRENT_VALUE_REGISTERS = range(560)
#: What ends a rung's contacts: the next STR or GSTR starts a new rung.
OUTPUTS = (Out, StoreByte, BlockMove, RegisterMove)

_INSTRUCTION = re.compile(
    r"(?P<combine>OR-STR|AND-STR)"
    r"|(?P<contact>STR-NOT|STR|AND-NOT|AND|OR-NOT|OR) #(?P<contact_signal>\d{5})"
    r"|OUT #(?P<out>\d{5})"
    r"|(?P<group>GSTR|GOUT) #(?P<group_byte>\d{5})"
    r"|BMOV #(?P<source>\d{5}), ?(?P<count>\d+), ?#(?P<target>\d{5})"
    r"|MOV M(?P<register>\d{1,3})(?:, ?| )#(?P<register_target>\d{5})"
    r"|(?P<clock>TMR|CNT) M(?P<current>\d{1,3}), ?(?P<preset>M?\d+)"
)


def parse_instructions(text: str) -> list[Instruction]:
    """The instructions in *text*, one after another; anything else is refused."""
    words = " ".join(text.split())
    instructions: list[Instruction] = []
    position = 0
    while position < len(words):
        match = _INSTRUCTION.match(words, position)
        if match is None:
            raise ValueError(f"ladder instruction {words[position:]!r} is not one the "
                             f"emulator runs")
        instructions.append(_instruction(match))
        position = match.end() + 1
    return instructions


def _instruction(match: re.Match[str]) -> Instruction:
    if match["combine"]:
        return Combine(match["combine"])
    if match["contact"]:
        kind = match["contact"].removesuffix("-NOT")
        return Contact(kind, _bit(match["contact_signal"]),
                       inverted=match["contact"].endswith("-NOT"))
    if match["out"]:
        return Out(_bit(match["out"]))
    return _data_instruction(match)


def _data_instruction(match: re.Match[str]) -> Instruction:
    """GSTR, GOUT, BMOV, MOV, TMR, CNT: the instructions on bytes and registers."""
    if match["group"]:
        byte = _byte(match["group_byte"])
        return LoadByte(byte) if match["group"] == "GSTR" else StoreByte(byte)
    if match["count"]:
        return BlockMove(_byte(match["source"]), int(match["count"]), _byte(match["target"]))
    if match["clock"]:
        current = int(match["current"])
        if current not in CURRENT_VALUE_REGISTERS:
            raise ValueError(f"{match['clock']} M{current}: a current value is kept in "
                             f"M000 to M559")
        preset = match["preset"]
        kind = Timer if match["clock"] == "TMR" else Counter
        return kind(current, int(preset.removeprefix("M")), preset.startswith("M"))
    return RegisterMove(int(match["register"]), _byte(match["register_target"]))


Rung = tuple[Instruction, ...]


def parse_rung(text: str) -> Rung:
    """One rung, as a scene's ladder option gives it: outputs after its contacts."""
    rung = tuple(parse_instructions(text))
    if not rung or not isinstance(rung[-1], OUTPUTS):
        raise ValueError(f"rung {text!r} ends without an output")
    return rung


def split_rungs(instructions: Iterable[Instruction]) -> list[Rung]:
    """An instruction list as rungs: each STR or GSTR after an output starts one."""
    rungs: list[list[Instruction]] = []
    for instruction in instructions:
        starts = isinstance(instruction, LoadByte) or (
            isinstance(instruction, Contact) and instruction.kind == "STR")
        if not rungs or (starts and isinstance(rungs[-1][-1], OUTPUTS)):
            rungs.append([])
        rungs[-1].append(instruction)
    return [tuple(rung) for rung in rungs]


def _byte(digits: str) -> int:
    number = int(digits)
    if number % 10:
        raise ValueError(f"#{digits} is not the first signal of a byte")
    return number


def _bit(digits: str) -> int:
    number = int(digits)
    if number % 10 > 7:
        raise ValueError(f"#{digits} is no signal: a byte holds bits 0 to 7")
    return number


@dataclass(slots=True)
class _Scan:
    """One rung's working state: its result, the branches kept, its byte."""

    io: MutableMapping[int, int]
    result: bool = True
    kept: list[bool] = field(default_factory=list)
    byte: int = 0
    started: bool = False

    def contact(self, contact: Contact) -> None:
        on = signal(self.io, contact.signal) != contact.inverted
        match contact.kind:
            case "STR":
                if self.started:
                    self.kept.append(self.result)
                self.result, self.started = on, True
            case "AND":
                self.result = self.result and on
            case "OR":
                self.result = self.result or on

    def combine(self, kind: str) -> None:
        if not self.kept:
            raise ValueError(f"{kind} without a branch before it")
        before = self.kept.pop()
        self.result = (before or self.result) if kind == "OR-STR" else (before and self.result)


@dataclass(slots=True)
class LadderMemory:
    """What the ladder keeps between scans: when each running timer started, and
    each counter's input at the last scan. Its timers and counters by (rung,
    instruction); one not yet scanned starts from its set value."""

    timers_since: dict[tuple[int, int], float | None] = field(default_factory=dict)
    counter_inputs: dict[tuple[int, int], bool] = field(default_factory=dict)


@dataclass(frozen=True, slots=True)
class Ladder:
    rungs: tuple[Rung, ...] = ()

    @classmethod
    def parse(cls, rungs: Iterable[str]) -> Ladder:
        """A scene's rungs, each given on its own."""
        return cls(tuple(parse_rung(rung) for rung in rungs))

    def __add__(self, other: Ladder) -> Ladder:
        return Ladder(self.rungs + other.rungs)

    def scan(self, io: MutableMapping[int, int], registers: MutableMapping[int, float],
             memory: LadderMemory | None = None, now: float = 0.0) -> None:
        """Run every rung once, in order, on the controller's bytes and registers;
        its timers and counters keep what they need in *memory*, at time *now*
        (seconds)."""
        memory = LadderMemory() if memory is None else memory
        for rung_index, rung in enumerate(self.rungs):
            state = _Scan(io)
            for index, instruction in enumerate(rung):
                match instruction:
                    case Timer():
                        _time(instruction, state, registers, memory, (rung_index, index), now)
                    case Counter():
                        _count(instruction, state, registers, memory, (rung_index, index))
                    case _:
                        _run(instruction, state, io, registers)


def _preset(clock: Timer | Counter, registers: MutableMapping[int, float]) -> int:
    if clock.preset_is_register:
        return int(registers.get(clock.preset, 0))
    return clock.preset


def _time(timer: Timer, state: _Scan, registers: MutableMapping[int, float],
          memory: LadderMemory, key: tuple[int, int], now: float) -> None:
    preset = _preset(timer, registers)
    since = memory.timers_since.get(key)
    if not state.result:
        memory.timers_since[key] = None
        current = preset
    else:
        if since is None:
            memory.timers_since[key] = since = now
        current = max(preset - int((now - since) / TIMER_TICK + 1e-9), 0)
    registers[timer.register] = current
    state.result = state.result and current == 0


def _count(counter: Counter, state: _Scan, registers: MutableMapping[int, float],
           memory: LadderMemory, key: tuple[int, int]) -> None:
    reset = state.result
    if not state.kept:
        raise ValueError("CNT without its counter input before its reset input")
    counting = state.kept.pop()
    preset = _preset(counter, registers)
    first_scan = key not in memory.counter_inputs
    current = preset if first_scan or reset else int(registers.get(counter.register, preset))
    if not reset and counting and not memory.counter_inputs.get(key, True) and current > 0:
        current -= 1
    memory.counter_inputs[key] = counting
    registers[counter.register] = current
    state.result = not reset and current == 0


def _run(instruction: Instruction, state: _Scan, io: MutableMapping[int, int],
         registers: MutableMapping[int, float]) -> None:
    match instruction:
        case Contact():
            state.contact(instruction)
        case Combine(kind):
            state.combine(kind)
        case Out(target):
            set_signal(io, target, state.result)
        case LoadByte(byte):
            state.byte = io.get(byte, 0)
        case StoreByte(byte):
            io[byte] = state.byte
        case BlockMove(source, count, target) if state.result:
            values = [io.get(source + 10 * i, 0) for i in range(count)]
            for i, value in enumerate(values):
                io[target + 10 * i] = value
        case RegisterMove(register, target) if state.result:
            value = int(registers.get(register, 0)) & 0xFFFF
            io[target], io[target + 10] = value & 0xFF, value >> 8


def signal(io: MutableMapping[int, int], number: int) -> bool:
    return bool(io.get(number - number % 10, 0) >> (number % 10) & 1)


def set_signal(io: MutableMapping[int, int], number: int, on: bool) -> None:
    byte, bit = number - number % 10, number % 10
    value = io.get(byte, 0)
    io[byte] = value | 1 << bit if on else value & ~(1 << bit)


#: The registers the //$M section of a CIOPRG.LST sets, ten to a line: M920 to M999,
#: the system registers the ladder editor's header sets (DX200 ladder editor manual,
#: HW1481795, 1.3.6.1).
FIRST_HEADER_REGISTER = 920


@dataclass(frozen=True, slots=True)
class LadderFile:
    """A saved CIOPRG.LST: the registers its header sets, and its rungs."""

    registers: dict[int, int]
    ladder: Ladder


def parse_ladder_file(text: str) -> LadderFile:
    """The //$M registers, then the instructions from //KEY to END, in every PART. What
    follows END (the alarm and message texts) is not the ladder's."""
    registers: dict[int, int] = {}
    instructions: list[Instruction] = []
    section = ""
    for raw in text.replace("\r", "").split("\n"):
        line = raw.strip()
        if line.startswith("//"):
            section = line[2:].split(" ")[0]
            continue
        if not line or line.startswith("/"):
            continue
        if section == "$M":
            for value in line.split(","):
                registers[FIRST_HEADER_REGISTER + len(registers)] = int(value)
        elif section == "KEY":
            if line == "END":
                break
            if not line.startswith("PART "):
                instructions.extend(parse_instructions(line))
    return LadderFile(registers, Ladder(tuple(split_rungs(instructions))))
