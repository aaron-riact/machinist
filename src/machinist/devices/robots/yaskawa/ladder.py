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
* ``MOV S,D``: while the result is on, S goes to D (11.2.24). Each is a register
  (``M020``), a byte (``#30010``: 8 bits), a word (``W#30010``: two bytes, the low one
  first, an order the manual does not give) or, as S only, a number. A value over 255
  moved to a byte keeps its low 8 bits and sets the carry flag ``#51400``, which is
  cleared at the start of every scan (11.3).

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
from collections.abc import Callable, Iterable, MutableMapping
from dataclasses import dataclass, field
from enum import StrEnum


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


class Place(StrEnum):
    """Where MOV takes or puts a value."""

    REGISTER = "M"
    BYTE = "#"
    WORD = "W#"
    NUMBER = ""


@dataclass(frozen=True, slots=True)
class Operand:
    place: Place
    number: int


@dataclass(frozen=True, slots=True)
class Move:
    """MOV."""

    source: Operand
    target: Operand


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


Instruction = (Contact | Combine | Out | LoadByte | StoreByte | BlockMove | Move
               | Timer | Counter)
#: A timer's clock, in seconds.
TIMER_TICK = 0.1
#: The registers a timer or counter may keep its current value in.
CURRENT_VALUE_REGISTERS = range(560)
#: The registers a ladder may write (11.1): general and analog output.
WRITABLE_REGISTERS = range(600)
#: The arithmetic flags (11.3.1): carry, zero, error, bits 0-2 of byte #51400.
FLAGS_BYTE = 51400
#: What ends a rung's contacts: the next STR or GSTR starts a new rung.
OUTPUTS = (Out, StoreByte, BlockMove, Move)

_INSTRUCTION = re.compile(
    r"(?P<combine>OR-STR|AND-STR)"
    r"|(?P<contact>STR-NOT|STR|AND-NOT|AND|OR-NOT|OR) #(?P<contact_signal>\d{5})"
    r"|OUT #(?P<out>\d{5})"
    r"|(?P<group>GSTR|GOUT) #(?P<group_byte>\d{5})"
    r"|BMOV #(?P<source>\d{5}), ?(?P<count>\d+), ?#(?P<target>\d{5})"
    r"|MOV (?P<move_source>M\d{1,3}|W?#\d{5}|\d+)(?:, ?| )(?P<move_target>M\d{1,3}|W?#\d{5})"
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
    source, target = _operand(match["move_source"]), _operand(match["move_target"])
    if target.place is Place.REGISTER and target.number not in WRITABLE_REGISTERS:
        raise ValueError(f"MOV to M{target.number}: only M000 to M599 can be written")
    return Move(source, target)


def _operand(text: str) -> Operand:
    if text.startswith("M"):
        return Operand(Place.REGISTER, int(text[1:]))
    if text.startswith("W#"):
        return Operand(Place.WORD, _byte(text[2:]))
    if text.startswith("#"):
        return Operand(Place.BYTE, _byte(text[1:]))
    return Operand(Place.NUMBER, int(text))


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
class LadderMemory:
    """What the ladder keeps between scans: when each running timer started, and
    each counter's input at the last scan. Its timers and counters by (rung,
    instruction); one not yet scanned starts from its set value."""

    timers_since: dict[tuple[int, int], float | None] = field(default_factory=dict)
    counter_inputs: dict[tuple[int, int], bool] = field(default_factory=dict)


#: A ladder as one function: on the controller's bytes and registers, with its
#: timers' and counters' memory, at a time in seconds.
ScanFunction = Callable[
    [MutableMapping[int, int], MutableMapping[int, float], LadderMemory, float], None]


@dataclass(frozen=True, slots=True)
class Ladder:
    """Rungs, and the function that runs them all once: generated as Python source
    when the ladder is made, each signal number written into it, as a PLC compiles
    its program. That runs a scan about five times faster than going through the
    instructions one by one, which matters at a scan every 4 ms."""

    rungs: tuple[Rung, ...] = ()
    _scan: ScanFunction = field(init=False, repr=False, compare=False)

    def __post_init__(self) -> None:
        object.__setattr__(self, "_scan", compile_ladder(self.rungs))

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
        self._scan(io, registers, LadderMemory() if memory is None else memory, now)


def compile_ladder(rungs: tuple[Rung, ...]) -> ScanFunction:
    """The rungs as one Python function. A rung that cannot run (OR-STR or CNT with no
    branch kept before it) is refused here, before any scan."""
    # The arithmetic flags are cleared before every scan (11.3.2).
    lines = ["def scan(io, regs, memory, now):", "    get = io.get", "    g = 0",
             f"    io[{FLAGS_BYTE}] = get({FLAGS_BYTE}, 0) & 0xF8"]
    clocks: list[Timer | Counter] = []
    for rung_index, rung in enumerate(rungs):
        lines.extend(_rung_source(rung, rung_index, clocks))
    namespace: dict[str, object] = {"_time": _time, "_count": _count, "CLOCKS": tuple(clocks)}
    exec(compile("\n".join(lines), "<ladder>", "exec"), namespace)
    scan = namespace["scan"]
    assert callable(scan)
    return scan


def _rung_source(rung: Rung, rung_index: int, clocks: list[Timer | Counter]) -> list[str]:
    """One rung as lines of the scan function: its result in r, its kept branches in
    k0, k1 ..., its byte in g."""
    lines = ["    r = 1"]
    kept: list[str] = []
    started = False

    def pop(why: str) -> str:
        if not kept:
            raise ValueError(why)
        return kept.pop()

    for index, instruction in enumerate(rung):
        match instruction:
            case Contact(kind, number, inverted):
                value = f"(get({number - number % 10}, 0) >> {number % 10} & 1)"
                value = f"({value} ^ 1)" if inverted else value
                if kind == "STR":
                    if started:
                        kept.append(f"k{len(kept)}")
                        lines.append(f"    {kept[-1]} = r")
                    lines.append(f"    r = {value}")
                    started = True
                else:
                    lines.append(f"    r = r {'&' if kind == 'AND' else '|'} {value}")
            case Combine(kind):
                before = pop(f"{kind} without a branch before it")
                lines.append(f"    r = {before} {'|' if kind == 'OR-STR' else '&'} r")
            case Out() | LoadByte() | StoreByte() | BlockMove() | Move():
                lines += _output_source(instruction)
            case Timer():
                clocks.append(instruction)
                lines.append(f"    r = _time(CLOCKS[{len(clocks) - 1}], r, regs, memory, "
                             f"({rung_index}, {index}), now)")
            case Counter():
                counting = pop("CNT without its counter input before its reset input")
                clocks.append(instruction)
                lines.append(f"    r = _count(CLOCKS[{len(clocks) - 1}], {counting}, r, regs, "
                             f"memory, ({rung_index}, {index}))")
    return lines


def _output_source(instruction: Out | LoadByte | StoreByte | BlockMove | Move,
                   ) -> list[str]:
    """What a rung's result, or its byte, does to the controller's bytes."""
    match instruction:
        case Out(number):
            byte, mask = number - number % 10, 1 << number % 10
            return [f"    io[{byte}] = get({byte}, 0) | {mask} if r "
                    f"else get({byte}, 0) & {0xFF ^ mask}"]
        case LoadByte(byte):
            return [f"    g = get({byte}, 0)"]
        case StoreByte(byte):
            return [f"    io[{byte}] = g"]
        case BlockMove(source, count, target):
            return ["    if r:",
                    f"        values = [get({source} + 10 * i, 0) for i in range({count})]",
                    "        for i, value in enumerate(values):",
                    f"            io[{target} + 10 * i] = value"]
        case Move(source, target):
            return ["    if r:", f"        v = {_read(source)}", *_write(target)]
    raise AssertionError(instruction)


def _read(operand: Operand) -> str:
    """A MOV source as an expression for its 16-bit value."""
    match operand.place:
        case Place.REGISTER:
            return f"int(regs.get({operand.number}, 0)) & 0xFFFF"
        case Place.BYTE:
            return f"get({operand.number}, 0)"
        case Place.WORD:
            return f"get({operand.number}, 0) | get({operand.number + 10}, 0) << 8"
    return str(operand.number & 0xFFFF)


def _write(operand: Operand) -> list[str]:
    """Lines that put v in a MOV destination; a byte keeps the low 8 bits, and the carry
    flag says whether more were lost."""
    flags = FLAGS_BYTE
    match operand.place:
        case Place.REGISTER:
            return [f"        regs[{operand.number}] = v"]
        case Place.BYTE:
            return [f"        io[{operand.number}] = v & 0xFF",
                    f"        io[{flags}] = get({flags}, 0) & 0xFE | (v > 0xFF)"]
    return [f"        io[{operand.number}] = v & 0xFF",
            f"        io[{operand.number + 10}] = v >> 8"]


def _preset(clock: Timer | Counter, registers: MutableMapping[int, float]) -> int:
    if clock.preset_is_register:
        return int(registers.get(clock.preset, 0))
    return clock.preset


def _time(timer: Timer, start: int, registers: MutableMapping[int, float],
          memory: LadderMemory, key: tuple[int, int], now: float) -> int:
    """TMR: the contact, 1 once the start input has been on for the set time."""
    preset = _preset(timer, registers)
    since = memory.timers_since.get(key)
    if not start:
        memory.timers_since[key] = None
        current = preset
    else:
        if since is None:
            memory.timers_since[key] = since = now
        current = max(preset - int((now - since) / TIMER_TICK + 1e-9), 0)
    registers[timer.register] = current
    return 1 if start and current == 0 else 0


def _count(counter: Counter, counting: int, reset: int,
           registers: MutableMapping[int, float], memory: LadderMemory,
           key: tuple[int, int]) -> int:
    """CNT: the contact, 1 once the counter input has turned on the set number of
    times since the reset input was last on."""
    preset = _preset(counter, registers)
    first_scan = key not in memory.counter_inputs
    current = preset if first_scan or reset else int(registers.get(counter.register, preset))
    if not reset and counting and not memory.counter_inputs.get(key, True) and current > 0:
        current -= 1
    memory.counter_inputs[key] = bool(counting)
    registers[counter.register] = current
    return 1 if not reset and current == 0 else 0


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
