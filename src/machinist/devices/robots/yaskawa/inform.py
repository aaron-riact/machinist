"""Running a job: the part of INFORM a cell's I/O jobs use, so that starting one
switches what it switches.

A job is parsed into typed instructions when it starts. What the emulator
cannot run (a move, ``EXPRESS``, ``IFTHEN`` ...) is kept as :class:`Unsupported`
and stops the job only when reached, so a job that jumps past it still runs.

Supported: comments, labels, ``NOP``/``END``/``RET``; ``DOUT OT#(..) ON|OFF``,
``DOUT OG#(..) value``, ``PULSE OT#(..) [T=..]``; ``SET`` on a B, I, D or R variable,
from a value or ``EXPRESS`` (see :mod:`.expression`), ``ADD``, ``SUB``, ``MUL``, ``DIV``,
``INC``, ``DEC`` and ``SQRT``; ``JUMP *LABEL``, ``JUMP JOB:X`` and ``CALL JOB:X``,
each with an optional ``IF`` on ``IN#(..)``/``OT#(..)`` or a variable, or several
joined by ``ANDIF`` or ``ORIF``; ``IFTHEN``/``IFTHENEXP`` blocks with ``ELSEIF``,
``ELSE`` and ``ENDIF``; P variables: ``SET P P``, ``GETE``, ``SETE``, ``ADD`` and ``SUB``
of two, ``MULMAT`` (see :mod:`.positions`); through the host: ``GETS`` of $PX000-$PX003
and $B002, ``CNVRT``, ``MFRAME``, ``GETTOOL``, ``SETTOOL``, ``TCPON``/``TCPOF``; moves:
``MOVJ`` and ``MOVL`` to a P variable or the job's own C position, ``IMOV``, each
with ``SRCH RIN#(..)`` or ``UNTIL IN#(..)``;
``DIN`` of a signal or group, ``SETREG``; ``MSG``; ``CLEAR STACK``; ``PAUSE``, which
stops the job;
``TIMER T=..``, in seconds, or in hundredths of a second from a variable;
``WAIT IN#(..)=ON|OFF [T=..]``. A signal or value is a number or a variable
(``OT#(D001)``, ``T=I020``).

Time passes through the host: ``TIMER`` waits its time, a ``PULSE`` switches its
output off after its time (0.3 s unless given), and a ``WAIT`` waits until its
signal is as asked, or until its ``T=`` time has passed, then goes on. The host may
stop the job at any of these, and between any two instructions.
"""

from __future__ import annotations

import math
import operator
import re
from collections.abc import Callable, MutableMapping
from contextlib import AbstractContextManager
from dataclasses import dataclass, field, replace
from typing import Protocol

from .expression import (
    OPERATIONS,
    VALUE,
    VARIABLE,
    Expression,
    Literal,
    Operand,
    Operation,
    Variable,
    evaluate,
    fit,
    operand,
    parse_expression,
    variable,
)
from .positions import (
    AXES,
    UNSET,
    DataType,
    Position,
    axes_of,
    parse_job_positions,
    transform_of,
)

#: How many instructions a job may run before the emulator calls it a loop.
MAX_STEPS = 100_000
#: How deep CALL JOB may nest.
MAX_CALL_DEPTH = 32
#: How long a PULSE without T= keeps its output on, in seconds.
PULSE_SECONDS = 0.3

_VARIABLE = VARIABLE
_VALUE = VALUE
_REF = rf"\(\s*({_VARIABLE}|\d+)\s*\)"
_IF = r"(?:\s+IF\s+(.+))?"
_OPERATORS: dict[str, Callable[[float, float], bool]] = {
    "=": operator.eq, "<>": operator.ne, "<=": operator.le, ">=": operator.ge,
    "<": operator.lt, ">": operator.gt,
}


class JobInterruptedError(Exception):
    """The host stopped the job (a hold, servo off) or cannot do what it asks: why."""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


class JobStoppedError(Exception):
    """The job stopped before its end: why, and at which line of which job."""

    def __init__(self, job: str, line: int, reason: str) -> None:
        super().__init__(f"{job} line {line}: {reason}")
        self.job, self.line, self.reason = job, line, reason


# --- Operands and conditions -------------------------------------------------


_operand = operand
_variable = variable


@dataclass(frozen=True, slots=True)
class SignalTest:
    """``IN#(..)=ON``: a general input or output, on or off."""

    output: bool
    ref: Operand
    on: bool


@dataclass(frozen=True, slots=True)
class Comparison:
    variable: Variable
    op: str
    value: Operand


@dataclass(frozen=True, slots=True)
class AllOf:
    """Tests joined by ANDIF (AND in IFTHENEXP): all must hold."""

    tests: tuple[SignalTest | Comparison, ...]


@dataclass(frozen=True, slots=True)
class AnyOf:
    """Tests joined by ORIF (OR in IFTHENEXP): one must hold."""

    tests: tuple[SignalTest | Comparison, ...]


Condition = SignalTest | Comparison | AllOf | AnyOf | None


def _condition(text: str | None) -> Condition:
    if text is None:
        return None
    text = " ".join(text.split())
    ands = re.split(r" (?:ANDIF|AND) ", text)
    ors = re.split(r" (?:ORIF|OR) ", text)
    if len(ands) > 1 and len(ors) > 1:
        raise ValueError(f"condition {text!r} mixes AND and OR")
    if len(ands) > 1:
        return AllOf(tuple(_test(part) for part in ands))
    if len(ors) > 1:
        return AnyOf(tuple(_test(part) for part in ors))
    return _test(text)


def _test(text: str) -> SignalTest | Comparison:
    text = text.strip()
    if text.startswith("(") and text.endswith(")"):
        text = text[1:-1].strip()
    if match := re.fullmatch(rf"(IN|OT)#{_REF}\s*=\s*(ON|OFF)", text):
        return SignalTest(match[1] == "OT", _operand(match[2]), match[3] == "ON")
    if match := re.fullmatch(rf"({_VARIABLE})\s*(<>|<=|>=|=|<|>)\s*({_VALUE})", text):
        return Comparison(_variable(match[1]), match[2], _operand(match[3]))
    raise ValueError(f"condition {text!r}")


# --- Instructions ------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Label:
    name: str


@dataclass(frozen=True, slots=True)
class End:
    """END, and RET: the job, or the called job, ends here."""


@dataclass(frozen=True, slots=True)
class SetOutput:
    ref: Operand
    on: bool


@dataclass(frozen=True, slots=True)
class SetOutputGroup:
    group: Operand
    value: Operand


@dataclass(frozen=True, slots=True)
class Pulse:
    ref: Operand
    seconds: Operand = field(default_factory=lambda: Literal(PULSE_SECONDS))


@dataclass(frozen=True, slots=True)
class Timer:
    seconds: Operand


@dataclass(frozen=True, slots=True)
class SetVariable:
    variable: Variable
    value: Operand


@dataclass(frozen=True, slots=True)
class Assign:
    """``SET D020 EXPRESS ...``: a variable takes the value of an expression."""

    variable: Variable
    expression: Expression


@dataclass(frozen=True, slots=True)
class Arithmetic:
    """``ADD``, ``SUB``, ``MUL``, ``DIV``, and ``INC``/``DEC`` (by 1): a variable
    takes its value combined with another."""

    symbol: str
    variable: Variable
    value: Operand


@dataclass(frozen=True, slots=True)
class SquareRoot:
    variable: Variable
    value: Operand


@dataclass(frozen=True, slots=True)
class CopyPosition:
    """``SET P010 P011``."""

    target: int
    source: int


@dataclass(frozen=True, slots=True)
class GetElement:
    """``GETE D010 P010 (1)``: a variable takes one axis of a position."""

    variable: Variable
    position: int
    element: int


@dataclass(frozen=True, slots=True)
class SetElement:
    """``SETE P010 (1) D020``: one axis of a position takes a value."""

    position: int
    element: int
    value: Operand


@dataclass(frozen=True, slots=True)
class CombinePositions:
    """``ADD P010 P012`` and ``SUB``: each axis of the first takes the sum or
    difference of both; the first keeps its data type, tool and frame."""

    symbol: str
    target: int
    other: int


@dataclass(frozen=True, slots=True)
class GetSystemPosition:
    """``GETS PX010 $PX003``: $PX000 the arm's pulses, $PX001 its tool point in base
    coordinates, $PX002 and $PX003 the same where the last search stopped."""

    target: int
    number: int


@dataclass(frozen=True, slots=True)
class GetSystemByte:
    """``GETS B000 $B002``: $B002 is 1 when the last search found what it looked for."""

    variable: Variable
    number: int


@dataclass(frozen=True, slots=True)
class Convert:
    """``CNVRT PX011 PX010 UF#(5) TL#(1)``: the target is the source's tool point in
    another frame (base, robot or a user frame), with another tool if one is named."""

    target: int
    source: int
    data_type: DataType
    user: Operand
    tool: Operand | None


@dataclass(frozen=True, slots=True)
class MakeFrame:
    """``MFRAME UF#(5) PX011 PX010 PX012``: a user frame from its origin, a point on
    its X axis and one on its XY plane."""

    user: Operand
    origin: int
    on_x: int
    on_xy: int


@dataclass(frozen=True, slots=True)
class MultiplyPositions:
    """``MULMAT P013 P010 P012``: the first is the second's transform times the third's."""

    target: int
    left: int
    right: int


@dataclass(frozen=True, slots=True)
class GetTool:
    target: int
    tool: Operand


@dataclass(frozen=True, slots=True)
class SetTool:
    tool: Operand
    source: int


@dataclass(frozen=True, slots=True)
class SelectTool:
    """``TCPON TL#(1)`` selects a tool for what follows; ``TCPOF`` goes back to each
    position's own."""

    tool: Operand | None


@dataclass(frozen=True, slots=True)
class PositionRef:
    """A position a move goes to: a P variable, or one of the job's own (C)."""

    own: bool
    number: int


@dataclass(frozen=True, slots=True)
class Watch:
    """What a move watches: ``SRCH RIN#(1)=ON T=0.20 DIS=2.0`` (a direct input, and the
    search is recorded) or ``UNTIL IN#(D030)=ON`` (a general input). T= and DIS= are
    how long and how far the move runs before it starts watching."""

    search: bool
    ref: Operand
    on: bool
    after_seconds: Operand = field(default_factory=lambda: Literal(0))
    after_mm: Operand = field(default_factory=lambda: Literal(0))


@dataclass(frozen=True, slots=True)
class WatchedInput:
    """A move's watch with its numbers worked out: what the host watches."""

    search: bool
    number: int
    on: bool
    after_seconds: float
    after_mm: float


@dataclass(frozen=True, slots=True)
class Move:
    """``MOVJ P0001 VJ=30.00`` (a joint move, at a percentage of the axes' top speed)
    or ``MOVL P010 V=200.0`` (a straight one, in mm/s). The job goes on when the arm
    is there. PL (how closely to pass through) is not emulated: every move stops."""

    linear: bool
    target: PositionRef
    speed: Operand
    watch: Watch | None = None


@dataclass(frozen=True, slots=True)
class IncrementalMove:
    """``IMOV P011 V=20.0 UF#(5)``: a straight move by the P variable's X, Y, Z (and
    turns), along the axes of the base, robot, tool or a user frame."""

    increment: int
    speed: Operand
    data_type: DataType
    user: Operand
    watch: Watch | None = None


@dataclass(frozen=True, slots=True)
class IfThen:
    """IFTHEN (and IFTHENEXP): its block runs when the condition holds; otherwise the
    job goes to *following*, the index of its next ELSEIF, ELSE or ENDIF."""

    condition: Condition
    following: int = -1


@dataclass(frozen=True, slots=True)
class ElseIf:
    """Reached from a branch not taken, it tests its condition as IFTHEN does;
    reached by running into it, the block is done and the job goes to *end*."""

    condition: Condition
    following: int = -1
    end: int = -1


@dataclass(frozen=True, slots=True)
class Else:
    end: int = -1


@dataclass(frozen=True, slots=True)
class EndIf:
    pass


@dataclass(frozen=True, slots=True)
class Jump:
    label: str
    condition: Condition


@dataclass(frozen=True, slots=True)
class JumpJob:
    job: str
    condition: Condition


@dataclass(frozen=True, slots=True)
class CallJob:
    job: str
    condition: Condition


@dataclass(frozen=True, slots=True)
class Wait:
    test: SignalTest
    #: How long to wait at most before going on; None for as long as it takes.
    seconds: Operand | None = None


@dataclass(frozen=True, slots=True)
class ReadSignal:
    """``DIN B000 OT#(5)``: a variable takes a signal (IN, OT, SIN, SOUT: 0 or 1) or
    a group of eight (IG, OG: their byte)."""

    variable: Variable
    kind: str
    ref: Operand


@dataclass(frozen=True, slots=True)
class SetRegister:
    """``SETREG MREG#(20) 100``: a ladder register takes a value."""

    ref: Operand
    value: Operand


@dataclass(frozen=True, slots=True)
class Message:
    """``MSG "DOOR NOT OPEN"``: the pendant shows the text."""

    text: str


@dataclass(frozen=True, slots=True)
class ClearStack:
    """``CLEAR STACK``: the jobs that called this one do not go on when it ends."""


@dataclass(frozen=True, slots=True)
class Pause:
    """``PAUSE``: a real controller holds the job until START. The emulator cannot
    go on from there, so the job stops."""


@dataclass(frozen=True, slots=True)
class Unsupported:
    text: str


Instruction = (Label | End | SetOutput | SetOutputGroup | Pulse | Timer | SetVariable | Assign
               | Arithmetic | SquareRoot | CopyPosition | GetElement | SetElement
               | CombinePositions | GetSystemPosition | GetSystemByte | Convert | MakeFrame
               | MultiplyPositions | GetTool | SetTool | SelectTool | Move | IncrementalMove
               | IfThen | ElseIf | Else | EndIf | Jump | JumpJob | CallJob | Wait | ReadSignal
               | SetRegister | Message | ClearStack | Pause | Unsupported)

_ARITHMETIC = {"ADD": "+", "SUB": "-", "MUL": "*", "DIV": "/"}
#: A P variable, as ``P010`` or, where a position instruction names it, ``PX010``.
_P = r"PX?(\d+)"


_FRAMES = {"BF": DataType.BASE, "RF": DataType.ROBOT}
_MOVE_FRAMES = {**_FRAMES, "TF": DataType.TOOL}
#: A move's target: a P variable or the job's own C position.
_TARGET = r"((?:P|C)\d+)"
#: A move's positioning level, which the emulator takes and ignores.
_PL = r"(?: PL=\d+)?"


#: What a move may watch, after its speed and PL: five groups, see :func:`_watch`.
_WATCH = (rf"(?: (SRCH RIN|UNTIL IN)#{_REF}=(ON|OFF)"
          rf"(?: T=({_VALUE}))?(?: DIS=({_VALUE}))?)?")


def _watch(match: re.Match[str], first: int) -> Watch | None:
    kind, ref, state, seconds, mm = (match[first + i] for i in range(5))
    if kind is None:
        return None
    return Watch(kind == "SRCH RIN", _operand(ref), state == "ON",
                 Literal(0) if seconds is None else _operand(seconds),
                 Literal(0) if mm is None else _operand(mm))


def _target(text: str) -> PositionRef:
    return PositionRef(text[0] == "C", int(text[1:]))


def _system(text: str, highest: int, lowest: int = 0) -> int:
    """A system variable's number; ValueError for one the emulator does not keep."""
    number = int(text)
    if not lowest <= number <= highest:
        raise ValueError(f"system variable {number}")
    return number


def _element(text: str) -> int:
    element = int(text)
    if not 1 <= element <= AXES:
        raise ValueError(f"element {element}")
    return element

#: Each instruction the emulator runs: its pattern, and what a match makes of it.
_PATTERNS: tuple[tuple[re.Pattern[str], Callable[[re.Match[str]], Instruction]], ...] = tuple(
    (re.compile(pattern), make) for pattern, make in (
        (r"\*(\S+)", lambda m: Label(m[1])),
        (r"END|RET", lambda m: End()),
        (rf"DOUT OT#{_REF} (ON|OFF)", lambda m: SetOutput(_operand(m[1]), m[2] == "ON")),
        (rf"DOUT OG#{_REF} ({_VALUE})",
         lambda m: SetOutputGroup(_operand(m[1]), _operand(m[2]))),
        (rf"PULSE OT#{_REF}", lambda m: Pulse(_operand(m[1]))),
        (rf"PULSE OT#{_REF} T=({_VALUE})", lambda m: Pulse(_operand(m[1]), _operand(m[2]))),
        (rf"TIMER T=({_VALUE})", lambda m: Timer(_operand(m[1]))),
        (rf"SET ({_VARIABLE}) ({_VALUE})", lambda m: SetVariable(_variable(m[1]), _operand(m[2]))),
        (rf"SET ({_VARIABLE}) EXPRESS (.+)",
         lambda m: Assign(_variable(m[1]), parse_expression(m[2]))),
        (rf"(ADD|SUB|MUL|DIV) ({_VARIABLE}) ({_VALUE})",
         lambda m: Arithmetic(_ARITHMETIC[m[1]], _variable(m[2]), _operand(m[3]))),
        (rf"(INC|DEC) ({_VARIABLE})",
         lambda m: Arithmetic("+" if m[1] == "INC" else "-", _variable(m[2]), Literal(1))),
        (rf"SQRT ({_VARIABLE}) ({_VALUE})",
         lambda m: SquareRoot(_variable(m[1]), _operand(m[2]))),
        (rf"SET {_P} {_P}", lambda m: CopyPosition(int(m[1]), int(m[2]))),
        (rf"GETE ({_VARIABLE}) {_P} \((\d)\)",
         lambda m: GetElement(_variable(m[1]), int(m[2]), _element(m[3]))),
        (rf"SETE {_P} \((\d)\) ({_VALUE})",
         lambda m: SetElement(int(m[1]), _element(m[2]), _operand(m[3]))),
        (rf"(ADD|SUB) {_P} {_P}",
         lambda m: CombinePositions(_ARITHMETIC[m[1]], int(m[2]), int(m[3]))),
        (rf"GETS {_P} \$PX(\d{{3}})", lambda m: GetSystemPosition(int(m[1]), _system(m[2], 3))),
        (rf"GETS ({_VARIABLE}) \$B(\d{{3}})",
         lambda m: GetSystemByte(_variable(m[1]), _system(m[2], 2, 2))),
        (rf"CNVRT {_P} {_P} (BF|RF|UF#{_REF})(?: TL#{_REF})?",
         lambda m: Convert(int(m[1]), int(m[2]), _FRAMES.get(m[3], DataType.USER),
                           Literal(0) if m[4] is None else _operand(m[4]),
                           None if m[5] is None else _operand(m[5]))),
        (rf"MFRAME UF#{_REF} {_P} {_P} {_P}",
         lambda m: MakeFrame(_operand(m[1]), int(m[2]), int(m[3]), int(m[4]))),
        (rf"MULMAT {_P} {_P} {_P}", lambda m: MultiplyPositions(int(m[1]), int(m[2]), int(m[3]))),
        (rf"GETTOOL {_P} TL#{_REF}", lambda m: GetTool(int(m[1]), _operand(m[2]))),
        (rf"SETTOOL TL#{_REF} {_P}", lambda m: SetTool(_operand(m[1]), int(m[2]))),
        (rf"TCPON TL#{_REF}", lambda m: SelectTool(_operand(m[1]))),
        (r"TCPOF", lambda m: SelectTool(None)),
        (rf"MOVJ {_TARGET} VJ=({_VALUE}){_PL}{_WATCH}",
         lambda m: Move(False, _target(m[1]), _operand(m[2]), _watch(m, 3))),
        (rf"MOVL {_TARGET} V=({_VALUE}){_PL}{_WATCH}",
         lambda m: Move(True, _target(m[1]), _operand(m[2]), _watch(m, 3))),
        (rf"IMOV {_P} V=({_VALUE}){_PL} (BF|RF|TF|UF#{_REF}){_WATCH}",
         lambda m: IncrementalMove(int(m[1]), _operand(m[2]), _MOVE_FRAMES.get(m[3], DataType.USER),
                                   Literal(0) if m[4] is None else _operand(m[4]), _watch(m, 5))),
        (rf"DIN ({_VARIABLE}) (IN|OT|SIN|SOUT|IG|OG)#{_REF}",
         lambda m: ReadSignal(_variable(m[1]), m[2], _operand(m[3]))),
        (rf"SETREG MREG#{_REF} ({_VALUE})", lambda m: SetRegister(_operand(m[1]), _operand(m[2]))),
        (r'MSG "(.*)"', lambda m: Message(m[1])),
        (r"PAUSE", lambda m: Pause()),
        (r"CLEAR STACK", lambda m: ClearStack()),
        (r"IFTHEN(?:EXP)? (.+)", lambda m: IfThen(_condition(m[1]))),
        (r"ELSEIF(?:EXP)? (.+)", lambda m: ElseIf(_condition(m[1]))),
        (r"ELSE", lambda m: Else()),
        (r"ENDIF", lambda m: EndIf()),
        (rf"JUMP \*(\S+){_IF}", lambda m: Jump(m[1], _condition(m[2]))),
        (rf"JUMP JOB:(\S+){_IF}", lambda m: JumpJob(m[1], _condition(m[2]))),
        (rf"CALL JOB:(\S+){_IF}", lambda m: CallJob(m[1], _condition(m[2]))),
        (rf"WAIT (IN|OT)#{_REF}=(ON|OFF)(?: T=({_VALUE}))?",
         lambda m: Wait(SignalTest(m[1] == "OT", _operand(m[2]), m[3] == "ON"),
                        None if m[4] is None else _operand(m[4]))),
    )
)


_NOTHING_TO_DO = re.compile(r"ADVSTOP|ADVINIT|SHCKRST|SHCKSET SSL#\(\d+\)")


def parse_instruction(text: str) -> Instruction | None:
    """One line of a job; None for one that does nothing (a comment, NOP, a line
    commented out with COMM), or nothing the emulator has: look-ahead (ADVSTOP,
    ADVINIT) and the collision sensor's sensitivity (SHCKSET, SHCKRST)."""
    line = " ".join(text.split())
    if not line or line.startswith("'") or line == "NOP" or line.startswith("COMM "):
        return None
    if _NOTHING_TO_DO.fullmatch(line):
        return None
    for pattern, make in _PATTERNS:
        if match := pattern.fullmatch(line):
            try:
                return make(match)
            except ValueError:
                break
    return Unsupported(line)


@dataclass(frozen=True, slots=True)
class Job:
    name: str
    #: Each instruction with its line number in the job (NOP is line 0).
    lines: tuple[tuple[int, Instruction], ...]
    #: The job's own positions (C00000 ...), from its //POS section.
    own_positions: dict[int, Position] = field(default_factory=dict)

    @classmethod
    def parse(cls, name: str, text: str) -> Job:
        body = text.replace("\r", "").split("\n")
        if "NOP" not in body:
            return cls(name, ())
        lines = []
        for number, raw in enumerate(body[body.index("NOP"):]):
            instruction = parse_instruction(raw)
            if instruction is not None:
                lines.append((number, instruction))
            if raw.strip() == "END":
                break
        return cls(name, _resolve_blocks(lines), parse_job_positions(text).own)

    def label(self, name: str) -> int | None:
        """The index of label *name* in lines, None when the job has none."""
        return next((i for i, (_, ins) in enumerate(self.lines) if ins == Label(name)), None)


def _resolve_blocks(
    lines: list[tuple[int, Instruction]],
) -> tuple[tuple[int, Instruction], ...]:
    """Each IFTHEN, ELSEIF and ELSE with where it goes; a clause without its IFTHEN
    or ENDIF stops the job when reached."""
    resolved = list(lines)
    #: For each open block: the index of each of its clauses so far.
    open_blocks: list[list[int]] = []
    for index, (number, instruction) in enumerate(resolved):
        match instruction:
            case IfThen():
                open_blocks.append([index])
            case ElseIf() | Else() if open_blocks:
                open_blocks[-1].append(index)
            case EndIf() if open_blocks:
                clauses = open_blocks.pop()
                for here, following in zip(clauses, [*clauses[1:], index], strict=True):
                    line, clause = resolved[here]
                    match clause:
                        case IfThen():
                            clause = replace(clause, following=following)
                        case ElseIf():
                            clause = replace(clause, following=following, end=index)
                        case Else():
                            clause = replace(clause, end=index)
                    resolved[here] = (line, clause)
            case ElseIf() | Else() | EndIf():
                resolved[index] = (number, Unsupported("a block clause without its IFTHEN"))
    for clauses in open_blocks:
        number, _ = resolved[clauses[0]]
        resolved[clauses[0]] = (number, Unsupported("IFTHEN without its ENDIF"))
    return tuple(resolved)


# --- Running ---------------------------------------------------------------


class JobHost(Protocol):
    """What a job acts on: the controller's jobs, bytes and variables."""

    @property
    def io(self) -> MutableMapping[int, int]: ...

    @property
    def variables(self) -> MutableMapping[str, dict[int, float]]: ...

    @property
    def io_lock(self) -> AbstractContextManager[object]:
        """Held while the job reads and writes the IO bytes together."""

    @property
    def positions(self) -> MutableMapping[str, dict[int, tuple[int, ...]]]:
        """Position variables by kind ("P"), then number, as HSES records."""

    def job_text(self, name: str) -> str | None:
        """Job *name*'s INFORM, None when the controller holds no such job."""

    def run_ladder(self) -> None: ...

    def checkpoint(self) -> None:
        """Go on; :class:`JobInterruptedError` when the job has been stopped."""

    # Each of these raises JobInterruptedError for what the host cannot do (a tool
    # or user frame it does not know, a frame the points do not span).

    def system_position(self, number: int) -> Position:
        """$PX000 to $PX003."""

    def system_byte(self, number: int) -> int:
        """$B002."""

    def convert(self, position: Position, *, data_type: DataType, user: int,
                tool: int) -> Position:
        """*position*'s tool point as tool *tool*'s, in the frame of *data_type*."""

    def make_user_frame(self, number: int, points: tuple[Position, Position, Position]) -> None:
        """Set user frame *number* from its origin, a point on X and one on XY."""

    def tool_position(self, number: int) -> Position:
        """Tool *number*'s offset from the flange, as a Cartesian position."""

    def set_tool_position(self, number: int, position: Position) -> None:
        """Set tool *number*'s offset from the flange to *position*'s first six axes."""

    def select_tool(self, number: int | None) -> None:
        """The tool what follows uses; None for each position's own."""

    def show_message(self, text: str) -> None:
        """Show *text* on the pendant."""

    def move_to(self, position: Position, *, linear: bool, speed: float,
                watch: WatchedInput | None) -> None:
        """Move the tool point to *position*, and return once it is there: a straight
        move at *speed* mm/s, or a joint move at *speed* percent. With *watch*, the
        move stops early where the input is as asked."""

    def move_by(self, increment: Position, *, data_type: DataType, user: int,
                speed: float, watch: WatchedInput | None) -> None:
        """Move the tool point straight by *increment*'s axes, along the axes of the
        frame of *data_type* (and *user*), and return once it is there (or, with
        *watch*, stopped on its input)."""

    def sleep(self, seconds: float) -> None:
        """Let *seconds* pass; :class:`JobInterruptedError` when the job is stopped
        meanwhile."""

    def wait_until(self, test: Callable[[], bool], seconds: float | None) -> None:
        """Wait until *test* holds, or *seconds* have passed (None: no limit);
        :class:`JobInterruptedError` when the job is stopped meanwhile."""

    def after(self, seconds: float, action: Callable[[], None]) -> None:
        """Do *action* once *seconds* have passed, while the job goes on."""


#: The first signal of the first byte of each kind of signal a job reads.
_FIRST_BYTES = {"IN": 10, "OT": 10010, "SIN": 40010, "SOUT": 50010}


def signal_byte(kind: str, number: int) -> int:
    """The first signal of the byte that holds signal *number* of *kind* (IN, OT, SIN,
    SOUT)."""
    return _FIRST_BYTES[kind] + (number - 1) // 8 * 10


def output_byte(number: int) -> int:
    """The first signal of the byte that holds general output OT#number."""
    return 10010 + (number - 1) // 8 * 10


def input_byte(number: int) -> int:
    """The first signal of the byte that holds general input IN#number."""
    return 10 + (number - 1) // 8 * 10


def run_job(host: JobHost, name: str) -> None:
    """Run job *name* to its end; :class:`JobStoppedError` when it cannot."""
    _Run(host).call(name, depth=0)


#: What a step does next: go on, go to an index in the job's lines, or end the job.
_END = -1


@dataclass(slots=True)
class _Run:
    host: JobHost
    steps: int = 0
    #: After CLEAR STACK at this depth, no job called from below it goes on.
    cleared_at: int | None = None

    def call(self, name: str, *, depth: int) -> None:
        if depth > MAX_CALL_DEPTH:
            raise JobStoppedError(name, 0, f"CALL JOB nests deeper than {MAX_CALL_DEPTH}")
        text = self.host.job_text(name)
        if text is None:
            raise JobStoppedError(name, 0, "no such job")
        job = Job.parse(name, text)
        index = 0
        while 0 <= index < len(job.lines):
            self.steps += 1
            number, instruction = job.lines[index]
            if self.steps > MAX_STEPS:
                raise JobStoppedError(name, number, f"more than {MAX_STEPS} steps")
            try:
                self.host.checkpoint()
                following = self._step(job, number, instruction, depth)
            except JobInterruptedError as interrupted:
                raise JobStoppedError(name, number, interrupted.reason) from None
            except ArithmeticError as error:
                raise JobStoppedError(name, number, str(error)) from None
            index = index + 1 if following is None else following

    def _step(self, job: Job, number: int, instruction: Instruction, depth: int) -> int | None:
        """Run one instruction; where to go next, None for the next line."""
        match instruction:
            case End():
                return _END
            case Move(linear, goal, speed, watch):
                position = (job.own_positions.get(goal.number, UNSET) if goal.own
                            else self._position(goal.number))
                self.host.move_to(position, linear=linear, speed=self._value(speed),
                                  watch=self._watched(watch))
            case IfThen() | ElseIf() | Else():
                return self._block(job, instruction)
            case Jump(label, condition) if self._holds(condition):
                target = job.label(label)
                if target is None:
                    raise JobStoppedError(job.name, number, f"no label *{label}")
                return target
            case JumpJob() | CallJob() | ClearStack():
                return self._between_jobs(instruction, depth)
            case Wait(test, seconds):
                self.host.wait_until(lambda: self._holds(test),
                                     None if seconds is None else self._value(seconds))
            case Unsupported(text):
                raise JobStoppedError(job.name, number, f"{text!r} is not emulated")
            case Pause():
                raise JobStoppedError(job.name, number, "PAUSE")
            case _:
                self._act(instruction)
        return None

    def _block(self, job: Job, instruction: IfThen | ElseIf | Else) -> int | None:
        match instruction:
            case IfThen(condition, following):
                return None if self._holds(condition) else self._branch(job, following)
            case ElseIf(end=end) | Else(end=end):
                return end

    def _between_jobs(self, instruction: JumpJob | CallJob | ClearStack, depth: int) -> int | None:
        match instruction:
            case JumpJob(target_job, condition) if self._holds(condition):
                self.call(target_job, depth=depth)
                return _END
            case CallJob(target_job, condition) if self._holds(condition):
                self.call(target_job, depth=depth + 1)
                if self.cleared_at is not None and depth < self.cleared_at:
                    return _END
            case ClearStack():
                self.cleared_at = depth
        return None

    def _act(self, instruction: Instruction) -> None:
        """What an instruction that does not change the flow does to the controller."""
        match instruction:
            case (SetOutput() | Pulse() | SetOutputGroup() | Timer() | Message() | ReadSignal()
                  | SetRegister()):
                self._act_on_signals(instruction)
            case SetVariable(target, value):
                self._store(target, self._value(value))
            case Assign(target, expression):
                self._store(target, evaluate(expression, self._value))
            case Arithmetic(symbol, target, value):
                self._store(target, evaluate(Operation(symbol, target, value), self._value))
            case CopyPosition() | GetElement() | SetElement() | CombinePositions():
                self._act_on_position(instruction)
            case (GetSystemPosition() | GetSystemByte() | Convert() | MakeFrame()
                  | MultiplyPositions() | GetTool() | SetTool() | SelectTool()):
                self._act_in_space(instruction)
            case IncrementalMove(increment, speed, data_type, user, watch):
                self.host.move_by(self._position(increment), data_type=data_type,
                                  user=self._int(user), speed=self._value(speed),
                                  watch=self._watched(watch))
            case SquareRoot(target, value):
                radicand = self._value(value)
                if radicand < 0:
                    raise ArithmeticError(f"square root of {radicand:g}")
                self._store(target, math.sqrt(radicand))

    def _act_on_signals(
        self, instruction: SetOutput | Pulse | SetOutputGroup | Timer | Message | ReadSignal
        | SetRegister,
    ) -> None:
        match instruction:
            case ReadSignal(target, kind, ref):
                number = self._int(ref)
                if kind in ("IG", "OG"):
                    first = signal_byte("IN" if kind == "IG" else "OT", number * 8)
                    self._store(target, self.host.io.get(first, 0))
                else:
                    byte = self.host.io.get(signal_byte(kind, number), 0)
                    self._store(target, byte >> (number - 1) % 8 & 1)
            case SetRegister(ref, value):
                self.host.variables.setdefault("register", {})[self._int(ref)] = self._value(value)
                self.host.run_ladder()
            case SetOutput(ref, on):
                self._set_output(self._int(ref), on)
            case Pulse(ref, seconds):
                number = self._int(ref)
                self._set_output(number, True)
                self.host.after(self._value(seconds), lambda: self._set_output(number, False))
            case Timer(seconds):
                self.host.sleep(self._timer_seconds(seconds))
            case SetOutputGroup(group, value):
                with self.host.io_lock:
                    self.host.io[output_byte(self._int(group) * 8)] = self._int(value) & 0xFF
                    self.host.run_ladder()
            case Message(text):
                self.host.show_message(text)

    def _act_on_position(
        self, instruction: CopyPosition | GetElement | SetElement | CombinePositions,
    ) -> None:
        match instruction:
            case CopyPosition(target, source):
                self._set_position(target, self._position(source))
            case GetElement(target, position, element):
                self._store(target, self._position(position).axes[element - 1])
            case SetElement(position, element, value):
                self._set_position(position, self._position(position).with_axis(
                    element, round(self._value(value))))
            case CombinePositions(symbol, target, other):
                mine, theirs = self._position(target), self._position(other)
                combined = tuple(round(OPERATIONS[symbol](a, b))
                                 for a, b in zip(mine.axes, theirs.axes, strict=True))
                self._set_position(target, replace(mine, axes=combined))

    def _act_in_space(
        self, instruction: GetSystemPosition | GetSystemByte | Convert | MakeFrame
        | MultiplyPositions | GetTool | SetTool | SelectTool,
    ) -> None:
        host = self.host
        match instruction:
            case GetSystemPosition(target, number):
                self._set_position(target, host.system_position(number))
            case GetSystemByte(target, number):
                self._store(target, host.system_byte(number))
            case Convert(target, source, data_type, user, tool):
                position = self._position(source)
                self._set_position(target, host.convert(
                    position, data_type=data_type, user=self._int(user),
                    tool=position.tool if tool is None else self._int(tool)))
            case MakeFrame(user, origin, on_x, on_xy):
                host.make_user_frame(self._int(user), (
                    self._position(origin), self._position(on_x), self._position(on_xy)))
            case MultiplyPositions(target, left, right):
                first, second = self._position(left), self._position(right)
                if not (first.cartesian and second.cartesian):
                    raise ArithmeticError("MULMAT needs two Cartesian positions")
                product = transform_of(first.axes) @ transform_of(second.axes)
                self._set_position(target, replace(first, axes=axes_of(product)))
            case GetTool(target, tool):
                self._set_position(target, host.tool_position(self._int(tool)))
            case SetTool(tool, source):
                host.set_tool_position(self._int(tool), self._position(source))
            case SelectTool(tool):
                host.select_tool(None if tool is None else self._int(tool))

    def _watched(self, watch: Watch | None) -> WatchedInput | None:
        if watch is None:
            return None
        return WatchedInput(watch.search, self._int(watch.ref), watch.on,
                            self._value(watch.after_seconds), self._value(watch.after_mm))

    def _position(self, number: int) -> Position:
        record = self.host.positions.get("P", {}).get(number)
        return UNSET if record is None else Position.from_record(record)

    def _set_position(self, number: int, position: Position) -> None:
        self.host.positions.setdefault("P", {})[number] = position.record()

    def _store(self, target: Variable, value: float) -> None:
        self.host.variables.setdefault(target.kind, {})[target.number] = fit(target.kind, value)

    def _value(self, operand: Operand) -> float:
        if isinstance(operand, Literal):
            return operand.value
        return self.host.variables.get(operand.kind, {}).get(operand.number, 0)

    def _timer_seconds(self, operand: Operand) -> float:
        """TIMER's time: a number is in seconds, a variable in hundredths of a second
        (the INFORM manuals of the NX100, DX100, DX200 and YRC1000)."""
        if isinstance(operand, Literal):
            return operand.value
        return self._value(operand) / 100

    def _int(self, operand: Operand) -> int:
        return int(self._value(operand))

    def _branch(self, job: Job, clause: int) -> int:
        """Where a block goes on from its clause at index *clause*, after the branches
        before it were not taken."""
        while True:
            match job.lines[clause][1]:
                case ElseIf(condition, following) if not self._holds(condition):
                    clause = following
                case _:
                    return clause + 1

    def _holds(self, condition: Condition) -> bool:
        match condition:
            case AllOf(tests):
                return all(self._holds(test) for test in tests)
            case AnyOf(tests):
                return any(self._holds(test) for test in tests)
            case SignalTest(output, ref, on):
                number = self._int(ref)
                byte = output_byte(number) if output else input_byte(number)
                return bool(self.host.io.get(byte, 0) >> ((number - 1) % 8) & 1) == on
            case Comparison(variable, op, value):
                return _OPERATORS[op](self._value(variable), self._value(value))
        return True

    def _set_output(self, number: int, on: bool) -> None:
        byte, bit = output_byte(number), (number - 1) % 8
        with self.host.io_lock:
            value = self.host.io.get(byte, 0)
            self.host.io[byte] = value | 1 << bit if on else value & ~(1 << bit)
            self.host.run_ladder()
