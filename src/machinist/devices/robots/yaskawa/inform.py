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
``ELSE`` and ``ENDIF``; ``TIMER T=..``;
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
from dataclasses import dataclass, field, replace
from typing import Protocol

from .expression import (
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
    """The host stopped the job (a hold, servo off): why."""

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
class Unsupported:
    text: str


Instruction = (Label | End | SetOutput | SetOutputGroup | Pulse | Timer | SetVariable | Assign
               | Arithmetic | SquareRoot | IfThen | ElseIf | Else | EndIf | Jump | JumpJob
               | CallJob | Wait | Unsupported)

_ARITHMETIC = {"ADD": "+", "SUB": "-", "MUL": "*", "DIV": "/"}

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


def parse_instruction(text: str) -> Instruction | None:
    """One line of a job; None for one that does nothing (a comment, NOP)."""
    line = " ".join(text.split())
    if not line or line.startswith("'") or line == "NOP":
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
        return cls(name, _resolve_blocks(lines))

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

    def job_text(self, name: str) -> str | None:
        """Job *name*'s INFORM, None when the controller holds no such job."""

    def run_ladder(self) -> None: ...

    def checkpoint(self) -> None:
        """Go on; :class:`JobInterruptedError` when the job has been stopped."""

    def sleep(self, seconds: float) -> None:
        """Let *seconds* pass; :class:`JobInterruptedError` when the job is stopped
        meanwhile."""

    def wait_until(self, test: Callable[[], bool], seconds: float | None) -> None:
        """Wait until *test* holds, or *seconds* have passed (None: no limit);
        :class:`JobInterruptedError` when the job is stopped meanwhile."""

    def after(self, seconds: float, action: Callable[[], None]) -> None:
        """Do *action* once *seconds* have passed, while the job goes on."""


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
            case IfThen(condition, following):
                return None if self._holds(condition) else self._branch(job, following)
            case ElseIf(end=end) | Else(end=end):
                return end
            case Jump(label, condition) if self._holds(condition):
                target = job.label(label)
                if target is None:
                    raise JobStoppedError(job.name, number, f"no label *{label}")
                return target
            case JumpJob(target_job, condition) if self._holds(condition):
                self.call(target_job, depth=depth)
                return _END
            case CallJob(target_job, condition) if self._holds(condition):
                self.call(target_job, depth=depth + 1)
            case Wait(test, seconds):
                self.host.wait_until(lambda: self._holds(test),
                                     None if seconds is None else self._value(seconds))
            case Unsupported(text):
                raise JobStoppedError(job.name, number, f"{text!r} is not emulated")
            case _:
                self._act(instruction)
        return None

    def _act(self, instruction: Instruction) -> None:
        """What an instruction that does not change the flow does to the controller."""
        match instruction:
            case SetOutput(ref, on):
                self._set_output(self._int(ref), on)
            case Pulse(ref, seconds):
                number = self._int(ref)
                self._set_output(number, True)
                self.host.after(self._value(seconds), lambda: self._set_output(number, False))
            case Timer(seconds):
                self.host.sleep(self._value(seconds))
            case SetOutputGroup(group, value):
                self.host.io[output_byte(self._int(group) * 8)] = self._int(value) & 0xFF
                self.host.run_ladder()
            case SetVariable(target, value):
                self._store(target, self._value(value))
            case Assign(target, expression):
                self._store(target, evaluate(expression, self._value))
            case Arithmetic(symbol, target, value):
                self._store(target, evaluate(Operation(symbol, target, value), self._value))
            case SquareRoot(target, value):
                radicand = self._value(value)
                if radicand < 0:
                    raise ArithmeticError(f"square root of {radicand:g}")
                self._store(target, math.sqrt(radicand))

    def _store(self, target: Variable, value: float) -> None:
        self.host.variables.setdefault(target.kind, {})[target.number] = fit(target.kind, value)

    def _value(self, operand: Operand) -> float:
        if isinstance(operand, Literal):
            return operand.value
        return self.host.variables.get(operand.kind, {}).get(operand.number, 0)

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
        value = self.host.io.get(byte, 0)
        self.host.io[byte] = value | 1 << bit if on else value & ~(1 << bit)
        self.host.run_ladder()
