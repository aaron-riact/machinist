"""Running a job: the part of INFORM a cell's I/O jobs use, so that starting one
switches what it switches.

A job is parsed into typed instructions when it starts. What the emulator
cannot run (a move, ``EXPRESS``, ``IFTHEN`` ...) is kept as :class:`Unsupported`
and stops the job only when reached, so a job that jumps past it still runs.

Supported: comments, labels, ``NOP``/``END``/``RET``; ``DOUT OT#(..) ON|OFF``,
``DOUT OG#(..) value``, ``PULSE OT#(..)``; ``SET Bnnn|Innn|Dnnn value``;
``JUMP *LABEL``, ``JUMP JOB:X`` and ``CALL JOB:X``, each with an optional
``IF`` on ``IN#(..)``/``OT#(..)`` or a variable; ``TIMER``; ``WAIT IN#(..)``.
A signal or value is a number or a variable (``OT#(D001)``, ``T=I020``).

The emulator does not wait: ``TIMER`` passes at once, a ``PULSE`` leaves its
output off, and a ``WAIT`` that is not met stops the job.
"""

from __future__ import annotations

import operator
import re
from collections.abc import Callable, MutableMapping
from dataclasses import dataclass
from typing import Protocol

#: How many instructions a job may run before the emulator calls it a loop.
MAX_STEPS = 100_000
#: How deep CALL JOB may nest.
MAX_CALL_DEPTH = 32

_VARIABLE = r"[BID]\d{3}"
_VALUE = rf"(?:{_VARIABLE}|-?\d+(?:\.\d+)?)"
_REF = rf"\(\s*({_VARIABLE}|\d+)\s*\)"
_IF = r"(?:\s+IF\s+(.+))?"
_OPERATORS: dict[str, Callable[[float, float], bool]] = {
    "=": operator.eq, "<>": operator.ne, "<=": operator.le, ">=": operator.ge,
    "<": operator.lt, ">": operator.gt,
}


class JobStoppedError(Exception):
    """The job stopped before its end: why, and at which line of which job."""

    def __init__(self, job: str, line: int, reason: str) -> None:
        super().__init__(f"{job} line {line}: {reason}")
        self.job, self.line, self.reason = job, line, reason


# --- Operands and conditions -------------------------------------------------


@dataclass(frozen=True, slots=True)
class Literal:
    value: float


@dataclass(frozen=True, slots=True)
class Variable:
    kind: str
    number: int


Operand = Literal | Variable


def _operand(text: str) -> Operand:
    if re.fullmatch(_VARIABLE, text):
        return _variable(text)
    return Literal(float(text))


def _variable(text: str) -> Variable:
    return Variable(text[0], int(text[1:]))


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


Condition = SignalTest | Comparison | None


def _condition(text: str | None) -> Condition:
    if text is None:
        return None
    text = text.strip()
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


@dataclass(frozen=True, slots=True)
class SetVariable:
    variable: Variable
    value: Operand


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


@dataclass(frozen=True, slots=True)
class Unsupported:
    text: str


Instruction = (Label | End | SetOutput | SetOutputGroup | Pulse | SetVariable | Jump
               | JumpJob | CallJob | Wait | Unsupported)

#: Each instruction the emulator runs: its pattern, and what a match makes of it.
_PATTERNS: tuple[tuple[re.Pattern[str], Callable[[re.Match[str]], Instruction]], ...] = tuple(
    (re.compile(pattern), make) for pattern, make in (
        (r"\*(\S+)", lambda m: Label(m[1])),
        (r"END|RET", lambda m: End()),
        (rf"DOUT OT#{_REF} (ON|OFF)", lambda m: SetOutput(_operand(m[1]), m[2] == "ON")),
        (rf"DOUT OG#{_REF} ({_VALUE})",
         lambda m: SetOutputGroup(_operand(m[1]), _operand(m[2]))),
        (rf"PULSE OT#{_REF}(?: T=\S+)?", lambda m: Pulse(_operand(m[1]))),
        (rf"SET ({_VARIABLE}) ({_VALUE})", lambda m: SetVariable(_variable(m[1]), _operand(m[2]))),
        (rf"JUMP \*(\S+){_IF}", lambda m: Jump(m[1], _condition(m[2]))),
        (rf"JUMP JOB:(\S+){_IF}", lambda m: JumpJob(m[1], _condition(m[2]))),
        (rf"CALL JOB:(\S+){_IF}", lambda m: CallJob(m[1], _condition(m[2]))),
        (rf"WAIT (IN|OT)#{_REF}=(ON|OFF)(?: T=\S+)?",
         lambda m: Wait(SignalTest(m[1] == "OT", _operand(m[2]), m[3] == "ON"))),
    )
)


def parse_instruction(text: str) -> Instruction | None:
    """One line of a job; None for one that does nothing (a comment, NOP, TIMER)."""
    line = " ".join(text.split())
    if not line or line.startswith("'") or line == "NOP" or line.startswith("TIMER "):
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
        return cls(name, tuple(lines))

    def label(self, name: str) -> int | None:
        """The index of label *name* in lines, None when the job has none."""
        return next((i for i, (_, ins) in enumerate(self.lines) if ins == Label(name)), None)


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
            following = self._step(job, number, instruction, depth)
            index = index + 1 if following is None else following

    def _step(self, job: Job, number: int, instruction: Instruction, depth: int) -> int | None:
        """Run one instruction; where to go next, None for the next line."""
        match instruction:
            case End():
                return _END
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
            case Wait(test) if not self._holds(test):
                raise JobStoppedError(job.name, number, "WAIT not met; the emulator does not wait")
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
            case Pulse(ref):
                self._set_output(self._int(ref), False)
            case SetOutputGroup(group, value):
                self.host.io[output_byte(self._int(group) * 8)] = self._int(value) & 0xFF
                self.host.run_ladder()
            case SetVariable(variable, value):
                store = self.host.variables.setdefault(variable.kind, {})
                store[variable.number] = self._value(value)

    def _value(self, operand: Operand) -> float:
        if isinstance(operand, Literal):
            return operand.value
        return self.host.variables.get(operand.kind, {}).get(operand.number, 0)

    def _int(self, operand: Operand) -> int:
        return int(self._value(operand))

    def _holds(self, condition: Condition) -> bool:
        match condition:
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
