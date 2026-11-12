"""INFORM arithmetic: the expression of ``SET D020 EXPRESS -I001 * 100 + D010``, and
how a value fits the variable it is stored in.

An expression is parsed when its job starts, into a tree of numbers, variables,
negations and the four operations, with the usual precedence and brackets.

Every value is worked out as a real number. Stored in an integer variable (B, I, D),
it is rounded to the nearest whole number, halves away from zero, and must lie in
the variable's range; an R variable keeps it as it is. A real controller's rounding
and range checks have not been compared with these.
"""

from __future__ import annotations

import math
import operator
import re
from collections.abc import Callable
from dataclasses import dataclass

#: A B, I, D or R variable, as ``D010``.
VARIABLE = r"[BIDR]\d{3}"
#: A number or a variable.
VALUE = rf"(?:{VARIABLE}|-?\d+(?:\.\d+)?)"

#: The values each integer variable kind holds.
RANGES: dict[str, tuple[int, int]] = {
    "B": (0, 255),
    "I": (-(2**15), 2**15 - 1),
    "D": (-(2**31), 2**31 - 1),
}


@dataclass(frozen=True, slots=True)
class Literal:
    value: float


@dataclass(frozen=True, slots=True)
class Variable:
    kind: str
    number: int

    def __str__(self) -> str:
        return f"{self.kind}{self.number:03d}"


Operand = Literal | Variable


def operand(text: str) -> Operand:
    """A number or a variable."""
    if re.fullmatch(VARIABLE, text):
        return variable(text)
    return Literal(float(text))


def variable(text: str) -> Variable:
    return Variable(text[0], int(text[1:]))


@dataclass(frozen=True, slots=True)
class Negate:
    inner: Expression


@dataclass(frozen=True, slots=True)
class Operation:
    symbol: str
    left: Expression
    right: Expression


Expression = Literal | Variable | Negate | Operation

OPERATIONS: dict[str, Callable[[float, float], float]] = {
    "+": operator.add, "-": operator.sub, "*": operator.mul, "/": operator.truediv,
}

_TOKEN = re.compile(rf"\s*(\d+(?:\.\d+)?|{VARIABLE}|[-+*/()])")


def parse_expression(text: str) -> Expression:
    """An EXPRESS expression; ValueError for one that is not."""
    tokens: list[str] = []
    position = 0
    text = text.rstrip()
    while position < len(text):
        match = _TOKEN.match(text, position)
        if match is None:
            raise ValueError(f"expression {text!r}: cannot read {text[position:]!r}")
        tokens.append(match[1])
        position = match.end()
    parser = _Parser(tokens)
    expression = parser.sum()
    if parser.index != len(tokens):
        raise ValueError(f"expression {text!r}: {tokens[parser.index]!r} is left over")
    return expression


@dataclass(slots=True)
class _Parser:
    tokens: list[str]
    index: int = 0

    def _peek(self) -> str | None:
        return self.tokens[self.index] if self.index < len(self.tokens) else None

    def _take(self) -> str:
        token = self._peek()
        if token is None:
            raise ValueError("expression ends too soon")
        self.index += 1
        return token

    def sum(self) -> Expression:
        left = self.product()
        while self._peek() in ("+", "-"):
            symbol = self._take()
            left = Operation(symbol, left, self.product())
        return left

    def product(self) -> Expression:
        left = self.factor()
        while self._peek() in ("*", "/"):
            symbol = self._take()
            left = Operation(symbol, left, self.factor())
        return left

    def factor(self) -> Expression:
        token = self._take()
        if token == "-":
            return Negate(self.factor())
        if token == "(":
            inner = self.sum()
            if self._take() != ")":
                raise ValueError("a bracket is not closed")
            return inner
        if token in OPERATIONS or token == ")":
            raise ValueError(f"{token!r} where a value belongs")
        return operand(token)


def evaluate(expression: Expression, read: Callable[[Variable], float]) -> float:
    """The value of *expression*, reading each variable with *read*."""
    match expression:
        case Literal(value):
            return value
        case Variable():
            return read(expression)
        case Negate(inner):
            return -evaluate(inner, read)
        case Operation(symbol, left, right):
            right_value = evaluate(right, read)
            if symbol == "/" and right_value == 0:
                raise ArithmeticError("division by zero")
            return OPERATIONS[symbol](evaluate(left, read), right_value)
    raise AssertionError(expression)


def fit(kind: str, value: float) -> float:
    """*value* as a variable of *kind* holds it; ArithmeticError when it cannot."""
    if kind == "R":
        return value
    whole = math.floor(abs(value) + 0.5) * (1 if value >= 0 else -1)
    low, high = RANGES[kind]
    if not low <= whole <= high:
        raise ArithmeticError(f"{value:g} does not fit a {kind} variable ({low} to {high})")
    return whole
