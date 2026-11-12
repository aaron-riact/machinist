"""INFORM arithmetic: expressions, ADD to SQRT, and how a value fits its variable."""

from __future__ import annotations

import pytest

from machinist.devices.robots.yaskawa.expression import (
    Literal,
    Negate,
    Operation,
    Variable,
    evaluate,
    fit,
    parse_expression,
)
from machinist.devices.robots.yaskawa.inform import (
    Arithmetic,
    Assign,
    JobStoppedError,
    SquareRoot,
    parse_instruction,
    run_job,
)

from .test_yaskawa_inform import _Host, _job

D010 = Variable("D", 10)
I001 = Variable("I", 1)


@pytest.mark.parametrize(("text", "expression"), [
    ("10000 + D010", Operation("+", Literal(10000), D010)),
    ("-I001 * 100 + D010",
     Operation("+", Operation("*", Negate(I001), Literal(100)), D010)),
    ("( D021 - D031 ) / 2",
     Operation("/", Operation("-", Variable("D", 21), Variable("D", 31)), Literal(2))),
    ("D000 + ( I001 * 0.5 - 200 ) * 10",
     Operation("+", Variable("D", 0),
               Operation("*", Operation("-", Operation("*", I001, Literal(0.5)), Literal(200)),
                         Literal(10)))),
    ("-( D001 )", Negate(Variable("D", 1))),
])
def test_an_expression_parses_with_precedence_and_brackets(text: str, expression: object) -> None:
    assert parse_expression(text) == expression


@pytest.mark.parametrize("text", ["D010 +", "( D010", "D010 D002", "* 2", "D010 + X"])
def test_an_expression_that_is_not_one_is_refused(text: str) -> None:
    with pytest.raises(ValueError, match=r"expression|bracket|value belongs"):
        parse_expression(text)


def test_an_expression_reads_its_variables() -> None:
    values = {D010: 7.0, I001: 2.0}
    assert evaluate(parse_expression("-I001 * 100 + D010"), values.__getitem__) == -193


@pytest.mark.parametrize(("kind", "value", "stored"), [
    ("D", 2.5, 3), ("D", -2.5, -3), ("D", 2.4, 2), ("I", -0.6, -1), ("B", 255.4, 255),
    ("R", 2.5, 2.5),
])
def test_an_integer_variable_holds_the_nearest_whole_number(kind: str, value: float,
                                                            stored: float) -> None:
    assert fit(kind, value) == stored


@pytest.mark.parametrize(("kind", "value"), [("B", 256), ("B", -1), ("I", 40000),
                                             ("D", 3e9)])
def test_a_value_past_the_variables_range_is_refused(kind: str, value: float) -> None:
    with pytest.raises(ArithmeticError, match="does not fit"):
        fit(kind, value)


@pytest.mark.parametrize(("text", "instruction"), [
    ("SET D020 EXPRESS 10000 + D010",
     Assign(Variable("D", 20), Operation("+", Literal(10000), D010))),
    ("ADD D006 1000", Arithmetic("+", Variable("D", 6), Literal(1000))),
    ("SUB D012 1000", Arithmetic("-", Variable("D", 12), Literal(1000))),
    ("MUL I001 D002", Arithmetic("*", Variable("I", 1), Variable("D", 2))),
    ("DIV R000 3", Arithmetic("/", Variable("R", 0), Literal(3))),
    ("INC D030", Arithmetic("+", Variable("D", 30), Literal(1))),
    ("DEC I005", Arithmetic("-", Variable("I", 5), Literal(1))),
    ("SQRT R002 R001", SquareRoot(Variable("R", 2), Variable("R", 1))),
])
def test_an_arithmetic_line_parses_to_what_it_does(text: str, instruction: object) -> None:
    assert parse_instruction(text) == instruction


def test_a_circles_centre_works_out_from_a_chord() -> None:
    """How far a 100 mm circle's centre lies from a chord 80 mm long."""
    host = _Host({"C": _job(
        "SET D021 80000", "SET D031 0",
        "SET D041  EXPRESS ( D021 - D031 ) / 2",
        "SET D002  EXPRESS I001 * 50",
        "SET R001  EXPRESS D002 * D002 - D041 * D041",
        "SQRT R002 R001",
        "SET D022 R002",
        "INC D022", "DEC D022", "ADD D022 1000", "SUB D022 1000",
    )}, variables={"I": {1: 1000}})
    run_job(host, "C")
    d = host.variables["D"]
    assert (d[41], d[2]) == (40000, 50000)
    assert host.variables["R"][2] == pytest.approx(30000.0, abs=0.1)
    assert d[22] == 30000


@pytest.mark.parametrize(("lines", "reason"), [
    (("SET B000 EXPRESS 200 + 100",), "does not fit a B variable"),
    (("SET R000 -4", "SQRT R001 R000"), "square root of -4"),
    (("SET D000 0", "SET D001 EXPRESS 5 / D000"), "division by zero"),
])
def test_arithmetic_that_cannot_be_done_stops_the_job(lines: tuple[str, ...],
                                                       reason: str) -> None:
    with pytest.raises(JobStoppedError, match=reason):
        run_job(_Host({"J": _job(*lines)}), "J")
