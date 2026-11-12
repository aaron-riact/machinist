"""IFTHEN blocks, and conditions joined by ANDIF and ORIF."""

from __future__ import annotations

import pytest

from machinist.devices.robots.yaskawa.inform import (
    AllOf,
    AnyOf,
    Comparison,
    Condition,
    JobStoppedError,
    Jump,
    Literal,
    SignalTest,
    Variable,
    parse_instruction,
    run_job,
)

from .test_yaskawa_inform import _Host, _job

BLOCK = ("IFTHEN I000=1", "SET D001 1",
         "ELSEIF I000=2", "SET D001 2",
         "ELSE", "SET D001 3",
         "ENDIF", "SET D002 9")


@pytest.mark.parametrize(("i000", "d001"), [(1, 1), (2, 2), (5, 3)])
def test_a_block_runs_the_first_branch_whose_condition_holds(i000: int, d001: int) -> None:
    host = _Host({"B": _job(*BLOCK)}, variables={"I": {0: i000}})
    run_job(host, "B")
    assert host.variables["D"] == {1: d001, 2: 9}


def test_a_block_without_else_runs_nothing_when_its_condition_fails() -> None:
    host = _Host({"B": _job("IFTHEN IN#(9)=ON", "SET D001 1", "ENDIF", "SET D002 2")})
    run_job(host, "B")
    assert host.variables["D"] == {2: 2}


def test_blocks_nest() -> None:
    host = _Host({"B": _job(
        "IFTHEN B000=1",
        "  IFTHEN B001=1", "SET D001 11", "ELSE", "SET D001 10", "ENDIF",
        "SET D002 1",
        "ELSE", "SET D002 0",
        "ENDIF")}, variables={"B": {0: 1, 1: 0}})
    run_job(host, "B")
    assert host.variables["D"] == {1: 10, 2: 1}


def test_a_jump_inside_a_block_leaves_it() -> None:
    """*1 is a label inside an IFTHEN block that a later JUMP goes to."""
    host = _Host({"B": _job("IFTHEN B000=1", "*1", "SET D001 1", "JUMP *OUT", "ENDIF",
                            "SET B000 1", "JUMP *1", "*OUT")}, variables={"B": {0: 0}})
    run_job(host, "B")
    assert host.variables["D"] == {1: 1}


@pytest.mark.parametrize(("text", "condition"), [
    ("JUMP *L IF I000=1 ANDIF I001=2",
     AllOf((Comparison(Variable("I", 0), "=", Literal(1)),
            Comparison(Variable("I", 1), "=", Literal(2))))),
    ("JUMP *L IF D000>2000 ORIF D000<-2000",
     AnyOf((Comparison(Variable("D", 0), ">", Literal(2000)),
            Comparison(Variable("D", 0), "<", Literal(-2000))))),
    ("JUMP *L IF IN#(D025)=ON ORIF IN#(D026)=ON",
     AnyOf((SignalTest(False, Variable("D", 25), True),
            SignalTest(False, Variable("D", 26), True)))),
])
def test_tests_join_with_andif_and_orif(text: str, condition: Condition) -> None:
    assert parse_instruction(text) == Jump("L", condition)


@pytest.mark.parametrize(("i0", "i1", "taken"), [(1, 2, True), (1, 3, False), (0, 2, False)])
def test_andif_needs_every_test(i0: int, i1: int, taken: bool) -> None:
    host = _Host({"A": _job("IFTHEN I000=1 ANDIF I001=2", "SET D000 1", "ENDIF")},
                 variables={"I": {0: i0, 1: i1}})
    run_job(host, "A")
    assert (host.variables.get("D", {}).get(0) == 1) is taken


def test_ifthenexp_tests_as_ifthen_does() -> None:
    host = _Host({"E": _job("IFTHENEXP I024<>0", "SET D000 1", "ENDIF")},
                 variables={"I": {24: 3}})
    run_job(host, "E")
    assert host.variables["D"] == {0: 1}


@pytest.mark.parametrize(("lines", "reason"), [
    (("ENDIF",), "without its IFTHEN"),
    (("ELSE",), "without its IFTHEN"),
    (("IFTHEN I000=0", "SET D000 1"), "without its ENDIF"),
])
def test_a_block_that_does_not_close_stops_the_job(lines: tuple[str, ...], reason: str) -> None:
    with pytest.raises(JobStoppedError, match=reason):
        run_job(_Host({"J": _job(*lines)}), "J")
