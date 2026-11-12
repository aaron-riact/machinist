"""Positions as the controller keeps them, read from VAR.DAT, a job's //POS and TOOL.CND."""

from __future__ import annotations

from pathlib import Path

import pytest

from machinist.devices.robots.yaskawa.backup import ControllerBackup, parse_position_variables
from machinist.devices.robots.yaskawa.inform import (
    CombinePositions,
    CopyPosition,
    GetElement,
    Literal,
    SetElement,
    Variable,
    parse_instruction,
    run_job,
)
from machinist.devices.robots.yaskawa.positions import (
    DataType,
    Position,
    Tool,
    parse_job_positions,
    parse_tools,
    parse_variable_line,
)

from .test_yaskawa_inform import _Host, _job

#: Lines in the layout a DX200 saves them in; the values are made up.
PULSE_LINE = '"PULSE"0,0,0,000000000000000000000000,0,0,10000,-2000,-30000,5000,20000,-3000,0,0'
USER_LINE = ('"RECTAN"0,5,1,000000000000000000000000,0,19,40.000,-5.000,0.020,'
             '0.0100,-0.0030,-60.0000,0.0000,0.0000')
BASE_LINE = ('"RECTAN"0,0,4,000000000000000000000000,0,16,1000.000,-250.500,300.000,'
             '180.0000,0.0000,90.0000,0.0000,0.0000')


def test_a_pulse_variable_keeps_its_pulses() -> None:
    assert parse_variable_line(PULSE_LINE) == Position(
        DataType.PULSE, tool=0, user=0, axes=(10000, -2000, -30000, 5000, 20000, -3000, 0, 0))


def test_a_cartesian_variable_is_in_micrometres_and_ten_thousandths_of_a_degree() -> None:
    assert parse_variable_line(USER_LINE) == Position(
        DataType.USER, tool=1, user=5, axes=(40000, -5000, 20, 100, -30, -600000, 0, 0))
    assert parse_variable_line(BASE_LINE) == Position(
        DataType.BASE, tool=4, user=0, axes=(1000000, -250500, 300000, 1800000, 0, 900000, 0, 0))


def test_an_unused_variable_is_none() -> None:
    assert parse_variable_line('"UNUSED"') is None


def test_a_position_is_its_hses_record() -> None:
    position = parse_variable_line(USER_LINE)
    assert position is not None
    assert position.record() == (19, 0, 1, 5, 0, 40000, -5000, 20, 100, -30, -600000, 0, 0)
    assert Position.from_record(position.record()) == position
    assert position.with_axis(2, 7).axes[1] == 7


def test_var_dat_numbers_its_p_variables_from_0() -> None:
    text = "\r\n".join(["///D", "0,0", "///P", '"UNUSED"', PULSE_LINE, '"UNUSED"', USER_LINE,
                       "///BP"])
    positions = parse_position_variables(text)
    assert set(positions) == {1, 3}
    assert positions[3][3] == 5


#: The //POS section of a job.
JOB = "\r\n".join([
    "/JOB", "//NAME SEARCH", "//POS", "///NPOS 1,0,0,3,0,0",
    "///TOOL 0", "///POSTYPE PULSE", "///PULSE",
    "C00000=10,20,30,40,50,60",
    "P00001=10000,-2000,-30000,5000,20000,-3000",
    "///USER 3", "///TOOL 1", "///POSTYPE USER", "///RECTAN", "///RCONF 0,0,0,0,0,0",
    "P00010=0.000,0.000,150.000,0.0000,0.0000,0.0000",
    "///USER 5",
    "P00011=40.000,-5.000,0.020,0.0100,-0.0030,-60.0000",
    "//INST", "NOP", "END",
])


def test_a_jobs_positions_take_the_user_tool_and_type_above_them() -> None:
    positions = parse_job_positions(JOB)
    assert positions.own == {0: Position(DataType.PULSE, 0, 0, (10, 20, 30, 40, 50, 60, 0, 0))}
    assert positions.variables[1].data_type is DataType.PULSE
    assert positions.variables[10] == Position(DataType.USER, 1, 3,
                                               (0, 0, 150000, 0, 0, 0, 0, 0))
    assert positions.variables[11].user == 5


TOOL_CND = "\r\n".join([
    "//TOOL 0", "///NAME STANDARD TOOL", "0.000,0.000,0.000,0.0000,0.0000,0.0000",
    "0.000,0.000,50.000", "10.000", "0.000,0.000,0.000", "0.000,0,3",
    "//TOOL 1", "///NAME LASER", "-300.000,0.000,100.000,90.0000,0.0000,90.0000",
    "0.000,0.000,50.000", "10.000", "0.000,0.000,0.000", "0.000,0,3",
])


def test_each_tool_has_its_name_and_offset_from_the_flange() -> None:
    assert parse_tools(TOOL_CND) == {
        0: Tool("STANDARD TOOL", (0, 0, 0, 0, 0, 0)),
        1: Tool("LASER", (-300, 0, 100, 90, 0, 90)),
    }


def test_a_backup_gives_the_controller_its_p_variables(tmp_path: Path) -> None:
    (tmp_path / "VAR.DAT").write_text("\r\n".join(["///P", '"UNUSED"', PULSE_LINE]))
    assert ControllerBackup.load(tmp_path).positions == {
        1: (0, 0, 0, 0, 0, 10000, -2000, -30000, 5000, 20000, -3000, 0, 0)}


@pytest.mark.parametrize("line", ['"PULSE"', "0,0,0", '"OTHER"1,2'])
def test_a_line_that_is_no_p_variable_is_refused(line: str) -> None:
    with pytest.raises(ValueError, match="P variable line"):
        parse_variable_line(line)


# --- What a job does with P variables ----------------------------------------------

P010 = Position(DataType.USER, 1, 3, (0, 0, 150000, 0, 0, 0, 0, 0))
P012 = Position(DataType.USER, 1, 6, (800000, -1000, 0, 0, 0, 5000, 0, 0))


def _run(*lines: str, positions: dict[int, Position], **variables: dict[int, float]) -> _Host:
    host = _Host({"J": _job(*lines)}, variables=dict(variables),
                 positions={"P": {n: p.record() for n, p in positions.items()}})
    run_job(host, "J")
    return host


def _p(host: _Host, number: int) -> Position:
    return Position.from_record(host.positions["P"][number])


@pytest.mark.parametrize(("text", "instruction"), [
    ("SET P010 P011", CopyPosition(10, 11)),
    ("GETE D010 P010 (1)", GetElement(Variable("D", 10), 10, 1)),
    ("SETE P011 (2) D021", SetElement(11, 2, Variable("D", 21))),
    ("SETE P010 (6) -300000", SetElement(10, 6, Literal(-300000))),
    ("ADD P010 P012", CombinePositions("+", 10, 12)),
    ("SUB P012 P012", CombinePositions("-", 12, 12)),
    ("COMM SETE P012 (3) -50000", None),
])
def test_a_position_line_parses_to_what_it_does(text: str, instruction: object) -> None:
    assert parse_instruction(text) == instruction


def test_a_job_moves_a_copy_of_a_position_along_its_x() -> None:
    host = _run("SET P011 P010", "GETE D010 P011 (3)", "SET D020  EXPRESS 10000 + D010",
                "SETE P011 (1) D020", positions={10: P010})
    assert _p(host, 11) == P010.with_axis(1, 160000)
    assert _p(host, 10) == P010
    assert host.variables["D"][10] == 150000


def test_adding_positions_adds_each_axis_and_keeps_the_firsts_frame() -> None:
    host = _run("ADD P010 P012", positions={10: P010, 12: P012})
    assert _p(host, 10) == Position(DataType.USER, 1, 3, (800000, -1000, 150000, 0, 0, 5000,
                                                          0, 0))
    host = _run("SUB P010 P010", positions={10: P010})
    assert _p(host, 10).axes == (0,) * 8


def test_an_unset_position_reads_as_pulses_at_zero() -> None:
    host = _run("GETE D000 P0007 (1)", "SETE P0008 (2) 5", positions={})
    assert host.variables["D"][0] == 0
    assert _p(host, 8) == Position(DataType.PULSE, 0, 0, (0, 5, 0, 0, 0, 0, 0, 0))
