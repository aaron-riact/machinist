"""Positions as the controller keeps them, and the files they are saved in.

A position is one record, the same one HSES carries for a P variable: its data
type, form, tool, user frame, extended form, and eight axes. A pulse position's
axes are encoder pulses; a Cartesian one's are X, Y, Z in micrometres and Rx, Ry,
Rz in ten-thousandths of a degree, turned about X, then Y, then Z.

Saved files write the same positions as text:

* VAR.DAT, section ``///P``: one line per P variable from P000, ``"UNUSED"``,
  ``"PULSE"…`` or ``"RECTAN"…``, the axes in pulses or mm and degrees.
* A job's ``//POS`` section: ``C00000=…`` (the job's own positions) and
  ``P00001=…``, each read with the ``///USER``, ``///TOOL`` and ``///POSTYPE`` lines
  above it.
* TOOL.CND: each tool's ``//TOOL n`` section, its offset from the flange first.

The form (RCONF) is not decoded: a position's form reads 0, and a move to it
takes the arm's nearest solution.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, replace
from enum import IntEnum

#: How many axes a position record carries.
AXES = 8
#: Micrometres in a millimetre, ten-thousandths of a degree in a degree.
MICROMETRES, DEGREE_UNITS = 1000, 10_000


class DataType(IntEnum):
    """A position's data type, as a P variable record numbers it."""

    PULSE = 0
    BASE = 16
    ROBOT = 17
    TOOL = 18
    USER = 19


@dataclass(frozen=True, slots=True)
class Position:
    data_type: DataType
    tool: int
    user: int
    axes: tuple[int, ...]
    form: int = 0
    extended_form: int = 0

    @classmethod
    def from_record(cls, record: tuple[int, ...]) -> Position:
        """A position from its HSES record; an unset one (all zeros) is pulses at 0."""
        data_type, form, tool, user, extended_form, *axes = record
        return cls(DataType(data_type), tool, user, tuple(axes), form, extended_form)

    def record(self) -> tuple[int, ...]:
        return (int(self.data_type), self.form, self.tool, self.user, self.extended_form,
                *self.axes)

    @property
    def cartesian(self) -> bool:
        return self.data_type is not DataType.PULSE

    def with_axis(self, element: int, value: int) -> Position:
        """This position with axis *element* (1 to 8) set to *value*."""
        axes = list(self.axes)
        axes[element - 1] = value
        return replace(self, axes=tuple(axes))


UNSET = Position(DataType.PULSE, 0, 0, (0,) * AXES)


def cartesian_axes(values: list[float]) -> tuple[int, ...]:
    """X, Y, Z in mm and Rx, Ry, Rz in degrees (then any external axes), as a record's."""
    scaled = [round(v * (MICROMETRES if i < 3 else DEGREE_UNITS)) for i, v in enumerate(values)]
    return tuple(scaled + [0] * (AXES - len(scaled)))


def pulse_axes(values: list[float]) -> tuple[int, ...]:
    return tuple([round(v) for v in values] + [0] * (AXES - len(values)))


# --- VAR.DAT ---------------------------------------------------------------------

_VAR_LINE = re.compile(r'"(UNUSED|PULSE|RECTAN)"(.*)')


def parse_variable_line(line: str) -> Position | None:
    """One VAR.DAT P variable: None for ``"UNUSED"``. Its fields before the axes are
    a control group, the user frame, the tool, the RCONF, a reserved field and the
    data type."""
    match = _VAR_LINE.fullmatch(line.strip())
    if match is None:
        raise ValueError(f"P variable line {line!r}")
    if match[1] == "UNUSED":
        return None
    fields = match[2].split(",")
    if len(fields) < 7:
        raise ValueError(f"P variable line {line!r}")
    _, user, tool, _, _, data_type = fields[:6]
    values = [float(v) for v in fields[6:]]
    if match[1] == "PULSE":
        return Position(DataType.PULSE, int(tool), int(user), pulse_axes(values))
    return Position(DataType(int(data_type)), int(tool), int(user), cartesian_axes(values))


# --- A job's //POS section ---------------------------------------------------------

_POSTYPES = {"PULSE": DataType.PULSE, "BASE": DataType.BASE, "ROBOT": DataType.ROBOT,
             "TOOL": DataType.TOOL, "USER": DataType.USER}
_POSITION = re.compile(r"(C|P)(\d{5})=(.*)")


@dataclass(frozen=True, slots=True)
class JobPositions:
    """The positions a job's file holds: its own (C) and P variables'."""

    own: dict[int, Position]
    variables: dict[int, Position]


def parse_job_positions(text: str) -> JobPositions:
    """The ``//POS`` section of a job's file, up to ``//INST``."""
    own: dict[int, Position] = {}
    variables: dict[int, Position] = {}
    user = tool = 0
    data_type = DataType.PULSE
    in_section = False
    for raw in text.replace("\r", "").split("\n"):
        line = raw.strip()
        if line == "//POS":
            in_section = True
            continue
        if line.startswith("//INST"):
            break
        if not in_section:
            continue
        if line.startswith("///USER "):
            user = int(line.split()[1])
        elif line.startswith("///TOOL "):
            tool = int(line.split()[1])
        elif line.startswith("///POSTYPE "):
            data_type = _POSTYPES[line.split()[1]]
        elif match := _POSITION.fullmatch(line):
            values = [float(v) for v in match[3].split(",")]
            axes = pulse_axes(values) if data_type is DataType.PULSE else cartesian_axes(values)
            position = Position(data_type, tool, user, axes)
            (own if match[1] == "C" else variables)[int(match[2])] = position
    return JobPositions(own, variables)


# --- TOOL.CND ----------------------------------------------------------------------


@dataclass(frozen=True, slots=True)
class Tool:
    """A tool file: its name and its offset from the flange, in mm and degrees."""

    name: str
    offset_mm_deg: tuple[float, float, float, float, float, float]


def parse_tools(text: str) -> dict[int, Tool]:
    """Each ``//TOOL n`` section of a TOOL.CND: its name, then its offset line."""
    tools: dict[int, Tool] = {}
    lines = [line.strip() for line in text.replace("\r", "").split("\n")]
    for index, line in enumerate(lines):
        if not line.startswith("//TOOL "):
            continue
        number = int(line.split()[1])
        name_line, offset_line = lines[index + 1], lines[index + 2]
        name = name_line.removeprefix("///NAME").strip()
        x, y, z, rx, ry, rz = (float(v) for v in offset_line.split(","))
        tools[number] = Tool(name, (x, y, z, rx, ry, rz))
    return tools
