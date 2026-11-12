"""A real controller's files, as saved from it (over HSES file control, FTP or the
pendant to USB), loaded so the emulator holds what that controller holds.

Every file in the folder becomes one of the controller's files: its jobs
(NAME.JBI), TOOL.CND, CIOPRG.LST and the rest, which take the place of the
emulator's own. VAR.DAT also gives the B, I, D and R variables the jobs read, and
the P variables (see :mod:`.positions`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .positions import parse_variable_line

#: The variable kinds VAR.DAT holds that the emulator keeps, by their section.
VARIABLE_SECTIONS = ("B", "I", "D", "R")


@dataclass(frozen=True, slots=True)
class ControllerBackup:
    files: dict[str, bytes]
    variables: dict[str, dict[int, float]]
    #: P variables by number, as HSES records; UNUSED ones are left out.
    positions: dict[int, tuple[int, ...]] = field(default_factory=dict)

    @classmethod
    def load(cls, folder: Path) -> ControllerBackup:
        if not folder.is_dir():
            raise ValueError(f"controller_files {folder} is not a folder")
        files = {path.name: path.read_bytes()
                 for path in sorted(folder.iterdir()) if path.is_file()}
        var_dat = files.get("VAR.DAT")
        text = var_dat.decode("ascii", errors="replace") if var_dat else ""
        return cls(files, parse_variables(text), parse_position_variables(text))


def parse_variables(text: str) -> dict[str, dict[int, float]]:
    """The B, I, D and R variables of a VAR.DAT, each by number from 0.

    A section starts with its ``///B``-style line; its values follow, ten to a
    line, the first being variable 0.
    """
    variables: dict[str, dict[int, float]] = {}
    store: dict[int, float] | None = None
    for raw in text.replace("\r", "").split("\n"):
        line = raw.strip()
        if line.startswith("/"):
            kind = line[3:].split(" ")[0] if line.startswith("///") else ""
            store = variables.setdefault(kind, {}) if kind in VARIABLE_SECTIONS else None
            continue
        if store is None:
            continue
        for value in line.split(","):
            if value.strip():
                store[len(store)] = float(value)
    return variables


def parse_position_variables(text: str) -> dict[int, tuple[int, ...]]:
    """The P variables of a VAR.DAT, by number from 0, as HSES records."""
    positions: dict[int, tuple[int, ...]] = {}
    number: int | None = None
    for raw in text.replace("\r", "").split("\n"):
        line = raw.strip()
        if line.startswith("/"):
            number = 0 if line == "///P" else None
            continue
        if number is None or not line:
            continue
        position = parse_variable_line(line)
        if position is not None:
            positions[number] = position.record()
        number += 1
    return positions
