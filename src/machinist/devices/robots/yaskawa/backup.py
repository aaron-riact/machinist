"""A real controller's files, as saved from it (over HSES file control, FTP or the
pendant to USB), loaded so the emulator holds what that controller holds.

Every file in the folder becomes one of the controller's files: its jobs
(NAME.JBI), TOOL.CND, CIOPRG.LST and the rest, which take the place of the
emulator's own. VAR.DAT also gives the B, I, D and R variables the jobs read, and
the P variables (see :mod:`.positions`). CIOPRG.LST gives the cell's ladder and the
registers its header sets (see :mod:`.ladder`).
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from .ladder import Ladder, LadderFile, parse_ladder_file
from .positions import parse_variable_line

#: The variable kinds VAR.DAT holds that the emulator keeps, by their section.
VARIABLE_SECTIONS = ("B", "I", "D", "R")


@dataclass(frozen=True, slots=True)
class SavedUserFrame:
    """A user frame as UFRAME.CND keeps it: the frame in base coordinates (BUSER: X Y Z
    in mm, Rx Ry Rz in degrees), the tool it was taught in, and ORG's external axes.

    The frame, not the points: a frame a job makes (MFRAME) keeps points that need not
    span a plane, while its BUSER is the frame the controller uses."""

    frame_mm_deg: tuple[float, ...]
    tool: int
    external: tuple[int, ...]


@dataclass(frozen=True, slots=True)
class ControllerBackup:
    files: dict[str, bytes]
    variables: dict[str, dict[int, float]]
    #: P variables by number, as HSES records; UNUSED ones are left out.
    positions: dict[int, tuple[int, ...]] = field(default_factory=dict)
    #: The saved ladder, empty when the folder has no CIOPRG.LST.
    ladder: LadderFile = field(default_factory=lambda: LadderFile({}, Ladder()))
    #: The user frames UFRAME.CND keeps, by number; the ones never taught are left out.
    user_frames: dict[int, SavedUserFrame] = field(default_factory=dict)

    @classmethod
    def load(cls, folder: Path) -> ControllerBackup:
        if not folder.is_dir():
            raise ValueError(f"controller_files {folder} is not a folder")
        files = {path.name: path.read_bytes()
                 for path in sorted(folder.iterdir()) if path.is_file()}
        var_dat = files.get("VAR.DAT")
        text = var_dat.decode("ascii", errors="replace") if var_dat else ""
        cioprg = files.get("CIOPRG.LST")
        ladder = (parse_ladder_file(cioprg.decode("ascii", errors="replace"))
                  if cioprg is not None else LadderFile({}, Ladder()))
        uframe = files.get("UFRAME.CND")
        user_frames = (parse_user_frames(uframe.decode("ascii", errors="replace"))
                       if uframe is not None else {})
        return cls(files, parse_variables(text), parse_position_variables(text), ladder,
                   user_frames)


#: How many of a UFRAME.CND point's pulses are the robot's; the rest are external axes.
ROBOT_AXES = 6
#: The external axes a user frame records.
EXTERNAL_AXES = 6


def parse_user_frames(text: str) -> dict[int, SavedUserFrame]:
    """The taught user frames of a UFRAME.CND, by number: each ``//UFRAME n`` section's
    ``///TOOL``, ``////BUSER`` and ORG's external axes. A frame whose points
    (``////RORG``, ``////RXX``, ``////RXY``) are all zero was never taught."""
    frames: dict[int, SavedUserFrame] = {}
    for section in text.replace("\r", "").split("//UFRAME ")[1:]:
        lines = section.split("\n")
        tool, frame, points = 0, None, {}
        for line in lines[1:]:
            if line.startswith("///TOOL "):
                tool = int(line.split()[1])
            elif line.startswith("////BUSER "):
                frame = tuple(float(v) for v in line.split(" ", 1)[1].split(","))
            elif line.startswith(("////RORG ", "////RXX ", "////RXY ")):
                key, _, values = line[4:].partition(" ")
                points[key] = tuple(int(v) for v in values.split("=", 1)[1].split(","))
        if frame is None or not any(any(p) for p in points.values()):
            continue
        org = points.get("RORG", ())
        frames[int(lines[0])] = SavedUserFrame(
            frame_mm_deg=frame[:6], tool=tool,
            external=(org[ROBOT_AXES:] + (0,) * EXTERNAL_AXES)[:EXTERNAL_AXES],
        )
    return frames


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
