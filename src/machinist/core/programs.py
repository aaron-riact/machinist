"""A directory of program files a device exposes (G-code, jobs, ...)."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar

from .events import Event


@dataclass(slots=True)
class ProgramLibrary:
    """Directory of program files exposed to the UI and file shares."""

    root: Path

    def __post_init__(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[str]:
        return sorted(p.name for p in self.root.iterdir() if p.is_file())

    def read(self, name: str) -> str:
        return (self.root / name).read_text(encoding="utf-8")

    def write(self, name: str, body: str) -> None:
        (self.root / name).write_text(body, encoding="utf-8")


@dataclass(frozen=True, slots=True, kw_only=True)
class ProgramsChanged(Event):
    """A device's program library now lists *names*."""

    KIND: ClassVar[str] = "programs"

    names: tuple[str, ...]
