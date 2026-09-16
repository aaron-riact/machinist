"""A directory of program files a device exposes (G-code, jobs, ...)."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import ClassVar

from .events import Event

Publish = Callable[[Event], None]


@dataclass(slots=True)
class ProgramLibrary:
    """Directory of program files exposed to the UI and file shares.

    Other programs write into the directory (over SMB, say), so the library
    cannot know when it changed. :meth:`refresh` looks, and publishes a
    :class:`ProgramsChanged` when the listing differs from the last one it
    announced; a device runs it on a poller.
    """

    root: Path
    owner: str = field(default="", kw_only=True)
    publish: Publish | None = field(default=None, kw_only=True)
    _announced: tuple[str, ...] | None = field(default=None, init=False)

    def __post_init__(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[str]:
        return sorted(p.name for p in self.root.iterdir() if p.is_file())

    def read(self, name: str) -> str:
        return (self.root / name).read_text(encoding="utf-8")

    def write(self, name: str, body: str) -> None:
        (self.root / name).write_text(body, encoding="utf-8")
        self.refresh()

    def refresh(self) -> tuple[str, ...]:
        """Re-read the directory; announce the listing if it changed."""
        names = tuple(self.list())
        if names != self._announced:
            self._announced = names
            if self.publish is not None:
                self.publish(ProgramsChanged(device=self.owner, names=names))
        return names


@dataclass(frozen=True, slots=True, kw_only=True)
class ProgramsChanged(Event):
    """A device's program library now lists *names*."""

    KIND: ClassVar[str] = "programs"

    names: tuple[str, ...]
