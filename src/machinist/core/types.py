"""Domain primitives shared across all emulators.

These tiny value objects sit at the heart of the framework. They are
deliberately *immutable* (`frozen=True`) so that producers and consumers
never have to worry about accidental mutation across thread/asyncio
boundaries.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import StrEnum, auto
from typing import Self


class DeviceState(StrEnum):
    """Lifecycle state shared by every emulator."""

    CREATED = auto()
    STARTING = auto()
    RUNNING = auto()
    STOPPING = auto()
    STOPPED = auto()
    FAULTED = auto()


@dataclass(frozen=True, slots=True)
class Endpoint:
    """A network endpoint (host + port) where an emulator listens."""

    host: str
    port: int

    def __str__(self) -> str:  # pragma: no cover - trivial
        return f"{self.host}:{self.port}"

    def with_host(self, host: str) -> Self:
        return type(self)(host=host, port=self.port)
