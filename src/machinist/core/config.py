"""Configuration models and YAML loader.

The whole system is driven by a single declarative document (or several
merged together). The schema is enforced by pydantic which gives us:

* validation with helpful error messages,
* JSON-schema generation if we ever expose it to the UI,
* immutable, hashable models for free.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, Field, model_validator

from .types import Endpoint

#: Default loopback host. Devices that do not pin a host inherit this and
#: are eligible for auto-bumping when their port collides.
DEFAULT_HOST = "127.0.0.1"


class _Frozen(BaseModel):
    model_config = ConfigDict(frozen=True, extra="forbid")


class DeviceConfig(_Frozen):
    """Per-device configuration entry from the YAML document."""

    name: str
    kind: str
    host: str | None = None
    port: int | None = None
    options: dict[str, Any] = Field(default_factory=dict)

    def desired_endpoint(self, default_port: int) -> Endpoint:
        return Endpoint(host=self.host or DEFAULT_HOST, port=self.port or default_port)

    @property
    def host_was_default(self) -> bool:
        return self.host is None


class IOLink(_Frozen):
    """A directed wire from one device's IO to another's."""

    source: str  # "device.signal"
    target: str  # "device.signal"


class FlangeLink(_Frozen):
    """A device wired onto a serial line, usually an arm's tool flange.

    The master owns that line and reaches the device by *slave_id*, so two
    entries naming the same master are how an OnRobot Dual Quick Changer's
    pair of grippers is expressed. Most masters are arms; a bare
    ``modbus_rtu_gateway`` is a line with no arm on the end of it.
    """

    master: str  # the device whose serial line carries this one
    slave: str  # the device wired onto it
    slave_id: int


class CylinderShape(_Frozen):
    """An upright cylinder."""

    diameter_mm: float = Field(gt=0)
    height_mm: float = Field(gt=0)


class BoxShape(_Frozen):
    """A box: its size along its own X, Y and Z."""

    size_mm: tuple[float, float, float]

    @model_validator(mode="after")
    def _every_side_has_a_length(self) -> BoxShape:
        if min(self.size_mm) <= 0:
            raise ValueError(f"size_mm {self.size_mm}: every side must be longer than 0")
        return self


class PartGrid(_Frozen):
    """Parts in rows: the first, how many along each row and across, and the
    distance between them, the rows turned by rz_deg about Z."""

    first_mm: tuple[float, float, float]
    count: tuple[int, int]
    pitch_mm: tuple[float, float]

    @model_validator(mode="after")
    def _at_least_one(self) -> PartGrid:
        if min(self.count) < 1:
            raise ValueError(f"count {self.count}: at least one each way")
        return self


class PartsConfig(_Frozen):
    """Parts of one shape standing in the cell, which sensors see.

    Each part is placed by the centre of its bottom face, in the cell frame: listed
    in at_mm, or laid out in a grid.
    """

    name: str
    cylinder: CylinderShape | None = None
    box: BoxShape | None = None
    at_mm: tuple[tuple[float, float, float], ...] = ()
    grid: PartGrid | None = None
    #: How far the grid's rows, and each box, are turned about Z.
    rz_deg: float = 0.0

    @model_validator(mode="after")
    def _one_shape_somewhere(self) -> PartsConfig:
        if (self.cylinder is None) == (self.box is None):
            raise ValueError(f"parts {self.name!r}: give one shape, cylinder or box")
        if not self.at_mm and self.grid is None:
            raise ValueError(f"parts {self.name!r}: place them with at_mm or grid")
        return self

    def bottoms_mm(self) -> list[tuple[float, float, float]]:
        """The centre of each part's bottom face."""
        bottoms = list(self.at_mm)
        if self.grid is not None:
            turn = math.radians(self.rz_deg)
            row, across = ((math.cos(turn), math.sin(turn)), (-math.sin(turn), math.cos(turn)))
            x0, y0, z0 = self.grid.first_mm
            for i in range(self.grid.count[0]):
                for j in range(self.grid.count[1]):
                    a, b = i * self.grid.pitch_mm[0], j * self.grid.pitch_mm[1]
                    bottoms.append((x0 + a * row[0] + b * across[0],
                                    y0 + a * row[1] + b * across[1], z0))
        return bottoms


class MountConfig(_Frozen):
    """A sensor mounted in the cell: on its carrier's mount point (an arm's flange),
    or fixed in the cell when *carrier* is left out. Its beam leaves at_mm_deg along
    that frame's +Z axis; the angles turn about X, then Y, then Z, as Yaskawa's
    Rx, Ry, Rz do."""

    device: str
    #: The device that carries the sensor. (Not "on": YAML reads that key as true.)
    carrier: str | None = None
    at_mm_deg: tuple[float, float, float, float, float, float]


class SystemConfig(_Frozen):
    """Top-level configuration document."""

    devices: tuple[DeviceConfig, ...] = Field(default_factory=tuple)
    io_links: tuple[IOLink, ...] = Field(default_factory=tuple)
    flange_links: tuple[FlangeLink, ...] = Field(default_factory=tuple)
    #: The parts in the cell, which sensors see.
    parts: tuple[PartsConfig, ...] = Field(default_factory=tuple)
    #: Where each sensor sits.
    mounts: tuple[MountConfig, ...] = Field(default_factory=tuple)


def load_config(paths: Iterable[Path]) -> SystemConfig:
    """Load and merge one or more YAML config files.

    Multiple files are concatenated: device lists, io_links, flange_links,
    parts and mounts are appended. Conflicting device *names* are rejected.
    """
    devices: list[DeviceConfig] = []
    io_links: list[IOLink] = []
    flange_links: list[FlangeLink] = []
    parts: list[PartsConfig] = []
    mounts: list[MountConfig] = []
    seen_names: set[str] = set()

    for path in paths:
        raw = yaml.safe_load(path.read_text()) or {}
        partial = SystemConfig.model_validate(raw)
        for dev in partial.devices:
            if dev.name in seen_names:
                raise ValueError(f"Duplicate device name {dev.name!r} (in {path})")
            seen_names.add(dev.name)
            devices.append(dev)
        io_links.extend(partial.io_links)
        flange_links.extend(partial.flange_links)
        parts.extend(partial.parts)
        mounts.extend(partial.mounts)

    return SystemConfig(
        devices=tuple(devices),
        io_links=tuple(io_links),
        flange_links=tuple(flange_links),
        parts=tuple(parts),
        mounts=tuple(mounts),
    )
