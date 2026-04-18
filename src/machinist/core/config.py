"""Configuration models and YAML loader.

The whole system is driven by a single declarative document (or several
merged together). The schema is enforced by pydantic which gives us:

* validation with helpful error messages,
* JSON-schema generation if we ever expose it to the UI,
* immutable, hashable models for free.
"""

from __future__ import annotations

from collections.abc import Iterable
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, Field

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


class SystemConfig(_Frozen):
    """Top-level configuration document."""

    devices: tuple[DeviceConfig, ...] = Field(default_factory=tuple)
    io_links: tuple[IOLink, ...] = Field(default_factory=tuple)


def load_config(paths: Iterable[Path]) -> SystemConfig:
    """Load and merge one or more YAML config files.

    Multiple files are concatenated: device lists and io_links are
    appended. Conflicting device *names* are rejected.
    """
    devices: list[DeviceConfig] = []
    io_links: list[IOLink] = []
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

    return SystemConfig(devices=tuple(devices), io_links=tuple(io_links))
