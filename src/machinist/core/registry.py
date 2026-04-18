"""A type-safe registry of device factories.

Devices declare themselves via :func:`register`. The CLI/loader looks
them up by ``kind`` and instantiates them with their per-device config.
This keeps adding a new device a one-liner:
``@register("ur_robot", default_port=29999)``.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .device import Device
from .events import EventBus
from .types import Endpoint

DeviceFactory = Callable[[str, Endpoint, EventBus, dict[str, Any]], Device]


@dataclass(frozen=True, slots=True)
class _Entry:
    factory: DeviceFactory
    default_port: int


class DeviceRegistry:
    """Maps device ``kind`` strings to factory callables."""

    def __init__(self) -> None:
        self._entries: dict[str, _Entry] = {}

    def register(self, kind: str, factory: DeviceFactory, *, default_port: int = 0) -> None:
        if kind in self._entries:
            raise ValueError(f"Device kind {kind!r} already registered")
        self._entries[kind] = _Entry(factory=factory, default_port=default_port)

    def create(
        self, kind: str, name: str, endpoint: Endpoint, bus: EventBus, config: dict[str, Any]
    ) -> Device:
        return self._entry(kind).factory(name, endpoint, bus, config)

    def default_port(self, kind: str) -> int:
        return self._entry(kind).default_port

    def kinds(self) -> tuple[str, ...]:
        return tuple(sorted(self._entries))

    def _entry(self, kind: str) -> _Entry:
        try:
            return self._entries[kind]
        except KeyError as exc:
            raise KeyError(f"Unknown device kind {kind!r}") from exc


#: Process-wide default registry. Devices populate this at import time.
default_registry = DeviceRegistry()


def register(kind: str, *, default_port: int = 0) -> Callable[[DeviceFactory], DeviceFactory]:
    """Decorator that registers a factory in :data:`default_registry`."""

    def decorator(factory: DeviceFactory) -> DeviceFactory:
        default_registry.register(kind, factory, default_port=default_port)
        return factory

    return decorator
