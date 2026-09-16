"""A type-safe registry of device factories.

Devices declare themselves via :func:`register`, naming the
:class:`~machinist.core.options.Options` model their YAML block parses into.
The registry does that parsing once, at creation, so a factory is handed
typed options and never a dict::

    @register("ur_dashboard", default_port=29999, options=ArmOptions)
    def factory(name, endpoint, bus, options: ArmOptions) -> Device: ...
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from .device import Device
from .events import EventBus
from .options import Options, parse_options
from .types import Endpoint

#: A factory is handed the device's options already parsed into its model.
DeviceFactory = Callable[[str, Endpoint, EventBus, Any], Device]


@dataclass(frozen=True, slots=True)
class _Entry:
    factory: DeviceFactory
    default_port: int
    options: type[Options]


class DeviceRegistry:
    """Maps device ``kind`` strings to factory callables."""

    def __init__(self) -> None:
        self._entries: dict[str, _Entry] = {}

    def register(
        self,
        kind: str,
        factory: DeviceFactory,
        *,
        options: type[Options],
        default_port: int = 0,
    ) -> None:
        if kind in self._entries:
            raise ValueError(f"Device kind {kind!r} already registered")
        self._entries[kind] = _Entry(factory=factory, default_port=default_port, options=options)

    def create(
        self, kind: str, name: str, endpoint: Endpoint, bus: EventBus, config: dict[str, Any]
    ) -> Device:
        """Build a device: parse *config* into the kind's options, then call its factory."""
        entry = self._entry(kind)
        return entry.factory(name, endpoint, bus, parse_options(entry.options, config, kind=kind))

    def options_for(self, kind: str) -> type[Options]:
        """The options model a kind's YAML block is parsed into."""
        return self._entry(kind).options

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


def register(
    kind: str, *, options: type[Options], default_port: int = 0
) -> Callable[[DeviceFactory], DeviceFactory]:
    """Decorator that registers a factory in :data:`default_registry`."""

    def decorator(factory: DeviceFactory) -> DeviceFactory:
        default_registry.register(kind, factory, default_port=default_port, options=options)
        return factory

    return decorator
