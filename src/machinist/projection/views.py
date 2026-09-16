"""Frozen views of the fleet, one level above the per-device views."""

from __future__ import annotations

from collections.abc import Iterable, Mapping
from dataclasses import dataclass, field, replace
from types import MappingProxyType

from ..core.io import Direction
from ..core.panel import Panel
from ..core.types import DeviceState
from ..devices.machines.state import MachineView
from ..devices.robots.arm import ArmStateView


def _frozen(mapping: Mapping[str, object] | None = None) -> Mapping:
    return MappingProxyType(dict(mapping or {}))


@dataclass(frozen=True, slots=True)
class SignalView:
    name: str
    direction: Direction
    value: bool


@dataclass(frozen=True, slots=True)
class DeviceView:
    """Everything a UI shows about one device."""

    name: str
    kind: str
    endpoint: str
    lifecycle: DeviceState = DeviceState.CREATED
    signals: Mapping[str, SignalView] = field(default_factory=_frozen)
    arm: ArmStateView | None = None
    machine: MachineView | None = None
    programs: tuple[str, ...] | None = None
    panel: Panel = Panel()
    fault: str | None = None

    def with_signal(self, signal: SignalView) -> DeviceView:
        return replace(self, signals=_frozen({**self.signals, signal.name: signal}))


@dataclass(frozen=True, slots=True)
class FleetState:
    """The whole fleet at one moment. ``version`` counts reductions that changed it."""

    devices: Mapping[str, DeviceView] = field(default_factory=_frozen)
    version: int = 0

    @classmethod
    def of(cls, views: Iterable[DeviceView]) -> FleetState:
        return cls(devices=_frozen({view.name: view for view in views}))

    def device(self, name: str) -> DeviceView:
        return self.devices[name]

    def with_device(self, view: DeviceView) -> FleetState:
        """This fleet with *view* swapped in and the version advanced."""
        if self.devices.get(view.name) == view:
            return self
        return FleetState(devices=_frozen({**self.devices, view.name: view}), version=self.version + 1)
