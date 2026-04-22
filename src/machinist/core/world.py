"""The :class:`World` orchestrates the lifecycle of a fleet of devices."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field

from .addressing import AddressAllocator
from .config import SystemConfig
from .device import Device
from .events import EventBus
from .io import IOMap, SignalBank
from .registry import DeviceRegistry, default_registry


@dataclass(slots=True)
class World:
    """A running fleet of emulated devices."""

    devices: tuple[Device, ...]
    bus: EventBus
    io_map: IOMap

    def start(self) -> None:
        for device in self.devices:
            device.start()

    def stop(self) -> None:
        for device in reversed(self.devices):
            device.stop()


@dataclass(slots=True)
class WorldBuilder:
    """Builds a :class:`World` from a :class:`SystemConfig`."""

    registry: DeviceRegistry = field(default_factory=lambda: default_registry)

    def build(self, config: SystemConfig) -> World:
        bus = EventBus()
        allocator = AddressAllocator()
        io_map = IOMap()
        devices: list[Device] = []

        for entry in config.devices:
            default_port = self.registry.default_port(entry.kind)
            desired = entry.desired_endpoint(default_port)
            endpoint = allocator.allocate(desired, host_was_default=entry.host_was_default)
            device = self.registry.create(entry.kind, entry.name, endpoint, bus, entry.options)
            _absorb_io(device, io_map)
            devices.append(device)

        for link in config.io_links:
            io_map.link(link.source, link.target)

        return World(devices=tuple(devices), bus=bus, io_map=io_map)


def _absorb_io(device: Device, io_map: IOMap) -> None:
    """Register a device's :class:`SignalBank` with the IO map, if any."""
    bank = getattr(device, "io", None)
    if isinstance(bank, SignalBank):
        io_map.adopt(bank)


@contextmanager
def running(config: SystemConfig) -> Iterator[World]:
    """Context manager: build, start, yield, stop."""
    world = WorldBuilder().build(config)
    world.start()
    try:
        yield world
    finally:
        world.stop()
