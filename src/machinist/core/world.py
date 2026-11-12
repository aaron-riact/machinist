"""The :class:`World` orchestrates the lifecycle of a fleet of devices."""

from __future__ import annotations

from collections.abc import Iterator
from contextlib import contextmanager
from dataclasses import dataclass, field

import numpy as np

from .addressing import AddressAllocator
from .capabilities import HasFlange, HasIO, HasMount, HasRegisters, SensesDistance
from .config import FlangeLink, MountConfig, SystemConfig
from .device import Device
from .environment import MountedSensor, Scene, mount_offset
from .events import EventBus
from .io import IOMap
from .registry import DeviceRegistry, default_registry


@dataclass(slots=True)
class World:
    """A running fleet of emulated devices."""

    devices: tuple[Device, ...]
    bus: EventBus
    io_map: IOMap
    #: The parts in the cell, which mounted sensors see.
    scene: Scene = field(default_factory=lambda: Scene(()))

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

        by_name = {d.name: d for d in devices}
        for flange in config.flange_links:
            _wire_flange(flange, by_name)

        scene = Scene.of(config.parts)
        for mount in config.mounts:
            _mount(mount, by_name, scene)

        return World(devices=tuple(devices), bus=bus, io_map=io_map, scene=scene)


def _wire_flange(link: FlangeLink, by_name: dict[str, Device]) -> None:
    """Attach a device to an arm's flange bus, once both devices exist.

    Runs after every device is built, like io_links do: a device factory is
    handed only its own config and cannot reach across the fleet.
    """
    master = by_name.get(link.master)
    if master is None:
        raise KeyError(f"flange_link names unknown master {link.master!r}")
    slave = by_name.get(link.slave)
    if slave is None:
        raise KeyError(f"flange_link names unknown slave {link.slave!r}")

    if not isinstance(master, HasFlange):
        raise TypeError(f"{link.master!r} has no tool flange to wire {link.slave!r} onto")
    if not isinstance(slave, HasRegisters):
        raise TypeError(f"{link.slave!r} has no registers to expose on a flange")
    master.flange.attach(link.slave_id, slave.register_port)


def _mount(mount: MountConfig, by_name: dict[str, Device], scene: Scene) -> None:
    """Mount a sensor: on a device that carries it, it sees the scene after each
    move of its carrier; fixed in the cell, it sees it once."""
    sensor = by_name.get(mount.device)
    if sensor is None:
        raise KeyError(f"mount names unknown device {mount.device!r}")
    if not isinstance(sensor, SensesDistance):
        raise TypeError(f"{mount.device!r} senses no distance; only a sensor is mounted")
    mounted = MountedSensor(sensor, mount_offset(mount), scene)
    if mount.carrier is None:
        mounted.see_from(np.eye(4))
        return
    carrier = by_name.get(mount.carrier)
    if carrier is None:
        raise KeyError(f"mount puts {mount.device!r} on unknown device {mount.carrier!r}")
    if not isinstance(carrier, HasMount):
        raise TypeError(f"{mount.carrier!r} has no mount point to carry {mount.device!r}")
    carrier.on_mount_moved(mounted.see_from)
    mounted.see_from(carrier.mount_transform())


def _absorb_io(device: Device, io_map: IOMap) -> None:
    """Register a device's :class:`SignalBank` with the IO map, if it has one."""
    if isinstance(device, HasIO):
        io_map.adopt(device.io)


@contextmanager
def running(config: SystemConfig) -> Iterator[World]:
    """Context manager: build, start, yield, stop."""
    world = WorldBuilder().build(config)
    world.start()
    try:
        yield world
    finally:
        world.stop()
