"""A live Projection follows the World's bus and equals the devices' own truth."""

from __future__ import annotations

import time

import pytest

from machinist.core.config import DeviceConfig, SystemConfig
from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import DeviceState, Endpoint
from machinist.core.world import World, WorldBuilder
from machinist.devices.machines.mazak_840d import MazakSinumerik840D, MazakSinumerik840DOptions
from machinist.projection import FleetState, Projection
from machinist.transport.s7_server import S7Server, S7Store


def _world() -> World:
    return WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
            ),
            io_links=(),
        )
    )


def _settled(predicate, *, timeout: float = 1.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.005)
    return predicate()


@pytest.mark.timeout(5)
def test_projection_follows_a_running_device_without_polling_it() -> None:
    world = _world()
    projection = Projection(world)
    seen: list[FleetState] = []
    projection.subscribe(seen.append)
    gripper = world.devices[0]

    world.start()
    try:
        assert gripper.wait_ready(timeout=2.0)
        assert _settled(lambda: projection.state.device("g1").lifecycle is DeviceState.RUNNING)

        world.io_map.signal("g1.cmd_open").set(True)
        assert _settled(lambda: projection.state.device("g1").signals["is_open"].value is True)
    finally:
        world.stop()
        projection.close()

    assert projection.state.device("g1").lifecycle is DeviceState.STOPPED
    assert seen, "listeners hear every change"
    assert seen[-1] is projection.state


def test_projection_equals_the_devices_truth_after_each_command() -> None:
    """The safety net: whatever a device believes, the projection believes too."""
    bus = EventBus()
    store = S7Store()
    device = MazakSinumerik840D(
        "m1", Endpoint("127.0.0.1", 0), bus, MazakSinumerik840DOptions(),
        io=SignalBank(owner="m1", publish=bus.publish), store=store,
        server=S7Server(host="127.0.0.1", port=0, store=store, backend="stub"),
    )
    world = World(devices=(device,), bus=bus, io_map=WorldBuilder().build(SystemConfig()).io_map)
    projection = Projection(world)

    for command in ("door_open_cmd", "door_close_cmd", "cycle_start_cmd"):
        device.io[command].set(True)
        view = projection.state.device("m1")
        assert view.machine == device.state.view
        assert {n: s.value for n, s in view.signals.items()} == {s.name: s.value for s in device.io}
        device.io[command].set(False)
