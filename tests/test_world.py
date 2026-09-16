from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from machinist.core.config import DeviceConfig, FlangeLink, SystemConfig
from machinist.core.device import Device
from machinist.core.registry import DeviceRegistry
from machinist.core.world import WorldBuilder
from machinist.transport.flange_bus import FlangeBus
from machinist.transport.registers import RegisterPort


def test_world_builds_from_yaml(tmp_path: Path) -> None:
    cfg = {
        "devices": [
            {"name": "g1", "kind": "pneumatic_gripper", "options": {"settle_seconds": 0.01}},
            {"name": "io1", "kind": "weidmuller_ur20", "host": "127.0.0.1", "port": 0,
             "options": {"inputs": 4, "outputs": 4}},
        ],
        "io_links": [
            {"source": "io1.o1", "target": "g1.cmd_open"},
        ],
    }
    path = tmp_path / "scene.yaml"
    path.write_text(yaml.safe_dump(cfg))

    from machinist.core.config import load_config
    config = load_config([path])
    assert isinstance(config, SystemConfig)
    world = WorldBuilder().build(config)
    assert {d.name for d in world.devices} == {"g1", "io1"}
    # IO link works without starting servers (we don't bind sockets here).
    world.io_map.bank("io1")["o1"].set(True)
    assert world.devices[0].io["cmd_open"].value is True


# --- flange links -------------------------------------------------------



class _Arm(Device):
    kind = "fake_arm"

    def __init__(self, name, endpoint, bus) -> None:
        super().__init__(name, endpoint, bus)
        self.flange = FlangeBus()

    def _run(self, stop) -> None:  # pragma: no cover - never started
        stop.wait()


class _Gripper(Device):
    kind = "fake_gripper"

    def __init__(self, name, endpoint, bus) -> None:
        super().__init__(name, endpoint, bus)
        self.cells: dict[int, int] = {}

    @property
    def register_port(self) -> RegisterPort:
        return RegisterPort(
            on_read=lambda a: self.cells.get(a, 0), on_write=self.cells.__setitem__
        )

    def _run(self, stop) -> None:  # pragma: no cover - never started
        stop.wait()


def _registry() -> DeviceRegistry:
    reg = DeviceRegistry()
    reg.register("fake_arm", lambda n, e, b, o: _Arm(n, e, b))
    reg.register("fake_gripper", lambda n, e, b, o: _Gripper(n, e, b))
    return reg


def _config(*flange_links: FlangeLink, slave_kind: str = "fake_gripper") -> SystemConfig:
    return SystemConfig(
        devices=(
            DeviceConfig(name="arm1", kind="fake_arm"),
            DeviceConfig(name="rg1", kind=slave_kind),
            DeviceConfig(name="rg2", kind=slave_kind),
        ),
        flange_links=flange_links,
    )


def test_flange_link_attaches_the_slave_to_the_arms_bus() -> None:
    config = _config(FlangeLink(master="arm1", slave="rg1", slave_id=0x41))

    world = WorldBuilder(registry=_registry()).build(config)

    arm = next(d for d in world.devices if d.name == "arm1")
    assert arm.flange.slave_ids == (0x41,)


def test_two_flange_links_share_one_arms_bus() -> None:
    """The OnRobot Dual Quick Changer case: two tools, one flange."""
    config = _config(
        FlangeLink(master="arm1", slave="rg1", slave_id=0x41),
        FlangeLink(master="arm1", slave="rg2", slave_id=0x42),
    )

    world = WorldBuilder(registry=_registry()).build(config)

    arm = next(d for d in world.devices if d.name == "arm1")
    assert arm.flange.slave_ids == (0x41, 0x42)


def test_flange_link_reaches_the_slaves_registers() -> None:
    config = _config(FlangeLink(master="arm1", slave="rg1", slave_id=0x41))
    world = WorldBuilder(registry=_registry()).build(config)
    arm = next(d for d in world.devices if d.name == "arm1")
    gripper = next(d for d in world.devices if d.name == "rg1")

    arm.flange.write_holding(0x41, 0, [400, 1100])

    assert gripper.cells == {0: 400, 1: 1100}


def test_flange_link_to_an_unknown_master_is_rejected() -> None:
    config = _config(FlangeLink(master="nope", slave="rg1", slave_id=0x41))

    with pytest.raises(KeyError, match="unknown master"):
        WorldBuilder(registry=_registry()).build(config)


def test_flange_link_to_an_unknown_slave_is_rejected() -> None:
    config = _config(FlangeLink(master="arm1", slave="nope", slave_id=0x41))

    with pytest.raises(KeyError, match="unknown slave"):
        WorldBuilder(registry=_registry()).build(config)


def test_flange_link_onto_a_device_with_no_flange_is_rejected() -> None:
    config = _config(FlangeLink(master="rg1", slave="rg2", slave_id=0x41))

    with pytest.raises(TypeError, match="no tool flange"):
        WorldBuilder(registry=_registry()).build(config)


def test_flange_link_of_a_device_with_no_registers_is_rejected() -> None:
    config = SystemConfig(
        devices=(
            DeviceConfig(name="arm1", kind="fake_arm"),
            DeviceConfig(name="arm2", kind="fake_arm"),
        ),
        flange_links=(FlangeLink(master="arm1", slave="arm2", slave_id=0x41),),
    )

    with pytest.raises(TypeError, match="no registers"):
        WorldBuilder(registry=_registry()).build(config)
