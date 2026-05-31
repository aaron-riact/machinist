from __future__ import annotations

from types import SimpleNamespace

import pytest

from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.world import World, WorldBuilder
from machinist.devices.machines.state import MachineState, Toggle
from machinist.devices.robots.arm import RobotArm
from machinist.web.api import (
    CommandError,
    dispatch_command,
    snapshot_device,
    snapshot_world,
)


def _gripper_world() -> World:
    config = SystemConfig(
        devices=(
            DeviceConfig(name="io1", kind="weidmuller_ur20", options={"inputs": 8, "outputs": 8}),
            DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
        ),
    )
    return WorldBuilder().build(config)


# --- serialization ------------------------------------------------------


def test_snapshot_device_reports_core_identity() -> None:
    device = SimpleNamespace(
        name="ur1",
        kind="ur_dashboard",
        endpoint="127.0.0.1:29999",
        lifecycle="running",
    )
    snap = snapshot_device(device)
    assert snap == {
        "name": "ur1",
        "kind": "ur_dashboard",
        "endpoint": "127.0.0.1:29999",
        "lifecycle": "running",
    }


def test_snapshot_device_includes_arm_snapshot() -> None:
    arm = RobotArm(joint_count=6)
    arm.estop()
    device = SimpleNamespace(
        name="arm1", kind="robot", endpoint="127.0.0.1:15001", lifecycle="running", arm=arm
    )
    snap = snapshot_device(device)
    assert snap["arm"]["mode"] == "estopped"
    assert snap["arm"]["estopped"] is True
    assert len(snap["arm"]["joints"]) == 6
    assert len(snap["arm"]["pose"]) == 6


def test_snapshot_device_includes_machine_state() -> None:
    state = MachineState()
    state.program = "O0001\nG0 X0"
    state.doors["main"] = Toggle(name="main", open=True)
    state.spindle_rpm = 1500.0
    state.tool = 3
    state.parts = 7
    state.position.x = 12.0
    device = SimpleNamespace(
        name="mill", kind="haas_ngc", endpoint="127.0.0.1:5051", lifecycle="running", state=state
    )
    machine = snapshot_device(device)["machine"]
    assert machine["program"] == "O0001"
    assert machine["doors"] == {"main": True}
    assert machine["spindle_rpm"] == 1500.0
    assert machine["tool"] == 3
    assert machine["parts"] == 7
    assert machine["position"]["x"] == 12.0


def test_snapshot_world_lists_signals_grouped_with_direction() -> None:
    world = _gripper_world()
    snap = snapshot_world(world)
    names = {d["name"] for d in snap["devices"]}
    assert {"io1", "g1"} <= names
    io1 = next(d for d in snap["devices"] if d["name"] == "io1")
    assert io1["signals"], "io controller should expose signals"
    directions = {s["direction"] for s in io1["signals"]}
    assert directions <= {"input", "output"}


# --- command dispatch ---------------------------------------------------


def test_dispatch_set_drives_a_signal_and_links_propagate() -> None:
    world = WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(
                    name="io1", kind="weidmuller_ur20", options={"inputs": 8, "outputs": 8}
                ),
                DeviceConfig(
                    name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}
                ),
            ),
            io_links=(IOLink(source="io1.o5", target="g1.cmd_open"),),
        )
    )
    result = dispatch_command(world, "set io1.o5 1")
    assert result["ok"] is True
    assert world.io_map._resolve("io1.o5").value is True
    assert world.io_map._resolve("g1.cmd_open").value is True


def test_dispatch_set_unknown_signal_raises() -> None:
    world = _gripper_world()
    with pytest.raises(CommandError):
        dispatch_command(world, "set io1.nope 1")


def test_dispatch_estop_requires_an_arm() -> None:
    world = _gripper_world()
    with pytest.raises(CommandError, match="no arm"):
        dispatch_command(world, "estop g1")


def test_dispatch_unknown_verb_and_empty_raise() -> None:
    world = _gripper_world()
    with pytest.raises(CommandError, match="unknown command"):
        dispatch_command(world, "frobnicate g1")
    with pytest.raises(CommandError, match="empty"):
        dispatch_command(world, "   ")


def test_dispatch_unknown_device_raises() -> None:
    world = _gripper_world()
    with pytest.raises(CommandError, match="unknown device"):
        dispatch_command(world, "reset nope")


def test_dispatch_help_lists_verbs() -> None:
    world = _gripper_world()
    result = dispatch_command(world, "help")
    assert result["ok"] is True
    assert "estop" in result["message"]
    assert "set" in result["message"]
