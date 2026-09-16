from __future__ import annotations

from types import SimpleNamespace

import pytest

from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.world import World, WorldBuilder
from machinist.devices.machines.state import MachineState
from machinist.devices.robots.arm import RobotArm
from machinist.web.api import (
    CommandError,
    dispatch_command,
    snapshot_device,
    snapshot_world,
)

from .fakes import FakeArmDevice, FakeDevice, FakeMachineDevice, RecordingDobot


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
    device = FakeDevice("ur1", kind="ur_dashboard", detail=None)
    snap = snapshot_device(device)
    assert snap == {
        "name": "ur1",
        "kind": "ur_dashboard",
        "endpoint": "127.0.0.1:29999",
        "lifecycle": "created",
    }


def test_snapshot_device_includes_arm_snapshot() -> None:
    arm = RobotArm(joint_count=6)
    arm.estop()
    device = FakeArmDevice("arm1", arm=arm)
    snap = snapshot_device(device)
    assert snap["arm"]["mode"] == "estopped"
    assert snap["arm"]["estopped"] is True
    assert len(snap["arm"]["joints"]) == 6
    assert len(snap["arm"]["pose"]) == 6


def test_snapshot_device_includes_machine_state() -> None:
    state = MachineState()
    state.update(program="O0001\nG0 X0", spindle_rpm=1500.0, tool=3, parts=7)
    state.set_door("main", open=True)
    state.move_to(x=12.0)
    device = FakeMachineDevice("mill", state=state)
    machine = snapshot_device(device)["machine"]
    assert machine["program"] == "O0001"
    assert machine["doors"] == {"main": True}
    assert machine["spindle_rpm"] == 1500.0
    assert machine["tool"] == 3
    assert machine["parts"] == 7
    assert machine["position"]["x"] == 12.0


def test_snapshot_device_includes_ethernetip_breakdown() -> None:
    device = FakeDevice(
        "smooth",
        kind="mazak_smooth",
        detail={
            "mode": "adapter",
            "transport_ready": True,
            "peer_connected": False,
            "input_block_hex": "00 00",
            "output_block_hex": "01 00",
            "input_fields": [{"signal": "DI100", "name": "Target work number data"}],
            "output_fields": [{"signal": "DO100", "name": "Current work number"}],
            "derived_fields": [{"signal": "STATE", "name": "Alarm message"}],
            "signals": [],
        },
    )
    snap = snapshot_device(device)
    assert snap["ethernetip"]["mode"] == "adapter"
    assert snap["ethernetip"]["input_fields"]
    assert snap["ethernetip"]["input_fields"][0]["signal"] == "DI100"


def test_snapshot_device_omits_ethernetip_when_snapshot_is_disabled() -> None:
    device = FakeDevice("smooth", kind="mazak_smooth", detail=None)
    snap = snapshot_device(device)
    assert "ethernetip" not in snap


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
    assert world.io_map.signal("io1.o5").value is True
    assert world.io_map.signal("g1.cmd_open").value is True


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


# --- fault injection commands -------------------------------------------


def _dobot_world() -> tuple[World, RecordingDobot]:
    dobot = RecordingDobot()
    return SimpleNamespace(devices=[dobot]), dobot


def test_dispatch_pstop_defaults_to_a_collision() -> None:
    world, dobot = _dobot_world()

    result = dispatch_command(world, "pstop dobot1")

    assert result["ok"] is True
    assert dobot.stops == [{"robot_mode": 11, "controller_ids": (), "sticky": False}]


def test_dispatch_pstop_accepts_a_mode_alarm_ids_and_sticky() -> None:
    world, dobot = _dobot_world()

    dispatch_command(world, "pstop dobot1 error ids=17,116 sticky")

    assert dobot.stops == [{"robot_mode": 9, "controller_ids": (17, 116), "sticky": True}]


def test_dispatch_pstop_clear_releases_the_stop() -> None:
    world, dobot = _dobot_world()

    dispatch_command(world, "pstop dobot1 clear")

    assert dobot.cleared == 1
    assert dobot.stops == []


def test_dispatch_pstop_rejects_an_unknown_option() -> None:
    world, _ = _dobot_world()

    with pytest.raises(CommandError, match="unknown pstop option"):
        dispatch_command(world, "pstop dobot1 sideways")


def test_dispatch_pstop_rejects_non_numeric_alarm_ids() -> None:
    world, _ = _dobot_world()

    with pytest.raises(CommandError, match="bad alarm ids"):
        dispatch_command(world, "pstop dobot1 ids=17,oops")


def test_dispatch_pstop_requires_a_device_that_can_be_stopped() -> None:
    world = _gripper_world()

    with pytest.raises(CommandError, match="cannot be put into a protective stop"):
        dispatch_command(world, "pstop g1")


def test_dispatch_failenable_sets_and_clears_the_failure() -> None:
    from machinist.devices.robots.dobot import EnableFailure

    world, dobot = _dobot_world()

    dispatch_command(world, "failenable dobot1 stuck")
    dispatch_command(world, "failenable dobot1 off")

    assert dobot.enable_failures == [EnableFailure.STUCK, None]


def test_dispatch_failenable_rejects_an_unknown_kind() -> None:
    world, _ = _dobot_world()

    with pytest.raises(CommandError, match="stuck|error|off"):
        dispatch_command(world, "failenable dobot1 explode")
