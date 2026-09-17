from __future__ import annotations

from machinist.core.config import DeviceConfig, SystemConfig
from machinist.core.panel import Field, Panel
from machinist.core.world import World, WorldBuilder
from machinist.devices.machines.state import MachineState
from machinist.devices.robots.arm import RobotArm
from machinist.projection import DeviceView, seed
from machinist.web.api import snapshot_world, view_to_dict


def _gripper_world() -> World:
    config = SystemConfig(
        devices=(
            DeviceConfig(name="io1", kind="weidmuller_ur20", options={"inputs": 8, "outputs": 8}),
            DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
        ),
    )
    return WorldBuilder().build(config)


# --- serialization ------------------------------------------------------


def _view(name: str = "ur1", kind: str = "ur_dashboard", **fields: object) -> DeviceView:
    return DeviceView(name=name, kind=kind, endpoint="127.0.0.1:29999", **fields)  # type: ignore[arg-type]


def test_view_to_dict_reports_core_identity() -> None:
    snap = view_to_dict(_view())
    assert snap["name"] == "ur1"
    assert snap["kind"] == "ur_dashboard"
    assert snap["endpoint"] == "127.0.0.1:29999"
    assert snap["lifecycle"] == "created"
    assert snap["signals"] == []
    assert snap["ethernetip"]["mode"] == "io"  # the plain panel of a device with nothing to add
    assert "arm" not in snap and "machine" not in snap and "programs" not in snap


def test_view_to_dict_includes_arm_snapshot() -> None:
    arm = RobotArm(joint_count=6)
    arm.estop()
    snap = view_to_dict(_view("arm1", "robot", arm=arm.state.view))
    assert snap["arm"]["mode"] == "estopped"
    assert snap["arm"]["estopped"] is True
    assert len(snap["arm"]["joints"]) == 6
    assert len(snap["arm"]["pose"]) == 6


def test_view_to_dict_includes_machine_state() -> None:
    state = MachineState()
    state.update(program="O0001\nG0 X0", spindle_rpm=1500.0, tool=3, parts=7)
    state.set_door("main", open=True)
    state.move_to(x=12.0)
    machine = view_to_dict(_view("mill", "haas_ngc", machine=state.view))["machine"]
    assert machine["program"] == "O0001"
    assert machine["doors"] == {"main": True}
    assert machine["spindle_rpm"] == 1500.0
    assert machine["tool"] == 3
    assert machine["parts"] == 7
    assert machine["position"]["x"] == 12.0


def test_view_to_dict_includes_ethernetip_breakdown() -> None:
    panel = Panel(
        mode="adapter",
        transport_ready=True,
        peer_connected=False,
        input_block_hex="00 00",
        output_block_hex="01 00",
        input_fields=(Field("DI100", "Target work number data"),),
        output_fields=(Field("DO100", "Current work number"),),
        status_fields=(Field("STATE", "Alarm message"),),
    )
    snap = view_to_dict(_view("smooth", "mazak_smooth", panel=panel))
    assert snap["ethernetip"]["mode"] == "adapter"
    assert snap["ethernetip"]["input_fields"]
    assert snap["ethernetip"]["input_fields"][0]["signal"] == "DI100"


def test_view_to_dict_files_a_modbus_panel_under_modbus() -> None:
    snap = view_to_dict(_view("g1", "onrobot_rg", panel=Panel(mode="modbus", clients=1)))
    assert snap["modbus"]["clients"] == 1
    assert "ethernetip" not in snap


def test_snapshot_world_lists_signals_grouped_with_direction() -> None:
    world = _gripper_world()
    snap = snapshot_world(seed(world))
    names = {d["name"] for d in snap["devices"]}
    assert {"io1", "g1"} <= names
    io1 = next(d for d in snap["devices"] if d["name"] == "io1")
    assert io1["signals"], "io controller should expose signals"
    directions = {s["direction"] for s in io1["signals"]}
    assert directions <= {"input", "output"}
