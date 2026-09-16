"""The fleet state is a pure reduction of typed events."""

from __future__ import annotations

from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.events import DeviceFaulted, LifecycleChanged, Note
from machinist.core.io import Direction, SignalChanged
from machinist.core.panel import Field, Panel, PanelChanged
from machinist.core.programs import ProgramsChanged
from machinist.core.types import DeviceState
from machinist.core.world import World, WorldBuilder
from machinist.devices.machines.state import MachineChanged, MachineState
from machinist.devices.robots.arm import ArmChanged, ArmMode, RobotArm
from machinist.projection import DeviceView, FleetState, reduce, replay, seed


def _world() -> World:
    return WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(name="io1", kind="weidmuller_ur20", options={"inputs": 2, "outputs": 2}),
                DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
            ),
            io_links=(IOLink(source="io1.o1", target="g1.cmd_open"),),
        )
    )


def _fleet(*names: str) -> FleetState:
    return FleetState.of(DeviceView(name=n, kind="fake", endpoint="127.0.0.1:1") for n in names)


# --- seeding --------------------------------------------------------------


def test_seed_reads_every_device_once() -> None:
    state = seed(_world())
    assert set(state.devices) == {"io1", "g1"}
    io1 = state.device("io1")
    assert io1.kind == "weidmuller_ur20"
    assert io1.lifecycle is DeviceState.CREATED
    assert set(io1.signals) == {"i1", "i2", "o1", "o2"}
    assert io1.signals["o1"].direction is Direction.OUTPUT
    assert io1.panel.mode == "io" and io1.panel.input_fields == ()  # IO comes from signals
    assert io1.arm is None and io1.machine is None and io1.programs is None


# --- the pure step --------------------------------------------------------


def test_lifecycle_and_fault_events_update_the_device() -> None:
    state = _fleet("d")
    state = reduce(state, LifecycleChanged(device="d", state=DeviceState.RUNNING))
    state = reduce(state, DeviceFaulted(device="d", message="port taken"))
    assert state.device("d").lifecycle is DeviceState.RUNNING
    assert state.device("d").fault == "port taken"
    assert state.version == 2


def test_signal_events_set_one_signal_and_keep_the_others() -> None:
    state = _fleet("d")
    state = reduce(state, SignalChanged(device="d", signal="i1", direction=Direction.INPUT, value=True))
    state = reduce(state, SignalChanged(device="d", signal="o1", direction=Direction.OUTPUT, value=False))
    state = reduce(state, SignalChanged(device="d", signal="i1", direction=Direction.INPUT, value=False))
    signals = state.device("d").signals
    assert (signals["i1"].value, signals["o1"].value) == (False, False)
    assert signals["o1"].direction is Direction.OUTPUT


def test_arm_machine_panel_and_programs_views_are_swapped_whole() -> None:
    arm = RobotArm(joint_count=3)
    arm.estop()
    machine = MachineState()
    machine.update(parts=4)
    panel = Panel(mode="modbus", derived_fields=(Field("W", "Width", value="42.0"),))

    state = _fleet("d")
    state = reduce(state, ArmChanged(device="d", view=arm.state.view))
    state = reduce(state, MachineChanged(device="d", view=machine.view))
    state = reduce(state, PanelChanged(device="d", panel=panel))
    state = reduce(state, ProgramsChanged(device="d", names=("O0001.nc",)))

    view = state.device("d")
    assert view.arm is not None and view.arm.mode is ArmMode.ESTOPPED
    assert view.machine is not None and view.machine.parts == 4
    assert view.panel is panel
    assert view.programs == ("O0001.nc",)


def test_unknown_devices_and_untyped_notes_leave_the_state_alone() -> None:
    state = _fleet("d")
    same = reduce(state, LifecycleChanged(device="ghost", state=DeviceState.RUNNING))
    same = reduce(same, Note(device="d", name="rx", data={"line": "hi"}))
    assert same is state
    assert same.version == 0


def test_an_event_that_changes_nothing_does_not_advance_the_version() -> None:
    state = _fleet("d")
    once = reduce(state, LifecycleChanged(device="d", state=DeviceState.RUNNING))
    twice = reduce(once, LifecycleChanged(device="d", state=DeviceState.RUNNING))
    assert twice is once


def test_replay_folds_a_recorded_stream() -> None:
    events = [
        LifecycleChanged(device="d", state=DeviceState.RUNNING),
        SignalChanged(device="d", signal="o1", direction=Direction.OUTPUT, value=True),
        LifecycleChanged(device="d", state=DeviceState.STOPPED),
    ]
    state = replay(events, start=_fleet("d"))
    assert state.device("d").lifecycle is DeviceState.STOPPED
    assert state.device("d").signals["o1"].value is True
    assert state.version == 3
