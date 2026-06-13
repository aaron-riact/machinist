"""Zero-arg reader mappings used by OPC-UA and other publishers."""

from __future__ import annotations

from machinist.devices.machines.state import MachineState, Toggle, machine_readers
from machinist.devices.robots.arm import RobotArm, arm_readers


def test_arm_readers_initial_state() -> None:
    arm = RobotArm(joint_count=6)
    readers = arm_readers(arm)
    assert set(readers) >= {"mode", "servo_on", "estopped", "moving", "command", "joints", "pose"}
    assert readers["mode"]() == "idle"
    assert readers["servo_on"]() is True
    assert readers["estopped"]() is False
    assert readers["moving"]() is False
    assert readers["command"]() == "none"
    assert readers["joints"]() == (0.0,) * 6
    assert readers["pose"]() == (0.0,) * 6


def test_arm_readers_reflect_state_change() -> None:
    arm = RobotArm(joint_count=6)
    readers = arm_readers(arm)
    arm.estop()
    assert readers["mode"]() == "estopped"
    assert readers["estopped"]() is True
    assert readers["command"]() == "none"


def test_arm_readers_during_move() -> None:
    arm = RobotArm(joint_count=6)
    readers = arm_readers(arm)
    target = (1.0,) * 6
    arm.movej(target, duration=5.0)
    assert readers["mode"]() == "moving"
    assert readers["moving"]() is True
    assert readers["command"]() == "movej"


def test_machine_readers_initial_state() -> None:
    state = MachineState()
    readers = machine_readers(state)
    assert set(readers) >= {"cycle", "program", "spindle_rpm", "feed", "tool", "parts", "x", "y", "z"}
    assert readers["cycle"]() == "idle"
    assert readers["program"]() == ""
    assert readers["spindle_rpm"]() == 0.0
    assert readers["feed"]() == 0.0
    assert readers["tool"]() == 0
    assert readers["parts"]() == 0
    assert readers["x"]() == 0.0
    assert readers["y"]() == 0.0
    assert readers["z"]() == 0.0


def test_machine_readers_reflect_state_change() -> None:
    state = MachineState()
    readers = machine_readers(state)
    state.parts = 42
    state.position.x = 1.5
    assert readers["parts"]() == 42
    assert readers["x"]() == 1.5


def test_machine_readers_with_doors() -> None:
    state = MachineState()
    state.doors["main"] = Toggle(name="main", open=True)
    state.doors["side"] = Toggle(name="side", open=False)
    readers = machine_readers(state)
    assert "door_main_open" in readers
    assert "door_side_open" in readers
    assert readers["door_main_open"]() is True
    assert readers["door_side_open"]() is False


def test_machine_readers_with_chucks() -> None:
    state = MachineState()
    state.chucks["left"] = Toggle(name="left", open=False)
    readers = machine_readers(state)
    assert "chuck_left_open" in readers
    assert readers["chuck_left_open"]() is False
