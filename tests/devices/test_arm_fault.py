"""A controller fault is a separate condition from an emergency stop.

A real controller reports a collision/alarm and an e-stop through different
registers, and a driver decides different things from each. The shared arm
state machine therefore keeps them in separate modes, with separate commands
to raise and to clear each one.
"""

from __future__ import annotations

import pytest

pytest.importorskip("numpy")

from machinist.devices.robots.arm import ArmMode, ArmOptions, arm_from_options


@pytest.fixture
def arm():
    return arm_from_options(ArmOptions(), name="arm1")


def test_fault_does_not_engage_the_estop(arm) -> None:
    arm.fault()

    s = arm.state.snapshot()
    assert s.mode is ArmMode.FAULTED
    assert s.faulted
    assert not s.estopped


def test_estop_does_not_raise_a_fault(arm) -> None:
    arm.estop()

    s = arm.state.snapshot()
    assert s.estopped
    assert not s.faulted


def test_fault_drops_the_move_in_progress(arm) -> None:
    arm.movej((0.1,) * 6, duration=10.0)
    assert arm.state.snapshot().moving

    arm.fault()
    assert arm.state.snapshot().mode is ArmMode.FAULTED


def test_reset_does_not_clear_a_fault(arm) -> None:
    arm.fault()
    arm.reset()

    assert arm.state.snapshot().faulted


def test_stop_does_not_clear_a_fault(arm) -> None:
    arm.fault()
    arm.stop()

    assert arm.state.snapshot().faulted


def test_stop_does_not_clear_an_estop(arm) -> None:
    arm.estop()
    arm.stop()

    assert arm.state.snapshot().estopped


def test_clear_fault_returns_the_arm_to_idle(arm) -> None:
    arm.fault()
    arm.clear_fault()

    assert arm.state.snapshot().mode is ArmMode.IDLE


def test_clear_fault_does_not_release_an_estop(arm) -> None:
    arm.estop()
    arm.clear_fault()

    assert arm.state.snapshot().estopped
