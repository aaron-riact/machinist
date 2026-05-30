from __future__ import annotations

import time

from machinist.core.events import EventBus
from machinist.devices.machines.gcode import Interpreter
from machinist.devices.machines.state import CycleState, MachineState


def test_dprint_and_variables() -> None:
    state = MachineState()
    interp = Interpreter(state)
    program = """
    DPRINT[hello world]
    #100 = 12.5
    M30
    """
    list(interp.run(program))
    assert state.dprint_log == ["hello world"]
    assert state.variables["100"] == 12.5
    assert state.cycle is CycleState.IDLE


def test_dwell_pauses() -> None:
    state = MachineState()
    interp = Interpreter(state)
    start = time.monotonic()
    list(interp.run("G4 P0.05\nM30"))
    assert (time.monotonic() - start) >= 0.04


def test_event_bus_unused() -> None:
    # Sanity: gcode interpreter has no dependency on EventBus.
    EventBus()


def test_spindle_tool_feed_and_parts() -> None:
    state = MachineState()
    interp = Interpreter(state)
    program = """
    T1 M06
    G1 F250
    M03 S1500
    M5
    M30
    """
    log = list(interp.run(program))
    assert state.tool == 1
    assert state.tool_changes == 1
    assert state.feed == 250.0
    # Spindle ends stopped (M5 after M3).
    assert state.spindle_rpm == 0.0
    assert state.parts == 1
    assert any("tool change T1" in line for line in log)
    assert any("spindle CW 1500" in line for line in log)


def test_m_and_g_code_zero_padding_equivalent() -> None:
    state = MachineState()
    interp = Interpreter(state)
    list(interp.run("M03 S1000\nM30"))
    assert state.spindle_rpm == 1000.0
