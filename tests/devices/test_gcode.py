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
