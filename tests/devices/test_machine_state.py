"""MachineState is the single writer of a frozen view, and every change is announced."""

from __future__ import annotations

import dataclasses

import pytest

from machinist.core.events import Event
from machinist.devices.machines.state import CycleState, MachineChanged, MachineState


def _recording_state(**kwargs: object) -> tuple[MachineState, list[Event]]:
    events: list[Event] = []
    state = MachineState(owner="mill", publish=events.append, **kwargs)  # type: ignore[arg-type]
    return state, events


def test_update_publishes_the_new_view() -> None:
    state, events = _recording_state()
    view = state.update(cycle=CycleState.RUNNING, program="O0001")

    assert state.view is view
    assert view.cycle is CycleState.RUNNING and view.program == "O0001"
    assert len(events) == 1
    event = events[0]
    assert isinstance(event, MachineChanged)
    assert event.device == "mill" and event.view is view


def test_an_update_that_changes_nothing_is_silent() -> None:
    state, events = _recording_state()
    state.update(parts=0)
    assert events == []


def test_the_view_cannot_be_written_directly() -> None:
    state = MachineState()
    with pytest.raises(dataclasses.FrozenInstanceError):
        state.view.parts = 3  # type: ignore[misc]
    with pytest.raises(TypeError):
        state.view.doors["main"] = True  # type: ignore[index]


def test_doors_are_declared_closed_and_set_by_name() -> None:
    state, events = _recording_state(doors=("main",))
    assert state.view.door_open("main") is False

    state.set_door("main", open=True)
    state.set_door("side", open=False)  # setting an undeclared door declares it

    assert state.view.doors == {"main": True, "side": False}
    assert [type(e) for e in events] == [MachineChanged, MachineChanged]


def test_bump_and_move_to_go_through_the_same_door() -> None:
    state, events = _recording_state()
    state.bump(parts=1, tool_changes=2)
    state.move_to(x=1.5, z=-2.0)

    assert state.view.parts == 1 and state.view.tool_changes == 2
    assert (state.view.position.x, state.view.position.y, state.view.position.z) == (1.5, 0.0, -2.0)
    assert len(events) == 2


def test_variables_are_written_as_one_batch() -> None:
    state, events = _recording_state()
    state.set_variables(alarm_code=7, alarm_message="door")
    assert state.view.variables == {"alarm_code": 7, "alarm_message": "door"}
    assert len(events) == 1


def test_a_silent_holder_still_works_without_a_bus() -> None:
    state = MachineState()
    state.update(tool=4)
    assert state.view.tool == 4
