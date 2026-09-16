from __future__ import annotations

import dataclasses

import pytest

from machinist.core.state import StateCell


@dataclasses.dataclass(frozen=True, slots=True)
class _Gripper:
    width: int = 0
    busy: bool = False


def test_update_replaces_fields_and_announces_the_new_view() -> None:
    seen: list[_Gripper] = []
    cell = StateCell(_Gripper(), on_change=seen.append)
    view = cell.update(width=40, busy=True)
    assert cell.view is view
    assert seen == [_Gripper(width=40, busy=True)]


def test_swap_derives_the_new_view_from_the_old_one_atomically() -> None:
    cell = StateCell(_Gripper(width=10))
    cell.swap(lambda g: dataclasses.replace(g, width=g.width + 5))
    assert cell.view.width == 15


def test_a_change_to_the_same_value_is_silent() -> None:
    seen: list[_Gripper] = []
    cell = StateCell(_Gripper(width=3), on_change=seen.append)
    cell.update(width=3)
    assert seen == []


def test_the_view_cannot_be_written_directly() -> None:
    cell = StateCell(_Gripper())
    with pytest.raises(dataclasses.FrozenInstanceError):
        cell.view.width = 1  # type: ignore[misc]
