"""Shared state for CNC machines, held by a single writer.

A machine has doors and chucks (open or closed), a cycle (idle / running /
paused / aborted), spindle, feed, tool and part counters, a position, and a
key/value variable store. Per-vendor modules expose that through their
wire protocol.

:class:`MachineView` is the state as everyone reads it: a frozen snapshot.
:class:`MachineState` is the only thing that changes it. Every change goes
through one of its methods, which swap in a new view and publish a
:class:`MachineChanged` event carrying it. A direct write to a view field
is a type error and a runtime error, so a change that forgets to announce
itself cannot be written.
"""

from __future__ import annotations

import threading
from abc import ABC
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass, field, replace
from enum import StrEnum, auto
from types import MappingProxyType
from typing import ClassVar

from ...core.events import Event

Publish = Callable[[Event], None]


class CycleState(StrEnum):
    IDLE = auto()
    RUNNING = auto()
    PAUSED = auto()
    ABORTED = auto()


@dataclass(frozen=True, slots=True)
class CartesianPosition:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def moved_to(
        self, *, x: float | None = None, y: float | None = None, z: float | None = None
    ) -> CartesianPosition:
        """This position with the given axes replaced."""
        return CartesianPosition(
            x=self.x if x is None else x,
            y=self.y if y is None else y,
            z=self.z if z is None else z,
        )


def _frozen(mapping: Mapping[str, object] | None = None) -> Mapping:
    return MappingProxyType(dict(mapping or {}))


@dataclass(frozen=True, slots=True)
class MachineView:
    """A point-in-time picture of a machine. Read it; never write to it."""

    cycle: CycleState = CycleState.IDLE
    program: str = ""
    spindle_rpm: float = 0.0
    feed: float = 0.0
    tool: int = 0
    tool_changes: int = 0
    parts: int = 0
    position: CartesianPosition = CartesianPosition()
    doors: Mapping[str, bool] = field(default_factory=_frozen)
    chucks: Mapping[str, bool] = field(default_factory=_frozen)
    variables: Mapping[str, float | str] = field(default_factory=_frozen)

    def door_open(self, name: str) -> bool:
        return self.doors[name]

    def chuck_open(self, name: str) -> bool:
        return self.chucks[name]


@dataclass(frozen=True, slots=True, kw_only=True)
class MachineChanged(Event):
    """A machine's state moved to *view*."""

    KIND: ClassVar[str] = "machine"

    view: MachineView


class MachineState:
    """The single writer of a machine's :class:`MachineView`.

    Construct it with the device's name and ``publish`` so every change is
    announced on the bus. Without them it is a plain, silent holder, which
    is what unit tests of the interpreter want.
    """

    def __init__(
        self,
        *,
        owner: str = "",
        publish: Publish | None = None,
        doors: Iterable[str] = (),
        chucks: Iterable[str] = (),
    ) -> None:
        self._owner = owner
        self._publish = publish
        self._lock = threading.Lock()
        self._view = MachineView(
            doors=_frozen({name: False for name in doors}),
            chucks=_frozen({name: False for name in chucks}),
        )
        self.dprint_log: list[str] = []
        self.dprint_subscribers: list[Callable[[str], None]] = []

    @property
    def view(self) -> MachineView:
        return self._view

    # ----- the one door for changes -----------------------------------

    def update(self, **changes: object) -> MachineView:
        """Replace the given fields. Publishes only if something changed."""
        return self._swap(lambda view: replace(view, **changes))

    def bump(self, **deltas: int) -> MachineView:
        """Add to integer counters atomically, e.g. ``bump(parts=1)``."""
        return self._swap(
            lambda view: replace(view, **{k: getattr(view, k) + d for k, d in deltas.items()})
        )

    def move_to(
        self, *, x: float | None = None, y: float | None = None, z: float | None = None
    ) -> MachineView:
        return self._swap(lambda view: replace(view, position=view.position.moved_to(x=x, y=y, z=z)))

    def declare_door(self, name: str) -> MachineView:
        """Add a closed door if it is not there yet."""
        return self._swap(lambda view: _with_toggle(view, "doors", name, view.doors.get(name, False)))

    def set_door(self, name: str, *, open: bool) -> MachineView:  # noqa: A002 - industrial vocab
        return self._swap(lambda view: _with_toggle(view, "doors", name, open))

    def declare_chuck(self, name: str) -> MachineView:
        return self._swap(lambda view: _with_toggle(view, "chucks", name, view.chucks.get(name, False)))

    def set_chuck(self, name: str, *, open: bool) -> MachineView:  # noqa: A002
        return self._swap(lambda view: _with_toggle(view, "chucks", name, open))

    def set_variables(self, **values: float | str) -> MachineView:
        """Write macro/status variables, one event for the whole batch."""
        return self._swap(lambda view: replace(view, variables=_frozen({**view.variables, **values})))

    def dprint(self, text: str) -> None:
        with self._lock:
            self.dprint_log.append(text)
            subs = list(self.dprint_subscribers)
        for sub in subs:
            sub(text)

    # ----- internals --------------------------------------------------

    def _swap(self, change: Callable[[MachineView], MachineView]) -> MachineView:
        with self._lock:
            new = change(self._view)
            changed = new != self._view
            if changed:
                self._view = new
        if changed and self._publish is not None:
            self._publish(MachineChanged(device=self._owner, view=new))
        return new


def _with_toggle(view: MachineView, group: str, name: str, open: bool) -> MachineView:  # noqa: A002
    current: Mapping[str, bool] = getattr(view, group)
    return replace(view, **{group: _frozen({**current, name: open})})


class HasMachineState(ABC):
    """A device (a CNC) whose domain state is a :class:`MachineState`."""

    state: MachineState


def machine_readers(state: MachineState) -> dict[str, Callable[[], object]]:
    """Zero-arg readers exposing machine state (e.g. for an OPC-UA server).

    Doors and chucks are captured by name at call time, so the set of
    nodes reflects whatever the machine declared; values stay live.
    """
    readers: dict[str, Callable[[], object]] = {
        "cycle": lambda: str(state.view.cycle),
        "program": lambda: state.view.program.splitlines()[0] if state.view.program else "",
        "spindle_rpm": lambda: state.view.spindle_rpm,
        "feed": lambda: state.view.feed,
        "tool": lambda: state.view.tool,
        "parts": lambda: state.view.parts,
        "x": lambda: state.view.position.x,
        "y": lambda: state.view.position.y,
        "z": lambda: state.view.position.z,
    }
    for name in state.view.doors:
        readers[f"door_{name}_open"] = _door_reader(state, name)
    for name in state.view.chucks:
        readers[f"chuck_{name}_open"] = _chuck_reader(state, name)
    return readers


def _door_reader(state: MachineState, name: str) -> Callable[[], object]:
    return lambda: state.view.door_open(name)


def _chuck_reader(state: MachineState, name: str) -> Callable[[], object]:
    return lambda: state.view.chuck_open(name)
