"""Fold events into a :class:`FleetState`, and keep one live for the UIs."""

from __future__ import annotations

import threading
from collections.abc import Callable, Iterable
from dataclasses import replace
from functools import reduce as _fold
from types import MappingProxyType

from ..core.capabilities import HasIO, HasPrograms
from ..core.device import Device
from ..core.events import DeviceFaulted, Event, LifecycleChanged
from ..core.io import SignalChanged
from ..core.panel import PanelChanged
from ..core.programs import ProgramsChanged
from ..core.world import World
from ..devices.machines.state import HasMachineState, MachineChanged
from ..devices.robots.arm import ArmChanged, HasArm
from .views import DeviceView, FleetState, SignalView

Listener = Callable[[FleetState], None]


def reduce(state: FleetState, event: Event) -> FleetState:
    """The pure step: one event in, the next fleet state out.

    Events about a device the fleet does not know, and untyped notes, leave
    the state untouched. The reducer never raises on an event, because it
    runs on the publishing device's thread.
    """
    current = state.devices.get(event.device)
    if current is None:
        return state
    match event:
        case LifecycleChanged(state=lifecycle):
            view = replace(current, lifecycle=lifecycle)
        case SignalChanged(signal=name, direction=direction, value=value):
            view = current.with_signal(SignalView(name=name, direction=direction, value=value))
        case ArmChanged(view=arm):
            view = replace(current, arm=arm)
        case MachineChanged(view=machine):
            view = replace(current, machine=machine)
        case PanelChanged(panel=panel):
            view = replace(current, panel=panel)
        case ProgramsChanged(names=names):
            view = replace(current, programs=names)
        case DeviceFaulted(message=message):
            view = replace(current, fault=message)
        case _:
            return state
    return state.with_device(view)


def replay(events: Iterable[Event], *, start: FleetState | None = None) -> FleetState:
    """Fold a recorded event stream into the fleet state it describes."""
    return _fold(reduce, events, start if start is not None else FleetState())


def seed(world: World) -> FleetState:
    """The fleet as the devices report it right now: the late joiner's snapshot."""
    return FleetState.of(_seed_device(device) for device in world.devices)


def _seed_device(device: Device) -> DeviceView:
    signals = {}
    if isinstance(device, HasIO):
        for sig in device.io:
            signals[sig.name] = SignalView(name=sig.name, direction=sig.direction, value=sig.value)
    return DeviceView(
        name=device.name,
        kind=device.kind,
        endpoint=str(device.endpoint),
        lifecycle=device.lifecycle,
        signals=MappingProxyType(signals),
        arm=device.arm.state.view if isinstance(device, HasArm) else None,
        machine=device.state.view if isinstance(device, HasMachineState) else None,
        programs=tuple(device.programs.list()) if isinstance(device, HasPrograms) else None,
        panel=device.build_detail(),
    )


class Projection:
    """A live :class:`FleetState`, kept current from the World's event bus.

    Reads are lock-free (the state is an immutable value). Listeners are
    called on the publishing thread after each change, so they must be
    cheap: set a flag, enqueue, wake a loop.
    """

    def __init__(self, world: World) -> None:
        self._lock = threading.Lock()
        self._state = seed(world)
        self._listeners: list[Listener] = []
        self._unsubscribe = world.bus.subscribe(self._apply)

    @property
    def state(self) -> FleetState:
        return self._state

    def subscribe(self, listener: Listener) -> Callable[[], None]:
        with self._lock:
            self._listeners.append(listener)

        def unsubscribe() -> None:
            with self._lock:
                if listener in self._listeners:
                    self._listeners.remove(listener)

        return unsubscribe

    def close(self) -> None:
        self._unsubscribe()

    def verify(self, world: World) -> list[str]:
        """Compare the projection with what the devices themselves hold right now.

        Returns one line per difference, empty when the projection is faithful.
        A difference means some state changed without announcing itself, which
        the design says must not happen; this is the net under that promise.
        """
        truth = seed(world)
        mine = self._state
        problems: list[str] = []
        for name, actual in truth.devices.items():
            shown = mine.devices.get(name)
            if shown is None:
                problems.append(f"{name}: missing from the projection")
                continue
            for field_name in ("lifecycle", "signals", "arm", "machine", "programs", "panel"):
                if getattr(shown, field_name) != getattr(actual, field_name):
                    problems.append(f"{name}.{field_name}: projection differs from the device")
        return problems

    def _apply(self, event: Event) -> None:
        with self._lock:
            before = self._state
            after = reduce(before, event)
            if after is before:
                return
            self._state = after
            listeners = list(self._listeners)
        for listener in listeners:
            listener(after)
