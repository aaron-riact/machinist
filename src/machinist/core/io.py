"""Discrete IO points + cross-device wiring.

Devices that expose IO (robots, IO controllers, machines, grippers)
register named *signals*. The :class:`IOMap` lets a config wire the
output of one signal to the input of another, so emulating "the IO
controller's output 5 commands the machine to open its door" is a
declarative one-liner in YAML.
"""

from __future__ import annotations

import threading
from collections.abc import Callable, Iterator
from dataclasses import dataclass, field
from enum import StrEnum, auto
from typing import ClassVar

from .events import Event

SignalListener = Callable[[bool], None]
Publish = Callable[[Event], None]


class Direction(StrEnum):
    """Which way a signal flows, *from the owning device's view*.

    ``INPUT``  — driven from outside (commands the device obeys).
    ``OUTPUT`` — driven by the device (status it reports).

    This is purely descriptive metadata: it never changes how a signal
    behaves, but it lets the UI group IO and documents intent.
    """

    INPUT = auto()
    OUTPUT = auto()


@dataclass(slots=True)
class Signal:
    """A single boolean IO point.

    Threadsafe. Listeners are called *outside* the lock so they can
    safely mutate other signals (forming chains) without deadlocking.
    """

    name: str
    direction: Direction = Direction.INPUT
    _value: bool = False
    _listeners: list[SignalListener] = field(default_factory=list)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    @property
    def value(self) -> bool:
        with self._lock:
            return self._value

    def set(self, value: bool) -> None:
        with self._lock:
            if self._value == value:
                return
            self._value = value
            listeners = list(self._listeners)
        for listener in listeners:
            listener(value)

    def subscribe(self, listener: SignalListener) -> None:
        with self._lock:
            self._listeners.append(listener)


@dataclass(frozen=True, slots=True, kw_only=True)
class SignalChanged(Event):
    """One discrete IO point of a device took a new value."""

    KIND: ClassVar[str] = "signal"

    signal: str
    direction: Direction
    value: bool


@dataclass(slots=True)
class SignalBank:
    """A namespaced collection of signals owned by one device.

    Give it ``publish`` and every value change of every signal it declares
    is announced as a :class:`SignalChanged`; the writer of a signal has
    nothing extra to do.
    """

    owner: str
    publish: Publish | None = field(default=None, kw_only=True)
    _signals: dict[str, Signal] = field(default_factory=dict)

    def declare(self, name: str, direction: Direction = Direction.INPUT) -> Signal:
        if name not in self._signals:
            signal = Signal(name=name, direction=direction)
            if self.publish is not None:
                signal.subscribe(self._announcer(signal))
            self._signals[name] = signal
        return self._signals[name]

    def _announcer(self, signal: Signal) -> SignalListener:
        def announce(value: bool) -> None:
            assert self.publish is not None
            self.publish(
                SignalChanged(
                    device=self.owner, signal=signal.name, direction=signal.direction, value=value
                )
            )

        return announce

    def __getitem__(self, name: str) -> Signal:
        return self._signals[name]

    def __iter__(self) -> Iterator[Signal]:
        return iter(self._signals.values())


class IOMap:
    """Connects ``device.signal`` outputs to ``device.signal`` inputs."""

    def __init__(self) -> None:
        self._banks: dict[str, SignalBank] = {}

    def bank(self, device: str) -> SignalBank:
        return self._banks.setdefault(device, SignalBank(owner=device))

    def adopt(self, bank: SignalBank) -> None:
        """Register a pre-built bank (from a device) into the map."""
        if bank.owner in self._banks and self._banks[bank.owner] is not bank:
            raise ValueError(f"Conflicting signal bank for {bank.owner!r}")
        self._banks[bank.owner] = bank

    def link(self, source: str, target: str) -> None:
        """Wire ``source`` -> ``target``. Both are ``device.signal`` paths."""
        src = self.signal(source)
        dst = self.signal(target)
        src.subscribe(dst.set)

    def signal(self, path: str) -> Signal:
        """Look up a signal by its ``device.signal`` path."""
        try:
            device, signal = path.split(".", 1)
        except ValueError as exc:
            raise ValueError(f"Signal path {path!r} must be 'device.signal'") from exc
        try:
            return self._banks[device][signal]
        except KeyError as exc:
            raise KeyError(f"Unknown signal {path!r}") from exc
