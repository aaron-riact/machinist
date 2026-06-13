"""Generic pneumatic gripper, controlled purely by digital IO.

Mirrors a plant-floor convention used by countless real grippers wired
into a PLC: two output coils (open / close) drive the air solenoids and
two input coils report the limit switches (fully open / fully closed).

The emulator wires up a small state machine: when ``cmd_open`` goes high
it transitions to "fully open" after a configurable settle time; same for
``cmd_close``. If both are high (or both low) it parks in transit.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.registry import register
from ...core.types import Endpoint


@dataclass(frozen=True, slots=True)
class PneumaticGripperOptions:
    settle_seconds: float = 0.3


class PneumaticGripper(Device):
    kind = "pneumatic_gripper"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: PneumaticGripperOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._settings = options
        self.io = SignalBank(owner=name)
        self._cmd_open = self.io.declare("cmd_open", Direction.INPUT)
        self._cmd_close = self.io.declare("cmd_close", Direction.INPUT)
        self._is_open = self.io.declare("is_open", Direction.OUTPUT)
        self._is_closed = self.io.declare("is_closed", Direction.OUTPUT)
        self._cmd_open.subscribe(lambda _: self._react())
        self._cmd_close.subscribe(lambda _: self._react())
        self._timer: threading.Timer | None = None

    def _react(self) -> None:
        # Cancel any in-flight settling.
        if self._timer is not None:
            self._timer.cancel()
        opening = self._cmd_open.value and not self._cmd_close.value
        closing = self._cmd_close.value and not self._cmd_open.value
        # Always transit through "neither limit reached" first.
        self._is_open.set(False)
        self._is_closed.set(False)
        if not (opening or closing):
            self.emit("idle")
            return
        target = "open" if opening else "closed"
        self.emit("moving", target=target)
        timer = threading.Timer(self._settings.settle_seconds, self._settle, args=(target,))
        timer.daemon = True
        self._timer = timer
        timer.start()

    def _settle(self, target: str) -> None:
        (self._is_open if target == "open" else self._is_closed).set(True)
        self.emit("settled", target=target)

    def _run(self, stop: threading.Event) -> None:
        # IO-only device: just block until shutdown.
        self._mark_running()
        stop.wait()
        if self._timer is not None:
            self._timer.cancel()


@register("pneumatic_gripper", default_port=0)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    return PneumaticGripper(name, endpoint, bus, PneumaticGripperOptions(**options))
