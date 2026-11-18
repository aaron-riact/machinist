"""Vacuum cup, controlled purely by digital IO.

The plant-floor wiring: one output switches the vacuum (an ejector or a
valve), an optional second blows air through the cup to drop the part, and
one input reports "vacuum OK" from the cup's pressure switch.

``vacuum_ok`` comes on a settle time after the vacuum is switched on with a
part on the cup and no blow-off, and goes off at once when any of those
stops. A ``part`` input says a part is on the cup; its start value is the
``part`` option.
"""

from __future__ import annotations

import threading

from ...core.capabilities import HasIO
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.options import Options
from ...core.registry import register
from ...core.types import Endpoint


class VacuumCupOptions(Options):
    settle_seconds: float = 0.2
    #: Whether a part is on the cup at power-up (the ``part`` input).
    part: bool = False


class VacuumCup(Device, HasIO):
    kind = "vacuum_cup"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: VacuumCupOptions,
        *, io: SignalBank,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._settings = options
        self.io = io
        self._vacuum_on = self.io.declare("vacuum_on", Direction.INPUT)
        self._blow_off = self.io.declare("blow_off", Direction.INPUT)
        self._part = self.io.declare("part", Direction.INPUT)
        self._vacuum_ok = self.io.declare("vacuum_ok", Direction.OUTPUT)
        self._part.set(options.part)
        self._timer: threading.Timer | None = None
        self._lock = threading.Lock()
        for signal in (self._vacuum_on, self._blow_off, self._part):
            signal.subscribe(lambda _: self._react())

    def _holding(self) -> bool:
        return self._vacuum_on.value and self._part.value and not self._blow_off.value

    def _react(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None
            if not self._holding():
                if self._vacuum_ok.value:
                    self.emit("lost")
                self._vacuum_ok.set(False)
                return
            if self._vacuum_ok.value:
                return
            timer = threading.Timer(self._settings.settle_seconds, self._settle)
            timer.daemon = True
            self._timer = timer
            timer.start()

    def _settle(self) -> None:
        with self._lock:
            if self._holding():
                self._vacuum_ok.set(True)
                self.emit("holding")

    def _shutdown(self) -> None:
        if self._timer is not None:
            self._timer.cancel()


@register("vacuum_cup", default_port=0, options=VacuumCupOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, opts: VacuumCupOptions) -> Device:
    return VacuumCup(name, endpoint, bus, opts, io=SignalBank(owner=name, publish=bus.publish))
