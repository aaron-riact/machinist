"""A laser distance switch: an IO device that switches outputs by distance.

It shines a beam and switches each of its outputs on while a surface is closer than
that output's switch point. It knows nothing of where it is: the scene works out the
distance along its beam (see :mod:`machinist.core.environment`) and hands it over.

Signals:

* ``on`` (input): the laser is switched on.
* ``ok`` (output): the laser is on and ready, which a cell waits for before measuring.
* one output per switch point, named in the options (``q1``, ``q2`` ...).

A switch point may have a hysteresis: the output goes on closer than ``on_within_mm``
and off again only beyond ``off_beyond_mm``, as a real sensor does to avoid chatter.
"""

from __future__ import annotations

from typing import Any

from pydantic import model_validator

from ...core.capabilities import HasIO, SensesDistance
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.options import Options
from ...core.registry import register
from ...core.types import Endpoint


class SwitchPoint(Options):
    """Where one output switches: on closer than on_within_mm, off beyond off_beyond_mm."""

    on_within_mm: float
    #: Unset: the same as on_within_mm, no hysteresis.
    off_beyond_mm: float | None = None

    @model_validator(mode="before")
    @classmethod
    def _a_number_is_the_switch_point(cls, raw: Any) -> Any:
        return {"on_within_mm": raw} if isinstance(raw, (int, float)) else raw

    @model_validator(mode="after")
    def _off_not_closer_than_on(self) -> SwitchPoint:
        if self.on_within_mm <= 0:
            raise ValueError(f"on_within_mm {self.on_within_mm} is not positive")
        if self.off_beyond_mm is not None and self.off_beyond_mm < self.on_within_mm:
            raise ValueError(
                f"off_beyond_mm {self.off_beyond_mm} is closer than on_within_mm "
                f"{self.on_within_mm}")
        return self

    @property
    def off_beyond(self) -> float:
        return self.on_within_mm if self.off_beyond_mm is None else self.off_beyond_mm


class LaserOptions(Options):
    #: Each output by name and where it switches: ``{q1: 180}`` or
    #: ``{q1: {on_within_mm: 175, off_beyond_mm: 200}}``.
    switches: dict[str, SwitchPoint]

    @model_validator(mode="after")
    def _outputs_have_their_own_names(self) -> LaserOptions:
        if not self.switches:
            raise ValueError("switches is empty; the laser would switch nothing")
        for name in self.switches:
            if name in ("on", "ok"):
                raise ValueError(f"switch {name!r} takes the name of the laser's own signal")
        return self


class Laser(Device, HasIO, SensesDistance):
    kind = "laser"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: LaserOptions,
        *, io: SignalBank,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.io = io
        self._switches = options.switches
        self._on = self.io.declare("on", Direction.INPUT)
        self._ok = self.io.declare("ok", Direction.OUTPUT)
        self._outputs = {name: self.io.declare(name, Direction.OUTPUT)
                         for name in options.switches}
        self._distance_mm: float | None = None
        self._on.subscribe(lambda _: self._update())

    def sense_distance(self, distance_m: float | None) -> None:
        self._distance_mm = None if distance_m is None else distance_m * 1000
        self._update()

    def _update(self) -> None:
        on = self._on.value
        self._ok.set(on)
        for name, point in self._switches.items():
            output = self._outputs[name]
            if not on or self._distance_mm is None:
                output.set(False)
            elif self._distance_mm < point.on_within_mm:
                output.set(True)
            elif self._distance_mm > point.off_beyond:
                output.set(False)

    def settings(self) -> list[str]:
        return [f"{name} on within {point.on_within_mm:g} mm"
                for name, point in self._switches.items()]


@register("laser", default_port=0, options=LaserOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: LaserOptions) -> Device:
    return Laser(name, endpoint, bus, options, io=SignalBank(owner=name, publish=bus.publish))
