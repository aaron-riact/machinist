"""Okuma LB3000EX lathe (OSP control), as a robot cell sees it: discrete IO only.

The real machine talks to the robot through Okuma's robot/loader interface,
which is hardwired IO. We do not have its I/O table yet, so this models the
part every robot-fed lathe has: an automatic door and a chuck, each with a
command in and a confirm out per position. Signal names are ours, chosen to
read well in ``io_links``; rename them once the real table is known.

Not modelled yet: cycle start (the OSP runs schedule programs, whose
handshake we do not know), interlocks, alarms.

Each command acts on its rising edge. The actuator drops both confirms,
travels for its configured time, then raises the confirm of the position it
reached. A new command mid-travel replaces the old one.
"""

from __future__ import annotations

import threading
from collections.abc import Callable

from ...core.capabilities import HasIO
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, Signal, SignalBank
from ...core.options import Options
from ...core.registry import register
from ...core.types import Endpoint
from .state import HasMachineState, MachineState

DOOR = "main"
CHUCK = "main"


class OkumaLB3000EXOptions(Options):
    door_open_seconds: float = 3.0
    door_close_seconds: float = 3.0
    chuck_unclamp_seconds: float = 0.5
    chuck_clamp_seconds: float = 0.5


def _on_rise(action: Callable[[], None]) -> Callable[[bool], None]:
    """A signal listener that runs *action* when the signal turns on."""

    def listener(value: bool) -> None:
        if value:
            action()

    return listener


class _TwoPosition:
    """A door or chuck: two confirms, moved by timed travel between them."""

    def __init__(
        self, *, open_confirm: Signal, closed_confirm: Signal,
        open_seconds: float, close_seconds: float,
        reached: Callable[[bool], None],
    ) -> None:
        self._open_confirm = open_confirm
        self._closed_confirm = closed_confirm
        self._seconds = {True: open_seconds, False: close_seconds}
        self._reached = reached
        self._timer: threading.Timer | None = None
        self._lock = threading.Lock()

    def move(self, *, open: bool) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
            self._open_confirm.set(False)
            self._closed_confirm.set(False)
            timer = threading.Timer(self._seconds[open], self._arrive, args=(open,))
            timer.daemon = True
            self._timer = timer
            timer.start()

    def cancel(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None

    def _arrive(self, open: bool) -> None:
        (self._open_confirm if open else self._closed_confirm).set(True)
        self._reached(open)


class OkumaLB3000EX(Device, HasMachineState, HasIO):
    kind = "okuma_lb3000ex"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: OkumaLB3000EXOptions,
        *, io: SignalBank,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.state = MachineState(owner=name, publish=self.publish, doors=(DOOR,), chucks=(CHUCK,))
        self.io = io
        door_open_cmd = io.declare("door_open_cmd", Direction.INPUT)
        door_close_cmd = io.declare("door_close_cmd", Direction.INPUT)
        chuck_unclamp_cmd = io.declare("chuck_unclamp_cmd", Direction.INPUT)
        chuck_clamp_cmd = io.declare("chuck_clamp_cmd", Direction.INPUT)
        self._door = _TwoPosition(
            open_confirm=io.declare("door_open", Direction.OUTPUT),
            closed_confirm=io.declare("door_closed", Direction.OUTPUT),
            open_seconds=options.door_open_seconds,
            close_seconds=options.door_close_seconds,
            reached=self._door_reached,
        )
        self._chuck = _TwoPosition(
            open_confirm=io.declare("chuck_unclamped", Direction.OUTPUT),
            closed_confirm=io.declare("chuck_clamped", Direction.OUTPUT),
            open_seconds=options.chuck_unclamp_seconds,
            close_seconds=options.chuck_clamp_seconds,
            reached=self._chuck_reached,
        )
        # Power-up: door shut, chuck clamped.
        io["door_closed"].set(True)
        io["chuck_clamped"].set(True)
        door_open_cmd.subscribe(_on_rise(lambda: self._door.move(open=True)))
        door_close_cmd.subscribe(_on_rise(lambda: self._door.move(open=False)))
        chuck_unclamp_cmd.subscribe(_on_rise(lambda: self._chuck.move(open=True)))
        chuck_clamp_cmd.subscribe(_on_rise(lambda: self._chuck.move(open=False)))

    def _door_reached(self, open: bool) -> None:
        self.state.set_door(DOOR, open=open)
        self.emit("door", open=open)

    def _chuck_reached(self, open: bool) -> None:
        self.state.set_chuck(CHUCK, open=open)
        self.emit("chuck", open=open)

    def _shutdown(self) -> None:
        self._door.cancel()
        self._chuck.cancel()


@register("okuma_lb3000ex", default_port=0, options=OkumaLB3000EXOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, opts: OkumaLB3000EXOptions) -> Device:
    return OkumaLB3000EX(name, endpoint, bus, opts, io=SignalBank(owner=name, publish=bus.publish))
