"""Okuma LB3000EX lathe (OSP control), as a robot cell sees it: discrete IO only.

The machine talks to the robot through Okuma's robot/loader interface, which
is hardwired IO. This models its Type-B form (OSP manual, section 38): the
NC calls the robot with M-codes, and the robot answers.

- An automatic door and a chuck, each with a command in and a confirm out per
  position. Each command acts on its rising edge. The actuator drops both
  confirms, travels for its configured time, then raises the confirm of the
  position it reached. A new command mid-travel replaces the old one.
- The part program the schedule program loops: it machines for
  ``machining_seconds``, its spindle turning only with the door closed, then
  calls the robot with its ``robot_call`` M-code (M180-M183 raise Request 1-4)
  and waits. The robot's request answer (rising edge) ends the call; the part
  program ends and the schedule program starts it again.
- Requests run only in link mode (``link``); out of it the NC runs no cycle.
- Status outputs: in link, spindle stopped, NC alarm (no alarm is modelled;
  ``nc_alarm_active_low`` sets how the line reads with none).

Signal names are ours, chosen to read well in ``io_links``.

Not modelled: spindle orientation, tailstock, air blow, cycle stop, the door
interlock stopping a cycle under way, alarms.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Literal

from ...core.capabilities import HasIO
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, Signal, SignalBank
from ...core.options import Options
from ...core.registry import register
from ...core.types import Endpoint
from .state import CycleState, HasMachineState, MachineState

DOOR = "main"
CHUCK = "main"


#: The robot/loader request M-codes: M180-M183 raise Request 1-4.
REQUESTS = {180: "request1", 181: "request2", 182: "request3", 183: "request4"}


class OkumaLB3000EXOptions(Options):
    door_open_seconds: float = 3.0
    door_close_seconds: float = 3.0
    chuck_unclamp_seconds: float = 0.5
    chuck_clamp_seconds: float = 0.5
    #: How long the part program machines before it calls the robot.
    machining_seconds: float = 10.0
    #: The M-code the part program calls the robot with.
    robot_call: Literal[180, 181, 182, 183] = 180
    #: At power-up the part program waits in its robot call, so the robot serves it first.
    robot_call_at_power_up: bool = True
    #: Link mode: the NC runs its schedule for the robot only in it.
    link: bool = True
    #: The NC alarm output is on while there is no alarm.
    nc_alarm_active_low: bool = False


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


class _PartProgram:
    """The part program the schedule program loops: machine, then call the robot
    and wait for its answer."""

    def __init__(
        self, *, request: Signal, spindle_stopped: Signal, door_closed: Signal,
        machining_seconds: float, state: MachineState, emit: Callable[..., None],
    ) -> None:
        self._request = request
        self._spindle_stopped = spindle_stopped
        self._door_closed = door_closed
        self._machining_seconds = machining_seconds
        self._state = state
        self._emit = emit
        self._calling = False
        self._waiting_for_door = False
        self._timer: threading.Timer | None = None
        self._lock = threading.RLock()

    def call_robot(self) -> None:
        with self._lock:
            self._timer = None
            self._calling = True
            self._spindle_stopped.set(True)
            self._state.update(cycle=CycleState.IDLE)
            self._request.set(True)
            self._emit("robot_call", request=self._request.name)

    def start(self) -> None:
        """Start the next run, as soon as the door is closed."""
        with self._lock:
            self._waiting_for_door = True
            self._run_if_door_closed()

    def answer(self) -> None:
        with self._lock:
            if not self._calling:
                return
            self._calling = False
            self._request.set(False)
            self._emit("answered", request=self._request.name)
            self.start()

    def door_closed(self) -> None:
        with self._lock:
            self._run_if_door_closed()

    def cancel(self) -> None:
        with self._lock:
            if self._timer is not None:
                self._timer.cancel()
                self._timer = None

    def _run_if_door_closed(self) -> None:
        if not (self._waiting_for_door and self._door_closed.value):
            return
        self._waiting_for_door = False
        self._spindle_stopped.set(False)
        self._state.update(cycle=CycleState.RUNNING)
        self._emit("machining", seconds=self._machining_seconds)
        timer = threading.Timer(self._machining_seconds, self._machined)
        timer.daemon = True
        self._timer = timer
        timer.start()

    def _machined(self) -> None:
        with self._lock:
            self._state.bump(parts=1)
            self.call_robot()


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
        requests = {code: io.declare(name, Direction.OUTPUT) for code, name in REQUESTS.items()}
        request_answer = io.declare("request_answer", Direction.INPUT)
        in_link = io.declare("in_link", Direction.OUTPUT)
        nc_alarm = io.declare("nc_alarm", Direction.OUTPUT)
        spindle_stopped = io.declare("spindle_stopped", Direction.OUTPUT)
        self._program = _PartProgram(
            request=requests[options.robot_call],
            spindle_stopped=spindle_stopped,
            door_closed=io["door_closed"],
            machining_seconds=options.machining_seconds,
            state=self.state,
            emit=self.emit,
        )
        # Power-up: door shut, chuck clamped, spindle stopped, no alarm.
        io["door_closed"].set(True)
        io["chuck_clamped"].set(True)
        spindle_stopped.set(True)
        nc_alarm.set(options.nc_alarm_active_low)
        in_link.set(options.link)
        if options.link:
            if options.robot_call_at_power_up:
                self._program.call_robot()
            else:
                self._program.start()
            request_answer.subscribe(_on_rise(self._program.answer))
            io["door_closed"].subscribe(_on_rise(self._program.door_closed))
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
        self._program.cancel()


@register("okuma_lb3000ex", default_port=0, options=OkumaLB3000EXOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, opts: OkumaLB3000EXOptions) -> Device:
    return OkumaLB3000EX(name, endpoint, bus, opts, io=SignalBank(owner=name, publish=bus.publish))
