"""Yaskawa Motoman NX100 Ethernet-server emulator.

Reference: *NX100 Options Instructions for Ethernet Server Function*
(HW0483202), in the repository as ``NX100_http_network_command.pdf``.

The NX100 serves on TCP port 80 but the protocol is **not** HTTP. A session
runs::

    CONNECT Robot_access[ Keep-Alive:<n>]<CR><LF>
    ← OK: NX Information Server (3.00)[ Keep-Alive:<n>].<CR><LF>
    HOSTCTRL_REQUEST <Command> <Size><CR><LF>
    ← OK: <Command><CR><LF>                 (or NG: <Message><CR><LF>, then close)
    [if Size > 0] <Command data><CR>
    ← <Data><CR>                            (an answer that carries data)
    ← 0000<CR><LF>                          (an answer without data)
    ← ERROR:<Command> is not successful (<code>).<CR><LF>, then close

Without ``Keep-Alive`` the controller answers one command and closes the
socket; with ``Keep-Alive:<n>`` it answers *n* (``-1``: no limit).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from enum import IntEnum, IntFlag

from ...core.device import Device
from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import CR
from ...transport.line_server import Hangup, Reply, SessionHandler
from .arm import ArmOptions, HasArm, RobotArm, arm_from_options

MOTOMAN_PORT = 80
SERVER_NAME = "OK: NX Information Server (3.00)"
#: The reply to a START request the controller cannot accept, before it closes.
START_ERROR = "NG: HTTP Error Response"
NORMAL_ANSWER = "0000"
MAX_KEEP_ALIVE = 32767


class InterpreterMessage(IntEnum):
    """The code in an ``ERROR:<Command> is not successful (<code>).`` answer.

    The manual leaves the list to the Data Transmission manual (RE-CKI-A445),
    which is not at hand, so these codes are stand-ins that only name why a
    command failed.
    """

    BAD_DATA = 9001


class Status1(IntFlag):
    """RSTATS Data-1: mode, cycle and operation."""

    STEP = 0x01
    ONE_CYCLE = 0x02
    AUTO = 0x04
    RUNNING = 0x08
    SAFETY_SPEED = 0x10
    TEACH = 0x20
    PLAY = 0x40
    COMMAND_REMOTE = 0x80


class Status2(IntFlag):
    """RSTATS Data-2: holds, alarm, error and servo."""

    HOLD_PENDANT = 0x02
    HOLD_EXTERNAL = 0x04
    HOLD_COMMAND = 0x08
    ALARM = 0x10
    ERROR = 0x20
    SERVO_ON = 0x40


class CommandError(Exception):
    """A command the controller refuses with an ERROR answer."""

    def __init__(self, code: InterpreterMessage, reason: str) -> None:
        super().__init__(reason)
        self.code = code


Command = Callable[[str], str | None]


@dataclass(slots=True)
class NX100Controller:
    """The controller state every session shares, and the commands that act on it."""

    arm: RobotArm
    cycle: Status1 = Status1.ONE_CYCLE

    def command(self, name: str) -> Command | None:
        """The command called *name*, or None for one the controller does not know.

        A command takes the command data and returns the answer's data, or
        None for an answer without data. It raises :class:`CommandError` when
        the controller refuses it.
        """
        commands: dict[str, Command] = {
            "RSTATS": self._rstats,
        }
        return commands.get(name)

    def _rstats(self, data: str) -> str:
        view = self.arm.state.view
        data1 = Status1.COMMAND_REMOTE | Status1.PLAY | self.cycle
        if view.moving:
            data1 |= Status1.RUNNING
        data2 = Status2(0)
        if view.servo_on and not view.estopped:
            data2 |= Status2.SERVO_ON
        if view.faulted:
            data2 |= Status2.ALARM
        return f"{int(data1)},{int(data2)}"


@dataclass(frozen=True, slots=True)
class _Request:
    name: str
    size: int
    run: Command


@dataclass(slots=True)
class _Session:
    """One connection: the START request, then up to Keep-Alive commands."""

    controller: NX100Controller
    #: Commands this connection may run; None until the START request.
    keep_alive: int | None = None
    answered: int = 0
    pending: _Request | None = None

    def handle(self, message: str) -> Reply:
        if self.keep_alive is None:
            return self._start(message)
        if self.pending is not None:
            request, self.pending = self.pending, None
            return self._answer(request, message)
        return self._request(message)

    # --- START request ------------------------------------------------

    def _start(self, message: str) -> Reply:
        words = message.split()
        if words[:2] != ["CONNECT", "Robot_access"] or len(words) > 3:
            return Hangup([START_ERROR + "\r\n"])
        if len(words) == 2:
            self.keep_alive = 1
            return SERVER_NAME + ".\r\n"
        keep_alive = _parse_keep_alive(words[2])
        if keep_alive is None:
            return Hangup([START_ERROR + "\r\n"])
        self.keep_alive = keep_alive
        return f"{SERVER_NAME} Keep-Alive:{keep_alive}.\r\n"

    # --- command ------------------------------------------------------

    def _request(self, message: str) -> Reply:
        words = message.split()
        if len(words) != 3 or words[0] != "HOSTCTRL_REQUEST" or not words[2].isdigit():
            return Hangup([f"NG: bad request {message!r}\r\n"])
        name, size = words[1], int(words[2])
        run = self.controller.command(name)
        if run is None:
            return Hangup([f"NG: unknown command {name}\r\n"])
        request = _Request(name=name, size=size, run=run)
        ok = f"OK: {name}\r\n"
        if request.size > 0:
            self.pending = request
            return ok
        answer = self._answer(request, "")
        lines = [ok, *answer]
        return Hangup(lines) if isinstance(answer, Hangup) else lines

    def _answer(self, request: _Request, data: str) -> list[str] | Hangup:
        sent = len(data.encode()) + 1  # the size counts the closing CR
        if request.size > 0 and sent != request.size:
            return Hangup([f"NG: size {request.size} for {sent} bytes\r\n"])
        try:
            answer = request.run(data)
        except CommandError as exc:
            return Hangup([f"ERROR:{request.name} is not successful ({exc.code:04d}).\r\n"])
        reply = NORMAL_ANSWER + "\r\n" if answer is None else answer + "\r"
        self.answered += 1
        if self.keep_alive != -1 and self.answered >= (self.keep_alive or 1):
            return Hangup([reply])
        return [reply]


def _parse_keep_alive(word: str) -> int | None:
    """``Keep-Alive:<n>`` (the manual's example ends it with a dot) → n, or None."""
    name, _, value = word.removesuffix(".").partition(":")
    if name != "Keep-Alive":
        return None
    try:
        n = int(value)
    except ValueError:
        return None
    return n if n == -1 or 2 <= n <= MAX_KEEP_ALIVE else None


class MotomanNX100(LineServerDevice, HasArm):
    kind = "motoman_nx100"
    DEFAULT_PORT = MOTOMAN_PORT
    FRAMER = CR

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: ArmOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options, name=name, publish=self.publish)
        self.add_service(self.arm)
        self.controller = NX100Controller(arm=self.arm)

    def make_session(self) -> SessionHandler:
        return _TracingSession(self, _Session(controller=self.controller))


@dataclass(slots=True)
class _TracingSession:
    """Thin wrapper that surfaces rx/tx events on the bus."""

    device: LineServerDevice
    inner: SessionHandler

    def handle(self, message: str) -> Reply:
        self.device.emit("rx", line=message)
        reply = self.inner.handle(message)
        if reply is not None:
            self.device.emit("tx", reply=reply if isinstance(reply, str) else list(reply))
        return reply


@register("motoman_nx100", default_port=MOTOMAN_PORT, options=ArmOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: ArmOptions) -> Device:
    return MotomanNX100(name, endpoint, bus, options)
