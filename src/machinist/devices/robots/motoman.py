"""Yaskawa Motoman NX100/DX100 Ethernet-server emulator.

Reference: *NX100 HTTP/Telnet Network Command Guide*.

The NX100 serves on TCP port 80 but the protocol is **not** HTTP.  The
session opens with::

    CONNECT Robot_access[ Keep-Alive:<n>]<CR><LF>
    ← OK: NX Information Server(Ver 1.10).<CR><LF>

Subsequent commands are framed as::

    HOSTCTRL_REQUEST <Command> <Size><CR><LF>
    ← OK: <Command><CR><LF>      (or  NG: <Message>)
    [if Size > 0] <Command data ending with CR>
    ← <answer ending with CRLF>

We model this with a stateful :class:`_Session` so CONNECT is a hard
gate before any verb runs — exactly what a real NX100 does.
"""

from __future__ import annotations

from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any, cast

from ...core.events import EventBus
from ...core.line_device import LineServerDevice
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import DHParams, Joints, KinematicsOptions, Pose
from ...transport.framing import CRLF
from ...transport.line_server import Reply, SessionHandler
from .arm import ArmOptions, ArmMode, RobotArm, arm_from_options

MOTOMAN_PORT = 80
SERVER_BANNER = "OK: NX Information Server(Ver 1.10)."


@dataclass(slots=True)
class _Session:
    """Per-connection Motoman handshake + command dispatcher."""

    arm: RobotArm
    connected: bool = False
    pending_cmd: str | None = None  # set after HOSTCTRL_REQUEST with Size>0
    _keep_alive: int | None = None

    # Public: SessionHandler.handle
    def handle(self, message: str) -> Reply:
        if self.pending_cmd is not None:
            cmd, self.pending_cmd = self.pending_cmd, None
            return self._answer(cmd, message.rstrip("\r"))

        if not self.connected:
            return self._handle_connect(message)

        if message.startswith("HOSTCTRL_REQUEST"):
            return self._handle_request(message)

        return "NG: not connected"

    # --- handshake ---------------------------------------------------

    def _handle_connect(self, message: str) -> str:
        head, _, rest = message.partition(" ")
        if head.upper() != "CONNECT" or not rest.startswith("Robot_access"):
            return "NG: bad CONNECT"
        # Optional Keep-Alive:<n>
        tail = rest[len("Robot_access"):].strip()
        if tail.startswith("Keep-Alive:"):
            try:
                self._keep_alive = int(tail.split(":", 1)[1])
            except ValueError:
                return "NG: bad keep-alive"
            self.connected = True
            return f"{SERVER_BANNER} Keep-Alive:{self._keep_alive}."
        self.connected = True
        return SERVER_BANNER

    # --- HOSTCTRL_REQUEST -------------------------------------------

    def _handle_request(self, message: str) -> Reply:
        parts = message.split()
        if len(parts) < 3:
            return "NG: bad request"
        _, cmd, size_s = parts[0], parts[1].upper(), parts[-1]
        try:
            size = int(size_s)
        except ValueError:
            return "NG: bad size"
        if size == 0:
            return [f"OK: {cmd}", self._answer(cmd, "")]
        self.pending_cmd = cmd
        return f"OK: {cmd}"

    # --- answer -----------------------------------------------------

    def _answer(self, cmd: str, data: str) -> str:
        arm, s = self.arm, self.arm.state.snapshot()
        match cmd:
            case "RPOSJ":
                return ",".join(f"{j:.4f}" for j in s.joints)
            case "RPOSC":
                return ",".join(f"{p:.4f}" for p in s.pose)
            case "RSTATS":
                return _state_word(s.mode)
            case "SVON":
                arm.set_servo(True); return "0000"
            case "SVOFF":
                arm.set_servo(False); return "0000"
            case "HOLD" | "CANCEL":
                arm.estop(); return "0000"
            case "RESET":
                arm.reset(); return "0000"
            case "MOVJ":
                arm.movej(cast(Joints, tuple(_parse_floats(data, count=len(s.joints)))))
                return "0000"
            case "MOVL":
                arm.movel(cast(Pose, tuple(_parse_floats(data, count=6))))
                return "0000"
            case _:
                return "ERROR:E2010"


class MotomanNX100(LineServerDevice):
    kind = "motoman_nx100"
    DEFAULT_PORT = MOTOMAN_PORT
    FRAMER = CRLF

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: ArmOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options)
        self.arm.start_ticker()

    def make_session(self) -> SessionHandler:
        return _TracingSession(self, _Session(arm=self.arm))

    def _shutdown(self) -> None:
        super()._shutdown()
        self.arm.stop_ticker()


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


def _parse_floats(text: str, *, count: int) -> list[float]:
    parts = [p for p in text.replace(",", " ").split() if p]
    if len(parts) != count:
        raise ValueError(f"expected {count} floats, got {len(parts)}")
    return [float(p) for p in parts]


def _state_word(mode: ArmMode) -> str:
    return {
        ArmMode.IDLE: "READY",
        ArmMode.MOVING: "RUNNING",
        ArmMode.ESTOPPED: "ESTOP",
        ArmMode.FAULTED: "ALARM",
    }[mode]


@register("motoman_nx100", default_port=MOTOMAN_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]):
    raw = dict(options)
    dh = DHParams(**raw.pop("dh_params")) if "dh_params" in raw else None
    kin = KinematicsOptions(**raw.pop("kinematics")) if "kinematics" in raw else None
    return MotomanNX100(name, endpoint, bus, ArmOptions(kinematics=kin, dh_params=dh, **raw))
