"""Client-side SRCI over a pluggable transport.

:class:`SrciClient` is a small, dependency-light wrapper that other
Python projects can lift wholesale to talk to any SRCI robot. It owns
job-id sequencing and the command vocabulary; it knows nothing about
sockets beyond the :class:`~machinist.transport.message.MessageTransport`
seam, so a caller chooses TCP, UDP (or a future Modbus channel) without
the client changing.
"""

from __future__ import annotations

from itertools import count

from ..transport.message import MessageTransport, open_transport
from .codec import CommandTelegram, Function, StatusTelegram

Pose = tuple[float, float, float, float, float, float]


class SrciClient:
    """Issue SRCI commands and read back robot status."""

    def __init__(self, transport: MessageTransport) -> None:
        self._transport = transport
        self._jobs = count(1)

    @classmethod
    def connect(
        cls, host: str, port: int, *, transport: str = "tcp", **kwargs: object
    ) -> SrciClient:
        """Open a client over the named transport (``tcp`` | ``udp``)."""
        return cls(open_transport(transport, host, port, **kwargs))

    # ----- commands ---------------------------------------------------

    def enable(self) -> StatusTelegram:
        return self._command(Function.ENABLE)

    def disable(self) -> StatusTelegram:
        return self._command(Function.DISABLE)

    def estop(self) -> StatusTelegram:
        return self._command(Function.STOP)

    def reset(self) -> StatusTelegram:
        return self._command(Function.RESET)

    def move_joint(self, target: tuple[float, ...], *, speed: float = 1.0) -> StatusTelegram:
        return self._command(Function.MOVE_JOINT, args=target, speed=speed)

    def move_linear(self, pose: Pose, *, speed: float = 1.0) -> StatusTelegram:
        return self._command(Function.MOVE_LINEAR, args=pose, speed=speed)

    def read_status(self) -> StatusTelegram:
        return self._command(Function.READ_STATUS)

    def close(self) -> None:
        self._transport.close()

    def __enter__(self) -> SrciClient:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -----------------------------------------------------------------

    def _command(
        self, function: Function, *, args: tuple[float, ...] = (), speed: float = 1.0
    ) -> StatusTelegram:
        telegram = CommandTelegram(
            job_id=next(self._jobs), function=function, args=args, speed=speed
        )
        reply = self._transport.request(telegram.encode())
        return StatusTelegram.decode(reply)
