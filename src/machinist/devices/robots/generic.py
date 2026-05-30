"""A generic, protocol-driven robot device.

Most of our robots are a vendor wire protocol bolted onto the same
:class:`RobotArm` physics. This device inverts that: you give it a
kinematic model (``joint_count`` + optional ``kinematics`` URDF/DH) and
name a ``protocol`` plus a ``transport``, and it serves that protocol
over that transport — no vendor module required.

Today the only registered protocol is SRCI, but the seam is a plain
``arm -> FrameHandler`` factory, so new telegram protocols drop in
without touching this device.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...srci import SrciServer
from ...transport.message import FrameHandler, open_server
from .arm import RobotArm, arm_from_options

#: Build a frame handler that drives an arm for a named protocol.
ProtocolFactory = Callable[[RobotArm], FrameHandler]

_PROTOCOLS: dict[str, ProtocolFactory] = {
    "srci": lambda arm: SrciServer(arm).handle,
}


def protocols() -> tuple[str, ...]:
    """Names of robot protocols this device can serve."""
    return tuple(_PROTOCOLS)


@register("robot", default_port=15001)
class RobotDevice(Device):
    """A robot arm served over a configurable protocol + transport."""

    kind = "robot"
    DEFAULT_PORT = 15001

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]
    ) -> None:
        super().__init__(name, endpoint, bus)
        protocol = str(options.get("protocol", "srci"))
        transport = str(options.get("transport", "tcp"))
        try:
            factory = _PROTOCOLS[protocol]
        except KeyError:
            raise ValueError(
                f"unknown robot protocol {protocol!r}; have {protocols()}"
            ) from None
        self.arm = arm_from_options(options)
        self._handler = factory(self.arm)
        self._server = open_server(transport, endpoint.host, endpoint.port)

    def _run(self, stop: threading.Event) -> None:
        self.arm.start_ticker()
        ready = threading.Event()
        thread = threading.Thread(
            target=self._server.serve_forever, args=(self._dispatch, ready), daemon=True
        )
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        stop.wait()
        self._server.shutdown()
        thread.join(timeout=2.0)

    def _dispatch(self, frame: bytes) -> bytes:
        self.emit("rx", bytes=len(frame))
        reply = self._handler(frame)
        self.emit("tx", bytes=len(reply))
        return reply

    def _shutdown(self) -> None:
        self._server.shutdown()
        self.arm.stop_ticker()
