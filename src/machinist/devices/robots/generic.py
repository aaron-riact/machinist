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
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...kinematics.api import DHParams, KinematicsOptions
from ...core.types import Endpoint
from ...srci import SrciServer
from ...transport.message import FrameHandler, MessageServer, open_server
from .arm import ArmOptions, RobotArm, arm_from_options, arm_readers

if TYPE_CHECKING:
    from ...transport.opcua_server import OpcUaServer

#: Build a frame handler that drives an arm for a named protocol.
ProtocolFactory = Callable[[RobotArm], FrameHandler]

_PROTOCOLS: dict[str, ProtocolFactory] = {
    "srci": lambda arm: SrciServer(arm).handle,
}


def protocols() -> tuple[str, ...]:
    """Names of robot protocols this device can serve."""
    return tuple(_PROTOCOLS)


@dataclass(frozen=True, slots=True)
class OpcUaClientOptions:
    port: int = 4840


@dataclass(frozen=True, slots=True)
class RobotDeviceOptions:
    joint_count: int = 6
    kinematics: dict[str, Any] | None = None
    backend: str | None = None
    dh_params: dict[str, list[float]] | None = None
    urdf: str | None = None
    protocol: str = "srci"
    transport: str = "tcp"
    opcua: OpcUaClientOptions | None = None


class RobotDevice(Device):
    """A robot arm served over a configurable protocol + transport."""

    kind = "robot"
    DEFAULT_PORT = 15001

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: RobotDeviceOptions,
        *, arm: RobotArm, server: MessageServer, opcua: OpcUaServer | None = None,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm
        self._server = server
        self._opcua = opcua
        protocol = options.protocol
        try:
            factory = _PROTOCOLS[protocol]
        except KeyError:
            raise ValueError(
                f"unknown robot protocol {protocol!r}; have {protocols()}"
            ) from None
        self._handler = factory(self.arm)

    def _run(self, stop: threading.Event) -> None:
        self.arm.start_ticker()
        ready = threading.Event()
        thread = threading.Thread(
            target=self._server.serve_forever, args=(self._dispatch, ready), daemon=True
        )
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        opcua_thread = self._start_opcua()
        self._mark_running()
        stop.wait()
        self._server.shutdown()
        thread.join(timeout=2.0)
        if self._opcua is not None:
            self._opcua.shutdown()
        if opcua_thread is not None:
            opcua_thread.join(timeout=2.0)

    def _start_opcua(self) -> threading.Thread | None:
        if self._opcua is None:
            return None
        ready = threading.Event()
        thread = threading.Thread(
            target=self._opcua.serve_forever, args=(ready,), daemon=True
        )
        thread.start()
        ready.wait(timeout=5.0)
        self.emit("opcua", state="ready")
        return thread

    def _dispatch(self, frame: bytes) -> bytes:
        self.emit("rx", bytes=len(frame))
        reply = self._handler(frame)
        self.emit("tx", bytes=len(reply))
        return reply

    def _shutdown(self) -> None:
        self._server.shutdown()
        if self._opcua is not None:
            self._opcua.shutdown()
        self.arm.stop_ticker()


def _maybe_opcua(
    name: str, host: str, config: OpcUaClientOptions | None, arm: RobotArm
) -> "OpcUaServer | None":
    """Build an OPC-UA server if the device config opts in, else None."""
    if not config:
        return None
    from ...transport.opcua_server import OpcUaServer  # noqa: PLC0415  (optional dep)

    return OpcUaServer(
        host,
        config.port,
        device_name=name,
        readers=arm_readers(arm),
    )


@register("robot", default_port=15001)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    raw_opcua = opts.pop("opcua", None)
    opcua_opts = OpcUaClientOptions(**raw_opcua) if raw_opcua else None
    opt = RobotDeviceOptions(opcua=opcua_opts, **opts)
    dh = DHParams(**opt.dh_params) if opt.dh_params is not None else None
    kin = KinematicsOptions(**opt.kinematics) if opt.kinematics is not None else None
    arm = arm_from_options(ArmOptions(
        joint_count=opt.joint_count,
        kinematics=kin,
        backend=opt.backend,
        dh_params=dh,
        urdf=opt.urdf,
    ))
    server = open_server(opt.transport, endpoint.host, endpoint.port)
    opcua = _maybe_opcua(name, endpoint.host, opt.opcua, arm)
    return RobotDevice(name, endpoint, bus, opt, arm=arm, server=server, opcua=opcua)
