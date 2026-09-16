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

from collections.abc import Callable
from dataclasses import dataclass
from typing import TYPE_CHECKING, Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...kinematics.api import DHParams, KinematicsOptions
from ...srci import SrciServer
from ...transport.message import FrameHandler, open_server
from .arm import ArmOptions, HasArm, RobotArm, arm_from_options, arm_readers

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


class RobotDevice(Device, HasArm):
    """A robot arm served over a configurable protocol + transport."""

    kind = "robot"
    DEFAULT_PORT = 15001

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: RobotDeviceOptions,
        *, arm: RobotArm,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm
        self.add_service(self.arm)
        protocol = options.protocol
        try:
            factory = _PROTOCOLS[protocol]
        except KeyError:
            raise ValueError(
                f"unknown robot protocol {protocol!r}; have {protocols()}"
            ) from None
        self._handler = factory(self.arm)

    def dispatch(self, frame: bytes) -> bytes:
        """Answer one request frame, reporting the exchange on the event bus.

        This is the :data:`FrameHandler` the factory hands to the message
        server.
        """
        self.emit("rx", bytes=len(frame))
        reply = self._handler(frame)
        self.emit("tx", bytes=len(reply))
        return reply


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
    device = RobotDevice(name, endpoint, bus, opt, arm=arm)
    device.add_service(open_server(opt.transport, endpoint.host, endpoint.port, device.dispatch))
    opcua = _maybe_opcua(name, endpoint.host, opt.opcua, arm)
    if opcua is not None:
        device.add_service(opcua)
    return device
