"""A Yaskawa arm and controller as a machinist device: its options, servers and kinds."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Literal

from pydantic import Field, model_validator

from ....core.device import Device
from ....core.events import EventBus
from ....core.registry import register
from ....core.types import Endpoint
from ....transport.framing import CR
from ....transport.line_server import LineServer, Reply, SessionHandler
from ....transport.message import UdpMessageServer
from ..arm import ArmOptions, HasArm, arm_from_options
from .controller import CONTROLLERS, ROBOTS, YaskawaController
from .ethernet_server import EthernetServer, _Session
from .hses import HighSpeedEthernetServer

MOTOMAN_PORT = 80
#: Where the High-Speed Ethernet Server (HSES) listens, over UDP.
HSES_PORT = 10040


Protocol = Literal["ethernet_server", "hses"]


class YaskawaOptions(ArmOptions):
    """The ``yaskawa`` block: the arm, its controller, and the protocols it serves."""

    #: Which controller generation answers: nx100, dx200 or yrc1000.
    controller: Literal["nx100", "dx200", "yrc1000"]
    #: The arm, from :data:`ROBOTS`. It supplies urdf, pulses_per_degree and
    #: max_joint_speed_deg_s wherever the scene does not give them.
    robot: str | None = None
    #: What the controller serves: the text Ethernet server on the device's
    #: port (TCP 80 by default), and HSES on hses_port (UDP).
    protocols: tuple[Protocol, ...] = ("ethernet_server",)
    hses_port: int = HSES_PORT

    #: Encoder pulses per degree of each joint, in joint order.
    pulses_per_degree: tuple[float, ...]
    #: Each joint's top speed in degrees per second, which 100 % of a joint
    #: move's speed means.
    max_joint_speed_deg_s: tuple[float, ...]

    #: What the cell's ladder copies: each network-input byte to another
    #: byte, as ``{22010: 30040}`` mirrors #22010-#22017 on #30040-#30047.
    #: Signal numbers name the first signal of each byte.
    ladder_copies: dict[int, int] = Field(default_factory=dict)

    @model_validator(mode="before")
    @classmethod
    def _fill_in_the_robot(cls, raw: Any) -> Any:
        if not isinstance(raw, dict):
            return raw
        name = raw.get("robot", cls.model_fields["robot"].default)
        if name is None:
            return raw
        if name not in ROBOTS:
            raise ValueError(f"unknown robot {name!r}; known: {', '.join(sorted(ROBOTS))}")
        arm = ROBOTS[name]
        return {
            "urdf": str(arm.urdf),
            "pulses_per_degree": arm.pulses_per_degree,
            "max_joint_speed_deg_s": arm.max_joint_speed_deg_s,
            **raw,
        }

    @model_validator(mode="after")
    def _protocols_the_controller_has(self) -> YaskawaOptions:
        if not self.protocols:
            raise ValueError("protocols is empty; the controller would serve nothing")
        if len(set(self.protocols)) != len(self.protocols):
            raise ValueError(f"protocols lists one twice: {list(self.protocols)}")
        if "hses" in self.protocols and self.controller == "nx100":
            raise ValueError("the NX100 has no HSES")
        return self

    @model_validator(mode="after")
    def _ladder_copies_name_bytes(self) -> YaskawaOptions:
        network_inputs = CONTROLLERS[self.controller].network_inputs
        for source, target in self.ladder_copies.items():
            if source not in network_inputs:
                raise ValueError(f"ladder copies from #{source}, not a network input")
            for signal in (source, target):
                if signal % 10:
                    raise ValueError(f"#{signal} is not the first signal of a byte")
        return self

    @model_validator(mode="after")
    def _one_value_per_joint(self) -> YaskawaOptions:
        for name in ("pulses_per_degree", "max_joint_speed_deg_s"):
            values = getattr(self, name)
            if len(values) != self.joint_count:
                raise ValueError(f"{len(values)} {name} for {self.joint_count} joints")
        return self


class MotomanNX100Options(YaskawaOptions):
    """The ``motoman_nx100`` block: a ``yaskawa`` block for an NX100 with a UP50N."""

    controller: Literal["nx100", "dx200", "yrc1000"] = "nx100"
    robot: str | None = "up50n"


class YaskawaRobot(Device, HasArm):
    """A Yaskawa arm and controller, serving the protocols its options name."""

    kind = "yaskawa"
    DEFAULT_PORT = MOTOMAN_PORT

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: YaskawaOptions
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.arm = arm_from_options(options, name=name, publish=self.publish)
        self.add_service(self.arm)
        self.protocols = options.protocols
        self.hses_port = options.hses_port
        self.controller = YaskawaController(
            arm=self.arm,
            profile=CONTROLLERS[options.controller],
            pulses_per_degree=options.pulses_per_degree,
            max_joint_speed_deg_s=options.max_joint_speed_deg_s,
            ladder_copies=dict(options.ladder_copies),
        )
        if "ethernet_server" in self.protocols:
            self.add_service(LineServer(
                endpoint.host, endpoint.port, session_factory=self.make_session, framer=CR,
            ))
        if "hses" in self.protocols:
            hses = HighSpeedEthernetServer(self.controller)
            self.add_service(UdpMessageServer(endpoint.host, self.hses_port, hses.handle))

    def make_session(self) -> SessionHandler:
        return _TracingSession(self, _Session(server=EthernetServer(self.controller)))


@dataclass(slots=True)
class _TracingSession:
    """Thin wrapper that surfaces rx/tx events on the bus."""

    device: Device
    inner: SessionHandler

    def handle(self, message: str) -> Reply:
        self.device.emit("rx", line=message)
        reply = self.inner.handle(message)
        if reply is not None:
            self.device.emit("tx", reply=reply if isinstance(reply, str) else list(reply))
        return reply


@register("yaskawa", default_port=MOTOMAN_PORT, options=YaskawaOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: YaskawaOptions) -> Device:
    return YaskawaRobot(name, endpoint, bus, options)


@register("motoman_nx100", default_port=MOTOMAN_PORT, options=MotomanNX100Options)
def _nx100_factory(
    name: str, endpoint: Endpoint, bus: EventBus, options: MotomanNX100Options
) -> Device:
    return YaskawaRobot(name, endpoint, bus, options)
