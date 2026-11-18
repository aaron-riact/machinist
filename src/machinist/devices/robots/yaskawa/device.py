"""A Yaskawa arm and controller as a machinist device: its options, servers and kinds."""

from __future__ import annotations

from dataclasses import dataclass
from functools import partial
from pathlib import Path
from typing import Any, Literal

from pydantic import Field, field_validator, model_validator

from ....core.capabilities import HasIO
from ....core.device import Device
from ....core.events import EventBus
from ....core.io import Direction, SignalBank
from ....core.registry import register
from ....core.types import Endpoint
from ....transport.framing import CR
from ....transport.line_server import LineServer, Reply, SessionHandler
from ....transport.message import UdpDatagramServer, UdpMessageServer
from ....transport.service import Ticker
from ..arm import ArmOptions, HasArm, arm_from_options
from .backup import ControllerBackup
from .controller import CONTROLLERS, JOB_SUFFIX, ROBOTS, YaskawaController, empty_job
from .ethernet_server import EthernetServer, _Session
from .hses import HighSpeedEthernetServer
from .hses_files import HsesFileServer
from .ladder import Ladder
from .signals import ControllerSignal, SignalKind

MOTOMAN_PORT = 80
#: Where the High-Speed Ethernet Server (HSES) listens, over UDP: robot
#: control, and file control next to it.
HSES_PORT = 10040
HSES_FILE_PORT = 10041
#: The pause between the OK line and the answer, while the controller runs the
#: command. Clients that read the two with separate recvs rely on it.
ES_LINE_GAP_SECONDS = 0.01


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
    hses_file_port: int = HSES_FILE_PORT

    #: Encoder pulses per degree of each joint, in joint order.
    pulses_per_degree: tuple[float, ...]
    #: Each joint's top speed in degrees per second, which 100 % of a joint
    #: move's speed means.
    max_joint_speed_deg_s: tuple[float, ...]
    #: Each joint's soft limits, its lowest and highest pulses: a move whose end lies
    #: past one raises alarm 4980 instead of moving. Unset, none is checked.
    soft_limit_pulses: tuple[tuple[int, int], ...] | None = None

    #: What the cell's ladder copies: each network-input byte to another
    #: byte, as ``{22010: 30040}`` mirrors #22010-#22017 on #30040-#30047.
    #: Signal numbers name the first signal of each byte.
    ladder_copies: dict[int, int] = Field(default_factory=dict)
    #: The cell's other ladder rungs, in the controller's instruction list, run after
    #: the ladder of controller_files' CIOPRG.LST, if it has one:
    #: ``GSTR #10010 GOUT #30010`` copies a byte, ``STR #27012 OUT #30134`` a
    #: signal, ``STR-NOT ...`` inverts it, ``MOV M560 W#30140`` puts a register on
    #: two bytes (see :mod:`.ladder`). They run after ladder_copies.
    ladder: tuple[str, ...] = ()
    #: The names of the jobs the controller holds, each as a NAME.JBI file
    #: that does nothing. A job that does something runs on its own thread
    #: when started, as far as the emulator runs INFORM (see :mod:`.inform`).
    jobs: tuple[str, ...] = ()
    #: A folder of files saved from a real controller (see :mod:`.backup`): its
    #: jobs, TOOL.CND, CIOPRG.LST ..., and the variables in its VAR.DAT. They take
    #: the place of the emulator's own files and of the jobs above of the same name.
    controller_files: Path | None = None
    #: How many move commands may wait behind the one running: a count, or
    #: "unlimited". Unset takes the controller's (the NX100 none, the DX200 and
    #: YRC1000 one).
    move_buffer: int | Literal["unlimited"] | None = None
    #: The NX100's running bit quirk: RSTATS does not show the moves that start
    #: from the buffer. Unset takes the controller's (on for the NX100, off for
    #: the others); each time it hides a move the device notes it.
    running_bit_quirk: bool | None = None
    #: Whether a job is selected (HSES 0x87, JSEQ, SETMJ) while the arm moves. The
    #: YRC1000 manual's interlock table (11.1.1) refuses it (2010), so by default it is.
    job_select_while_moving: bool = False
    #: How often the ladder scans on its own, in ms, besides after every write: a
    #: DX200 scans every 4 ms. Its timers count real time whatever this is; its
    #: counters count scans, so a counter clocked by a relay the ladder turns over
    #: each scan slows as this grows. None: only after writes, as before.
    ladder_scan_ms: float | None = 4.0
    #: The controller's signals other devices wire to (see :mod:`.signals`):
    #: ``in10`` a general input, ``ot9`` a general output, ``rin1`` a direct input.
    signals: tuple[ControllerSignal, ...] = ()

    @field_validator("signals", mode="before")
    @classmethod
    def _signals_by_name(cls, raw: Any) -> Any:
        if isinstance(raw, (list, tuple)):
            return tuple(ControllerSignal.parse(n) if isinstance(n, str) else n for n in raw)
        return raw

    @model_validator(mode="after")
    def _ladder_scans_at_all(self) -> YaskawaOptions:
        if self.ladder_scan_ms is not None and self.ladder_scan_ms <= 0:
            raise ValueError(f"ladder_scan_ms {self.ladder_scan_ms}: not more than 0")
        return self

    @model_validator(mode="after")
    def _each_signal_once(self) -> YaskawaOptions:
        names = [signal.name for signal in self.signals]
        if len(set(names)) != len(names):
            raise ValueError(f"signals lists one twice: {names}")
        return self

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
            "soft_limit_pulses": arm.soft_limit_pulses,
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
    def _move_buffer_counts_moves(self) -> YaskawaOptions:
        if isinstance(self.move_buffer, int) and self.move_buffer < 0:
            raise ValueError(f"move_buffer {self.move_buffer} is negative")
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
    def _controller_files_is_a_folder(self) -> YaskawaOptions:
        if self.controller_files is not None and not self.controller_files.is_dir():
            raise ValueError(f"controller_files {self.controller_files} is not a folder")
        return self

    @model_validator(mode="after")
    def _ladder_rungs_parse(self) -> YaskawaOptions:
        self.cell_ladder()
        return self

    def cell_ladder(self) -> Ladder:
        """ladder_copies as byte-copy rungs, then the other rungs."""
        copies = [f"GSTR #{source:05d} GOUT #{target:05d}"
                  for source, target in self.ladder_copies.items()]
        return Ladder.parse([*copies, *self.ladder])

    @model_validator(mode="after")
    def _one_value_per_joint(self) -> YaskawaOptions:
        for name in ("pulses_per_degree", "max_joint_speed_deg_s", "soft_limit_pulses"):
            values = getattr(self, name)
            if values is not None and len(values) != self.joint_count:
                raise ValueError(f"{len(values)} {name} for {self.joint_count} joints")
        return self


class MotomanNX100Options(YaskawaOptions):
    """The ``motoman_nx100`` block: a ``yaskawa`` block for an NX100 with a UP50N."""

    controller: Literal["nx100", "dx200", "yrc1000"] = "nx100"
    robot: str | None = "up50n"


class YaskawaRobot(Device, HasArm, HasIO):
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
        self.controller_name = options.controller
        self.robot = options.robot
        self.hses_port = options.hses_port
        self.hses_file_port = options.hses_file_port
        backup = (ControllerBackup.load(options.controller_files)
                  if options.controller_files is not None else ControllerBackup({}, {}))
        self.controller = YaskawaController(
            arm=self.arm,
            profile=CONTROLLERS[options.controller],
            pulses_per_degree=options.pulses_per_degree,
            max_joint_speed_deg_s=options.max_joint_speed_deg_s,
            soft_limit_pulses=options.soft_limit_pulses,
            ladder=backup.ladder.ladder + options.cell_ladder(),
            files={**{name + JOB_SUFFIX: empty_job(name) for name in options.jobs},
                   **backup.files},
            variables={**backup.variables,
                       "register": {**backup.variables.get("register", {}),
                                    **backup.ladder.registers}},
            positions={"P": dict(backup.positions)},
            model=ROBOTS[options.robot].model if options.robot is not None else "",
            move_buffer=_move_buffer(options),
            running_bit_quirk=(CONTROLLERS[options.controller].running_bit_quirk
                               if options.running_bit_quirk is None
                               else options.running_bit_quirk),
            job_select_while_moving=options.job_select_while_moving,
            note=self.emit,
        )
        for number, saved in backup.user_frames.items():
            self.controller.restore_user_frame(number, frame_mm_deg=saved.frame_mm_deg,
                                               tool=saved.tool, external=saved.external)
        self.io = SignalBank(owner=name, publish=bus.publish)
        self._wire_signals(options.signals)
        # A real ladder scans from power-up: an inverted rung is on before any write.
        self.controller.run_ladder()
        if options.ladder_scan_ms is not None:
            self.add_service(Ticker(self.controller.run_ladder,
                                    interval=options.ladder_scan_ms / 1000))
        if "ethernet_server" in self.protocols:
            self.add_service(LineServer(
                endpoint.host, endpoint.port, session_factory=self.make_session, framer=CR,
                line_gap_seconds=ES_LINE_GAP_SECONDS,
            ))
        if "hses" in self.protocols:
            hses = HighSpeedEthernetServer(self.controller)
            self.add_service(UdpMessageServer(endpoint.host, self.hses_port, hses.handle))
            files = HsesFileServer(self.controller)
            self.add_service(
                UdpDatagramServer(endpoint.host, self.hses_file_port, files.handle)
            )

    def _wire_signals(self, signals: tuple[ControllerSignal, ...]) -> None:
        """Inputs switch the controller's; outputs follow the controller's after each
        write to its IO."""
        outputs = []
        for signal in signals:
            declared = self.io.declare(signal.name, signal.direction)
            match signal.kind, signal.direction:
                case SignalKind.IN, _:
                    declared.subscribe(partial(self.controller.set_input, signal.number))
                case SignalKind.RIN, _:
                    declared.subscribe(partial(self.controller.set_direct_input, signal.number))
                case SignalKind.EXTERNAL, Direction.INPUT:
                    declared.subscribe(partial(self.controller.set_signal, signal.number))
                case SignalKind.OT, _:
                    outputs.append((partial(self.controller.output, signal.number), declared))
                case SignalKind.EXTERNAL, Direction.OUTPUT:
                    outputs.append((partial(self.controller.signal, signal.number), declared))

        def follow_outputs() -> None:
            for read, declared in outputs:
                declared.set(read())
        self.controller.io_written = follow_outputs

    def settings(self) -> list[str]:
        """The controller and arm, each protocol it serves and where, and how it
        takes moves."""
        host = self.endpoint.host
        arm = f", arm {self.robot}" if self.robot is not None else ""
        lines = [f"controller {self.controller_name.upper()}{arm}"]
        if "ethernet_server" in self.protocols:
            lines.append(f"Ethernet server on TCP {host}:{self.endpoint.port}")
        if "hses" in self.protocols:
            lines.append(f"HSES on UDP {host}:{self.hses_port}, files on :{self.hses_file_port}")
        buffer = self.controller.move_buffer
        lines.append(f"moves taken while moving: {'unlimited' if buffer is None else buffer}")
        if self.controller.running_bit_quirk:
            lines.append("running bit quirk on")
        if self.controller.job_select_while_moving:
            lines.append("jobs selected while moving")
        if not self.arm.state.view.servo_on:
            lines.append("servos off")
        return lines

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


def _move_buffer(options: YaskawaOptions) -> int | None:
    if options.move_buffer is None:
        return CONTROLLERS[options.controller].move_buffer
    return None if options.move_buffer == "unlimited" else options.move_buffer


@register("yaskawa", default_port=MOTOMAN_PORT, options=YaskawaOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: YaskawaOptions) -> Device:
    return YaskawaRobot(name, endpoint, bus, options)


@register("motoman_nx100", default_port=MOTOMAN_PORT, options=MotomanNX100Options)
def _nx100_factory(
    name: str, endpoint: Endpoint, bus: EventBus, options: MotomanNX100Options
) -> Device:
    return YaskawaRobot(name, endpoint, bus, options)
