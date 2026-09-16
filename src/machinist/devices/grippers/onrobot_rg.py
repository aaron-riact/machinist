"""OnRobot RG2 / RG6 two-finger gripper.

The RG is a parallel gripper whose whole interface is a handful of Modbus
holding registers. Register addresses and semantics here follow the layout
riact's own driver speaks (``onrobot/grippers/rg/definitions.py``), so the
emulator answers the same registers a real RG does.

Write registers
---------------
=========  =====================================================
0x0000     Target force, 1/10 N
0x0001     Target width, 1/10 mm
0x0002     Command: 1 grip, 8 stop, 16 grip with offset
0x0407     Set fingertip offset, 1/10 mm (signed)
=========  =====================================================

Read registers
--------------
=========  =====================================================
0x0102     Fingertip offset, 1/10 mm (signed)
0x0107     Actual depth, 1/10 mm (signed)
0x0108     Actual relative depth, 1/10 mm (signed)
0x010B     Actual width, 1/10 mm
0x010C     Status flags (see below)
0x0113     Actual width with fingertip offset, 1/10 mm
=========  =====================================================

The status word's bit order comes from how riact's ``BitSet`` packs the
``Status`` set: sequences are laid out most-significant bit first in
declaration order, so ``Status``'s 16-bit width lands in 0x010B and its nine
padding bits plus seven flags land in 0x010C, flags in the low bits.

Reachability is what makes this gripper worth emulating twice over: it
serves the same :class:`RegisterPort` to a Modbus/TCP client and to a robot
reading the RS485 line on its tool flange.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

from ...core.capabilities import HasRegisters
from ...core.device import Device, DetailField, DetailSignal, DeviceDetail
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.modbus_server import HoldingRegisterServer
from ...transport.registers import RegisterPort

# --- write registers --------------------------------------------------------
REG_TARGET_FORCE = 0x0000
REG_TARGET_WIDTH = 0x0001
REG_COMMAND = 0x0002
REG_SET_FINGERTIP_OFFSET = 0x0407

# --- read registers ---------------------------------------------------------
REG_FINGERTIP_OFFSET = 0x0102
REG_ACTUAL_DEPTH = 0x0107
REG_ACTUAL_RELATIVE_DEPTH = 0x0108
REG_ACTUAL_WIDTH = 0x010B
REG_STATUS = 0x010C
REG_WIDTH_WITH_OFFSET = 0x0113

# --- commands, from riact's Command register --------------------------------
CMD_GRIP = 1
CMD_STOP = 8
CMD_GRIP_WITH_OFFSET = 16

# --- status flags, low bits of REG_STATUS -----------------------------------
STATUS_BUSY = 1 << 0
STATUS_GRIP_DETECTED = 1 << 1
STATUS_SAFETY_1_PUSHED = 1 << 2
STATUS_SAFETY_1_TRIGGERED = 1 << 3
STATUS_SAFETY_2_PUSHED = 1 << 4
STATUS_SAFETY_2_TRIGGERED = 1 << 5
STATUS_SAFETY_ERROR = 1 << 6

#: Mover tick period. Travel speed sets how far the fingers move per tick.
_STEP_SECONDS = 0.02


@dataclass(frozen=True, slots=True)
class _ModelLimits:
    """Force and opening limits, from riact's RG2 / RG6 GripperLimits."""

    strongest_tenth_newtons: int
    fully_open_tenths: int


RG_MODELS: dict[str, _ModelLimits] = {
    "rg2": _ModelLimits(strongest_tenth_newtons=400, fully_open_tenths=1100),
    "rg6": _ModelLimits(strongest_tenth_newtons=1200, fully_open_tenths=1600),
}


@dataclass(slots=True)
class OnRobotRGOptions:
    """Typed schema for the ``onrobot_rg`` YAML ``options`` section."""

    model: str = "rg2"
    initial_width_mm: float = 110.0
    travel_mm_per_sec: float = 110.0
    fingertip_offset_mm: float = 0.0
    #: Width of the object between the fingers, if any. A grip narrower than
    #: this stops on the object and reports grip detected, which is how a real
    #: RG tells "I am holding something" from "I closed on nothing".
    held_object_mm: float | None = None


@dataclass(slots=True)
class _State:
    actual_width_tenths: int = 0
    target_width_tenths: int = 0
    target_force_tenths: int = 400
    command: int = 0
    fingertip_offset_tenths: int = 0
    held_object_tenths: int | None = None
    #: Distance covered per mover tick, from the configured travel speed.
    step_tenths: int = 22
    busy: bool = False
    grip_detected: bool = False
    limits: _ModelLimits = RG_MODELS["rg2"]
    lock: threading.Lock = field(default_factory=threading.Lock)


def _to_signed(value: int) -> int:
    """16-bit two's complement, as the signed read registers carry."""
    return value - 0x10000 if value >= 0x8000 else value


def _from_signed(value: int) -> int:
    return value & 0xFFFF


class OnRobotRG(Device, HasRegisters):
    kind = "onrobot_rg"
    DEFAULT_PORT = 502

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: OnRobotRGOptions,
        *, state: _State,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._settings = options
        self._state = state
        self._server: HoldingRegisterServer | None = None
        self._mover: threading.Thread | None = None

    # ----- register model -----------------------------------------------

    @property
    def register_port(self) -> RegisterPort:
        """This gripper's holding registers, servable over any front end."""
        return RegisterPort(on_read=self._on_read, on_write=self._on_write)

    def _status_word(self) -> int:
        s = self._state
        return (STATUS_BUSY if s.busy else 0) | (STATUS_GRIP_DETECTED if s.grip_detected else 0)

    def _on_read(self, address: int) -> int:
        s = self._state
        with s.lock:
            return {
                REG_TARGET_FORCE: s.target_force_tenths,
                REG_TARGET_WIDTH: s.target_width_tenths,
                REG_COMMAND: s.command,
                REG_FINGERTIP_OFFSET: _from_signed(s.fingertip_offset_tenths),
                REG_ACTUAL_DEPTH: 0,
                REG_ACTUAL_RELATIVE_DEPTH: 0,
                REG_ACTUAL_WIDTH: s.actual_width_tenths,
                REG_STATUS: self._status_word(),
                REG_WIDTH_WITH_OFFSET: max(
                    0, s.actual_width_tenths - 2 * s.fingertip_offset_tenths
                ),
                REG_SET_FINGERTIP_OFFSET: _from_signed(s.fingertip_offset_tenths),
            }.get(address, 0)

    def _on_write(self, address: int, value: int) -> None:
        s = self._state
        start = False
        with s.lock:
            if address == REG_TARGET_FORCE:
                s.target_force_tenths = min(value, s.limits.strongest_tenth_newtons)
            elif address == REG_TARGET_WIDTH:
                s.target_width_tenths = min(value, s.limits.fully_open_tenths)
            elif address in (REG_FINGERTIP_OFFSET, REG_SET_FINGERTIP_OFFSET):
                s.fingertip_offset_tenths = _to_signed(value)
            elif address == REG_COMMAND:
                s.command = value
                if value == CMD_STOP:
                    s.target_width_tenths = s.actual_width_tenths
                    s.busy = False
                elif value in (CMD_GRIP, CMD_GRIP_WITH_OFFSET):
                    s.grip_detected = False
                    start = True
        if start:
            self._kick()

    # ----- motion --------------------------------------------------------

    def _kick(self) -> None:
        if self._mover is None or not self._mover.is_alive():
            self._mover = threading.Thread(target=self._move_loop, daemon=True)
            self._mover.start()

    def _stop_width(self) -> int:
        """Where the fingers actually come to rest.

        Closing onto an object stops at the object's width, not the commanded
        one -- that difference is what grip detection reports.
        """
        s = self._state
        obstacle = s.held_object_tenths
        if obstacle is not None and s.target_width_tenths < obstacle <= s.actual_width_tenths:
            return obstacle
        return s.target_width_tenths

    def _move_loop(self) -> None:
        s = self._state
        while not self._stop_event.is_set():
            with s.lock:
                goal = self._stop_width()
                if s.actual_width_tenths == goal:
                    s.busy = False
                    s.grip_detected = goal != s.target_width_tenths
                    width, gripped = s.actual_width_tenths, s.grip_detected
                    self.emit("settled", width_mm=width / 10, gripped=gripped)
                    return
                s.busy = True
                delta = goal - s.actual_width_tenths
                step = min(abs(delta), s.step_tenths)
                s.actual_width_tenths += step if delta > 0 else -step
                width = s.actual_width_tenths
            self.emit("moving", width_mm=width / 10)
            self._stop_event.wait(_STEP_SECONDS)

    # ----- device --------------------------------------------------------

    def build_detail(self) -> DeviceDetail:
        s = self._state
        server = self._server
        io = getattr(self, "io", None)
        signals: list[DetailSignal] = []
        if io is not None:
            signals = [
                DetailSignal(name=sig.name, direction=str(sig.direction), value=sig.value)
                for sig in io
            ]

        return DeviceDetail(
            mode="modbus",
            transport_ready=server is not None and server._sock is not None,
            peer_connected=server is not None and server.client_count > 0,
            clients=server.client_count if server is not None else 0,
            input_block_hex="",
            output_block_hex="",
            input_fields=[
                _reg("T_FORCE", "Target force", "0x0000", "int", f"{s.target_force_tenths} (.1 N)"),
                _reg("T_WIDTH", "Target width", "0x0001", "int", f"{s.target_width_tenths} (.1 mm)"),
                _reg("CMD", "Command", "0x0002", "int", str(s.command)),
                _reg("SET_FTOF", "Set fingertip offset", "0x0407", "int", f"{s.fingertip_offset_tenths} (.1 mm)"),
            ],
            output_fields=[
                _reg("WIDTH", "Actual width", "0x010B", "int", f"{s.actual_width_tenths} (.1 mm)"),
                _reg("STATUS", "Status flags", "0x010C", "hex", f"0x{self._status_word():04X}"),
                _reg("W_OFF", "Width w/ offset", "0x0113", "int", f"{max(0, s.actual_width_tenths - 2 * s.fingertip_offset_tenths)} (.1 mm)"),
                _reg("FTOFFSET", "Fingertip offset", "0x0102", "int", f"{s.fingertip_offset_tenths} (.1 mm)"),
            ],
            derived_fields=[
                _reg("MODEL", "Model", "", "str", self._settings.model.upper()),
                _reg("WIDTH_MM", "Actual width", "", "mm", f"{s.actual_width_tenths / 10:.1f}"),
                _reg("BUSY", "Moving", "", "bit", "1" if s.busy else "0"),
                _reg("GRIPPED", "Object gripped", "", "bit", "1" if s.grip_detected else "0"),
                _reg("OBJECT", "Object width", "", "mm",
                     "-" if s.held_object_tenths is None else f"{s.held_object_tenths / 10:.1f}"),
            ],
            signals=signals,
        )

    def _run(self, stop: threading.Event) -> None:
        ready = threading.Event()
        thread = threading.Thread(
            target=self._server.serve_forever, args=(ready,), daemon=True
        )
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        stop.wait()
        self._server.shutdown()
        thread.join(timeout=2.0)


def _reg(signal: str, name: str, offset: str, type_: str, value: str) -> DetailField:
    return DetailField(signal=signal, name=name, offset=offset, type=type_, value=value)


@register("onrobot_rg", default_port=502)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = OnRobotRGOptions(**options)
    try:
        limits = RG_MODELS[opts.model.lower()]
    except KeyError:
        raise ValueError(
            f"unknown RG model {opts.model!r}; expected one of {sorted(RG_MODELS)}"
        ) from None

    width = min(int(round(opts.initial_width_mm * 10)), limits.fully_open_tenths)
    state = _State(
        actual_width_tenths=width,
        target_width_tenths=width,
        target_force_tenths=limits.strongest_tenth_newtons,
        fingertip_offset_tenths=int(round(opts.fingertip_offset_mm * 10)),
        held_object_tenths=(
            None if opts.held_object_mm is None else int(round(opts.held_object_mm * 10))
        ),
        step_tenths=max(1, int(round(opts.travel_mm_per_sec * 10 * _STEP_SECONDS))),
        limits=limits,
    )
    device = OnRobotRG(name, endpoint, bus, opts, state=state)
    device._server = HoldingRegisterServer(
        host=endpoint.host,
        port=endpoint.port,
        registers=device.register_port,
        on_connect_change=lambda count: device.emit("snapshot", clients=count),
    )
    return device
