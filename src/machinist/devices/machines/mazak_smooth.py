"""Mazak Smooth robot-interface emulator with IO and EtherNet/IP support."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable, Iterable, Mapping
from dataclasses import dataclass
from typing import Literal

from pydantic import field_validator, model_validator

from ...core.capabilities import HasIO
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.options import Options
from ...core.panel import Field, Panel, PanelChanged
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.ethernetip import (
    EtherNetIPAdapter,
    EtherNetIPAdapterConfig,
    EtherNetIPScanner,
    EtherNetIPScannerConfig,
    EtherNetIPTransport,
    MazakEthernetIPAdapter,
)
from ...transport.mtconnect import MTConnectAgent, render_mtconnect
from .state import CycleState, HasMachineState, MachineState

BLOCK_SIZE = 110
PROGRAM_OFFSET = 44
PROGRAM_LENGTH = 32
CONTROL_OFFSET = 12
HEARTBEAT_ALARM = 1362
# Raised when a work-number search cannot be satisfied. The assembly carries only
# DO004, never the alarm number, so this code is a placeholder sitting next to
# HEARTBEAT_ALARM rather than an observed value.
WORK_SEARCH_ALARM = 1363


@dataclass(frozen=True, slots=True)
class BitPoint:
    number: int
    signal: str
    byte: int
    bit: int
    description: str


@dataclass(frozen=True, slots=True)
class TextField:
    number: int
    signal: str
    offset: int
    length: int
    description: str


@dataclass(frozen=True, slots=True)
class BitField:
    number: int
    signal: str
    byte: int
    bit: int
    width: int
    description: str


def _word_bit(base: int, index: int) -> tuple[int, int]:
    return base + (index // 8), index % 8


def _point(number: int, signal: str, byte: int, bit: int, description: str) -> BitPoint:
    return BitPoint(number=number, signal=signal, byte=byte, bit=bit, description=description)


def _control_point(number: int, signal: str, word_bit: int, description: str) -> BitPoint:
    byte, bit = _word_bit(CONTROL_OFFSET, word_bit)
    return _point(number, signal, byte, bit, description)


INPUT_SIGNAL_POINTS: dict[int, BitPoint] = {
    0: _point(0, "di000", 0, 0, "Communication check input"),
    1: _point(1, "di001", 0, 1, "Robot ready"),
    2: _point(2, "di002", 0, 2, "Machine stop request"),
    3: _point(3, "di003", 0, 3, "Robot operating"),
    4: _point(4, "di004", 0, 4, "Robot alarm status"),
    5: _point(5, "di005", 0, 5, "Operator interruption request"),
    6: _point(6, "di006", 0, 6, "Automatic power shut-off request"),
    8: _point(8, "di008", 1, 0, "Fixture 1 clamp command"),
    9: _point(9, "di009", 1, 1, "Fixture 1 unclamp command"),
    10: _point(10, "di010", 1, 2, "Fixture 2 clamp command"),
    11: _point(11, "di011", 1, 3, "Fixture 2 unclamp command"),
    12: _point(12, "di012", 1, 4, "Fixture 3 clamp command"),
    13: _point(13, "di013", 1, 5, "Fixture 3 unclamp command"),
    14: _point(14, "di014", 1, 6, "Fixture 4 clamp command"),
    15: _point(15, "di015", 1, 7, "Fixture 4 unclamp command"),
    16: _point(16, "di016", 2, 0, "Workpiece seating detection request 1"),
    17: _point(17, "di017", 2, 1, "Workpiece seating detection request 2"),
    18: _point(18, "di018", 2, 2, "Ignore workpiece seating detection alarm 1"),
    19: _point(19, "di019", 2, 3, "Ignore workpiece seating detection alarm 2"),
    101: _control_point(101, "di101", 0, "Work number search start"),
    102: _control_point(102, "di102", 1, "Cycle start command"),
    103: _control_point(103, "di103", 2, "NC reset"),
    104: _control_point(104, "di104", 3, "All machining complete"),
    106: _control_point(106, "di106", 5, "Robot service finished"),
    107: _control_point(107, "di107", 6, "Door open command"),
    108: _control_point(108, "di108", 7, "Door close command"),
    109: _control_point(109, "di109", 8, "Robot clear"),
}

OUTPUT_SIGNAL_POINTS: dict[int, BitPoint] = {
    0: _point(0, "do000", 0, 0, "Communication check output"),
    1: _point(1, "do001", 0, 1, "Machine ready"),
    2: _point(2, "do002", 0, 2, "Robot stop request"),
    3: _point(3, "do003", 0, 3, "Machine operating panel retract position"),
    4: _point(4, "do004", 0, 4, "Machine alarm status"),
    6: _point(6, "do006", 0, 6, "Automatic power shut-off request received"),
    8: _point(8, "do008", 1, 0, "Fixture 1 clamp complete"),
    9: _point(9, "do009", 1, 1, "Fixture 1 unclamp complete"),
    10: _point(10, "do010", 1, 2, "Fixture 2 clamp complete"),
    11: _point(11, "do011", 1, 3, "Fixture 2 unclamp complete"),
    12: _point(12, "do012", 1, 4, "Fixture 3 clamp complete"),
    13: _point(13, "do013", 1, 5, "Fixture 3 unclamp complete"),
    14: _point(14, "do014", 1, 6, "Fixture 4 clamp complete"),
    15: _point(15, "do015", 1, 7, "Fixture 4 unclamp complete"),
    16: _point(16, "do016", 2, 0, "Seating detection complete 1"),
    17: _point(17, "do017", 2, 1, "Seating detection complete 2"),
    18: _point(18, "do018", 2, 2, "Workpiece seating detection alarm 1"),
    19: _point(19, "do019", 2, 3, "Workpiece seating detection alarm 2"),
    101: _control_point(101, "do101", 0, "Work number search complete"),
    102: _control_point(102, "do102", 1, "Cycle start enable"),
    103: _control_point(103, "do103", 2, "Machine running"),
    104: _control_point(104, "do104", 3, "Machining complete"),
    106: _control_point(106, "do106", 8, "Robot service request"),
    107: _control_point(107, "do107", 9, "Door open finished"),
    108: _control_point(108, "do108", 10, "Door close finished"),
    109: _control_point(109, "do109", 11, "Robot access permitted"),
    110: _control_point(110, "do110", 12, "Robot service finished confirmation"),
}

INPUT_TEXT_FIELDS = {
    100: TextField(100, "di100", PROGRAM_OFFSET, PROGRAM_LENGTH, "Target work number data")
}

OUTPUT_TEXT_FIELDS = {
    100: TextField(100, "do100", PROGRAM_OFFSET, PROGRAM_LENGTH, "Current work number")
}

OUTPUT_BIT_FIELDS = {
    105: BitField(105, "do105", CONTROL_OFFSET, 4, 4, "Robot service code")
}

SMOOTH_AI_INPUT_OVERRIDES: dict[int, BitPoint] = {
    107: _control_point(107, "di107", 6, "Side door open command"),
    108: _control_point(108, "di108", 7, "Side door close command"),
    110: _control_point(110, "di110", 9, "Front door open command"),
    111: _control_point(111, "di111", 10, "Front door close command"),
}

SMOOTH_AI_OUTPUT_OVERRIDES: dict[int, BitPoint] = {
    107: _control_point(107, "do107", 9, "Side door open finished"),
    108: _control_point(108, "do108", 10, "Side door close finished"),
    110: _control_point(110, "do110", 12, "Front door open finished"),
    111: _control_point(111, "do111", 13, "Front door close finished"),
}


class MTConnectOptions(Options):
    port: int


class EtherNetIPOptions(Options):
    """The ``ethernetip`` block: which end of the link this machine is, and how it is set up.

    In ``adapter`` mode the machine listens on the device endpoint. In ``scanner``
    mode it dials out to the robot at *host*:*port*.
    """

    mode: Literal["adapter", "scanner"] = "adapter"
    udp_port: int = 2222
    requested_packet_rate_ms: int = 20
    o_t_realtime_format: str = "header32bit"
    # adapter only
    behaviour: str = "mazak"
    # scanner only
    host: str | None = None
    port: int = 44818
    originator_udp_port: int = 2222
    target_udp_port: int = 2222
    assembly_object_class: int = 0x04
    configuration_assembly_instance_id: int = 0x01
    output_assembly_instance_id: int = 0x64
    input_assembly_instance_id: int = 0x65
    o_t_connection_type: str = "point_to_point"
    t_o_connection_type: str = "point_to_point"


class MazakSmoothOptions(Options):
    variant: str = "smoothx"
    scan_interval_seconds: float = 0.02
    # Door travel. `door_move_seconds` is the symmetric default; the per-direction
    # options override it. A real SmoothAi is slower and asymmetric -- mazak6.pcap
    # (10 door cycles) measures 5.44s from DI107 to DO107 opening and 4.30s from
    # DI108 to DO108 closing -- so set 5.4/4.3 to mimic that machine.
    door_move_seconds: float = 2.0
    door_open_seconds: float | None = None
    door_close_seconds: float | None = None
    # Front door (SmoothAi only). Off by default: the SmoothAi in mazak6.pcap
    # never asserts DO110/DO111 -- T->O byte13 only ever holds 0x00/0x02/0x04
    # across the whole 4.6h capture -- so the front-door bits are only driven
    # when a machine actually has one.
    front_door: bool = False
    cycle_duration_seconds: float = 1.0
    work_search_seconds: float = 0.5
    # Work numbers the NC holds. None (the default) accepts any search; give a
    # list and a search for anything outside it fails the way mazak3.pcap does.
    programs: tuple[str | int, ...] | None = None
    # How long DO102 (cycle-start permission) stays OFF after a work-number
    # search *loads a different program*. The machine withholds permission while
    # the NC actually swaps programs; a search that resolves to the already-
    # loaded program is a no-op and permission never drops. Both captures come
    # from the same machine at 192.168.10.1:
    #   cyclestart.pcap  '9'->'6' at t=3.700 and '6'->'9' at t=7.000, each
    #                    dipping DO102 for exactly 1.0s (t=4.700 / t=8.000).
    #   mazak6.pcap      11 searches, all for the already-loaded '202', T->O
    #                    work# never changes and DO102 never dips -- byte12 runs
    #                    0x0B -> 0x0A -> 0x0B -> 0x03 -> 0x05 with bit1 set the
    #                    whole way. mazak3/mazak4 add 2 more no-op searches.
    # Set to 0.0 to disable the dip entirely.
    work_search_settle_seconds: float = 1.0
    heartbeat_interval_seconds: float = 2.0
    # Real mazak is ~10s and mazak6.pcap proves it tolerates more: the robot left
    # 5 gaps over 6s between DI000 toggles (13.35s, 11.67s, 9.32s, 9.08s, 8.68s)
    # and the machine raised no comms alarm at any of them. 6.0s would have
    # false-tripped 5 times in that 4.6h session -- kept short on purpose so the
    # emulator surfaces a stalled scanner quickly.
    heartbeat_timeout_seconds: float = 6.0
    #: Interfaces to serve: ``io`` (signals) and/or ``ethernetip``. A bare string,
    #: a list, or a ``{name: flag}`` map are all accepted; ``ethernetip`` is
    #: implied by an ``ethernetip`` block.
    interfaces: frozenset[str] = frozenset({"io"})
    ethernetip: EtherNetIPOptions | None = None
    mtconnect: MTConnectOptions | None = None

    @field_validator("interfaces", mode="before")
    @classmethod
    def _interfaces_as_names(cls, raw: object) -> object:
        if raw is None:
            return frozenset({"io"})
        if isinstance(raw, str):
            return {raw.strip().lower()}
        if isinstance(raw, Mapping):
            return {str(name).strip().lower() for name, flag in raw.items() if flag}
        if isinstance(raw, Iterable):
            return {str(item).strip().lower() for item in raw}
        return raw

    @model_validator(mode="before")
    @classmethod
    def _mtconnect_port_shorthand(cls, data: object) -> object:
        """``mtconnect_port: 5000`` is the short spelling of ``mtconnect: {port: 5000}``."""
        if isinstance(data, dict) and "mtconnect_port" in data:
            data = dict(data)
            port = data.pop("mtconnect_port")
            if port is not None:
                data["mtconnect"] = {"port": int(port)}
        return data

    def enabled_interfaces(self) -> frozenset[str]:
        enabled = set(self.interfaces)
        if self.ethernetip is not None:
            enabled.add("ethernetip")
        return frozenset(enabled)

    @property
    def ethernetip_mode(self) -> str:
        return self.ethernetip.mode if self.ethernetip is not None else "adapter"


@dataclass
class _Door:
    """One door's motion, driven each scan by the DI bits that command it.

    A rising edge on *open_cmd* or *close_cmd* starts a timed move; the
    matching "finished" DO bits drop while it travels and one of them comes
    back when it lands. Releasing the command mid-travel abandons the move.
    The side door of a SmoothAi (and the main door of a SmoothX) also needs
    *close_gate* (DI109) held for a close and stops dead if the stop request
    (DI002) drops; the optional front door has neither.
    """

    name: str
    open_cmd: int
    close_cmd: int
    opened: int
    closed: int
    close_gate: int | None = None
    needs_stop_request: bool = False
    deadline: float | None = None
    target_open: bool | None = None
    _prev_open_cmd: bool = False
    _prev_close_cmd: bool = False

    def tick(
        self,
        now: float,
        *,
        read: Callable[[int], bool],
        write: Callable[[int, bool], None],
        open_seconds: float,
        close_seconds: float,
    ) -> bool | None:
        """Advance one scan. Returns the door's new state when a move lands, else None."""
        open_cmd = read(self.open_cmd)
        close_cmd = read(self.close_cmd)
        gate_ok = self.close_gate is None or read(self.close_gate)

        if open_cmd and not self._prev_open_cmd and self.deadline is None:
            self._start(True, now + open_seconds, write)
        if close_cmd and not self._prev_close_cmd and gate_ok and self.deadline is None:
            self._start(False, now + close_seconds, write)

        if self.needs_stop_request and not read(2):
            self._abandon()
        elif self.deadline is not None:
            released = (self.target_open and not open_cmd) or (
                self.target_open is False and (not close_cmd or not gate_ok)
            )
            if released:
                self._abandon()

        landed: bool | None = None
        if self.deadline is not None and now >= self.deadline:
            landed = bool(self.target_open)
            write(self.opened, landed)
            write(self.closed, not landed)
            self._abandon()

        self._prev_open_cmd = open_cmd
        self._prev_close_cmd = close_cmd
        return landed

    def _start(self, target_open: bool, deadline: float, write: Callable[[int, bool], None]) -> None:
        self.target_open = target_open
        self.deadline = deadline
        write(self.opened, False)
        write(self.closed, False)

    def _abandon(self) -> None:
        self.deadline = None
        self.target_open = None


class MazakSmoothEmulator(Device, HasMachineState, HasIO):
    kind = "mazak_smooth"

    input_signal_points = INPUT_SIGNAL_POINTS
    output_signal_points = OUTPUT_SIGNAL_POINTS
    input_text_fields = INPUT_TEXT_FIELDS
    output_text_fields = OUTPUT_TEXT_FIELDS

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: MazakSmoothOptions,
        *, io: SignalBank,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._variant = options.variant
        self._front_door = bool(options.front_door) and options.variant == "smoothai"
        # A SmoothAi has a side door (plus an optional front door); a SmoothX
        # has one main door. Declaring them here is what lets the state view
        # refuse a typo later instead of inventing a door.
        doors = ["side"] if self._variant == "smoothai" else ["main"]
        if self._front_door:
            doors.append("front")
        self.state = MachineState(owner=name, publish=self.publish, doors=doors)

        self._input_signal_points = _build_input_points(options.variant)
        self._output_signal_points = _build_output_points(options.variant)

        self._lock = threading.RLock()
        self._input_block = bytearray(BLOCK_SIZE)
        self._output_block = bytearray(BLOCK_SIZE)
        self._state_snapshot: dict[str, object] = {}
        self._scan_interval = options.scan_interval_seconds
        self._door_open_seconds = (
            options.door_move_seconds
            if options.door_open_seconds is None
            else options.door_open_seconds
        )
        self._door_close_seconds = (
            options.door_move_seconds
            if options.door_close_seconds is None
            else options.door_close_seconds
        )
        self._cycle_seconds = options.cycle_duration_seconds
        self._work_search_seconds = options.work_search_seconds
        self._search_settle_seconds = options.work_search_settle_seconds
        self._programs: frozenset[str] | None = (
            None
            if options.programs is None
            else frozenset(str(item) for item in options.programs)
        )
        self._work_search_failed = False
        self._heartbeat_interval = options.heartbeat_interval_seconds
        self._heartbeat_timeout = options.heartbeat_timeout_seconds
        self._interfaces = options.enabled_interfaces()
        self._ethernetip_mode = options.ethernetip_mode
        self._io_writable = "io" in self._interfaces
        self._alarm_code: int | None = None
        self._alarm_message = ""
        self._connection_up = False
        self._last_panel: Panel | None = None
        main_door = "side" if self._variant == "smoothai" else "main"
        self._doors = [
            _Door(main_door, open_cmd=107, close_cmd=108, opened=107, closed=108,
                  close_gate=109, needs_stop_request=True),
        ]
        if self._front_door:
            self._doors.append(_Door("front", open_cmd=110, close_cmd=111, opened=110, closed=111))
        self._cycle_complete_deadline: float | None = None
        self._work_search_deadline: float | None = None
        self._cycle_start_blocked_until: float | None = None
        self._pending_program = ""
        self._prev_di101 = False
        self._prev_di102 = False
        self._cycle_start_armed = False
        self._feed_hold = False
        self._machining_complete_latched = False
        self._last_heartbeat_toggle_at = -self._heartbeat_interval
        self._di000_toggle: tuple[float, bool | None] = (0.0, None)
        self._last_connection_gen = -1

        self.io = io
        self._declare_signals()

        self._ethernetip: EtherNetIPTransport | None = None
        self._next_connect_attempt = 0.0

        self._initialize_defaults()

    @property
    def input_block(self) -> bytes:
        with self._lock:
            return bytes(self._input_block)

    @property
    def output_block(self) -> bytes:
        with self._lock:
            return bytes(self._output_block)

    @property
    def active_program(self) -> str:
        return self.state.view.program

    @property
    def ethernetip_mode(self) -> str:
        return self._ethernetip_mode

    @property
    def connection_up(self) -> bool:
        return self._connection_up

    @property
    def alarm_code(self) -> int | None:
        return self._alarm_code

    @property
    def state_snapshot(self) -> dict[str, object]:
        with self._lock:
            return dict(self._state_snapshot)

    def build_detail(self) -> Panel:
        """Assemble the normalized detail dict for this Smooth device."""
        if "ethernetip" not in self._interfaces:
            return Panel(mode="io", transport_ready=False, peer_connected=False)

        with self._lock:
            input_block = bytes(self._input_block)
            output_block = bytes(self._output_block)
            alarm_code = self._alarm_code
            alarm_message = self._alarm_message
            connection_up = self._connection_up
            active_program = self.state.view.program
            transport = self._ethernetip

        transport_ready = transport is not None and transport.connected
        peer_connected = transport is not None and transport.peer_connected

        input_fields = _field_rows(
            prefix="DI",
            block=input_block,
            bit_points=self._input_signal_points,
            text_fields=INPUT_TEXT_FIELDS,
            bit_fields={},
        )
        output_fields = _field_rows(
            prefix="DO",
            block=output_block,
            bit_points=self._output_signal_points,
            text_fields=OUTPUT_TEXT_FIELDS,
            bit_fields=OUTPUT_BIT_FIELDS,
        )
        status_fields = (
            Field("STATE", "Active program", "-", "string", active_program or ""),
            Field("STATE", "Connection up", "-", "bool", "ON" if connection_up else "OFF", on=connection_up),
            Field("STATE", "Alarm code", "-", "int", "" if alarm_code is None else str(alarm_code)),
            Field("STATE", "Alarm message", "-", "string", alarm_message),
        )

        return Panel(
            mode=self._ethernetip_mode,
            transport_ready=transport_ready,
            peer_connected=peer_connected,
            clients=None,
            input_block_hex=input_block.hex(" "),
            output_block_hex=output_block.hex(" "),
            input_fields=input_fields,
            output_fields=output_fields,
            status_fields=status_fields,
        )

    def write_input_block(self, data: bytes | bytearray, *, offset: int = 0) -> None:
        chunk = bytes(data)
        if offset < 0 or offset + len(chunk) > BLOCK_SIZE:
            raise ValueError("input block write exceeds the 100-byte block")
        with self._lock:
            current = bytes(self._input_block[offset : offset + len(chunk)])
            if current == chunk:
                return
            self._input_block[offset : offset + len(chunk)] = chunk
            snapshot = bytes(self._input_block)
        self._sync_input_signals(snapshot)
        self._emit_snapshot_change("input")

    def write_output_block(self, data: bytes | bytearray, *, offset: int = 0) -> None:
        chunk = bytes(data)
        if offset < 0 or offset + len(chunk) > BLOCK_SIZE:
            raise ValueError("output block write exceeds the 100-byte block")
        with self._lock:
            current = bytes(self._output_block[offset : offset + len(chunk)])
            if current == chunk:
                return
            self._output_block[offset : offset + len(chunk)] = chunk
            snapshot = bytes(self._output_block)
        self._sync_output_signals(snapshot)
        self._emit_snapshot_change("output")

    def set_input_bit(self, number: int, value: bool) -> None:
        self._write_input_bit(number, value, sync_signal=True)

    def set_target_work_number(self, program: str) -> None:
        self._set_text(self._input_block, INPUT_TEXT_FIELDS[100], program)

    def inject_alarm(self, code: int, message: str) -> None:
        self._set_alarm(code, message)

    def clear_alarm(self) -> None:
        was_set = False
        with self._lock:
            if self._alarm_code is not None:
                was_set = True
                self._alarm_code = None
                self._alarm_message = ""
        if was_set:
            self._refresh_outputs()
            self.emit("alarm", code=0, message="cleared")
            self._publish_panel()

    def _declare_signals(self) -> None:
        for point in self._input_signal_points.values():
            signal = self.io.declare(point.signal, Direction.INPUT)
            if self._io_writable:
                signal.subscribe(
                    lambda value, number=point.number: self._write_input_bit(
                        number, value, sync_signal=False
                    )
                )
        for point in self._output_signal_points.values():
            self.io.declare(point.signal, Direction.OUTPUT)

    def _initialize_defaults(self) -> None:
        self._set_output_text(100, "")
        self._write_output_field(105, 0)
        self._write_input_bit(2, True, sync_signal=True)
        self._write_output_bit(2, True)
        self._write_output_bit(101, True)
        self._write_output_bit(108, True)
        # DO104 idles ON on a real machine: mazak6.pcap opens with control word
        # byte12=0x0B (DO101 + DO102 + DO104) and the bit only drops once a cycle
        # start is accepted. The bit is driven from the latch in _refresh_outputs.
        self._machining_complete_latched = True
        if self._front_door:
            self._write_output_bit(111, True)
        self._refresh_outputs()

    def attach_ethernetip(self, transport: EtherNetIPTransport) -> None:
        """Give the machine its EtherNet/IP link. Call before :meth:`start`.

        An adapter listens, so it runs as one of the device's services and
        is bound before the device reports running. A scanner dials out and
        is opened lazily by the scan loop, with retries.
        """
        self._ethernetip = transport
        if isinstance(transport, EtherNetIPAdapter):
            self.add_service(transport)

    def _serve(self, stop: threading.Event) -> None:
        try:
            while not stop.is_set():
                now = time.monotonic()
                self._poll_ethernetip(now)
                self._scan_cycle(now=now)
                self._publish_panel()  # link state and program changes land here
                stop.wait(self._scan_interval)
        finally:
            if self._ethernetip is not None:
                self._ethernetip.close()

    def _poll_ethernetip(self, now: float) -> None:
        transport = self._ethernetip
        if transport is None:
            self._connection_up = True
            return
        if self._alarm_code == HEARTBEAT_ALARM and not isinstance(
            transport, EtherNetIPAdapter
        ):
            self._connection_up = False
            return
        if not transport.connected:
            if now < self._next_connect_attempt:
                self._connection_up = False
                return
            try:
                transport.open()
                self._next_connect_attempt = 0.0
            except Exception as exc:
                self._next_connect_attempt = now + 1.0
                self._connection_up = False
                self.emit("ethernetip.error", message=str(exc))
                return
        try:
            transport.write_output_block(self.output_block)
            incoming = transport.read_input_block()
        except Exception as exc:
            transport.close()
            self._next_connect_attempt = now + 1.0
            self._connection_up = False
            self.emit("ethernetip.error", message=str(exc))
            return
        was_down = not self._connection_up
        self._connection_up = transport.peer_connected
        if was_down and self._connection_up:
            self.emit("ethernetip.connected", message="connection established")
        self.write_input_block(incoming)
        gen = transport.connection_generation
        if gen != self._last_connection_gen:
            self._last_connection_gen = gen
            self.clear_alarm()
            self._last_heartbeat_toggle_at = -self._heartbeat_interval
            self._di000_toggle = (now, None)

    def _scan_cycle(self, *, now: float) -> None:
        self._update_heartbeat(now)
        self._handle_program_search(now)
        self._handle_door_motion(now)
        self._handle_cycle(now)
        self._refresh_outputs()

    def _update_heartbeat(self, now: float) -> None:
        di000 = self._read_input_bit(0)
        if di000 or now - self._last_heartbeat_toggle_at >= self._heartbeat_interval:
            self._write_output_bit(0, not di000)
            self._last_heartbeat_toggle_at = now

        prev_time, prev_value = self._di000_toggle
        if prev_value is not None and di000 != prev_value:
            self._di000_toggle = (now, di000)
            if self._alarm_code == HEARTBEAT_ALARM:
                self.clear_alarm()
        else:
            self._di000_toggle = (prev_time, di000)

        if now - self._di000_toggle[0] > self._heartbeat_timeout:
            self._set_alarm(HEARTBEAT_ALARM, "Robot Communication Error")

    def _handle_program_search(self, now: float) -> None:
        di101 = self._read_input_bit(101)
        if self._work_search_failed and self._alarm_code is None:
            # Alarm cleared: DO101 returns to its idle high state, as it does at
            # mazak3.pcap t=3445 when DO004 drops in the same frame.
            self._work_search_failed = False
            self._write_output_bit(101, True)
        # A machine sitting in alarm does not accept a new search: the robot's two
        # further DI101 pulses at mazak3.pcap t=1826.6 and t=1832.7 get no
        # response at all.
        if di101 and not self._prev_di101 and self._alarm_code is None:
            self._pending_program = self._read_text(self._input_block, INPUT_TEXT_FIELDS[100])
            self._write_output_bit(101, False)
            self._work_search_deadline = now + self._work_search_seconds
        if self._work_search_deadline is not None and now >= self._work_search_deadline:
            self._work_search_deadline = None
            if not self._program_available(self._pending_program):
                # A search the NC cannot satisfy never finishes. mazak3.pcap
                # f31402-f31411: DO101 drops, DO004 comes up 0.500s later --
                # exactly the normal search duration -- and DO101 never returns.
                # The program is not loaded and DO100 keeps its old value.
                self._work_search_failed = True
                self._set_alarm(
                    WORK_SEARCH_ALARM,
                    f"Work Number Search Error ({self._pending_program or '(blank)'})",
                    abort_cycle=False,
                )
                self.emit("program.search_failed", program=self._pending_program)
                self._prev_di101 = di101
                return
            program_changed = self._pending_program != self.state.view.program
            self.state.update(program=self._pending_program)
            self._set_output_text(100, self._pending_program)
            self._write_output_bit(101, True)
            self.emit("program", program=self._pending_program)
            # Only an actual program swap withholds cycle-start permission --
            # see `work_search_settle_seconds`. A freshly started emulator has no
            # program loaded, so its first search is a real load and does dip.
            if program_changed and self._search_settle_seconds > 0.0:
                self._cycle_start_blocked_until = now + self._search_settle_seconds
        if (
            self._cycle_start_blocked_until is not None
            and now >= self._cycle_start_blocked_until
        ):
            self._cycle_start_blocked_until = None
        self._prev_di101 = di101

    def _program_available(self, program: str) -> bool:
        """Whether the NC holds *program*; always true when no library is set."""
        if self._programs is None:
            return True
        return program in self._programs

    def _handle_door_motion(self, now: float) -> None:
        for door in self._doors:
            landed = door.tick(
                now,
                read=self._read_input_bit,
                write=self._write_output_bit,
                open_seconds=self._door_open_seconds,
                close_seconds=self._door_close_seconds,
            )
            if landed is not None:
                self.state.set_door(door.name, open=landed)
                self.emit("door", name=door.name, open=landed)

    def _handle_cycle(self, now: float) -> None:
        robot_ready = self._read_input_bit(1)
        stop_request = self._read_input_bit(2)
        cycle_start = self._read_input_bit(102)
        nc_reset = self._read_input_bit(103)
        all_machining_complete = self._read_input_bit(104)

        if not stop_request:
            self._feed_hold = True
            self.state.update(cycle=CycleState.PAUSED)
            self._cycle_complete_deadline = None
            self._cycle_start_armed = False
            self._write_output_bit(103, False)
        elif self._feed_hold and stop_request and self.state.view.cycle is CycleState.PAUSED:
            self._feed_hold = False

        if nc_reset:
            self.state.update(cycle=CycleState.IDLE)
            self._cycle_complete_deadline = None
            self._cycle_start_armed = False
            self._machining_complete_latched = False
            self._write_output_bit(103, False)

        _door_name = "side" if self._variant == "smoothai" else "main"
        _cycle_blocked = (
            self._cycle_start_blocked_until is not None
            and now < self._cycle_start_blocked_until
        )
        can_cycle = (
            robot_ready
            and stop_request
            and self._alarm_code is None
            and not self.state.view.door_open(_door_name)
            and self.state.view.cycle is not CycleState.RUNNING
            and not _cycle_blocked
        )
        self._write_output_bit(102, can_cycle)

        # Arm on DI102's *level*, not its rising edge: this robot raises DI102 in
        # the same frame it drops DI101 (mazak6.pcap f5362, and 0.108s later at
        # f277914), so an edge-triggered arm inside a settle window is lost
        # forever and the cycle never starts. Levels let the command wait out the
        # window instead -- a settle window may delay a start, never lose one.
        if cycle_start and can_cycle:
            if not self._cycle_start_armed:
                # The real machine clears DO104 as soon as it accepts the command
                # -- 70-163ms after DI102 rises, over 9 cycles in mazak6.pcap --
                # well before the cycle itself starts on the falling edge below.
                self._machining_complete_latched = False
            self._cycle_start_armed = True
        elif cycle_start and not can_cycle:
            self._cycle_start_armed = False

        if not cycle_start and self._prev_di102:
            if self._cycle_start_armed and can_cycle:
                self.state.update(cycle=CycleState.RUNNING)
                self._cycle_complete_deadline = now + self._cycle_seconds
                self._write_output_bit(103, True)
                self.emit("cycle.start", program=self.state.view.program or self._pending_program)
            self._cycle_start_armed = False

        if self._cycle_complete_deadline is not None and now >= self._cycle_complete_deadline:
            self._complete_cycle()

        if all_machining_complete:
            self._complete_cycle()

        self._prev_di102 = cycle_start

    def _complete_cycle(self) -> None:
        if self.state.view.cycle is not CycleState.RUNNING:
            return
        self.state.update(cycle=CycleState.IDLE)
        self.state.bump(parts=1)
        self._cycle_complete_deadline = None
        self._machining_complete_latched = True
        self._write_output_bit(103, False)
        self.emit("cycle.end", parts=self.state.view.parts)

    def _refresh_outputs(self) -> None:
        robot_ready = self._read_input_bit(1)
        stop_request = self._read_input_bit(2)
        has_alarm = self._alarm_code is not None
        self._write_output_bit(1, robot_ready and not has_alarm)
        self._write_output_bit(2, stop_request and not has_alarm)
        # DO003, DO004, DO102 and DO104 are only presented while the robot
        # interface is enabled by DI001. Every DI001 blip in mazak6.pcap drops
        # DO003/DO102/DO104 for the same ~100ms window (5 of them), and the five
        # DI001 retries during the mazak3.pcap alarm at t=1842-1863s drop DO004
        # with them. DO003 is not gated by the alarm: it stayed ON right through
        # the mazak6.pcap machine alarm at t=5853s.
        self._write_output_bit(3, robot_ready)
        self._write_output_bit(4, has_alarm and robot_ready)
        # DO109 (robot access permitted) is deliberately never driven: across the
        # 4.6h mazak6.pcap capture T->O byte13 only ever holds 0x00/0x02/0x04, so
        # a real SmoothAi leaves DO106/DO109/DO110/DO111 clear.
        self._write_output_bit(104, self._machining_complete_latched and robot_ready)
        _door_name = "side" if self._variant == "smoothai" else "main"
        with self._lock:
            self.state.set_variables(
                alarm_code=self._alarm_code or 0,
                alarm_message=self._alarm_message,
                connection_up=self._connection_up,
                robot_ready=robot_ready,
                machine_stop_request=stop_request,
            )
            self._state_snapshot = {
                "alarm_code": self._alarm_code,
                "alarm_message": self._alarm_message,
                "connection_up": self._connection_up,
                "active_program": self.state.view.program,
                "cycle": self.state.view.cycle.value,
                "door_open": self.state.view.door_open(_door_name),
                "feed_hold": self._feed_hold,
            }

    def _set_alarm(self, code: int, message: str, *, abort_cycle: bool = True) -> None:
        with self._lock:
            if self._alarm_code == code and self._alarm_message == message:
                return
            self._alarm_code = code
            self._alarm_message = message
        if abort_cycle:
            # A failed work search is the exception: it does not stop the running
            # program. DO103 stayed set for 38s after the alarm in mazak3.pcap
            # (t=1821.074 to t=1859.220).
            self.state.update(cycle=CycleState.ABORTED)
            self._write_output_bit(103, False)
        self._write_output_bit(4, True)
        self.emit("alarm", code=code, message=message)
        self._publish_panel()

    def _read_input_bit(self, number: int) -> bool:
        with self._lock:
            return _get_bit(self._input_block, self._input_signal_points[number])

    def _read_output_bit(self, number: int) -> bool:
        with self._lock:
            return _get_bit(self._output_block, self._output_signal_points[number])

    def _write_input_bit(self, number: int, value: bool, *, sync_signal: bool) -> None:
        point = self._input_signal_points[number]
        with self._lock:
            changed = _set_bit(self._input_block, point, value)
        if not changed:
            return
        if sync_signal:
            self.io[point.signal].set(value)
        self._emit_snapshot_change("input")

    def _write_output_bit(self, number: int, value: bool) -> None:
        point = self._output_signal_points[number]
        with self._lock:
            changed = _set_bit(self._output_block, point, value)
        if not changed:
            return
        self.io[point.signal].set(value)
        self._emit_snapshot_change("output")

    def _write_output_field(self, number: int, value: int) -> None:
        field = OUTPUT_BIT_FIELDS[number]
        with self._lock:
            changed = _set_field(self._output_block, field, value)
        if changed:
            self._emit_snapshot_change("output")

    def _sync_input_signals(self, snapshot: bytes) -> None:
        for point in self._input_signal_points.values():
            self.io[point.signal].set(_bit_value(snapshot, point.byte, point.bit))

    def _sync_output_signals(self, snapshot: bytes) -> None:
        for point in self._output_signal_points.values():
            self.io[point.signal].set(_bit_value(snapshot, point.byte, point.bit))

    def _set_output_text(self, number: int, value: str) -> None:
        self._set_text(self._output_block, OUTPUT_TEXT_FIELDS[number], value)

    def _set_text(self, block: bytearray, field: TextField, value: str) -> None:
        encoded = value.encode("ascii", "ignore")[: field.length]
        payload = encoded.ljust(field.length, b"\x00")
        with self._lock:
            current = bytes(block[field.offset : field.offset + field.length])
            if current == payload:
                return
            block[field.offset : field.offset + field.length] = payload
        direction = "input" if block is self._input_block else "output"
        self._emit_snapshot_change(direction)

    def _read_text(self, block: bytearray, field: TextField) -> str:
        with self._lock:
            raw = bytes(block[field.offset : field.offset + field.length])
        return raw.split(b"\x00", 1)[0].decode("ascii", "ignore").strip()

    def _emit_snapshot_change(self, direction: str) -> None:
        self.emit("snapshot", interface="ethernetip", direction=direction)
        self._publish_panel()

    def _publish_panel(self) -> None:
        """Announce the detail panel if it reads differently from the last one published."""
        panel = self.build_detail()
        if panel != self._last_panel:
            self._last_panel = panel
            self.publish(PanelChanged(device=self.name, panel=panel))


def _bit_value(block: bytes | bytearray, byte: int, bit: int) -> bool:
    return bool(block[byte] & (1 << bit))


def _get_bit(block: bytes | bytearray, point: BitPoint) -> bool:
    return _bit_value(block, point.byte, point.bit)


def _set_bit(block: bytearray, point: BitPoint, value: bool) -> bool:
    mask = 1 << point.bit
    current = bool(block[point.byte] & mask)
    if current == value:
        return False
    if value:
        block[point.byte] |= mask
    else:
        block[point.byte] &= ~mask
    return True


def _set_field(block: bytearray, field: BitField, value: int) -> bool:
    mask = ((1 << field.width) - 1) << field.bit
    next_value = (block[field.byte] & ~mask) | ((value << field.bit) & mask)
    if block[field.byte] == next_value:
        return False
    block[field.byte] = next_value
    return True


def _field_rows(
    *,
    prefix: str,
    block: bytes,
    bit_points: dict[int, BitPoint],
    text_fields: dict[int, TextField],
    bit_fields: dict[int, BitField],
) -> tuple[Field, ...]:
    rows: list[Field] = []
    numbers = sorted(set(bit_points) | set(text_fields) | set(bit_fields))
    for number in numbers:
        if number in text_fields:
            field = text_fields[number]
            value = _read_text_from_bytes(block, field)
            rows.append(
                Field(
                    signal=f"{prefix}{number:03d}",
                    name=field.description,
                    offset=f"bytes {field.offset}-{field.offset + field.length - 1}",
                    type=f"ascii[{field.length}]",
                    value=value,
                )
            )
        if number in bit_points:
            point = bit_points[number]
            on = _get_bit(block, point)
            rows.append(
                Field(
                    signal=f"{prefix}{number:03d}",
                    name=point.description,
                    offset=f"byte {point.byte} bit {point.bit}",
                    type="bit",
                    value="ON" if on else "OFF",
                    on=on,
                )
            )
        if number in bit_fields:
            field = bit_fields[number]
            rows.append(
                Field(
                    signal=f"{prefix}{number:03d}",
                    name=field.description,
                    offset=f"byte {field.byte} bits {field.bit}-{field.bit + field.width - 1}",
                    type=f"u{field.width}",
                    value=str(_get_field(block, field)),
                )
            )
    return tuple(rows)


def _get_field(block: bytes | bytearray, field: BitField) -> int:
    mask = (1 << field.width) - 1
    return (block[field.byte] >> field.bit) & mask


def _read_text_from_bytes(block: bytes, field: TextField) -> str:
    raw = block[field.offset : field.offset + field.length]
    return raw.split(b"\x00", 1)[0].decode("ascii", "ignore").strip()


def _build_input_points(variant: str) -> dict[int, BitPoint]:
    points = dict(INPUT_SIGNAL_POINTS)
    if variant == "smoothai":
        points.update(SMOOTH_AI_INPUT_OVERRIDES)
    return points


def _build_output_points(variant: str) -> dict[int, BitPoint]:
    points = dict(OUTPUT_SIGNAL_POINTS)
    if variant == "smoothai":
        points.update(SMOOTH_AI_OUTPUT_OVERRIDES)
    return points


def _build_ethernetip_transport(
    endpoint: Endpoint, options: MazakSmoothOptions
) -> EtherNetIPTransport:
    """The machine's end of the EtherNet/IP link, from its typed ``ethernetip`` block."""
    eip = options.ethernetip if options.ethernetip is not None else EtherNetIPOptions()
    if eip.mode == "scanner":
        return _build_scanner(_scanner_config(eip))
    return _build_adapter(_adapter_config(endpoint, eip))


def _adapter_config(endpoint: Endpoint, eip: EtherNetIPOptions) -> EtherNetIPAdapterConfig:
    return EtherNetIPAdapterConfig(
        host=endpoint.host,
        port=endpoint.port,
        udp_port=eip.udp_port,
        output_length=BLOCK_SIZE,
        input_length=BLOCK_SIZE,
        requested_packet_rate_ms=eip.requested_packet_rate_ms,
        o_t_realtime_format=eip.o_t_realtime_format,
        behaviour=eip.behaviour,
    )


def _scanner_config(eip: EtherNetIPOptions) -> EtherNetIPScannerConfig:
    if not eip.host or eip.host.strip() in {"0.0.0.0", "::"}:
        raise ValueError(
            "ethernetip.host must be the remote robot adapter address; "
            "mazak_smooth acts as an outbound scanner and does not listen for inbound "
            "EtherNet/IP connections"
        )
    return EtherNetIPScannerConfig(
        host=eip.host.strip(),
        port=eip.port,
        originator_udp_port=eip.originator_udp_port,
        target_udp_port=eip.target_udp_port,
        assembly_object_class=eip.assembly_object_class,
        configuration_assembly_instance_id=eip.configuration_assembly_instance_id,
        output_assembly_instance_id=eip.output_assembly_instance_id,
        input_assembly_instance_id=eip.input_assembly_instance_id,
        output_length=BLOCK_SIZE,
        input_length=BLOCK_SIZE,
        requested_packet_rate_ms=eip.requested_packet_rate_ms,
        o_t_realtime_format=eip.o_t_realtime_format,
        o_t_connection_type=eip.o_t_connection_type,
        t_o_connection_type=eip.t_o_connection_type,
    )


def _build_adapter(config: EtherNetIPAdapterConfig) -> EtherNetIPAdapter:
    if config.behaviour == "mazak":
        return MazakEthernetIPAdapter(config)
    return EtherNetIPAdapter(config)


def _build_scanner(config: EtherNetIPScannerConfig) -> EtherNetIPScanner:
    return EtherNetIPScanner(config)


def make_device(
    name: str, endpoint: Endpoint, bus: EventBus, options_obj: MazakSmoothOptions,
) -> MazakSmoothEmulator:
    """Build a :class:`MazakSmoothEmulator` with full service wiring. Does NOT start services."""
    device = MazakSmoothEmulator(name, endpoint, bus, options_obj, io=SignalBank(owner=name, publish=bus.publish))
    if options_obj.mtconnect is not None:
        device.add_service(
            MTConnectAgent(
                endpoint.host,
                options_obj.mtconnect.port,
                render=lambda render_endpoint: render_mtconnect(device.state.view, render_endpoint),
            )
        )
    if "ethernetip" in device._interfaces:
        device.attach_ethernetip(_build_ethernetip_transport(endpoint, options_obj))
    return device


@register("mazak_smooth", default_port=44818, options=MazakSmoothOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: MazakSmoothOptions) -> Device:
    return make_device(name, endpoint, bus, options)
