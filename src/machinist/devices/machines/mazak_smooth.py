"""Mazak Smooth robot-interface emulator with IO and EtherNet/IP support."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Any

from ...core.device import Device, DetailField, DetailSignal, DeviceDetail
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.ethernetip import (
    EtherNetIPAdapter,
    EtherNetIPAdapterConfig,
    EtherNetIPScanner,
    EtherNetIPScannerConfig,
    MazakEthernetIPAdapter,
)
from ...transport.mtconnect import MTConnectAgent, render_mtconnect
from .state import CycleState, MachineState

BLOCK_SIZE = 110
PROGRAM_OFFSET = 44
PROGRAM_LENGTH = 32
CONTROL_OFFSET = 12
HEARTBEAT_ALARM = 1362


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


@dataclass(frozen=True, slots=True)
class MTConnectOptions:
    port: int


@dataclass(frozen=True, slots=True)
class MazakSmoothOptions:
    scan_interval_seconds: float = 0.02
    door_move_seconds: float = 2.0
    cycle_duration_seconds: float = 1.0
    work_search_seconds: float = 0.1
    heartbeat_interval_seconds: float = 2.0
    heartbeat_timeout_seconds: float = 6.0 # real mazak is ~10, but we are impatient
    interfaces: Any = None
    main_interface: Any = None
    ethernetip: dict[str, Any] | None = None
    ethernetip_mode: str = "adapter"
    ethernetip_adapter_config: EtherNetIPAdapterConfig | None = None
    ethernetip_scanner_config: EtherNetIPScannerConfig | None = None
    mtconnect: MTConnectOptions | None = None
    _transport_factory: Any = None
    _eeip_client_factory: Any = None


class MazakSmoothEmulator(Device):
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
        self.state = MachineState()
        self.state.door("main").set(open=False)

        self._lock = threading.RLock()
        self._input_block = bytearray(BLOCK_SIZE)
        self._output_block = bytearray(BLOCK_SIZE)
        self._state_snapshot: dict[str, object] = {}
        self._scan_interval = options.scan_interval_seconds
        self._door_seconds = options.door_move_seconds
        self._cycle_seconds = options.cycle_duration_seconds
        self._work_search_seconds = options.work_search_seconds
        self._heartbeat_interval = options.heartbeat_interval_seconds
        self._heartbeat_timeout = options.heartbeat_timeout_seconds
        self._interfaces = _enabled_interfaces(options)
        self._ethernetip_mode = options.ethernetip_mode
        self._io_writable = "io" in self._interfaces
        self._alarm_code: int | None = None
        self._alarm_message = ""
        self._connection_up = False
        self._door_motion_deadline: float | None = None
        self._door_target_open: bool | None = None
        self._cycle_complete_deadline: float | None = None
        self._work_search_deadline: float | None = None
        self._pending_program = ""
        self._prev_di101 = False
        self._prev_di102 = False
        self._prev_di107 = False
        self._prev_di108 = False
        self._cycle_start_armed = False
        self._feed_hold = False
        self._machining_complete_latched = False
        self._last_heartbeat_toggle_at = -self._heartbeat_interval
        self._di000_toggle: tuple[float, bool | None] = (0.0, None)
        self._last_connection_gen = -1

        self.io = io
        self._declare_signals()

        self._mtconnect: MTConnectAgent | None = None
        self._ethernetip: EtherNetIPAdapter | EtherNetIPScanner | None = None
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
        return self.state.program

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

    def build_detail(self) -> DeviceDetail:
        """Assemble the normalized detail dict for this Smooth device."""
        if "ethernetip" not in self._interfaces:
            return DeviceDetail(
                mode="io",
                transport_ready=False,
                peer_connected=False,
                clients=None,
                input_block_hex="",
                output_block_hex="",
                input_fields=[],
                output_fields=[],
                derived_fields=[],
                signals=[],
            )

        with self._lock:
            input_block = bytes(self._input_block)
            output_block = bytes(self._output_block)
            alarm_code = self._alarm_code
            alarm_message = self._alarm_message
            connection_up = self._connection_up
            active_program = self.state.program
            transport = self._ethernetip

        transport_ready = bool(getattr(transport, "connected", False))
        peer_connected = bool(
            getattr(transport, "peer_connected", transport_ready)
        )

        io = getattr(self, "io", None)
        signals: list[DetailSignal] = []
        if io is not None:
            signals = [
                DetailSignal(name=sig.name, direction=str(sig.direction), value=sig.value)
                for sig in io
            ]

        input_fields = _field_rows(
            prefix="DI",
            block=input_block,
            bit_points=INPUT_SIGNAL_POINTS,
            text_fields=INPUT_TEXT_FIELDS,
            bit_fields={},
        )
        output_fields = _field_rows(
            prefix="DO",
            block=output_block,
            bit_points=OUTPUT_SIGNAL_POINTS,
            text_fields=OUTPUT_TEXT_FIELDS,
            bit_fields=OUTPUT_BIT_FIELDS,
        )
        derived_fields: list[DetailField] = [
            {
                "signal": "STATE",
                "name": "Active program",
                "offset": "-",
                "type": "string",
                "value": active_program or "",
            },
            {
                "signal": "STATE",
                "name": "Connection up",
                "offset": "-",
                "type": "bool",
                "value": "ON" if connection_up else "OFF",
            },
            {
                "signal": "STATE",
                "name": "Alarm code",
                "offset": "-",
                "type": "int",
                "value": "" if alarm_code is None else str(alarm_code),
            },
            {
                "signal": "STATE",
                "name": "Alarm message",
                "offset": "-",
                "type": "string",
                "value": alarm_message,
            },
        ]

        return DeviceDetail(
            mode=self._ethernetip_mode,
            transport_ready=transport_ready,
            peer_connected=peer_connected,
            clients=None,
            input_block_hex=input_block.hex(" "),
            output_block_hex=output_block.hex(" "),
            input_fields=input_fields,
            output_fields=output_fields,
            derived_fields=derived_fields,
            signals=signals,
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

    def _declare_signals(self) -> None:
        for point in INPUT_SIGNAL_POINTS.values():
            signal = self.io.declare(point.signal, Direction.INPUT)
            if self._io_writable:
                signal.subscribe(
                    lambda value, number=point.number: self._write_input_bit(
                        number, value, sync_signal=False
                    )
                )
        for point in OUTPUT_SIGNAL_POINTS.values():
            self.io.declare(point.signal, Direction.OUTPUT)

    def _build_mtconnect(
        self, host: str, options: MazakSmoothOptions
    ) -> MTConnectAgent | None:
        if options.mtconnect is None:
            return None
        return MTConnectAgent(
            host,
            options.mtconnect.port,
            render=lambda render_endpoint: render_mtconnect(self.state, render_endpoint),
        )

    def _initialize_defaults(self) -> None:
        self._set_output_text(100, "")
        self._write_output_field(105, 0)
        self._write_input_bit(2, True, sync_signal=True)
        self._write_output_bit(2, True)
        self._write_output_bit(3, True)
        self._write_output_bit(108, True)
        self._refresh_outputs()

    def _run(self, stop: threading.Event) -> None:
        mtconnect_thread: threading.Thread | None = None
        if self._mtconnect is not None:
            ready = threading.Event()
            mtconnect_thread = threading.Thread(
                target=self._mtconnect.serve_forever, args=(ready,), daemon=True
            )
            mtconnect_thread.start()
            ready.wait(timeout=2.0)

        if isinstance(self._ethernetip, EtherNetIPAdapter):
            self._ethernetip.open()
        self._mark_running()
        while not stop.is_set():
            now = time.monotonic()
            self._poll_ethernetip(now)
            self._scan_cycle(now=now)
            stop.wait(self._scan_interval)

        if self._ethernetip is not None:
            self._ethernetip.close()
        if self._mtconnect is not None:
            self._mtconnect.shutdown()
        if mtconnect_thread is not None:
            mtconnect_thread.join(timeout=2.0)

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
        self._connection_up = bool(getattr(transport, "peer_connected", transport.connected))
        if was_down and self._connection_up:
            self.emit("ethernetip.connected", message="connection established")
        self.write_input_block(incoming)
        gen = getattr(transport, "connection_generation", -1)
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
        if di101 and not self._prev_di101:
            self._pending_program = self._read_text(self._input_block, INPUT_TEXT_FIELDS[100])
            self._write_output_bit(101, False)
            self._work_search_deadline = now + self._work_search_seconds
        if self._work_search_deadline is not None and now >= self._work_search_deadline:
            self.state.program = self._pending_program
            self._set_output_text(100, self._pending_program)
            self._write_output_bit(101, True)
            self.emit("program", program=self._pending_program)
            self._work_search_deadline = None
        if not di101 and self._read_output_bit(101):
            self._write_output_bit(101, False)
        self._prev_di101 = di101

    def _handle_door_motion(self, now: float) -> None:
        stop_request = self._read_input_bit(2)
        di107 = self._read_input_bit(107)
        di108 = self._read_input_bit(108)
        di109 = self._read_input_bit(109)

        if di107 and not self._prev_di107 and self._door_motion_deadline is None:
            self._door_target_open = True
            self._door_motion_deadline = now + self._door_seconds
            self._write_output_bit(107, False)
            self._write_output_bit(108, False)

        if di108 and not self._prev_di108 and di109 and self._door_motion_deadline is None:
            self._door_target_open = False
            self._door_motion_deadline = now + self._door_seconds
            self._write_output_bit(107, False)
            self._write_output_bit(108, False)

        if not stop_request:
            self._door_motion_deadline = None
            self._door_target_open = None
        elif self._door_motion_deadline is not None:
            if (self._door_target_open and not di107) or (
                self._door_target_open is False and (not di108 or not di109)
            ):
                self._door_motion_deadline = None
                self._door_target_open = None

        if self._door_motion_deadline is not None and now >= self._door_motion_deadline:
            target_open = bool(self._door_target_open)
            self.state.door("main").set(open=target_open)
            self._write_output_bit(107, target_open)
            self._write_output_bit(108, not target_open)
            self._door_motion_deadline = None
            self._door_target_open = None
            self.emit("door", open=target_open)

        self._prev_di107 = di107
        self._prev_di108 = di108

    def _handle_cycle(self, now: float) -> None:
        robot_ready = self._read_input_bit(1)
        stop_request = self._read_input_bit(2)
        cycle_start = self._read_input_bit(102)
        nc_reset = self._read_input_bit(103)
        all_machining_complete = self._read_input_bit(104)

        if not stop_request:
            self._feed_hold = True
            self.state.cycle = CycleState.PAUSED
            self._cycle_complete_deadline = None
            self._cycle_start_armed = False
            self._write_output_bit(103, False)
        elif self._feed_hold and stop_request and self.state.cycle is CycleState.PAUSED:
            self._feed_hold = False

        if nc_reset:
            self.state.cycle = CycleState.IDLE
            self._cycle_complete_deadline = None
            self._cycle_start_armed = False
            self._machining_complete_latched = False
            self._write_output_bit(103, False)
            self._write_output_bit(104, False)

        can_cycle = (
            robot_ready
            and stop_request
            and self._alarm_code is None
            and not self.state.door("main").open
            and self.state.cycle is not CycleState.RUNNING
        )
        self._write_output_bit(102, can_cycle)

        if cycle_start and not self._prev_di102 and can_cycle:
            self._cycle_start_armed = True
        elif cycle_start and not can_cycle:
            self._cycle_start_armed = False

        if not cycle_start and self._prev_di102 and self._cycle_start_armed and can_cycle:
            self.state.cycle = CycleState.RUNNING
            self._cycle_complete_deadline = now + self._cycle_seconds
            self._cycle_start_armed = False
            self._machining_complete_latched = False
            self._write_output_bit(103, True)
            self._write_output_bit(104, False)
            self.emit("cycle.start", program=self.state.program or self._pending_program)
        elif not cycle_start and self._prev_di102:
            self._cycle_start_armed = False

        if self._cycle_complete_deadline is not None and now >= self._cycle_complete_deadline:
            self._complete_cycle()

        if all_machining_complete:
            self._complete_cycle()

        self._prev_di102 = cycle_start

    def _complete_cycle(self) -> None:
        if self.state.cycle is not CycleState.RUNNING:
            return
        self.state.cycle = CycleState.IDLE
        self.state.parts += 1
        self._cycle_complete_deadline = None
        self._machining_complete_latched = True
        self._write_output_bit(103, False)
        self._write_output_bit(104, True)
        self.emit("cycle.end", parts=self.state.parts)

    def _refresh_outputs(self) -> None:
        robot_ready = self._read_input_bit(1)
        stop_request = self._read_input_bit(2)
        has_alarm = self._alarm_code is not None
        self._write_output_bit(1, robot_ready and not has_alarm)
        self._write_output_bit(2, stop_request and not has_alarm)
        self._write_output_bit(4, has_alarm)
        self._write_output_bit(
            109,
            robot_ready and not has_alarm and self.state.cycle is not CycleState.RUNNING,
        )
        with self._lock:
            self.state.variables["alarm_code"] = self._alarm_code or 0
            self.state.variables["alarm_message"] = self._alarm_message
            self.state.variables["connection_up"] = self._connection_up
            self.state.variables["robot_ready"] = robot_ready
            self.state.variables["machine_stop_request"] = stop_request
            self._state_snapshot = {
                "alarm_code": self._alarm_code,
                "alarm_message": self._alarm_message,
                "connection_up": self._connection_up,
                "active_program": self.state.program,
                "cycle": self.state.cycle.value,
                "door_open": self.state.door("main").open,
                "feed_hold": self._feed_hold,
            }

    def _set_alarm(self, code: int, message: str) -> None:
        with self._lock:
            if self._alarm_code == code and self._alarm_message == message:
                return
            self._alarm_code = code
            self._alarm_message = message
        self.state.cycle = CycleState.ABORTED
        self._write_output_bit(103, False)
        self._write_output_bit(4, True)
        self.emit("alarm", code=code, message=message)

    def _read_input_bit(self, number: int) -> bool:
        with self._lock:
            return _get_bit(self._input_block, INPUT_SIGNAL_POINTS[number])

    def _read_output_bit(self, number: int) -> bool:
        with self._lock:
            return _get_bit(self._output_block, OUTPUT_SIGNAL_POINTS[number])

    def _write_input_bit(self, number: int, value: bool, *, sync_signal: bool) -> None:
        point = INPUT_SIGNAL_POINTS[number]
        with self._lock:
            changed = _set_bit(self._input_block, point, value)
        if not changed:
            return
        if sync_signal:
            self.io[point.signal].set(value)
        self._emit_snapshot_change("input")

    def _write_output_bit(self, number: int, value: bool) -> None:
        point = OUTPUT_SIGNAL_POINTS[number]
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
        for point in INPUT_SIGNAL_POINTS.values():
            self.io[point.signal].set(_bit_value(snapshot, point.byte, point.bit))

    def _sync_output_signals(self, snapshot: bytes) -> None:
        for point in OUTPUT_SIGNAL_POINTS.values():
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
) -> list[DetailField]:
    rows: list[DetailField] = []
    numbers = sorted(set(bit_points) | set(text_fields) | set(bit_fields))
    for number in numbers:
        if number in text_fields:
            field = text_fields[number]
            value = _read_text_from_bytes(block, field)
            rows.append(
                {
                    "signal": f"{prefix}{number:03d}",
                    "name": field.description,
                    "offset": (
                        f"bytes {field.offset}-{field.offset + field.length - 1}"
                    ),
                    "type": f"ascii[{field.length}]",
                    "value": value,
                }
            )
        if number in bit_points:
            point = bit_points[number]
            rows.append(
                {
                    "signal": f"{prefix}{number:03d}",
                    "name": point.description,
                    "offset": f"byte {point.byte} bit {point.bit}",
                    "type": "bit",
                    "value": "ON" if _get_bit(block, point) else "OFF",
                }
            )
        if number in bit_fields:
            field = bit_fields[number]
            rows.append(
                {
                    "signal": f"{prefix}{number:03d}",
                    "name": field.description,
                    "offset": (
                        f"byte {field.byte} bits "
                        f"{field.bit}-{field.bit + field.width - 1}"
                    ),
                    "type": f"u{field.width}",
                    "value": str(_get_field(block, field)),
                }
            )
    return rows


def _get_field(block: bytes | bytearray, field: BitField) -> int:
    mask = (1 << field.width) - 1
    return (block[field.byte] >> field.bit) & mask


def _read_text_from_bytes(block: bytes, field: TextField) -> str:
    raw = block[field.offset : field.offset + field.length]
    return raw.split(b"\x00", 1)[0].decode("ascii", "ignore").strip()


def _enabled_interfaces(options: MazakSmoothOptions) -> set[str]:
    enabled = {"io"}
    raw = options.interfaces
    if isinstance(raw, str):
        enabled = {raw.strip().lower()}
    elif isinstance(raw, dict):
        enabled = {name.strip().lower() for name, flag in raw.items() if flag}
    elif raw is not None:
        enabled = {str(item).strip().lower() for item in raw}
    main_interface = options.main_interface
    if isinstance(main_interface, str):
        enabled.add(main_interface.strip().lower())
    elif isinstance(main_interface, (list, tuple, set)):
        enabled.update(str(item).strip().lower() for item in main_interface)
    if options.ethernetip is not None or options.ethernetip_adapter_config is not None or options.ethernetip_scanner_config is not None:
        enabled.add("ethernetip")
    return enabled


def _build_ethernetip_transport(
    endpoint: Endpoint, options: MazakSmoothOptions
) -> EtherNetIPAdapter | EtherNetIPScanner:
    mode = options.ethernetip_mode
    if mode == "scanner":
        config = options.ethernetip_scanner_config
        assert config is not None, "ethernetip_scanner_config must be set for scanner mode"
        return _build_scanner(config, options)
    if mode == "adapter":
        config = options.ethernetip_adapter_config
        assert config is not None, "ethernetip_adapter_config must be set for adapter mode"
        return _build_adapter(config)
    raise ValueError("ethernetip.mode must be either 'adapter' or 'scanner'")


def _build_adapter(config: EtherNetIPAdapterConfig) -> EtherNetIPAdapter:
    if config.behaviour == "mazak":
        return MazakEthernetIPAdapter(config)
    return EtherNetIPAdapter(config)


def _build_scanner(
    config: EtherNetIPScannerConfig,
    options: MazakSmoothOptions,
) -> EtherNetIPScanner:
    host = config.host
    if host in {"", "0.0.0.0", "::"}:
        raise ValueError(
            "ethernetip.host must be the remote robot adapter address; "
            "mazak_smooth acts as an outbound scanner and does not listen for inbound "
            "EtherNet/IP connections"
        )
    transport_factory = options._transport_factory or EtherNetIPScanner
    client_factory = options._eeip_client_factory
    if client_factory is None:
        return transport_factory(config)
    return transport_factory(config, client_factory=client_factory)


def make_device(
    name: str, endpoint: Endpoint, bus: EventBus, options_obj: MazakSmoothOptions,
) -> MazakSmoothEmulator:
    """Build a :class:`MazakSmoothEmulator` with full service wiring. Does NOT start services."""
    device = MazakSmoothEmulator(name, endpoint, bus, options_obj, io=SignalBank(owner=name))
    if options_obj.mtconnect is not None:
        device._mtconnect = MTConnectAgent(
            endpoint.host,
            options_obj.mtconnect.port,
            render=lambda render_endpoint: render_mtconnect(device.state, render_endpoint),
        )
    if "ethernetip" in device._interfaces:
        device._ethernetip = _build_ethernetip_transport(endpoint, options_obj)
    return device


@register("mazak_smooth", default_port=44818)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    raw_mtconnect_port = opts.pop("mtconnect_port", None)
    raw_mtconnect = opts.pop("mtconnect", None)
    if raw_mtconnect_port is not None:
        opts["mtconnect"] = MTConnectOptions(port=int(raw_mtconnect_port))
    elif raw_mtconnect is not None:
        opts["mtconnect"] = MTConnectOptions(**raw_mtconnect) if isinstance(raw_mtconnect, dict) else raw_mtconnect
    raw_ethernetip = opts.get("ethernetip")
    if isinstance(raw_ethernetip, dict):
        mode = str(raw_ethernetip.get("mode", "adapter")).strip().lower()
        opts["ethernetip_mode"] = mode
        if mode == "adapter":
            opts["ethernetip_adapter_config"] = EtherNetIPAdapterConfig(
                host=endpoint.host,
                port=endpoint.port,
                udp_port=int(raw_ethernetip.get("udp_port", 2222)),
                output_length=BLOCK_SIZE,
                input_length=BLOCK_SIZE,
                requested_packet_rate_ms=int(raw_ethernetip.get("requested_packet_rate_ms", 20)),
                o_t_realtime_format=str(raw_ethernetip.get("o_t_realtime_format", "header32bit")),
                behaviour=str(raw_ethernetip.get("behaviour", "mazak")),
            )
        elif mode == "scanner":
            opts["ethernetip_scanner_config"] = EtherNetIPScannerConfig(
                host=str(raw_ethernetip["host"]).strip(),
                port=int(raw_ethernetip.get("port", 44818)),
                originator_udp_port=int(raw_ethernetip.get("originator_udp_port", 2222)),
                target_udp_port=int(raw_ethernetip.get("target_udp_port", 2222)),
                assembly_object_class=int(raw_ethernetip.get("assembly_object_class", 0x04)),
                configuration_assembly_instance_id=int(
                    raw_ethernetip.get("configuration_assembly_instance_id", 0x01)
                ),
                output_assembly_instance_id=int(raw_ethernetip.get("output_assembly_instance_id", 0x64)),
                input_assembly_instance_id=int(raw_ethernetip.get("input_assembly_instance_id", 0x65)),
                output_length=BLOCK_SIZE,
                input_length=BLOCK_SIZE,
                requested_packet_rate_ms=int(raw_ethernetip.get("requested_packet_rate_ms", 20)),
                o_t_realtime_format=str(raw_ethernetip.get("o_t_realtime_format", "header32bit")),
                o_t_connection_type=str(raw_ethernetip.get("o_t_connection_type", "point_to_point")),
                t_o_connection_type=str(raw_ethernetip.get("t_o_connection_type", "point_to_point")),
            )
    options_obj = MazakSmoothOptions(**opts)
    return make_device(name, endpoint, bus, options_obj)
