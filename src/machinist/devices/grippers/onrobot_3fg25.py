"""OnRobot 3FG25 three-finger gripper, controlled over Modbus/TCP.

Spec reference (public): https://onrobot.com/en/products/3fg25
The physical gripper exposes its full state via Modbus holding
registers on TCP port 502.

Device-specific registers
=========================

============ ========== ===================================
Address      Access     Meaning
============ ========== ===================================
0x0000       Write      Target force (10*%)
0x0001       Write      Target diameter (.1 mm)
0x0002       Write      Grip type
0x0003       Write      Control
0x0100       Read       Status flags
0x0101       Read       Raw diameter (.1 mm)
0x0102       Read       Diameter w/ fingertip offset (.1 mm)
0x0103       Read       Force applied (1/10 %)
0x010E       Read       Finger length (.1 mm)
0x0110       Read       Finger position (index 1/2/3)
0x0111       Read       Fingertip offset (.01 mm)
0x0113       Read       Actual width with offset (.1 mm)
0x0201       Read       Minimum diameter (.1 mm)
0x0202       Read       Maximum diameter (.1 mm)
0x0401       Read/Write Set finger length (.1 mm)
0x0403       Read/Write Set finger position (index 1/2/3)
0x0404       Read/Write Set fingertip offset (.01 mm)
============ ========== ===================================

Common registers (all OnRobot grippers)
========================================

============ ========== ===============================================
Address      Access     Meaning
============ ========== ===============================================
0x0600       Read       Product code (3FG25 = 0x71, 3FG15 = 0x70)
0x0604       Read       Firmware version: upper byte = major, lower = minor
0x0605       Read       Firmware build number
0x0609-0x0618  Read     Serial number (32 bytes, 2 ASCII chars per register)
============ ========== ===============================================
"""

from __future__ import annotations

import math
import threading
from dataclasses import dataclass, field
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.modbus_server import HoldingRegisterServer

# --- write registers --------------------------------------------------------
REG_TARGET_FORCE = 0x0000
REG_TARGET_DIAMETER = 0x0001
REG_GRIP_TYPE = 0x0002
REG_CONTROL = 0x0003

# --- read-only registers ----------------------------------------------------
REG_STATUS = 0x0100
REG_RAW_DIAMETER = 0x0101
REG_DIAMETER_WITH_OFFSET = 0x0102
REG_FORCE_APPLIED = 0x0103
REG_FINGER_LENGTH = 0x010E
REG_FINGER_POSITION = 0x0110
REG_FINGERTIP_OFFSET = 0x0111
REG_MIN_DIAMETER = 0x0201
REG_MAX_DIAMETER = 0x0202

# --- read/write registers ---------------------------------------------------
REG_SET_FINGER_LENGTH = 0x0401
REG_SET_FINGER_POSITION = 0x0403
REG_SET_FINGERTIP_OFFSET = 0x0404

STATUS_BUSY = 0x01
STATUS_GRIPPED = 0x02

# --- common registers (all OnRobot grippers) --------------------------------
REG_PRODUCT_CODE = 0x0600
REG_FW_MAJOR_MINOR = 0x0604
REG_FW_BUILD = 0x0605
REG_SERIAL_BASE = 0x0609
REG_SERIAL_END = 0x0618

#: Product codes from the OnRobot common register spec.
PRODUCT_3FG25 = 0x71
PRODUCT_3FG15 = 0x70

# --- finger geometry ---------------------------------------------------------

#: Distance from gripper center to each finger motor (.1 mm).
MOTOR_RADIUS_TENTHS = 370  # 37.0 mm

#: Offsets for the three finger mounting positions (.1 mm).
_POSITION_OFFSETS: dict[int, int] = {1: -180, 2: -60, 3: 60}

#: Angular step per tick at 50 Hz (~36 °/s).
ANGLE_STEP_TENTHS = 7  # 0.7 degree per tick

#: Default finger geometry for the 3FG25.
#: The 3FG15 shares the same register map but with different defaults.
FINGER_LENGTH_25_TENTHS = 485  # 48.5 mm
FINGERTIP_OFFSET_25_HUNDREDTHS = 650  # 6.5 mm
FINGER_POSITION_25 = 2  # mounting position (1 / 2 / 3)


def _angle_to_width(
    angle_tenths: int,
    finger_length_tenths: int,
    position_offset_tenths: int,
    fingertip_offset_tenths: int,
) -> int:
    angle_rad = math.radians(angle_tenths / 10)
    eff_len = finger_length_tenths + position_offset_tenths
    radius_sq = (
        MOTOR_RADIUS_TENTHS**2
        + eff_len**2
        + 2 * MOTOR_RADIUS_TENTHS * eff_len * math.cos(angle_rad)
    )
    radial = math.sqrt(radius_sq) - fingertip_offset_tenths
    return max(0, int(2 * radial))


def _width_to_angle(
    width_tenths: int,
    finger_length_tenths: int,
    position_offset_tenths: int,
    fingertip_offset_tenths: int,
) -> int:
    eff_len = finger_length_tenths + position_offset_tenths
    radial_to_contact = width_tenths / 2
    radial_to_joint = radial_to_contact + fingertip_offset_tenths
    cos_a = (
        radial_to_joint**2
        - MOTOR_RADIUS_TENTHS**2
        - eff_len**2
    ) / (2 * MOTOR_RADIUS_TENTHS * eff_len)
    cos_a = max(-1.0, min(1.0, cos_a))
    return int(round(math.degrees(math.acos(cos_a)) * 10))


@dataclass(slots=True)
class OnRobot3FG25Options:
    initial_diameter_mm: float = 75.0
    travel_mm_per_sec: float = 60.0
    finger_length_tenths: int = FINGER_LENGTH_25_TENTHS
    fingertip_offset_hundredths: int = FINGERTIP_OFFSET_25_HUNDREDTHS
    finger_position: int = FINGER_POSITION_25


@dataclass(slots=True)
class _State:
    actual_angle_tenths: int = 0
    target_angle_tenths: int = 0
    force: int = 40
    grip: bool = False
    grip_type: int = 0
    control: int = 0
    finger_length_tenths: int = FINGER_LENGTH_25_TENTHS
    finger_position: int = FINGER_POSITION_25
    fingertip_offset_hundredths: int = FINGERTIP_OFFSET_25_HUNDREDTHS
    busy: bool = False
    gripped: bool = False
    product_code: int = PRODUCT_3FG25
    fw_major: int = 0
    fw_minor: int = 0
    fw_build: int = 0
    serial: str = ""
    lock: threading.Lock = field(default_factory=threading.Lock)


class OnRobot3FG25(Device):
    kind = "onrobot_3fg25"
    DEFAULT_PORT = 502

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: OnRobot3FG25Options,
        *, state: _State,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._settings = options
        self._state = state
        self._server: HoldingRegisterServer | None = None
        self._mover: threading.Thread | None = None

    def _width(self) -> int:
        s = self._state
        pos_offset = _POSITION_OFFSETS.get(s.finger_position, 0)
        return _angle_to_width(
            s.actual_angle_tenths,
            s.finger_length_tenths,
            pos_offset,
            s.fingertip_offset_hundredths//10,
        )

    def _target_width(self) -> int:
        s = self._state
        pos_offset = _POSITION_OFFSETS.get(s.finger_position, 0)
        return _angle_to_width(
            s.target_angle_tenths,
            s.finger_length_tenths,
            pos_offset,
            s.fingertip_offset_hundredths//10,
        )

    def _on_read(self, address: int) -> int:
        s = self._state
        with s.lock:
            if REG_SERIAL_BASE <= address <= REG_SERIAL_END:
                return self._serial_register(address, s.serial)
            return {
                REG_TARGET_FORCE: s.force,
                REG_TARGET_DIAMETER: self._target_width(),
                REG_GRIP_TYPE: s.grip_type,
                REG_CONTROL: s.control,
                REG_STATUS: (STATUS_BUSY if s.busy else 0) | (STATUS_GRIPPED if s.gripped else 0),
                REG_RAW_DIAMETER: self._width(),
                REG_DIAMETER_WITH_OFFSET: self._width() - s.fingertip_offset_hundredths // 10 * 2,
                REG_FORCE_APPLIED: s.force,
                REG_FINGER_LENGTH: s.finger_length_tenths,
                REG_FINGER_POSITION: s.finger_position,
                REG_FINGERTIP_OFFSET: s.fingertip_offset_hundredths,
                REG_MIN_DIAMETER: 0,
                REG_MAX_DIAMETER: 1000,
                REG_SET_FINGER_LENGTH: s.finger_length_tenths,
                REG_SET_FINGER_POSITION: s.finger_position,
                REG_SET_FINGERTIP_OFFSET: s.fingertip_offset_hundredths,
                REG_PRODUCT_CODE: s.product_code,
                REG_FW_MAJOR_MINOR: (s.fw_major << 8) | s.fw_minor,
                REG_FW_BUILD: s.fw_build,
            }.get(address, 0)

    def modbus_snapshot(self) -> dict[str, object]:
        """Expose Modbus register values for the TUI and web interfaces."""
        s = self._state
        status = (STATUS_BUSY if s.busy else 0) | (STATUS_GRIPPED if s.gripped else 0)
        pos_offset = _POSITION_OFFSETS.get(s.finger_position, 0)
        actual_tenths = _angle_to_width(
            s.actual_angle_tenths, s.finger_length_tenths,
            pos_offset, s.fingertip_offset_hundredths // 10,
        )
        target_tenths = _angle_to_width(
            s.target_angle_tenths, s.finger_length_tenths,
            pos_offset, s.fingertip_offset_hundredths // 10,
        )

        def _reg(signal: str, name: str, offset: str, type_: str, value: object) -> dict[str, str]:
            return {"signal": signal, "name": name, "offset": offset, "type": type_, "value": str(value)}

        server = self._server
        return {
            "mode": "modbus",
            "transport_ready": server is not None and server._sock is not None,
            "peer_connected": server is not None and server.client_count > 0,
            "clients": server.client_count if server is not None else 0,
            "input_block_hex": "",
            "output_block_hex": "",
            "input_fields": [
                _reg("T_FORCE", "Target force", "0x0000", "int", f"{s.force} (10*%)"),
                _reg("T_DIA", "Target diameter", "0x0001", "int", f"{target_tenths} (.1 mm)"),
                _reg("GRIP_TYPE", "Grip type", "0x0002", "int", str(s.grip_type)),
                _reg("CTRL", "Control", "0x0003", "hex", f"0x{s.control:04X}"),
                _reg("SET_FLEN", "Set finger length", "0x0401", "int", f"{s.finger_length_tenths} (.1 mm)"),
                _reg("SET_FPOS", "Set finger position", "0x0403", "int", str(s.finger_position)),
                _reg("SET_FTOF", "Set fingertip offset", "0x0404", "int", f"{s.fingertip_offset_hundredths} (.01 mm)"),
            ],
            "output_fields": [
                _reg("STATUS", "Status flags", "0x0100", "hex", f"0x{status:04X}"),
                _reg("RAW_DIA", "Raw diameter", "0x0101", "int", f"{actual_tenths} (.1 mm)"),
                _reg("DIA_OFF", "Diameter w/ offset", "0x0102", "int", f"{actual_tenths - s.fingertip_offset_hundredths // 10 * 2} (.1 mm)"),
                _reg("FORCE", "Force applied", "0x0103", "int", f"{s.force} (1/10 %)"),
                _reg("FLENGTH", "Finger length", "0x010E", "int", f"{s.finger_length_tenths} (.1 mm)"),
                _reg("FPOS", "Finger position", "0x0110", "int", str(s.finger_position)),
                _reg("FTOFFSET", "Fingertip offset", "0x0111", "int", f"{s.fingertip_offset_hundredths} (.01 mm)"),
                _reg("MIN_DIA", "Minimum diameter", "0x0201", "int", "250 (.1 mm)"),
                _reg("MAX_DIA", "Maximum diameter", "0x0202", "int", "1400 (.1 mm)"),
                _reg("PROD", "Product code", "0x0600", "hex", f"0x{s.product_code:02X}"),
                _reg("FW", "Firmware version", "0x0604", "hex", f"{s.fw_major}.{s.fw_minor}"),
            ],
            "derived_fields": [
                _reg("DIAMETER", "Actual diameter", "", "mm", f"{actual_tenths / 10:.1f}"),
                _reg("ANGLE", "Finger angle", "", "deg", f"{s.actual_angle_tenths / 10:.1f}"),
                _reg("BUSY", "Moving", "", "bit", "1" if s.busy else "0"),
                _reg("GRIPPED", "Object gripped", "", "bit", "1" if s.gripped else "0"),
            ],
        }

    @staticmethod
    def _serial_register(address: int, serial: str) -> int:
        idx = (address - REG_SERIAL_BASE) * 2
        hi = ord(serial[idx]) if idx < len(serial) else 0
        lo = ord(serial[idx + 1]) if idx + 1 < len(serial) else 0
        return (hi << 8) | lo

    def _on_write(self, address: int, value: int) -> None:
        s = self._state
        with s.lock:
            if address == REG_TARGET_FORCE:
                s.force = value
            elif address == REG_TARGET_DIAMETER:
                pos_offset = _POSITION_OFFSETS.get(s.finger_position, 0)
                s.target_angle_tenths = _width_to_angle(
                    value, s.finger_length_tenths, pos_offset, s.fingertip_offset_hundredths//10,
                )
            elif address == REG_GRIP_TYPE:
                s.grip_type = value
            elif address == REG_CONTROL:
                s.grip = bool(value & 0x01)
            elif address == REG_SET_FINGER_LENGTH:
                s.finger_length_tenths = value
            elif address == REG_SET_FINGER_POSITION:
                s.finger_position = value
            elif address == REG_SET_FINGERTIP_OFFSET:
                s.fingertip_offset_hundredths = value
        if address in (REG_TARGET_DIAMETER, REG_CONTROL):
            self._kick()

    def _kick(self) -> None:
        if self._mover is None or not self._mover.is_alive():
            self._mover = threading.Thread(target=self._move_loop, daemon=True)
            self._mover.start()

    def _move_loop(self) -> None:
        s = self._state
        while not self._stop_event.is_set():
            with s.lock:
                if s.actual_angle_tenths == s.target_angle_tenths:
                    s.busy = False
                    s.gripped = bool(s.grip)
                    width = self._width()
                    self.emit("settled", diameter_mm=width / 10)
                    return
                s.busy = True
                delta = s.target_angle_tenths - s.actual_angle_tenths
                step = min(abs(delta), ANGLE_STEP_TENTHS)
                s.actual_angle_tenths += step if delta > 0 else -step
            width = self._width()
            self.emit("moving", diameter_mm=width / 10)
            self._stop_event.wait(0.02)

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


@register("onrobot_3fg25", default_port=502)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = OnRobot3FG25Options(**options)
    pos_offset = _POSITION_OFFSETS.get(opts.finger_position, 0)
    initial_angle = _width_to_angle(
        int(opts.initial_diameter_mm * 10),
        opts.finger_length_tenths,
        pos_offset,
        opts.fingertip_offset_hundredths//10,
    )
    state = _State(
        actual_angle_tenths=initial_angle,
        finger_length_tenths=opts.finger_length_tenths,
        finger_position=opts.finger_position,
        fingertip_offset_hundredths=opts.fingertip_offset_hundredths,
    )
    state.target_angle_tenths = state.actual_angle_tenths
    device = OnRobot3FG25(name, endpoint, bus, opts, state=state)
    device._server = HoldingRegisterServer(
        host=endpoint.host,
        port=endpoint.port,
        on_read=device._on_read,
        on_write=device._on_write,
        on_connect_change=lambda count: device.emit("snapshot", clients=count),
    )
    return device
