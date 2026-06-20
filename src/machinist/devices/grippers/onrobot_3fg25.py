"""OnRobot 3FG25 three-finger gripper, controlled over Modbus/TCP.

Spec reference (public): https://onrobot.com/en/products/3fg25
The physical gripper exposes its full state via Modbus holding
registers on TCP port 502.

Device-specific registers
=========================

============ ========== ===================================
Address      Access     Meaning
============ ========== ===================================
0x0000       Write      Target force (N)
0x0001       Write      Target diameter (.1 mm)
0x0002       Write      Grip type
0x0003       Write      Control
0x0100       Read       Status flags
0x0101       Read       Raw diameter (.1 mm)
0x0102       Read       Diameter w/ fingertip offset (.1 mm)
0x0103       Read       Force applied (N)
0x010E       Read       Finger length (.1 mm)
0x0110       Read       Finger position (.1 mm)
0x0111       Read       Fingertip offset (.1 mm)
0x0201       Read       Minimum diameter (.1 mm)
0x0202       Read       Maximum diameter (.1 mm)
0x0401       Read/Write Set finger length (.1 mm)
0x0403       Read/Write Set finger position (.1 mm)
0x0404       Read/Write Set fingertip offset (.1 mm)
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

#: Default finger geometry for the 3FG25 gripper (.1 mm units).
#: The 3FG15 shares the same register map but with different defaults.
FINGER_LENGTH_25_TENTHS = 0  # 48.5 mm
FINGERTIP_OFFSET_25_HUNDREDTHS = 0  # 6.5 mm (.01 mm units)
FINGER_POSITION_25_TENTHS = 0  # 2.0 mm


@dataclass(slots=True)
class OnRobot3FG25Options:
    initial_diameter_mm: float = 75.0
    travel_mm_per_sec: float = 60.0
    finger_length_tenths: int = FINGER_LENGTH_25_TENTHS
    fingertip_offset_hundredths: int = FINGERTIP_OFFSET_25_HUNDREDTHS
    finger_position_tenths: int = FINGER_POSITION_25_TENTHS


@dataclass(slots=True)
class _State:
    actual_tenths: int = 750
    target_tenths: int = 750
    force: int = 40
    grip: bool = False
    grip_type: int = 0
    control: int = 0
    finger_length_tenths: int = FINGER_LENGTH_25_TENTHS
    finger_position_tenths: int = FINGER_POSITION_25_TENTHS
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

    def _on_read(self, address: int) -> int:
        s = self._state
        with s.lock:
            if REG_SERIAL_BASE <= address <= REG_SERIAL_END:
                return self._serial_register(address, s.serial)
            return {
                REG_TARGET_FORCE: s.force,
                REG_TARGET_DIAMETER: s.target_tenths,
                REG_GRIP_TYPE: s.grip_type,
                REG_CONTROL: s.control,
                REG_STATUS: (STATUS_BUSY if s.busy else 0) | (STATUS_GRIPPED if s.gripped else 0),
                REG_RAW_DIAMETER: s.actual_tenths,
                REG_DIAMETER_WITH_OFFSET: max(0, s.actual_tenths - s.fingertip_offset_hundredths // 10),
                REG_FORCE_APPLIED: s.force,
                REG_FINGER_LENGTH: s.finger_length_tenths,
                REG_FINGER_POSITION: s.finger_position_tenths,
                REG_FINGERTIP_OFFSET: s.fingertip_offset_hundredths,
                REG_MIN_DIAMETER: 0,
                REG_MAX_DIAMETER: 1000,  # 100 mm
                REG_SET_FINGER_LENGTH: s.finger_length_tenths,
                REG_SET_FINGER_POSITION: s.finger_position_tenths,
                REG_SET_FINGERTIP_OFFSET: s.fingertip_offset_hundredths,
                REG_PRODUCT_CODE: s.product_code,
                REG_FW_MAJOR_MINOR: (s.fw_major << 8) | s.fw_minor,
                REG_FW_BUILD: s.fw_build,
            }.get(address, 0)

    @staticmethod
    def _serial_register(address: int, serial: str) -> int:
        """Encode serial bytes at ``address`` as two ASCII chars in a 16-bit word."""
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
                s.target_tenths = value
            elif address == REG_GRIP_TYPE:
                s.grip_type = value
            elif address == REG_CONTROL:
                s.grip = bool(value & 0x01)
            elif address == REG_SET_FINGER_LENGTH:
                s.finger_length_tenths = value
            elif address == REG_SET_FINGER_POSITION:
                s.finger_position_tenths = value
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
        step = max(1, int(self._settings.travel_mm_per_sec * 10 / 50))  # 50 Hz
        while not self._stop_event.is_set():
            with s.lock:
                if s.actual_tenths == s.target_tenths:
                    s.busy = False
                    s.gripped = bool(s.grip)
                    self.emit("settled", diameter_mm=s.actual_tenths / 10)
                    return
                s.busy = True
                delta = s.target_tenths - s.actual_tenths
                s.actual_tenths += step if delta > 0 else -step
                s.actual_tenths = (
                    min(s.actual_tenths, s.target_tenths)
                    if delta > 0
                    else max(s.actual_tenths, s.target_tenths)
                )
            self.emit("moving", diameter_mm=s.actual_tenths / 10)
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
    state = _State(
        actual_tenths=int(opts.initial_diameter_mm * 10),
        finger_length_tenths=opts.finger_length_tenths,
        finger_position_tenths=opts.finger_position_tenths,
        fingertip_offset_hundredths=opts.fingertip_offset_hundredths,
    )
    state.target_tenths = state.actual_tenths
    device = OnRobot3FG25(name, endpoint, bus, opts, state=state)
    device._server = HoldingRegisterServer(
        host=endpoint.host,
        port=endpoint.port,
        on_read=device._on_read,
        on_write=device._on_write,
    )
    return device
