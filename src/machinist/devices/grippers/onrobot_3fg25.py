"""OnRobot 3FG25 three-finger gripper, controlled over Modbus/TCP.

Spec reference (public): https://onrobot.com/en/products/3fg25
The physical gripper exposes its full state via Modbus holding
registers on TCP port 502. We emulate the canonical subset:

============ ====== ====================
Register     Mode   Meaning
============ ====== ====================
0x0100       R/W    Target diameter (.1 mm)
0x0101       R/W    Target force (N)
0x0102       R/W    Grip command (0/1)
0x0200       R      Actual diameter (.1 mm)
0x0201       R      Status flags (bit0=busy, bit1=gripped)
============ ====== ====================
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

REG_TARGET_DIAMETER = 0x0100
REG_TARGET_FORCE = 0x0101
REG_GRIP_COMMAND = 0x0102
REG_ACTUAL_DIAMETER = 0x0200
REG_STATUS = 0x0201

STATUS_BUSY = 0x01
STATUS_GRIPPED = 0x02


@dataclass(slots=True)
class OnRobot3FG25Options:
    initial_diameter_mm: float = 75.0
    travel_mm_per_sec: float = 60.0


@dataclass(slots=True)
class _State:
    actual_tenths: int = 750
    target_tenths: int = 750
    force: int = 40
    grip: bool = False
    busy: bool = False
    gripped: bool = False
    lock: threading.Lock = field(default_factory=threading.Lock)


class OnRobot3FG25(Device):
    kind = "onrobot_3fg25"
    DEFAULT_PORT = 502

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, *,
        options: OnRobot3FG25Options, state: _State,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._settings = options
        self._state = state
        self._server: HoldingRegisterServer | None = None
        self._mover: threading.Thread | None = None

    def _on_read(self, address: int) -> int:
        s = self._state
        with s.lock:
            return {
                REG_TARGET_DIAMETER: s.target_tenths,
                REG_TARGET_FORCE: s.force,
                REG_GRIP_COMMAND: int(s.grip),
                REG_ACTUAL_DIAMETER: s.actual_tenths,
                REG_STATUS: (STATUS_BUSY if s.busy else 0) | (STATUS_GRIPPED if s.gripped else 0),
            }.get(address, 0)

    def _on_write(self, address: int, value: int) -> None:
        s = self._state
        with s.lock:
            if address == REG_TARGET_DIAMETER:
                s.target_tenths = value
            elif address == REG_TARGET_FORCE:
                s.force = value
            elif address == REG_GRIP_COMMAND:
                s.grip = bool(value)
        if address in (REG_TARGET_DIAMETER, REG_GRIP_COMMAND):
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
    state = _State(actual_tenths=int(opts.initial_diameter_mm * 10))
    state.target_tenths = state.actual_tenths
    device = OnRobot3FG25(name, endpoint, bus, options=opts, state=state)
    device._server = HoldingRegisterServer(
        host=endpoint.host,
        port=endpoint.port,
        on_read=device._on_read,
        on_write=device._on_write,
    )
    return device
