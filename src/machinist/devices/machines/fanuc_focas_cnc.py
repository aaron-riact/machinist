"""FANUC CNC emulator serving the FOCAS1/2 protocol on TCP port 8193.

Maps FOCAS function codes to a shared :class:`MachineState`, exposing
data reads (sysinfo, status, axes, PMC, alarms, macros, time) and
control writes (PMC → door, cycle, tool).
"""

from __future__ import annotations

import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.focas import (
    FocasSubpacket,
    FocasFrame,
    VAR_REQ,
    CONNECT_REQ,
    CLOSE_REQ,
)
from ...transport.focas_server import FocasServer
from .state import MachineState, CycleState


_EMPTY_BLOCK = object()
_WORD_FORMAT = struct.Struct(">H")


@dataclass(frozen=True, slots=True)
class FanucFocasCncOptions:
    model: str = "0i-TF"
    series: str = "3000"
    version: str = "1.00"
    max_axes: int = 8
    axis_names: tuple[str, ...] = ("X", "Y", "Z", "A", "B", "C", "U", "V")
    door_count: int = 1
    initial_diagnostics: dict[int, int] = field(default_factory=dict)


class FanucFocasCnc(Device):
    kind = "fanuc_focas_cnc"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: FanucFocasCncOptions,
        *, state: MachineState | None = None,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._options = options
        self._state = state or MachineState()
        for i in range(1, options.door_count + 1):
            self._state.doors[str(i)] = self._state.doors.get(str(i), self._state.door(str(i)))
        self._pmc_store: dict[tuple[int, int], int] = {}  # (section, address) -> value
        self._diag_store: dict[int, int] = dict(options.initial_diagnostics)
        self._server = FocasServer(
            host=endpoint.host,
            port=endpoint.port,
            on_request=self._on_request,
            on_connect=self._on_connect,
            on_disconnect=self._on_disconnect,
        )

    # ----- FOCAS handlers -------------------------------------------------

    def _on_connect(self) -> bytes | None:
        return None

    def _on_disconnect(self) -> None:
        pass

    def _on_request(self, sp: FocasSubpacket) -> bytes:
        key = (sp.c1, sp.c2, sp.c3)
        handler = _REQUEST_HANDLERS.get(key)
        if handler is None:
            return sp.encode_response_error(1)  # unknown function
        try:
            return handler(self, sp)
        except Exception:
            return sp.encode_response_error(6)  # internal error

    # ----- FOCAS function implementations ---------------------------------

    def _handle_sysinfo(self, sp: FocasSubpacket) -> bytes:
        opts = self._options
        data = struct.pack(
            ">HH2s2s4s4s2s",
            0,                  # addinfo
            opts.max_axes,
            opts.model.encode("ascii").ljust(2)[:2],
            b"  ",             # mttype
            opts.series.encode("ascii").ljust(4)[:4],
            opts.version.encode("ascii").ljust(4)[:4],
            b"\x20\x20",       # axes suffix
        )
        return sp.encode_response_ok(data)

    def _handle_status(self, sp: FocasSubpacket) -> bytes:
        s = self._state
        aut = 1 if s.cycle in (CycleState.RUNNING, CycleState.PAUSED) else 0
        run = 1 if s.cycle == CycleState.RUNNING else 0
        motion = 0
        mstb = 0
        emergency = 0
        alarm = 0
        edit = 1 if s.cycle == CycleState.IDLE else 0
        data = struct.pack(">HHHHHHH", aut, run, motion, mstb, emergency, alarm, edit)
        return sp.encode_response_ok(data)

    def _handle_alarm(self, sp: FocasSubpacket) -> bytes:
        return sp.encode_response_ok(struct.pack(">I", 0))

    def _handle_prognum(self, sp: FocasSubpacket) -> bytes:
        return sp.encode_response_ok(struct.pack(">ii", 0, 0))  # run=0, main=0

    def _handle_feedrate(self, sp: FocasSubpacket) -> bytes:
        feed = self._state.feed
        raw = struct.pack(">ii", int(feed * 1000), 0x0002000A)
        return sp.encode_response_ok(raw)

    def _handle_spindle_speed(self, sp: FocasSubpacket) -> bytes:
        rpm = self._state.spindle_rpm
        raw = struct.pack(">ii", int(rpm * 1000), 0x0002000A)
        return sp.encode_response_ok(raw)

    def _handle_get_time(self, sp: FocasSubpacket) -> bytes:
        t = time.localtime()
        data = struct.pack(">HHH", t.tm_year, t.tm_mon, t.tm_mday)
        if sp.v1 == 1:  # time only
            data = struct.pack(">HHH", t.tm_hour, t.tm_min, t.tm_sec)
        return sp.encode_response_ok(struct.pack(">H", len(data) + 2) + data)

    def _handle_axes(self, sp: FocasSubpacket) -> bytes:
        opts = self._options
        pos = self._state.position
        values = b"".join(
            struct.pack(">ii", int(v * 1000), 0x0002000A)
            for v in (pos.x, pos.y, pos.z)
        )
        count = len(values) // 8
        data = struct.pack(">HH", sp.v1 & 0xFFFF, count) + values
        return sp.encode_response_ok(data)

    def _handle_read_pmc(self, sp: FocasSubpacket) -> bytes:
        section = sp.v3 & 0xFF
        address = sp.v1 & 0xFFFF
        count = sp.v4
        if count < 1:
            count = 1
        values = bytearray()
        for offset in range(count):
            val = self._pmc_store.get((section, address + offset), 0)
            if sp.v5 == 0:  # byte
                values.append(val & 0xFF)
            elif sp.v5 == 1:  # word
                values.extend(_WORD_FORMAT.pack(val & 0xFFFF))
            else:  # dword
                values.extend(struct.pack(">I", val & 0xFFFFFFFF))
        return sp.encode_response_ok(bytes(values))

    def _handle_write_pmc(self, sp: FocasSubpacket) -> bytes:
        section = sp.v3 & 0xFF
        address = sp.v1 & 0xFFFF
        payload = sp.payload
        if sp.v5 == 0:
            for i, b in enumerate(payload):
                self._pmc_store[(section, address + i)] = b
        elif sp.v5 == 1:
            for i in range(0, len(payload), 2):
                val = struct.unpack(">H", payload[i:i + 2])[0]
                self._pmc_store[(section, address + i // 2)] = val
        else:
            for i in range(0, len(payload), 4):
                val = struct.unpack(">I", payload[i:i + 4])[0]
                self._pmc_store[(section, address + i // 4)] = val
        self._check_pmc_actions(section, address)
        return sp.encode_response_ok()

    def _handle_mdi(self, sp: FocasSubpacket) -> bytes:
        prog = self._state.program[:sp.v1] if sp.v1 > 0 else self._state.program
        return sp.encode_response_ok(struct.pack(">i", 0) + prog.encode("ascii"))

    def _handle_set_time(self, sp: FocasSubpacket) -> bytes:
        return sp.encode_response_ok()  # accept silently

    def _handle_diag(self, sp: FocasSubpacket) -> bytes:
        address = sp.v1 & 0xFFFF
        val = self._diag_store.get(address, 0)
        data = struct.pack(">IHh", address, val, 0)
        return sp.encode_response_ok(data)

    # ----- PMC action dispatch --------------------------------------------

    _PMC_DOOR = 0x100
    _PMC_CYCLE = 0x200

    def _check_pmc_actions(self, section: int, address: int) -> None:
        if section != 9:  # D (data table)
            return
        if address == self._PMC_DOOR:
            val = self._pmc_store.get((9, address), 0)
            door = self._state.door("1")
            door.set(open=bool(val))
            self.emit("door", name="1", open=door.open)
        elif address == self._PMC_CYCLE:
            val = self._pmc_store.get((9, address), 0)
            if val:
                self._state.cycle = CycleState.RUNNING
                self.emit("cycle", state=str(self._state.cycle))
            else:
                self._state.cycle = CycleState.IDLE
                self.emit("cycle", state=str(self._state.cycle))

    # ----- lifecycle ------------------------------------------------------

    def _run(self, stop: threading.Event) -> None:
        ready = threading.Event()
        thread = threading.Thread(
            target=self._server.serve_forever, args=(ready,), daemon=True,
        )
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        stop.wait()
        self._server.shutdown()
        thread.join(timeout=2.0)

    def _shutdown(self) -> None:
        self._server.shutdown()


_REQUEST_HANDLERS: dict[tuple[int, int, int], Any] = {
    (1, 1, 0x18): FanucFocasCnc._handle_sysinfo,
    (1, 1, 0x19): FanucFocasCnc._handle_status,
    (1, 1, 0x1a): FanucFocasCnc._handle_alarm,
    (1, 1, 0x1c): FanucFocasCnc._handle_prognum,
    (1, 1, 0x20): FanucFocasCnc._handle_mdi,
    (1, 1, 0x24): FanucFocasCnc._handle_feedrate,
    (1, 1, 0x25): FanucFocasCnc._handle_spindle_speed,
    (1, 1, 0x26): FanucFocasCnc._handle_axes,
    (1, 1, 0x30): FanucFocasCnc._handle_diag,
    (1, 1, 0x45): FanucFocasCnc._handle_get_time,
    (1, 1, 0x46): FanucFocasCnc._handle_set_time,
    (2, 1, 0x8001): FanucFocasCnc._handle_read_pmc,
    (2, 1, 0x8002): FanucFocasCnc._handle_write_pmc,
}


@register("fanuc_focas_cnc", default_port=8193)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    if "axis_names" in opts:
        opts["axis_names"] = tuple(opts["axis_names"])
    if "initial_diagnostics" in opts:
        opts["initial_diagnostics"] = dict(opts["initial_diagnostics"])
    opt = FanucFocasCncOptions(**opts)
    state = MachineState()
    for i in range(1, opt.door_count + 1):
        state.doors[str(i)] = state.door(str(i))
    return FanucFocasCnc(name, endpoint, bus, opt, state=state)
