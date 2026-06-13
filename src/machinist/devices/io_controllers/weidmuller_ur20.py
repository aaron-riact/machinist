"""Weidmuller UR20 IO controller (Modbus/TCP) emulator.

The UR20 series exposes its discrete IO as Modbus holding registers,
mapped one bit per IO point. Configuration declares ``inputs`` and
``outputs`` counts; we expose a :class:`SignalBank` with names ``i1..in``
and ``o1..on`` so other devices can wire to them via ``io_links`` in YAML.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.modbus_server import HoldingRegisterServer

REG_INPUTS = 0x0000
REG_OUTPUTS = 0x0100


@dataclass(slots=True)
class WeidmullerUR20Options:
    inputs: int = 16
    outputs: int = 16


class WeidmullerUR20(Device):
    kind = "weidmuller_ur20"
    DEFAULT_PORT = 502

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: WeidmullerUR20Options
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._cfg = options
        self.io = SignalBank(owner=name)
        for i in range(1, self._cfg.inputs + 1):
            self.io.declare(f"i{i}", Direction.INPUT)
        for i in range(1, self._cfg.outputs + 1):
            sig = self.io.declare(f"o{i}", Direction.OUTPUT)
            sig.subscribe(lambda v, idx=i: self.emit("output", index=idx, value=v))
        self._server = HoldingRegisterServer(
            host=endpoint.host,
            port=endpoint.port,
            on_read=self._on_read,
            on_write=self._on_write,
        )

    def _on_read(self, address: int) -> int:
        if REG_INPUTS <= address < REG_INPUTS + 16:
            return self._pack_signals("i", base=(address - REG_INPUTS) * 16, count=16)
        if REG_OUTPUTS <= address < REG_OUTPUTS + 16:
            return self._pack_signals("o", base=(address - REG_OUTPUTS) * 16, count=16)
        return 0

    def _on_write(self, address: int, value: int) -> None:
        if not (REG_OUTPUTS <= address < REG_OUTPUTS + 16):
            return
        base = (address - REG_OUTPUTS) * 16
        for bit in range(16):
            idx = base + bit + 1
            if idx > self._cfg.outputs:
                break
            self.io[f"o{idx}"].set(bool(value & (1 << bit)))

    def _pack_signals(self, prefix: str, *, base: int, count: int) -> int:
        word = 0
        limit = self._cfg.inputs if prefix == "i" else self._cfg.outputs
        for bit in range(count):
            idx = base + bit + 1
            if idx > limit:
                break
            if self.io[f"{prefix}{idx}"].value:
                word |= 1 << bit
        return word

    def _run(self, stop: threading.Event) -> None:
        ready = threading.Event()
        thread = threading.Thread(target=self._server.serve_forever, args=(ready,), daemon=True)
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        stop.wait()
        self._server.shutdown()
        thread.join(timeout=2.0)


@register("weidmuller_ur20", default_port=502)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    return WeidmullerUR20(name, endpoint, bus, WeidmullerUR20Options(**options))
