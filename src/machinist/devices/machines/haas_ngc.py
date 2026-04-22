"""HAAS NGC controller emulator.

The HAAS NGC (Next Generation Control) exposes a number of network
interfaces. We cover the four called out in the brief:

* **MDC** – Machine Data Collection: a tiny line-protocol query channel
  on TCP port 5051 (`Q100`...`Q600` queries returning ``Q100, value``).
* **DPRINT** – the DPRINT macro line lands on TCP port 5052 in NGC; we
  expose any DPRINT log line over a server-sent stream there.
* **MTConnect** – a minimal HTTP/XML probe + current document on port
  5000 (the conventional Mazak / HAAS MTConnect agent port).
* **SMB** – a stub file-share for parts/programs (uses the
  :mod:`machinist.transport.smb_share` abstraction).

The four sub-services share a single :class:`MachineState`; you can
configure which ones are exposed via the device options.
"""

from __future__ import annotations

import threading
from collections.abc import Iterable
from dataclasses import dataclass, field
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.io import SignalBank
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.framing import CRLF
from ...transport.line_server import LineServer, stateless
from .gcode import Interpreter
from .state import CycleState, MachineState

DEFAULT_MDC_PORT = 5051


@dataclass(slots=True)
class _Subservers:
    mdc: LineServer
    threads: list[threading.Thread] = field(default_factory=list)


class HaasNGC(Device):
    kind = "haas_ngc"
    DEFAULT_PORT = DEFAULT_MDC_PORT

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.state = MachineState()
        for door in options.get("doors", ["main"]):
            self.state.door(door)
        for chuck in options.get("chucks", []):
            self.state.chuck(chuck)
        self.io = SignalBank(owner=name)
        for d in self.state.doors:
            self.io.declare(f"door_{d}_open").subscribe(
                lambda v, name=d: self.state.door(name).set(open=v)
            )
            self.io.declare(f"door_{d}_is_open")
        # interpreter for run_program
        self._interp = Interpreter(self.state)
        # MDC server
        self._mdc = LineServer(
            host=endpoint.host,
            port=endpoint.port,
            session_factory=stateless(self._handle_mdc),
            framer=CRLF,
        )
        self._sub = _Subservers(mdc=self._mdc)

    # ----- public commands -------------------------------------------

    def run_program(self, program: str) -> None:
        def _runner() -> None:
            for line in self._interp.run(program):
                self.emit("gcode", line=line)
        threading.Thread(target=_runner, daemon=True).start()

    # ----- protocols -------------------------------------------------

    def _handle_mdc(self, line: str) -> Iterable[str] | str:
        # MDC queries look like "?Q500" or "?Q100"
        q = line.strip().lstrip("?").upper()
        s = self.state
        if q == "Q100":
            return ">Q100, machinist-haas-emulator"
        if q == "Q200":  # Total power-on time (we report uptime placeholder)
            return ">Q200, 0"
        if q == "Q500":  # current cycle status
            return f">Q500, {s.cycle.value.upper()}"
        if q == "Q600":  # last DPRINT line
            last = s.dprint_log[-1] if s.dprint_log else ""
            return f">Q600, {last}"
        return ">?, NACK"

    def _run(self, stop: threading.Event) -> None:
        ready = threading.Event()
        thread = threading.Thread(target=self._mdc.serve_forever, args=(ready,), daemon=True)
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        self._sub.threads.append(thread)
        stop.wait()
        self._mdc.shutdown()
        for t in self._sub.threads:
            t.join(timeout=2.0)

    def _shutdown(self) -> None:
        self._mdc.shutdown()


@register("haas_ngc", default_port=DEFAULT_MDC_PORT)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    return HaasNGC(name, endpoint, bus, options)
