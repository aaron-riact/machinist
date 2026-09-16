"""HAAS Next Generation Control emulator.

Sub-services (all optional except MDC):

* **MDC** (line TCP) — the machine Q-command serial protocol.
* **DPRINT** (line broadcast) — receive one line per ``DPRINT[...]``
  macro executed by the gcode engine.
* **MTConnect** (HTTP) — minimal ``/probe`` + ``/current`` XML.
* **SMB share** — exposes the machine's program folder. Back-end is
  configurable (impacket / pysmb / smbprotocol / aiosmb).

The device owns a :class:`ProgramLibrary` rooted at ``program_folder``.
The TUI file navigator lists and runs programs from this library.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

from ...core.capabilities import HasPrograms
from ...core.device import Device
from ...core.events import EventBus
from ...core.programs import ProgramLibrary
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.broadcast import BroadcastServer
from ...transport.framing import CRLF
from ...transport.line_server import LineServer, stateless
from ...transport.mtconnect import MTConnectAgent, render_mtconnect
from ...transport.smb_share import SmbConfig, build_share
from .gcode import Interpreter
from .state import HasMachineState, MachineState, Toggle, machine_readers


@dataclass(frozen=True, slots=True)
class SmbDeviceOptions:
    backend: str = "impacket"
    port: int = 445
    share_name: str = "PROGRAMS"
    smb1: bool = True


@dataclass(frozen=True, slots=True)
class OpcUaDeviceOptions:
    port: int = 4840


@dataclass(frozen=True, slots=True)
class HaasNGCOptions:
    doors: tuple[str, ...] = ("main",)
    program_folder: str | None = None
    dprint_port: int | None = None
    mtconnect_port: int | None = None
    smb: SmbDeviceOptions | None = None
    opcua: OpcUaDeviceOptions | None = None


class HaasNGC(Device, HasMachineState, HasPrograms):
    kind = "haas_ngc"

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: HaasNGCOptions,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self.state = MachineState()
        for d in options.doors or ("main",):
            self.state.doors[d] = Toggle(name=d)

        folder = options.program_folder
        root = Path(folder).expanduser() if folder else (
            Path.cwd() / ".machinist_programs" / name
        )
        self.programs = ProgramLibrary(root=root)
        self.interpreter = Interpreter(state=self.state)

        self._runner: threading.Thread | None = None
        self._run_lock = threading.Lock()

    # ----- MDC handler ------------------------------------------------

    def _handle_mdc(self, line: str) -> Iterable[str]:
        line = line.strip()
        if not line:
            return []
        if line.startswith("Q100"):
            return [f"SERIAL NUMBER, {self.name.upper()}"]
        if line.startswith("Q104"):
            return [f"MODE, {self.state.cycle.value.upper()}"]
        if line.startswith("Q200"):
            return [f"TOOL CHANGES, {self.state.tool_changes}"]
        if line.startswith("Q201"):
            return [f"USING TOOL, {self.state.tool}"]
        if line.startswith("Q402"):
            return [f"M30 #1, {self.state.parts}"]
        if line.startswith("Q500"):
            first = self.state.program.splitlines()[0] if self.state.program else "NONE"
            return [
                f"PROGRAM, {first}, {self.state.cycle.value.upper()}, "
                f"PARTS, {self.state.parts}"
            ]
        if line.startswith("Q600"):
            _, _, var = line.partition(" ")
            var = var.strip()
            value = self.state.variables.get(var, 0)
            return [f"MACRO, {var}, {value}"]
        return [f"?{line}"]

    # ----- program execution -----------------------------------------

    def run_program(self, name: str) -> None:
        body = self.programs.read(name)
        with self._run_lock:
            if self._runner is not None and self._runner.is_alive():
                raise RuntimeError("program already running")
            self._runner = threading.Thread(
                target=self._run_program, args=(name, body), daemon=True,
            )
            self._runner.start()

    def _run_program(self, name: str, body: str) -> None:
        self.emit("program.start", program=name)
        for line in self.interpreter.run(body):
            self.emit("program.step", line=line)
        self.emit("program.end", program=name, cycle=self.state.cycle.value)


def make_device(
    name: str,
    endpoint: Endpoint,
    bus: EventBus,
    options: HaasNGCOptions,
    *,
    dprint: BroadcastServer | None = None,
) -> HaasNGC:
    """Build a :class:`HaasNGC` with full service wiring. Does NOT start services."""
    device = HaasNGC(name, endpoint, bus, options)
    device.add_service(
        LineServer(
            endpoint.host, endpoint.port,
            session_factory=stateless(device._handle_mdc),
            framer=CRLF,
        )
    )
    if dprint is not None:
        device.state.dprint_subscribers.append(dprint.broadcast)
        device.add_service(dprint)
    if options.mtconnect_port is not None:
        device.add_service(
            MTConnectAgent(
                endpoint.host, options.mtconnect_port,
                render=lambda ep: render_mtconnect(device.state, ep),
            )
        )
    if options.smb is not None:
        smb_opts = options.smb
        cfg = SmbConfig(
            host=endpoint.host,
            port=smb_opts.port,
            share_name=smb_opts.share_name,
            root=device.programs.root,
            smb1=smb_opts.smb1,
        )
        device.add_service(build_share(smb_opts.backend, cfg))
    if options.opcua is not None:
        from ...transport.opcua_server import OpcUaServer

        device.add_service(
            OpcUaServer(
                endpoint.host, options.opcua.port,
                device_name=name,
                readers=machine_readers(device.state),
            )
        )
    return device


@register("haas_ngc", default_port=5051)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    if "doors" in opts:
        opts["doors"] = tuple(opts["doors"])
    if "smb" in opts:
        opts["smb"] = SmbDeviceOptions(**opts["smb"]) if opts["smb"] else None
    if "opcua" in opts:
        opts["opcua"] = OpcUaDeviceOptions(**opts["opcua"]) if opts["opcua"] else None
    opt = HaasNGCOptions(**opts)
    dprint = BroadcastServer(endpoint.host, opt.dprint_port) if opt.dprint_port is not None else None
    return make_device(name, endpoint, bus, opt, dprint=dprint)
