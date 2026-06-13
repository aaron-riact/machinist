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

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.broadcast import BroadcastServer
from ...transport.framing import CRLF
from ...transport.line_server import LineServer, stateless
from ...transport.mtconnect import MTConnectAgent, render_mtconnect
from ...transport.smb_share import SmbConfig, build_share
from .gcode import Interpreter
from .state import MachineState, Toggle, machine_readers


@dataclass(frozen=True, slots=True)
class HaasNGCOptions:
    doors: tuple[str, ...] = ("main",)
    program_folder: str | None = None
    dprint_port: int | None = None
    mtconnect_port: int | None = None
    smb: dict[str, Any] | None = None
    opcua: dict[str, Any] | None = None


@dataclass(slots=True)
class ProgramLibrary:
    """Directory of G-code files exposed to the TUI and SMB share."""

    root: Path

    def __post_init__(self) -> None:
        self.root.mkdir(parents=True, exist_ok=True)

    def list(self) -> list[str]:
        return sorted(p.name for p in self.root.iterdir() if p.is_file())

    def read(self, name: str) -> str:
        return (self.root / name).read_text(encoding="utf-8")

    def write(self, name: str, body: str) -> None:
        (self.root / name).write_text(body, encoding="utf-8")


class HaasNGC(Device):
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

        self._mdc = LineServer(
            endpoint.host, endpoint.port,
            session_factory=stateless(self._handle_mdc),
            framer=CRLF,
        )

        self._dprint: BroadcastServer | None = None
        if options.dprint_port is not None:
            self._dprint = BroadcastServer(endpoint.host, options.dprint_port)
            self.state.dprint_subscribers.append(self._dprint.broadcast)

        self._mtc: MTConnectAgent | None = None
        if options.mtconnect_port is not None:
            self._mtc = MTConnectAgent(
                endpoint.host, options.mtconnect_port,
                render=lambda endpoint: render_mtconnect(self.state, endpoint),
            )

        self._smb = None
        if options.smb is not None:
            smb = options.smb
            cfg = SmbConfig(
                host=endpoint.host,
                port=int(smb.get("port", 445)),
                share_name=smb.get("share", "PROGRAMS"),
                root=self.programs.root,
                smb1=bool(smb.get("smb1", True)),
            )
            self._smb = build_share(smb.get("backend", "impacket"), cfg)

        self._opcua = None
        if options.opcua is not None:
            opc = options.opcua
            from ...transport.opcua_server import OpcUaServer

            self._opcua = OpcUaServer(
                endpoint.host,
                int(opc.get("port", 4840)),
                device_name=name,
                readers=machine_readers(self.state),
            )

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

    # ----- lifecycle --------------------------------------------------

    def _run(self, stop: threading.Event) -> None:
        threads = [_spawn(self._mdc.serve_forever)]
        for sub in (self._dprint, self._mtc, self._smb, self._opcua):
            if sub is not None:
                threads.append(_spawn(sub.serve_forever))
        self._mark_running()
        stop.wait()
        for sub in (self._mdc, self._dprint, self._mtc, self._smb, self._opcua):
            if sub is not None:
                sub.shutdown()
        for t in threads:
            t.join(timeout=2.0)


def _spawn(target) -> threading.Thread:
    ready = threading.Event()
    t = threading.Thread(target=target, args=(ready,), daemon=True)
    t.start()
    ready.wait(timeout=2.0)
    return t


@register("haas_ngc", default_port=5051)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    if "doors" in opts:
        opts["doors"] = tuple(opts["doors"])
    return HaasNGC(name, endpoint, bus, HaasNGCOptions(**opts))
