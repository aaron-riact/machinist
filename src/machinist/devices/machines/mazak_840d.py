"""Mazak (Sinumerik 840D) S7 protocol emulator.

The 840D speaks S7 over TCP/102 (ISO-on-TCP). The user maps machine
*functions* (door open command, cycle start, …) to specific DB
addresses, and we honour reads/writes against those.

We use a *very* small native S7 server good enough for python-snap7
clients to read/write byte/word data blocks — full S7 ranges from
trivial (DB R/W) to rich (PI services, PDU negotiation). We implement
only what's needed for emulation.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass, field
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.s7_server import S7Server, S7Store
from .state import MachineState


@dataclass(slots=True)
class _DBMapping:
    """Maps a machine *function* to an S7 (DB, byte, bit) address."""

    function: str
    db: int
    byte: int
    bit: int


@dataclass(slots=True)
class _Mappings:
    door_open_cmd: _DBMapping
    door_close_cmd: _DBMapping
    cycle_start_cmd: _DBMapping
    door_is_open: _DBMapping
    door_is_closed: _DBMapping
    cycle_running: _DBMapping
    extra: tuple[_DBMapping, ...] = field(default_factory=tuple)


from .state import MachineState


def _default_mappings() -> _Mappings:
    return _build_mappings({})


@dataclass(frozen=True, slots=True)
class MazakSinumerik840DOptions:
    mappings: _Mappings = field(default_factory=_default_mappings)
    s7_backend: str = "stub"


class MazakSinumerik840D(Device):
    kind = "mazak_840d"
    DEFAULT_PORT = 102

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: MazakSinumerik840DOptions,
        *, io: SignalBank, store: S7Store, server: S7Server,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._maps = options.mappings
        self.state = MachineState()
        self.state.door("main")
        self.io = io
        self._store = store
        self._server = server
        self.io.declare("door_open_cmd", Direction.INPUT)
        self.io.declare("door_close_cmd", Direction.INPUT)
        self.io.declare("cycle_start_cmd", Direction.INPUT)
        self.io.declare("door_is_open", Direction.OUTPUT)
        self.io.declare("door_is_closed", Direction.OUTPUT)
        self.io.declare("cycle_running", Direction.OUTPUT)
        self._wire_signals()

    # -----------------------------------------------------------------

    def _wire_signals(self) -> None:
        for cmd_name, mp in [
            ("door_open_cmd", self._maps.door_open_cmd),
            ("door_close_cmd", self._maps.door_close_cmd),
            ("cycle_start_cmd", self._maps.cycle_start_cmd),
        ]:
            # External clients writing the DB toggles the IO signal.
            self._store.subscribe_bit(mp.db, mp.byte, mp.bit, self.io[cmd_name].set)
            # Local IO writes propagate back to the DB so reads agree.
            self.io[cmd_name].subscribe(
                lambda v, mp=mp: self._store.write_bit(mp.db, mp.byte, mp.bit, v)
            )
        # Status signals propagate the *other* way (machine -> DB).
        for sig_name, mp in [
            ("door_is_open", self._maps.door_is_open),
            ("door_is_closed", self._maps.door_is_closed),
            ("cycle_running", self._maps.cycle_running),
        ]:
            self.io[sig_name].subscribe(
                lambda v, mp=mp: self._store.write_bit(mp.db, mp.byte, mp.bit, v)
            )
        # Hook the door command to physical motion.
        self.io["door_open_cmd"].subscribe(
            lambda v: v and self._move_door(open=True)
        )
        self.io["door_close_cmd"].subscribe(
            lambda v: v and self._move_door(open=False)
        )

    def _move_door(self, *, open: bool) -> None:  # noqa: A002
        self.state.door("main").set(open=open)
        self.io["door_is_open"].set(open)
        self.io["door_is_closed"].set(not open)
        self.emit("door", open=open)

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


# -----------------------------------------------------------------------


def _build_mappings(raw: dict[str, Any]) -> _Mappings:
    def _entry(name: str, *, default: tuple[int, int, int]) -> _DBMapping:
        spec = raw.get(name) or {}
        return _DBMapping(
            function=name,
            db=int(spec.get("db", default[0])),
            byte=int(spec.get("byte", default[1])),
            bit=int(spec.get("bit", default[2])),
        )

    return _Mappings(
        door_open_cmd=_entry("door_open_cmd", default=(1, 0, 0)),
        door_close_cmd=_entry("door_close_cmd", default=(1, 0, 1)),
        cycle_start_cmd=_entry("cycle_start_cmd", default=(1, 0, 2)),
        door_is_open=_entry("door_is_open", default=(1, 1, 0)),
        door_is_closed=_entry("door_is_closed", default=(1, 1, 1)),
        cycle_running=_entry("cycle_running", default=(1, 1, 2)),
    )


@register("mazak_840d", default_port=102)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = dict(options)
    raw_maps = opts.pop("mappings", {}) or {}
    opt = MazakSinumerik840DOptions(mappings=_build_mappings(raw_maps), **opts)
    store = S7Store()
    io = SignalBank(owner=name)
    server = S7Server(host=endpoint.host, port=endpoint.port, store=store, backend=opt.s7_backend)
    return MazakSinumerik840D(name, endpoint, bus, opt, io=io, store=store, server=server)
