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

from ...core.capabilities import HasIO
from ...core.device import Device
from ...core.events import EventBus
from ...core.io import Direction, SignalBank
from ...core.options import Options
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.s7_server import S7Server, S7Store
from .state import HasMachineState, MachineState


class DBMapping(Options):
    """An S7 (DB, byte, bit) address a machine function lives at."""

    db: int
    byte: int
    bit: int


class Mappings(Options):
    """Where each machine function sits in the S7 data blocks."""

    door_open_cmd: DBMapping = DBMapping(db=1, byte=0, bit=0)
    door_close_cmd: DBMapping = DBMapping(db=1, byte=0, bit=1)
    cycle_start_cmd: DBMapping = DBMapping(db=1, byte=0, bit=2)
    door_is_open: DBMapping = DBMapping(db=1, byte=1, bit=0)
    door_is_closed: DBMapping = DBMapping(db=1, byte=1, bit=1)
    cycle_running: DBMapping = DBMapping(db=1, byte=1, bit=2)


class MazakSinumerik840DOptions(Options):
    mappings: Mappings = Mappings()
    s7_backend: str = "stub"


class MazakSinumerik840D(Device, HasMachineState, HasIO):
    kind = "mazak_840d"
    DEFAULT_PORT = 102

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: MazakSinumerik840DOptions,
        *, io: SignalBank, store: S7Store, server: S7Server,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._maps = options.mappings
        self.state = MachineState(owner=name, publish=self.publish, doors=("main",))
        self.io = io
        self._store = store
        self.add_service(server)
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
        self.state.set_door("main", open=open)
        self.io["door_is_open"].set(open)
        self.io["door_is_closed"].set(not open)
        self.emit("door", open=open)


# -----------------------------------------------------------------------


@register("mazak_840d", default_port=102, options=MazakSinumerik840DOptions)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, opt: MazakSinumerik840DOptions) -> Device:
    store = S7Store()
    io = SignalBank(owner=name, publish=bus.publish)
    server = S7Server(host=endpoint.host, port=endpoint.port, store=store, backend=opt.s7_backend)
    return MazakSinumerik840D(name, endpoint, bus, opt, io=io, store=store, server=server)
