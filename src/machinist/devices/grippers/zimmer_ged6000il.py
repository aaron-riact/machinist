"""Zimmer GED6000IL gripper, exposed via an IO-Link master gateway.

The real device is wired into an IO-Link master (we emulate the IFM
AL1350 master). Configuration tools talk to the master via a small REST
API that we surface here. The gateway (transport) and the gripper
behaviour are decoupled so other IO-Link adapters can plug in later.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass
from typing import Any

from ...core.device import Device
from ...core.events import EventBus
from ...core.registry import register
from ...core.types import Endpoint
from ...transport.iolink_http_master import IOLinkHttpMaster, IOLinkPort, ProcessData


@dataclass(slots=True)
class ZimmerGED6000ILOptions:
    initial_diameter_mm: float = 75.0


@dataclass(slots=True)
class _State:
    diameter_mm: float = 75.0
    target_mm: float = 75.0
    grip_force_n: int = 50
    moving: bool = False


class ZimmerGED6000IL(Device, IOLinkPort):
    """Zimmer GED6000IL emulated through an IO-Link HTTP gateway."""

    kind = "zimmer_ged6000il"
    DEFAULT_PORT = 80

    def __init__(
        self, name: str, endpoint: Endpoint, bus: EventBus, options: ZimmerGED6000ILOptions,
        *, master: IOLinkHttpMaster | None = None,
    ) -> None:
        super().__init__(name, endpoint, bus)
        self._state = _State(diameter_mm=options.initial_diameter_mm)
        self._state.target_mm = self._state.diameter_mm
        self._state_lock = threading.Lock()
        self._master: IOLinkHttpMaster | None = None

    # ----- IOLinkPort protocol ---------------------------------------

    def read_process_data(self) -> ProcessData:
        with self._state_lock:
            return ProcessData(
                diameter_mm=self._state.diameter_mm,
                target_mm=self._state.target_mm,
                grip_force_n=self._state.grip_force_n,
                moving=self._state.moving,
            )

    def write_process_data(self, data: ProcessData) -> None:
        with self._state_lock:
            self._state.target_mm = data.target_mm
            self._state.moving = data.target_mm != self._state.diameter_mm
            self._state.grip_force_n = data.grip_force_n
        self.emit("command", target_mm=data.target_mm, grip_force_n=data.grip_force_n)
        if self._state.moving:
            threading.Thread(target=self._settle, daemon=True).start()

    def _settle(self) -> None:
        with self._state_lock:
            self._state.diameter_mm = self._state.target_mm
            self._state.moving = False
        self.emit("settled", diameter_mm=self._state.diameter_mm)

    def _run(self, stop: threading.Event) -> None:
        ready = threading.Event()
        thread = threading.Thread(
            target=self._master.serve_forever, args=(ready,), daemon=True
        )
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        stop.wait()
        self._master.shutdown()
        thread.join(timeout=2.0)


@register("zimmer_ged6000il", default_port=80)
def _factory(name: str, endpoint: Endpoint, bus: EventBus, options: dict[str, Any]) -> Device:
    opts = ZimmerGED6000ILOptions(**options)
    device = ZimmerGED6000IL(name, endpoint, bus, opts)
    device._master = IOLinkHttpMaster(host=endpoint.host, port=endpoint.port, port_device=device)
    return device
