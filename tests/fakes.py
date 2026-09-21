"""Minimal concrete devices for UI-layer tests.

They bind no ports and start no threads. Each declares exactly the
capabilities a test needs, so the isinstance checks in the web API and TUI
see the same shapes real devices present.
"""

from __future__ import annotations

import threading

from machinist.core.capabilities import HasIO, HasPrograms
from machinist.core.device import Device
from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.panel import Panel
from machinist.core.types import Endpoint
from machinist.devices.machines.state import HasMachineState, MachineState
from machinist.devices.robots.arm import ArmOptions, HasArm, RobotArm
from machinist.devices.robots.dobot import DobotDashboard
from machinist.transport.flange_bus import FlangeBus

_LOOPBACK = Endpoint("127.0.0.1", 29999)


class FakeDevice(Device):
    """A bare device whose detail panel content is whatever the test supplies."""

    kind = "fake"

    def __init__(
        self,
        name: str,
        *,
        kind: str | None = None,
        endpoint: Endpoint = _LOOPBACK,
        detail: Panel | None = None,
    ) -> None:
        super().__init__(name, endpoint, EventBus())
        if kind is not None:
            self.kind = kind
        self._detail = detail

    def build_detail(self) -> Panel:
        return self._detail if self._detail is not None else super().build_detail()

    def _run(self, stop: threading.Event) -> None:  # pragma: no cover - never started
        stop.wait()


class FakeArmDevice(FakeDevice, HasArm):
    def __init__(self, name: str, *, arm: RobotArm) -> None:
        super().__init__(name, kind="robot")
        self.arm = arm


class FakeMachineDevice(FakeDevice, HasMachineState):
    def __init__(self, name: str, *, state: MachineState) -> None:
        super().__init__(name, kind="haas_ngc")
        self.state = state


class FakeIODevice(FakeDevice, HasIO):
    def __init__(self, name: str, *, io: SignalBank, detail: Panel | None = None) -> None:
        super().__init__(name, detail=detail)
        self.io = io


class FakeLibrary:
    """Stands in for a ProgramLibrary. Keeps *names* by reference so a test
    can grow the listing after the fact."""

    def __init__(self, names: list[str] | None = None) -> None:
        self.names = [] if names is None else names

    def list(self) -> list[str]:
        return list(self.names)


class FakeProgramDevice(FakeDevice, HasPrograms):
    def __init__(self, name: str, *, programs: FakeLibrary, run=None) -> None:
        super().__init__(name, kind="haas_ngc")
        self.programs = programs  # type: ignore[assignment]
        self._run_hook = run
        self.ran: list[str] = []

    def run_program(self, name: str) -> None:
        self.ran.append(name)
        if self._run_hook is not None:
            self._run_hook(name)


class RecordingDobot(DobotDashboard):
    """A real Dobot (no ports bound) that records what fault injection asked for."""

    def __init__(self, name: str = "dobot1") -> None:
        super().__init__(
            name, Endpoint("127.0.0.1", 0), EventBus(), ArmOptions(),
            flange=FlangeBus(), feedback_enabled=False,
        )
        self.stops: list[dict] = []
        self.cleared = 0
        self.enable_failures: list[object] = []

    def inject_protective_stop(self, *, robot_mode, controller_ids, sticky) -> None:
        self.stops.append(
            {"robot_mode": robot_mode, "controller_ids": tuple(controller_ids), "sticky": sticky}
        )

    def clear_protective_stop(self) -> None:
        self.cleared += 1

    def set_enable_failure(self, failure) -> None:
        self.enable_failures.append(failure)
