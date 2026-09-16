"""Every built-in device declares the capabilities the framework relies on."""

from __future__ import annotations

import pytest

from machinist.core.capabilities import HasFlange, HasIO, HasPrograms, HasRegisters
from machinist.devices.grippers.onrobot_3fg25 import OnRobot3FG25
from machinist.devices.grippers.onrobot_rg import OnRobotRG
from machinist.devices.grippers.pneumatic import PneumaticGripper
from machinist.devices.grippers.zimmer_ged6000il import ZimmerGED6000IL
from machinist.devices.io_controllers.weidmuller_ur20 import WeidmullerUR20
from machinist.devices.machines.fanuc_focas_cnc import FanucFocasCnc
from machinist.devices.machines.haas_ngc import HaasNGC
from machinist.devices.machines.mazak_840d import MazakSinumerik840D
from machinist.devices.machines.mazak_smooth import MazakSmoothEmulator
from machinist.devices.machines.state import HasMachineState
from machinist.devices.robots.arm import HasArm
from machinist.devices.robots.dobot import DobotDashboard
from machinist.devices.robots.fanuc import FanucFocasRobot, FanucKarelServer
from machinist.devices.robots.generic import RobotDevice
from machinist.devices.robots.motoman import MotomanNX100
from machinist.devices.robots.ur import URDashboardServer

ALL = {HasArm, HasFlange, HasIO, HasMachineState, HasPrograms, HasRegisters}

EXPECTED = {
    DobotDashboard: {HasArm, HasIO, HasFlange},
    FanucKarelServer: {HasArm, HasIO},
    FanucFocasRobot: {HasArm, HasIO},
    RobotDevice: {HasArm},
    MotomanNX100: {HasArm},
    URDashboardServer: {HasArm},
    HaasNGC: {HasMachineState, HasPrograms},
    MazakSinumerik840D: {HasMachineState, HasIO},
    MazakSmoothEmulator: {HasMachineState, HasIO},
    FanucFocasCnc: set(),
    PneumaticGripper: {HasIO},
    OnRobot3FG25: {HasRegisters},
    OnRobotRG: {HasRegisters},
    ZimmerGED6000IL: set(),
    WeidmullerUR20: {HasIO, HasRegisters},
}


@pytest.mark.parametrize("cls", EXPECTED, ids=lambda c: c.__name__)
def test_device_declares_exactly_its_capabilities(cls: type) -> None:
    declared = {cap for cap in ALL if issubclass(cls, cap)}
    assert declared == EXPECTED[cls]


def test_has_programs_requires_run_program() -> None:
    class Incomplete(HasPrograms):
        pass

    with pytest.raises(TypeError):
        Incomplete()  # type: ignore[abstract]
