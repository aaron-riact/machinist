"""Every built-in device declares the capabilities the framework relies on."""

from __future__ import annotations

import pytest

from machinist.core.capabilities import (
    HasFlange,
    HasIO,
    HasMount,
    HasPrograms,
    HasRegisters,
    SensesDistance,
)
from machinist.devices.gateways.modbus_rtu import ModbusRtuGatewayDevice
from machinist.devices.grippers.onrobot_3fg25 import OnRobot3FG25
from machinist.devices.grippers.onrobot_rg import OnRobotRG
from machinist.devices.grippers.pneumatic import PneumaticGripper
from machinist.devices.grippers.zimmer_ged6000il import ZimmerGED6000IL
from machinist.devices.io_controllers.weidmuller_ur20 import WeidmullerUR20
from machinist.devices.machines.fanuc_focas_cnc import FanucFocasCnc
from machinist.devices.machines.haas_ngc import HaasNGC
from machinist.devices.machines.mazak_840d import MazakSinumerik840D
from machinist.devices.machines.mazak_smooth import MazakSmoothEmulator
from machinist.devices.machines.okuma_lb3000ex import OkumaLB3000EX
from machinist.devices.machines.state import HasMachineState
from machinist.devices.robots.arm import HasArm
from machinist.devices.robots.dobot import DobotDashboard
from machinist.devices.robots.fanuc import FanucFocasRobot, FanucKarelServer
from machinist.devices.robots.generic import RobotDevice
from machinist.devices.robots.ur import URDashboardServer
from machinist.devices.robots.yaskawa import YaskawaRobot
from machinist.devices.sensors.laser import Laser

ALL = {HasArm, HasFlange, HasIO, HasMachineState, HasMount, HasPrograms, HasRegisters,
       SensesDistance}

EXPECTED = {
    DobotDashboard: {HasArm, HasMount, HasIO, HasFlange},
    FanucKarelServer: {HasArm, HasMount, HasIO},
    FanucFocasRobot: {HasArm, HasMount, HasIO},
    RobotDevice: {HasArm, HasMount},
    YaskawaRobot: {HasArm, HasMount, HasIO},
    URDashboardServer: {HasArm, HasMount, HasFlange},
    HaasNGC: {HasMachineState, HasPrograms},
    MazakSinumerik840D: {HasMachineState, HasIO},
    MazakSmoothEmulator: {HasMachineState, HasIO},
    OkumaLB3000EX: {HasMachineState, HasIO},
    FanucFocasCnc: set(),
    PneumaticGripper: {HasIO},
    OnRobot3FG25: {HasRegisters},
    OnRobotRG: {HasRegisters},
    ZimmerGED6000IL: set(),
    WeidmullerUR20: {HasIO, HasRegisters},
    ModbusRtuGatewayDevice: {HasFlange},
    Laser: {HasIO, SensesDistance},
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
