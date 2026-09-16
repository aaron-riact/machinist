"""The arm's physics tick is a Service a robot device can register."""

from __future__ import annotations

import threading
import time

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions, RobotArm
from machinist.devices.robots.ur import URDashboardServer
from machinist.kinematics.units import Radians
from machinist.transport.service import Service


def test_arm_ticks_while_served_and_stops_on_shutdown() -> None:
    arm = RobotArm(joint_count=3)
    assert isinstance(arm, Service)
    ready = threading.Event()
    thread = threading.Thread(target=arm.serve_forever, args=(ready,), daemon=True)
    thread.start()
    assert ready.wait(timeout=1.0)

    arm.movej((Radians(0.5), Radians(0.0), Radians(0.0)), duration=0.01)
    time.sleep(0.1)
    assert arm.state.snapshot().joints[0] == Radians(0.5)

    arm.shutdown()
    thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_robot_device_registers_its_arm_as_a_service() -> None:
    ur = URDashboardServer("ur1", Endpoint("127.0.0.1", 0), EventBus(), ArmOptions())
    assert ur.arm in ur.services
