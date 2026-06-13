from __future__ import annotations

import time

from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import Endpoint
from machinist.devices.grippers.pneumatic import PneumaticGripper, PneumaticGripperOptions


def test_open_close_cycle() -> None:
    gripper = PneumaticGripper(
        "g1", Endpoint("127.0.0.1", 0), EventBus(),
        options=PneumaticGripperOptions(settle_seconds=0.05), io=SignalBank(owner="g1"),
    )
    gripper.start()
    try:
        gripper.io["cmd_open"].set(True)
        time.sleep(0.15)
        assert gripper.io["is_open"].value is True
        assert gripper.io["is_closed"].value is False

        gripper.io["cmd_open"].set(False)
        gripper.io["cmd_close"].set(True)
        time.sleep(0.15)
        assert gripper.io["is_closed"].value is True
        assert gripper.io["is_open"].value is False
    finally:
        gripper.stop()
