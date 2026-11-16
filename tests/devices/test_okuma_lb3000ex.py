from __future__ import annotations

import time

from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import Endpoint
from machinist.devices.machines.okuma_lb3000ex import OkumaLB3000EX, OkumaLB3000EXOptions

FAST = OkumaLB3000EXOptions(
    door_open_seconds=0.05, door_close_seconds=0.05,
    chuck_unclamp_seconds=0.05, chuck_clamp_seconds=0.05,
)


def _lathe(options: OkumaLB3000EXOptions = FAST) -> OkumaLB3000EX:
    return OkumaLB3000EX(
        "lathe1", Endpoint("127.0.0.1", 0), EventBus(), options, io=SignalBank(owner="lathe1"),
    )


def test_powers_up_with_door_closed_and_chuck_clamped() -> None:
    lathe = _lathe()
    assert lathe.io["door_closed"].value is True
    assert lathe.io["door_open"].value is False
    assert lathe.io["chuck_clamped"].value is True
    assert lathe.io["chuck_unclamped"].value is False
    assert lathe.state.view.door_open("main") is False
    assert lathe.state.view.chuck_open("main") is False


def test_door_drops_both_confirms_while_travelling_then_confirms_open() -> None:
    lathe = _lathe(OkumaLB3000EXOptions(door_open_seconds=0.2))
    lathe.io["door_open_cmd"].set(True)
    assert lathe.io["door_closed"].value is False
    assert lathe.io["door_open"].value is False
    time.sleep(0.3)
    assert lathe.io["door_open"].value is True
    assert lathe.state.view.door_open("main") is True


def test_door_closes_on_close_command() -> None:
    lathe = _lathe()
    lathe.io["door_open_cmd"].set(True)
    time.sleep(0.1)
    lathe.io["door_open_cmd"].set(False)
    lathe.io["door_close_cmd"].set(True)
    time.sleep(0.1)
    assert lathe.io["door_closed"].value is True
    assert lathe.io["door_open"].value is False
    assert lathe.state.view.door_open("main") is False


def test_chuck_unclamps_and_clamps() -> None:
    lathe = _lathe()
    lathe.io["chuck_unclamp_cmd"].set(True)
    time.sleep(0.1)
    assert lathe.io["chuck_unclamped"].value is True
    assert lathe.io["chuck_clamped"].value is False
    assert lathe.state.view.chuck_open("main") is True

    lathe.io["chuck_unclamp_cmd"].set(False)
    lathe.io["chuck_clamp_cmd"].set(True)
    time.sleep(0.1)
    assert lathe.io["chuck_clamped"].value is True
    assert lathe.state.view.chuck_open("main") is False


def test_a_new_command_mid_travel_replaces_the_old_one() -> None:
    lathe = _lathe(OkumaLB3000EXOptions(door_open_seconds=0.2, door_close_seconds=0.05))
    lathe.io["door_open_cmd"].set(True)
    lathe.io["door_close_cmd"].set(True)
    time.sleep(0.3)
    assert lathe.io["door_closed"].value is True
    assert lathe.io["door_open"].value is False
