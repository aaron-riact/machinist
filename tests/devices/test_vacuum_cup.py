from __future__ import annotations

import time

from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import Endpoint
from machinist.devices.grippers.vacuum import VacuumCup, VacuumCupOptions


def _cup(*, part: bool) -> VacuumCup:
    return VacuumCup(
        "cup", Endpoint("127.0.0.1", 0), EventBus(),
        VacuumCupOptions(settle_seconds=0.05, part=part), io=SignalBank(owner="cup"),
    )


def test_vacuum_on_a_part_reports_vacuum_ok_after_settling() -> None:
    cup = _cup(part=True)
    cup.io["vacuum_on"].set(True)
    assert cup.io["vacuum_ok"].value is False
    time.sleep(0.15)
    assert cup.io["vacuum_ok"].value is True


def test_vacuum_without_a_part_never_reports_vacuum_ok() -> None:
    cup = _cup(part=False)
    cup.io["vacuum_on"].set(True)
    time.sleep(0.15)
    assert cup.io["vacuum_ok"].value is False


def test_blow_off_drops_vacuum_ok_at_once() -> None:
    cup = _cup(part=True)
    cup.io["vacuum_on"].set(True)
    time.sleep(0.15)
    cup.io["blow_off"].set(True)
    assert cup.io["vacuum_ok"].value is False


def test_switching_the_vacuum_off_drops_vacuum_ok_at_once() -> None:
    cup = _cup(part=True)
    cup.io["vacuum_on"].set(True)
    time.sleep(0.15)
    cup.io["vacuum_on"].set(False)
    assert cup.io["vacuum_ok"].value is False


def test_vacuum_switched_off_before_settling_never_reports_vacuum_ok() -> None:
    cup = _cup(part=True)
    cup.io["vacuum_on"].set(True)
    cup.io["vacuum_on"].set(False)
    time.sleep(0.15)
    assert cup.io["vacuum_ok"].value is False
