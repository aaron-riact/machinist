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


def _program(**options: object) -> OkumaLB3000EX:
    return _lathe(OkumaLB3000EXOptions(
        door_open_seconds=0.05, door_close_seconds=0.05, machining_seconds=0.1, **options,
    ))


def test_powers_up_in_link_waiting_in_its_robot_call() -> None:
    lathe = _program()
    assert lathe.io["in_link"].value is True
    assert lathe.io["request1"].value is True
    assert [lathe.io[f"request{n}"].value for n in (2, 3, 4)] == [False, False, False]
    assert lathe.io["spindle_stopped"].value is True
    assert lathe.io["nc_alarm"].value is False


def test_the_robot_call_raises_its_m_codes_request() -> None:
    lathe = _program(robot_call=182)
    assert lathe.io["request3"].value is True
    assert lathe.io["request1"].value is False


def test_an_active_low_nc_alarm_line_is_on_with_no_alarm() -> None:
    assert _program(nc_alarm_active_low=True).io["nc_alarm"].value is True


def test_the_answer_ends_the_call_then_the_next_run_machines_and_calls_again() -> None:
    lathe = _program()
    lathe.io["request_answer"].set(True)
    assert lathe.io["request1"].value is False
    assert lathe.io["spindle_stopped"].value is False
    assert lathe.state.view.cycle == "running"
    time.sleep(0.2)
    assert lathe.io["request1"].value is True
    assert lathe.io["spindle_stopped"].value is True
    assert lathe.state.view.parts == 1


def test_the_spindle_waits_for_the_door_to_close() -> None:
    lathe = _program()
    lathe.io["door_open_cmd"].set(True)
    time.sleep(0.1)
    lathe.io["request_answer"].set(True)
    assert lathe.io["request1"].value is False
    assert lathe.io["spindle_stopped"].value is True
    lathe.io["door_open_cmd"].set(False)
    lathe.io["door_close_cmd"].set(True)
    time.sleep(0.08)
    assert lathe.io["spindle_stopped"].value is False


def test_an_answer_with_no_call_does_nothing() -> None:
    lathe = _program()
    lathe.io["request_answer"].set(True)
    lathe.io["request_answer"].set(False)
    lathe.io["request_answer"].set(True)
    time.sleep(0.05)
    assert lathe.io["request1"].value is False
    assert lathe.io["spindle_stopped"].value is False


def test_without_robot_call_at_power_up_it_machines_first() -> None:
    lathe = _program(robot_call_at_power_up=False)
    assert lathe.io["request1"].value is False
    assert lathe.io["spindle_stopped"].value is False
    time.sleep(0.2)
    assert lathe.io["request1"].value is True


def test_out_of_link_it_calls_no_robot_and_runs_no_cycle() -> None:
    lathe = _program(link=False)
    lathe.io["request_answer"].set(True)
    time.sleep(0.2)
    assert lathe.io["in_link"].value is False
    assert lathe.io["request1"].value is False
    assert lathe.io["spindle_stopped"].value is True
