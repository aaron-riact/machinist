"""The RG2/RG6 answers the registers riact's own RG driver speaks."""

from __future__ import annotations

import time

import pytest

from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.grippers.onrobot_rg import (
    CMD_GRIP,
    CMD_STOP,
    REG_ACTUAL_WIDTH,
    REG_COMMAND,
    REG_STATUS,
    REG_TARGET_FORCE,
    REG_TARGET_WIDTH,
    REG_WIDTH_WITH_OFFSET,
    RG_MODELS,
    STATUS_BUSY,
    STATUS_GRIP_DETECTED,
    OnRobotRG,
)

from ..conftest import free_port


def _rg(**options) -> OnRobotRG:
    options.setdefault("travel_mm_per_sec", 1000.0)
    return default_registry.create(
        "onrobot_rg", "rg1", Endpoint("127.0.0.1", free_port()), EventBus(), options
    )


def _settle(rg: OnRobotRG, timeout: float = 2.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if not rg.register_port.read(REG_STATUS, 1)[0] & STATUS_BUSY:
            mover = rg._mover
            if mover is None or not mover.is_alive():
                return
        time.sleep(0.005)
    raise AssertionError("gripper never settled")


# --- register model ---------------------------------------------------


def test_starts_at_the_configured_width() -> None:
    rg = _rg(initial_width_mm=80.0)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [800]


def test_initial_width_is_clamped_to_the_models_opening() -> None:
    rg = _rg(model="rg2", initial_width_mm=200.0)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [RG_MODELS["rg2"].fully_open_tenths]


def test_target_registers_read_back_what_was_written() -> None:
    rg = _rg()

    rg.register_port.write(REG_TARGET_FORCE, [250])
    rg.register_port.write(REG_TARGET_WIDTH, [600])

    assert rg.register_port.read(REG_TARGET_FORCE, 2) == [250, 600]


def test_target_force_is_clamped_to_the_models_strongest() -> None:
    rg = _rg(model="rg2")

    rg.register_port.write(REG_TARGET_FORCE, [9999])

    assert rg.register_port.read(REG_TARGET_FORCE, 1) == [400]


def test_rg6_allows_a_force_an_rg2_would_clamp() -> None:
    rg = _rg(model="rg6")

    rg.register_port.write(REG_TARGET_FORCE, [1200])

    assert rg.register_port.read(REG_TARGET_FORCE, 1) == [1200]


def test_an_unknown_model_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown RG model"):
        _rg(model="rg99")


# --- motion -----------------------------------------------------------


def test_a_grip_command_moves_the_fingers_to_the_target() -> None:
    rg = _rg(initial_width_mm=110.0)

    rg.register_port.write(REG_TARGET_WIDTH, [400])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [400]


def test_writing_a_target_alone_does_not_move_the_fingers() -> None:
    """A real RG moves on the command register, not on the setpoint."""
    rg = _rg(initial_width_mm=110.0)

    rg.register_port.write(REG_TARGET_WIDTH, [400])
    time.sleep(0.05)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [1100]


def test_the_busy_flag_is_set_while_moving() -> None:
    rg = _rg(initial_width_mm=110.0)

    rg.register_port.write(REG_TARGET_WIDTH, [0])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])

    assert rg.register_port.read(REG_STATUS, 1)[0] & STATUS_BUSY


def test_the_busy_flag_clears_once_settled() -> None:
    rg = _rg(initial_width_mm=110.0)

    rg.register_port.write(REG_TARGET_WIDTH, [1000])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    assert not rg.register_port.read(REG_STATUS, 1)[0] & STATUS_BUSY


def test_a_stop_command_halts_the_fingers_where_they_are() -> None:
    rg = _rg(initial_width_mm=110.0)
    rg.register_port.write(REG_TARGET_WIDTH, [0])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])

    rg.register_port.write(REG_COMMAND, [CMD_STOP])
    _settle(rg)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1)[0] > 0


# --- grip detection ---------------------------------------------------


def test_closing_on_nothing_reports_no_grip() -> None:
    rg = _rg(initial_width_mm=110.0)

    rg.register_port.write(REG_TARGET_WIDTH, [0])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    assert not rg.register_port.read(REG_STATUS, 1)[0] & STATUS_GRIP_DETECTED


def test_closing_onto_an_object_stops_on_it_and_reports_a_grip() -> None:
    rg = _rg(initial_width_mm=110.0, held_object_mm=50.0)

    rg.register_port.write(REG_TARGET_WIDTH, [0])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [500]
    assert rg.register_port.read(REG_STATUS, 1)[0] & STATUS_GRIP_DETECTED


def test_opening_past_an_object_does_not_report_a_grip() -> None:
    rg = _rg(initial_width_mm=110.0, held_object_mm=50.0)

    rg.register_port.write(REG_TARGET_WIDTH, [1000])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [1000]
    assert not rg.register_port.read(REG_STATUS, 1)[0] & STATUS_GRIP_DETECTED


def test_a_second_grip_clears_the_previous_grip_flag() -> None:
    rg = _rg(initial_width_mm=110.0, held_object_mm=50.0)
    rg.register_port.write(REG_TARGET_WIDTH, [0])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    rg.register_port.write(REG_TARGET_WIDTH, [1000])
    rg.register_port.write(REG_COMMAND, [CMD_GRIP])
    _settle(rg)

    assert not rg.register_port.read(REG_STATUS, 1)[0] & STATUS_GRIP_DETECTED


# --- fingertip offset -------------------------------------------------


def test_width_with_offset_subtracts_both_fingertips() -> None:
    rg = _rg(initial_width_mm=100.0, fingertip_offset_mm=5.0)

    assert rg.register_port.read(REG_WIDTH_WITH_OFFSET, 1) == [1000 - 2 * 50]


def test_a_negative_fingertip_offset_round_trips_as_twos_complement() -> None:
    rg = _rg(fingertip_offset_mm=-2.5)

    assert rg.register_port.read(0x0102, 1) == [0xFFFF - 25 + 1]


# --- the whole Status window riact scans -------------------------------


def test_the_status_scan_window_reads_as_one_block() -> None:
    """riact scans {'addr': 0x010B, 'count': 2} -- width then flags."""
    rg = _rg(initial_width_mm=75.0)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 2) == [750, 0]


def test_rg6_opens_wider_than_an_rg2() -> None:
    rg = _rg(model="rg6", initial_width_mm=160.0)

    assert rg.register_port.read(REG_ACTUAL_WIDTH, 1) == [1600]


def test_an_rg2_clamps_a_target_width_an_rg6_would_allow() -> None:
    rg = _rg(model="rg2")

    rg.register_port.write(REG_TARGET_WIDTH, [1600])

    assert rg.register_port.read(REG_TARGET_WIDTH, 1) == [1100]
