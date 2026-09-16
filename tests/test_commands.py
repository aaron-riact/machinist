"""One command surface for both UIs: parse, find the device, check the capability, act."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from machinist.commands import CommandError, Commands
from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.world import World, WorldBuilder

from .fakes import RecordingDobot


def _gripper_world() -> World:
    return WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(name="io1", kind="weidmuller_ur20", options={"inputs": 8, "outputs": 8}),
                DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
            ),
            io_links=(IOLink(source="io1.o5", target="g1.cmd_open"),),
        )
    )


def _dobot_world() -> tuple[Commands, RecordingDobot]:
    dobot = RecordingDobot()
    return Commands(SimpleNamespace(devices=[dobot])), dobot  # type: ignore[arg-type]


def test_set_drives_a_signal_and_links_propagate() -> None:
    world = _gripper_world()
    result = Commands(world).dispatch("set io1.o5 1")
    assert result.message == "set io1.o5 = True"
    assert world.io_map.signal("io1.o5").value is True
    assert world.io_map.signal("g1.cmd_open").value is True


def test_set_unknown_signal_raises() -> None:
    with pytest.raises(CommandError):
        Commands(_gripper_world()).dispatch("set io1.nope 1")


def test_estop_requires_an_arm() -> None:
    with pytest.raises(CommandError, match="no arm"):
        Commands(_gripper_world()).dispatch("estop g1")


def test_unknown_verb_and_empty_raise() -> None:
    commands = Commands(_gripper_world())
    with pytest.raises(CommandError, match="unknown command"):
        commands.dispatch("frobnicate g1")
    with pytest.raises(CommandError, match="empty"):
        commands.dispatch("   ")


def test_unknown_device_raises() -> None:
    with pytest.raises(CommandError, match="unknown device"):
        Commands(_gripper_world()).dispatch("reset nope")


def test_help_lists_verbs() -> None:
    result = Commands(_gripper_world()).dispatch("help")
    assert "estop" in result.message and "set" in result.message


def test_ls_needs_a_program_library() -> None:
    with pytest.raises(CommandError, match="no program library"):
        Commands(_gripper_world()).dispatch("ls g1")


def test_a_missing_device_name_falls_back_to_the_selection() -> None:
    commands, dobot = _dobot_world()
    result = commands.dispatch("pstop", selected="dobot1")
    assert "dobot1" in result.message
    assert len(dobot.stops) == 1
    commands.dispatch("pstop clear", selected="dobot1")
    assert dobot.cleared == 1


def test_without_a_name_or_a_selection_the_command_asks_for_one() -> None:
    commands, _ = _dobot_world()
    with pytest.raises(CommandError, match="which device"):
        commands.dispatch("estop")


def test_pstop_defaults_to_a_collision() -> None:
    commands, dobot = _dobot_world()
    result = commands.dispatch("pstop dobot1")
    assert "collision" in result.message
    assert dobot.stops == [{"robot_mode": 11, "controller_ids": (), "sticky": False}]


def test_pstop_accepts_a_mode_alarm_ids_and_sticky() -> None:
    commands, dobot = _dobot_world()
    commands.dispatch("pstop dobot1 error ids=17,116 sticky")
    assert dobot.stops == [{"robot_mode": 9, "controller_ids": (17, 116), "sticky": True}]


def test_pstop_clear_releases_the_stop() -> None:
    commands, dobot = _dobot_world()
    commands.dispatch("pstop dobot1 clear")
    assert dobot.cleared == 1 and dobot.stops == []


def test_pstop_rejects_an_unknown_option() -> None:
    commands, _ = _dobot_world()
    with pytest.raises(CommandError, match="unknown pstop option"):
        commands.dispatch("pstop dobot1 sideways")


def test_pstop_rejects_non_numeric_alarm_ids() -> None:
    commands, _ = _dobot_world()
    with pytest.raises(CommandError, match="bad alarm ids"):
        commands.dispatch("pstop dobot1 ids=17,oops")


def test_pstop_requires_a_device_that_can_be_stopped() -> None:
    with pytest.raises(CommandError, match="cannot be put into a protective stop"):
        Commands(_gripper_world()).dispatch("pstop g1")


def test_failenable_sets_and_clears_the_failure() -> None:
    from machinist.devices.robots.dobot import EnableFailure

    commands, dobot = _dobot_world()
    commands.dispatch("failenable dobot1 stuck")
    commands.dispatch("failenable dobot1 off")
    assert dobot.enable_failures == [EnableFailure.STUCK, None]


def test_failenable_rejects_an_unknown_kind() -> None:
    commands, _ = _dobot_world()
    with pytest.raises(CommandError, match="stuck|error|off"):
        commands.dispatch("failenable dobot1 explode")
