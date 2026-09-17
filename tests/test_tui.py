"""The TUI: pure renderers, and the app driven headless through Textual's pilot."""

from __future__ import annotations

import re
import time
from types import MappingProxyType

import pytest

from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.events import Note
from machinist.core.io import Direction
from machinist.core.panel import Field, Panel
from machinist.core.types import DeviceState
from machinist.core.world import World, WorldBuilder
from machinist.devices.machines.state import MachineState
from machinist.devices.robots.arm import RobotArm
from machinist.projection import DeviceView, SignalView
from machinist.tui.app import MachinistApp
from machinist.tui.render import (
    arm_summary,
    detail_header,
    format_event,
    io_rows,
    machine_summary,
    paint_lifecycle,
    panel_summary,
)
from machinist.tui.widgets import DetailPane, DeviceList, EventLogPanel


def _view(name: str = "dev1", kind: str = "fake", **fields: object) -> DeviceView:
    return DeviceView(name=name, kind=kind, endpoint="127.0.0.1:1", **fields)  # type: ignore[arg-type]


def _signals(**values: bool) -> MappingProxyType:
    return MappingProxyType(
        {
            name: SignalView(name=name, direction=Direction.INPUT if name.startswith("i") else Direction.OUTPUT, value=v)
            for name, v in values.items()
        }
    )


# --- pure renderers -------------------------------------------------------


def test_format_event_is_compact_and_readable() -> None:
    at = time.mktime((2026, 9, 17, 14, 5, 32, 0, 0, -1)) + 0.123
    ev = Note(device="ur1", name="rx", data={"line": "power on"}, timestamp=at)
    out = format_event(ev)
    assert "14:05:32.123" in out, "wall-clock time, not epoch seconds"
    assert not re.search(r"\d{9,}\.\d", out)
    assert "ur1" in out and "rx" in out and "power on" in out


def test_paint_lifecycle_uses_expected_colours() -> None:
    assert paint_lifecycle(DeviceState.RUNNING).startswith("[green]")
    assert paint_lifecycle(DeviceState.FAULTED).startswith("[red]")


def test_arm_summary_reports_estop_and_pose() -> None:
    assert arm_summary(None) == ""
    arm = RobotArm(joint_count=6)
    arm.estop()
    out = arm_summary(arm.state.view)
    assert "estopped" in out and "ENGAGED" in out and "joints" in out and "pose" in out


def test_machine_summary_reports_cycle_and_tooling() -> None:
    assert machine_summary(None) == ""
    state = MachineState()
    state.set_door("main", open=True)
    state.update(program="O0001\nG0 X0", spindle_rpm=1500.0, tool=3, parts=7)
    state.move_to(x=12.0, y=-3.5, z=8.25)
    out = machine_summary(state.view)
    for needle in ("O0001", "+12.000", "1500", "T3", "parts 7", "main:"):
        assert needle in out


def test_detail_header_combines_static_and_dynamic_sections() -> None:
    state = MachineState()
    state.update(program="O0001")
    out = detail_header(_view("mill", kind="haas_ngc", machine=state.view, fault="boom"))
    for needle in ("mill", "haas_ngc", "program", "boom"):
        assert needle in out


def test_panel_summary_reports_mode_and_link_state() -> None:
    out = panel_summary(Panel(mode="adapter", transport_ready=True, peer_connected=False))
    assert "adapter" in out and "waiting" in out
    assert panel_summary(Panel()) == ""


def test_detail_header_shows_the_panels_status_rows() -> None:
    panel = Panel(
        mode="modbus",
        status_fields=(Field("MODEL", "Model", value="RG2"), Field("BUSY", "Moving", type="bit", value="1", on=True)),
    )
    out = detail_header(_view("rg1", kind="onrobot_rg", panel=panel))
    assert "Model [cyan]RG2[/]" in out
    assert "Moving [green]on[/]" in out


def test_io_rows_fall_back_to_signals_when_the_panel_has_none() -> None:
    inputs, outputs = io_rows(_view(signals=_signals(i1=True, o1=False)))
    assert [(f.signal, f.value, f.on) for f in inputs] == [("I1", "ON", True)]
    assert [(f.signal, f.value, f.on) for f in outputs] == [("O1", "OFF", False)]


def test_io_rows_prefer_the_panels_own_rows() -> None:
    panel = Panel(input_fields=(Field("T_FORCE", "Target force", value="400"),))
    inputs, outputs = io_rows(_view(panel=panel, signals=_signals(i1=True)))
    assert [f.signal for f in inputs] == ["T_FORCE"] and outputs == ()


# --- the app, headless -----------------------------------------------------


def _world() -> World:
    return WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(name="io1", kind="weidmuller_ur20", options={"inputs": 2, "outputs": 2}),
                DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
            ),
            io_links=(IOLink(source="io1.o1", target="g1.cmd_open"),),
        )
    )


def _rows(table) -> list[list[str]]:  # type: ignore[no-untyped-def]
    return [[str(c) for c in table.get_row_at(i)] for i in range(table.row_count)]


async def _type(pilot, text: str) -> None:  # type: ignore[no-untyped-def]
    """Type *text* into the focused widget and press Enter (the pilot wants key names)."""
    await pilot.press(*("space" if ch == " " else ch for ch in text), "enter")


@pytest.mark.timeout(10)
async def test_the_fleet_and_the_first_device_are_painted_on_mount() -> None:
    app = MachinistApp(_world())
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        devices = app.query_one(DeviceList)
        assert [r[0] for r in _rows(devices)] == ["io1", "g1"]
        detail = app.query_one(DetailPane)
        assert detail.view is not None and detail.view.name == "io1"
        assert [r[0].split()[-1] for r in _rows(detail.inputs)] == ["i1", "i2"]
        assert [r[0].split()[-1] for r in _rows(detail.outputs)] == ["o1", "o2"]


@pytest.mark.timeout(10)
async def test_a_command_repaints_the_detail_without_polling() -> None:
    world = _world()
    app = MachinistApp(world)
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        detail = app.query_one(DetailPane)
        assert _rows(detail.outputs)[0][-1] == "OFF"

        app.query_one("#cmd").focus()
        await _type(pilot, "set io1.o1 1")
        await pilot.pause()
        await pilot.pause()

        assert _rows(detail.outputs)[0][-1] == "ON"
        assert world.io_map.signal("g1.cmd_open").value is True, "the link fired too"
        lines = [str(line) for line in app.query_one(EventLogPanel).lines]
        assert any("set io1.o1 = True" in line for line in lines)
        assert any("signal" in line and "o1" in line for line in lines), "the signal event was logged"


@pytest.mark.timeout(10)
async def test_moving_the_cursor_selects_the_device() -> None:
    app = MachinistApp(_world())
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        app.query_one(DeviceList).focus()
        await pilot.press("down")
        await pilot.pause()
        detail = app.query_one(DetailPane)
        assert detail.view is not None and detail.view.name == "g1"
        assert [r[0].split()[-1] for r in _rows(detail.inputs)] == ["cmd_open", "cmd_close"]


@pytest.mark.timeout(10)
async def test_the_program_list_appears_only_for_devices_that_have_programs(tmp_path) -> None:  # type: ignore[no-untyped-def]
    world = WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}),
                DeviceConfig(
                    name="mill", kind="haas_ngc", port=0,
                    options={"program_folder": str(tmp_path)},
                ),
            ),
        )
    )
    app = MachinistApp(world)
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        detail = app.query_one(DetailPane)
        assert detail.files.display is False, "a gripper has no programs, so no list"

        app.query_one(DeviceList).focus()
        await pilot.press("down")
        await pilot.pause()
        assert detail.view is not None and detail.view.name == "mill"
        assert detail.files.display is True

        await pilot.press("f")
        await pilot.pause()
        assert detail.files.display is False, "F hides the list"
        await pilot.press("f")
        await pilot.pause()
        assert detail.files.display is True


@pytest.mark.timeout(10)
async def test_colon_focuses_the_command_bar_and_escape_leaves_it() -> None:
    app = MachinistApp(_world())
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        app.query_one(DeviceList).focus()
        await pilot.press("colon")
        assert app.focused is app.query_one("#cmd")
        await pilot.press("escape")
        assert app.focused is app.query_one(DeviceList)


@pytest.mark.timeout(10)
async def test_help_lists_the_shared_verbs_and_quit_exits() -> None:
    app = MachinistApp(_world())
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        app.query_one("#cmd").focus()
        await _type(pilot, "help")
        await pilot.pause()
        lines = [str(line) for line in app.query_one(EventLogPanel).lines]
        assert any("estop <device>" in line and "quit" in line for line in lines)
        await _type(pilot, "quit")
        await pilot.pause()
    assert app.return_code is not None or not app.is_running


@pytest.mark.timeout(10)
async def test_an_unknown_command_is_logged_not_raised() -> None:
    app = MachinistApp(_world())
    async with app.run_test(size=(140, 45)) as pilot:
        await pilot.pause()
        app.query_one("#cmd").focus()
        await _type(pilot, "frobnicate")
        await pilot.pause()
        lines = [str(line) for line in app.query_one(EventLogPanel).lines]
        assert any("unknown command" in line for line in lines)
