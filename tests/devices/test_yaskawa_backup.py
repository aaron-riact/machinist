"""A real controller's saved files, loaded so the emulator holds what it holds."""

from __future__ import annotations

from pathlib import Path

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.backup import ControllerBackup, parse_variables

from ..conftest import free_port

#: The layout of a DX200's VAR.DAT, cut short: ten values to a line, from 0.
VAR_DAT = "\r\n".join([
    "//VAR", "///PFNUM 6,0,0,0", "///SHARE 1000,1000,1000,100,100,1600,0,0",
    "///B", "0,1,2,3,4,5,6,7,8,9", "10,11",
    "///I", "0,-5,0,0,0,0,0,0,0,0",
    "///D", "0,1,2,3,4,5,6,0,8,0", "14",
    "///R", "0.000000,1.500000",
    "///S", "\"HELLO\"",
    "///P", "0,0,0,0,0,0",
]) + "\r\n"


def test_var_dat_gives_each_kind_by_number() -> None:
    variables = parse_variables(VAR_DAT)
    assert variables["B"][11] == 11
    assert variables["I"][1] == -5
    assert variables["D"][1] == 1
    assert variables["D"][10] == 14
    assert variables["R"][1] == 1.5
    assert set(variables) == {"B", "I", "D", "R"}


def _job(*lines: str) -> bytes:
    return ("\r\n".join(["/JOB", "//NAME X", "//INST", "NOP", *lines, "END"]) + "\r\n").encode()


@pytest.fixture
def saved(tmp_path: Path) -> Path:
    (tmp_path / "VAR.DAT").write_text(VAR_DAT)
    (tmp_path / "GRIP_OPEN.JBI").write_bytes(_job("DOUT OT#(D002) OFF", "DOUT OT#(D001) ON"))
    (tmp_path / "TOOL.CND").write_bytes(b"//TOOL 0\r\n///NAME SAVED\r\n")
    return tmp_path


def test_every_saved_file_is_the_controllers_own(saved: Path) -> None:
    backup = ControllerBackup.load(saved)
    assert set(backup.files) == {"VAR.DAT", "GRIP_OPEN.JBI", "TOOL.CND"}
    assert backup.variables["D"][1] == 1


def test_a_scene_runs_a_saved_job_with_its_saved_variables(saved: Path) -> None:
    options = YaskawaOptions(controller="dx200", robot="up50n", controller_files=saved,
                             ladder=("GSTR #10010 GOUT #30010",), jobs=("MAIN",))
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    controller = robot.controller
    assert controller.files["TOOL.CND"] == b"//TOOL 0\r\n///NAME SAVED\r\n"
    assert {"MAIN", "GRIP_OPEN"} <= controller.jobs
    controller.start_job("GRIP_OPEN")
    assert controller.wait_job(2)
    assert controller.read_io([30010]) == [0b01]


def test_a_folder_that_is_not_there_fails_when_the_scene_loads(tmp_path: Path) -> None:
    with pytest.raises(ValueError, match="not a folder"):
        YaskawaOptions(controller="dx200", robot="up50n", controller_files=tmp_path / "gone")
