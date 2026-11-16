"""A real controller's saved files, loaded so the emulator holds what it holds."""

from __future__ import annotations

import math
from pathlib import Path

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.devices.robots.yaskawa.backup import ControllerBackup, parse_variables
from machinist.kinematics.dh_backend import pose_to_mat

from ..conftest import free_port

#: The layout of a DX200's VAR.DAT, cut short: ten values to a line, from 0.
VAR_DAT = "\r\n".join([
    "//VAR", "///PFNUM 6,0,0,0", "///SHARE 1000,1000,1000,100,100,1600,0,0",
    "///B", "0,1,2,3,4,5,6,7,8,9", "10,11",
    "///I", "0,-5,0,0,0,0,0,0,0,0",
    "///D", "0,1,2,3,4,5,6,0,8,0", "14",
    "///R", "0.000000,1.500000",
    "///S", "\"HELLO\"",
    "///P", "\"UNUSED\"",
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


#: A CIOPRG.LST as a DX200 saves it, cut short: M920 starts at 7, and the ladder copies
#: terminals #20020-#20027 to IN#9-IN#16 and OT#9-OT#16 to #30020-#30027.
CIOPRG = "\r\n".join([
    "/CIOPRG DN", "//NAME STANDARD", "//ATTR SC", "//CIOAP GENERAL", "//$M",
    "7,0,0,0,0,0,0,0,0,0", *["0,0,0,0,0,0,0,0,0,0"] * 7, "//KEY 0",
    "PART 1", "STR-NOT #70110", "OUT #70110",
    "PART 2", "GSTR #20020", "GOUT #00020", "GSTR #10020", "GOUT #30020",
    "END", "//ALARM1", "Example alarm",
]) + "\r\n"


def test_a_saved_ladder_runs_with_the_scenes_rungs_after_it(saved: Path) -> None:
    (saved / "CIOPRG.LST").write_text(CIOPRG)
    options = YaskawaOptions(controller="dx200", robot="up50n", controller_files=saved,
                             ladder=("GSTR #00020 GOUT #30010",), signals=("s20021",))
    robot = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    controller = robot.controller
    assert controller.variables["register"][920] == 7
    robot.io["s20021"].set(True)
    assert controller.io[20] == 0b10           # the saved ladder: terminal to IN#10
    assert controller.io[30010] == 0b10         # then the scene's rung


#: A real MH50's user frame 11 (Welltec's DX200, UFRAME.CND): ORG, XX and XY as the
#: pulses they were taught at, in tool 3; BUSER is where the controller put the frame.
UFRAME_CND = "\r\n".join([
    "//UFRAME 11", "///NAME MULTUS", "///TOOL 3", "///GROUP 1" + ",0" * 31,
    "///BASICFRM 1,16,0", "///BASEP 0.000,0.000,0.000,0.0000,0.0000,0.0000", "///PULSE",
    "////RORG C000=10794,95543,45323,-12474,20107,6007,0,0,0",
    "////RXX C001=-9303,95542,45176,12305,20146,-6064,0,0,0",
    "////RXY C002=7893,98588,61863,-11485,14855,5973,0,0,0",
    "////BUSER 2139.407,337.361,-69.060,51.8855,0.1512,-89.5443",
]) + "\r\n"
#: Its tool 3 (PZN A).
TOOL_3_CND = "\r\n".join([
    "//TOOL 3", "///NAME PZN A", "-0.500,-90.000,146.500,90.0000,0.0000,180.0000",
    "0.000,0.000,0.000", "0.000", "0.000,0.000,0.000", "0.000,0,3",
]) + "\r\n"


def test_a_saved_user_frame_is_taught_where_the_controller_put_it(tmp_path: Path) -> None:
    """UFRAME.CND keeps a frame's points as pulses; through the arm and its tool they
    give the frame the controller stored (BUSER)."""
    (tmp_path / "UFRAME.CND").write_text(UFRAME_CND)
    (tmp_path / "TOOL.CND").write_text(TOOL_3_CND)
    options = YaskawaOptions(controller="dx200", robot="mh50", protocols=("hses",),
                             controller_files=tmp_path)
    controller = YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options).controller
    frame = controller.user_frame_transform(11)
    assert tuple(frame[:3, 3] * 1000) == pytest.approx((2139.407, 337.361, -69.060), abs=0.1)
    stored = pose_to_mat((0, 0, 0, *(math.radians(a) for a in (51.8855, 0.1512, -89.5443))))
    assert frame[:3, :3] == pytest.approx(stored[:3, :3], abs=1e-3)
    assert controller.taught_user_frame(11).tool == 3


def test_a_user_frame_a_job_made_keeps_its_frame_not_its_points(tmp_path: Path) -> None:
    """The cell's UF7 came from a job's MFRAME: its three points lie within 0.01 mm of
    each other, yet the controller holds the frame BUSER gives."""
    (tmp_path / "UFRAME.CND").write_text("\r\n".join([
        "//UFRAME 7", "///NAME LASER OPRET", "///TOOL 3", "///PULSE",
        "////RORG C000=96622,86999,-77489,15825,71082,42569,0,0,0",
        "////RXX C001=96615,87000,-77484,15829,71080,42568,0,0,0",
        "////RXY C002=96625,87001,-77477,15824,71078,42568,0,0,0",
        "////BUSER 379.059,1255.950,-616.266,0.0000,0.0000,-1.1458",
    ]) + "\r\n")
    (tmp_path / "TOOL.CND").write_text(TOOL_3_CND)
    options = YaskawaOptions(controller="dx200", robot="mh50", protocols=("hses",),
                             controller_files=tmp_path)
    controller = YaskawaRobot("r1", Endpoint("127.0.0.1", 0), EventBus(), options).controller
    frame = controller.user_frame_transform(7)
    assert tuple(frame[:3, 3] * 1000) == pytest.approx((379.059, 1255.950, -616.266), abs=0.001)
    assert math.degrees(math.atan2(frame[1, 0], frame[0, 0])) == pytest.approx(-1.1458, abs=1e-3)
