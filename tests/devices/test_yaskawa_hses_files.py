"""HSES file control against the DX200 manual (HW1481977), 3.2.3-3.2.6 and 3.4."""

from __future__ import annotations

import socket
from collections.abc import Iterator

import pytest

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.robots.yaskawa import YaskawaOptions, YaskawaRobot
from machinist.transport.yaskawa_hses import (
    LAST_BLOCK,
    AddedStatus,
    Answer,
    Division,
    Request,
    Status,
    encode_request,
    parse_answer,
)

from ..conftest import free_port, wait_running

DELETE, LOAD, SAVE, LIST = 0x09, 0x15, 0x16, 0x32


@pytest.fixture
def yrc() -> Iterator[YaskawaRobot]:
    options = YaskawaOptions(
        controller="yrc1000", robot="up50n", protocols=("hses",), hses_port=free_port(),
        hses_file_port=free_port(), jobs=("MAIN", "PICK"),
    )
    m = YaskawaRobot("r1", Endpoint("127.0.0.1", free_port()), EventBus(), options)
    m.start()
    try:
        wait_running(m)
        yield m
    finally:
        m.stop()


@pytest.fixture
def host(yrc: YaskawaRobot) -> Iterator[socket.socket]:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.settimeout(1)
        s.connect(("127.0.0.1", yrc.hses_file_port))
        yield s


def _exchange(host: socket.socket, service: int, data: bytes = b"", *, ack: bool = False,
              block: int = 0, division: Division = Division.FILE) -> Answer | None:
    """Send a packet; its answer, or None when the controller sends nothing."""
    host.send(encode_request(Request(
        division=division, request_id=3, command=0, instance=0, attribute=0,
        service=service, data=data, ack=ack, block=block,
    )))
    try:
        return parse_answer(host.recv(1024))
    except TimeoutError:
        return None


def _send(host: socket.socket, service: int, data: bytes = b"", *, ack: bool = False,
          block: int = 0, division: Division = Division.FILE) -> Answer:
    answer = _exchange(host, service, data, ack=ack, block=block, division=division)
    assert answer is not None, "no answer"
    return answer


def _receive(host: socket.socket, service: int, data: bytes) -> tuple[bytes, list[int]]:
    """Ask for a file or list and acknowledge each block, as the host does in 3.2.3."""
    answer: Answer | None = _send(host, service, data)
    received, blocks = b"", []
    while answer is not None:
        assert answer.status is Status.OK, answer
        received += answer.data
        blocks.append(answer.block)
        answer = _exchange(host, service, ack=True, block=answer.block)
    return received, blocks


def test_list_names_the_jobs(host: socket.socket) -> None:
    """3.2.5: no pattern lists the jobs, each name ending in CR LF, in one block."""
    listing, blocks = _receive(host, LIST, b"")
    assert listing == b"MAIN.JBI\r\nPICK.JBI\r\n"
    assert blocks == [LAST_BLOCK + 1]


def test_list_by_pattern(yrc: YaskawaRobot, host: socket.socket) -> None:
    yrc.controller.files["TOOL.CND"] = b"x"
    assert _receive(host, LIST, b"*.CND")[0] == b"TOOL.CND\r\n"


def test_save_sends_a_job_file(host: socket.socket) -> None:
    content, _ = _receive(host, SAVE, b"PICK.JBI")
    assert content.startswith(b"/JOB\r\n//NAME PICK\r\n")


def test_a_large_file_travels_in_blocks_of_479(yrc: YaskawaRobot, host: socket.socket) -> None:
    yrc.controller.files["BIG.DAT"] = bytes(range(256)) * 4
    content, blocks = _receive(host, SAVE, b"BIG.DAT")
    assert content == bytes(range(256)) * 4
    assert blocks == [1, 2, LAST_BLOCK + 3]


def test_load_keeps_the_file_after_its_last_block(yrc: YaskawaRobot,
                                                  host: socket.socket) -> None:
    """3.2.4: the request is acknowledged with block 0, then each block with its own."""
    answer = _send(host, LOAD, b"NEW.JBI")
    assert (answer.service, answer.block) == (0x95, 0)
    first = _send(host, LOAD, b"/JOB\r\n//NAME NEW\r\n", ack=True, block=1)
    assert first.block == 1
    assert "NEW.JBI" not in yrc.controller.files
    last = _send(host, LOAD, b"NOP\r\nEND\r\n", ack=True, block=LAST_BLOCK + 2)
    assert last.block == LAST_BLOCK + 2
    assert yrc.controller.files["NEW.JBI"] == b"/JOB\r\n//NAME NEW\r\nNOP\r\nEND\r\n"
    assert "NEW" in yrc.controller.jobs


def test_delete(yrc: YaskawaRobot, host: socket.socket) -> None:
    answer = _send(host, DELETE, b"MAIN.JBI")
    assert (answer.status, answer.service) == (Status.OK, 0x89)
    assert yrc.controller.jobs == {"PICK"}


def test_a_deleted_file_is_noted(yrc: YaskawaRobot, host: socket.socket) -> None:
    notes: list[tuple[str, dict[str, object]]] = []
    yrc.controller.note = lambda kind, **details: notes.append((kind, details))
    _send(host, DELETE, b"MAIN.JBI")
    assert notes == [("hses file", {"action": "deleted", "name": "MAIN.JBI"})]


def test_a_loaded_file_is_noted(yrc: YaskawaRobot, host: socket.socket) -> None:
    notes: list[tuple[str, dict[str, object]]] = []
    yrc.controller.note = lambda kind, **details: notes.append((kind, details))
    _send(host, LOAD, b"NEW.JBI")
    _send(host, LOAD, b"NOP\r\nEND\r\n", ack=True, block=LAST_BLOCK + 1)
    assert notes == [("hses file", {"action": "loaded", "name": "NEW.JBI", "size": 10})]


@pytest.mark.parametrize("service", [DELETE, SAVE])
def test_a_file_the_controller_lacks_is_not_found(host: socket.socket, service: int) -> None:
    answer = _send(host, service, b"NONE.JBI")
    assert answer.added_status == AddedStatus.FILE_NOT_FOUND


def test_an_ack_for_the_wrong_block_ends_the_transfer(yrc: YaskawaRobot,
                                                      host: socket.socket) -> None:
    yrc.controller.files["BIG.DAT"] = bytes(1000)
    _send(host, SAVE, b"BIG.DAT")
    answer = _send(host, SAVE, ack=True, block=2)
    assert answer.added_status == AddedStatus.BLOCK_NUMBER_ERROR


def test_an_ack_without_a_transfer_is_refused(host: socket.socket) -> None:
    answer = _send(host, SAVE, ack=True, block=1)
    assert answer.added_status == AddedStatus.BLOCK_NUMBER_ERROR


def test_the_file_port_takes_no_robot_control(host: socket.socket) -> None:
    answer = _send(host, 0x01, division=Division.ROBOT)
    assert answer.status is Status.UNDEFINED_COMMAND


def test_the_controller_holds_the_files_ymconnect_reads(yrc: YaskawaRobot) -> None:
    """In a DX200 backup's layout: what YMConnect parses is where it looks."""
    files = yrc.controller.files
    system = files["SYSTEM.SYS"].split(b"\r\n")
    assert system[0] == b"/SYSTEM 7"
    assert b"//SYSTEM NO : YAS2.80.00-00" in system
    robot = system[system.index(b"//ROBOT NAME") + 1]
    assert robot.startswith(b"R1 : YR-UP50N-A00")
    assert robot.endswith(b"0011_1111")
    assert system[system.index(b"//SERVO POWER") + 2].startswith(b"R1    : ")
    assert files["TOOL.CND"].count(b"//TOOL ") == 64
    rs = files["RS.PRM"].split(b"\r\n")
    assert (rs[0], len(rs)) == (b"//RS YA", 102)  # header, 100 lines, then the last CR LF
    assert rs[3].split(b",")[2] == b"1"  # RS022


def test_a_list_of_jobs_leaves_out_the_system_files(host: socket.socket) -> None:
    assert _receive(host, LIST, b"*.*")[0] == b"MAIN.JBI\r\nPICK.JBI\r\n"
