"""SRCI codec round-trips and end-to-end client/server over a transport."""

from __future__ import annotations

import threading

import pytest

from machinist.devices.robots.arm import RobotArm
from machinist.srci import (
    CommandTelegram,
    Function,
    SrciClient,
    SrciServer,
    StatusFlag,
    StatusTelegram,
)
from machinist.srci.codec import MAGIC
from machinist.transport.message import open_server

from .conftest import free_port


def test_command_round_trip() -> None:
    cmd = CommandTelegram(job_id=7, function=Function.MOVE_JOINT, args=(0.1, 0.2), speed=0.5)
    assert CommandTelegram.decode(cmd.encode()) == cmd


def test_status_round_trip() -> None:
    st = StatusTelegram(
        job_id=7,
        flags=StatusFlag.BUSY | StatusFlag.SERVO_ON,
        active_function=Function.MOVE_JOINT,
        joints=(0.1, 0.2, 0.3),
        pose=(1.0, 2.0, 3.0, 0.0, 0.0, 0.0),
        error_code=0,
    )
    assert StatusTelegram.decode(st.encode()) == st


def test_decode_rejects_foreign_frame() -> None:
    with pytest.raises(ValueError, match="not an SRCI frame"):
        CommandTelegram.decode(b"\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00")
    assert MAGIC == 0x53524349


def test_server_reports_garbage_as_error() -> None:
    server = SrciServer(RobotArm(joint_count=6))
    status = StatusTelegram.decode(server.handle(b"junk-frame"))
    assert StatusFlag.ERROR in status.flags
    assert status.error_code == 1


def test_server_drives_arm_directly() -> None:
    arm = RobotArm(joint_count=3)
    arm.start_ticker()
    server = SrciServer(arm)
    try:
        status = StatusTelegram.decode(
            server.handle(CommandTelegram(job_id=1, function=Function.ENABLE).encode())
        )
        assert StatusFlag.SERVO_ON in status.flags
        status = StatusTelegram.decode(
            server.handle(CommandTelegram(job_id=2, function=Function.STOP).encode())
        )
        assert StatusFlag.ESTOP in status.flags
    finally:
        arm.stop_ticker()


@pytest.mark.parametrize("transport", ["tcp", "udp"])
def test_client_server_end_to_end(transport: str) -> None:
    arm = RobotArm(joint_count=6)
    arm.start_ticker()
    srci = SrciServer(arm)
    port = free_port()
    channel = open_server(transport, "127.0.0.1", port)
    ready = threading.Event()
    thread = threading.Thread(
        target=channel.serve_forever, args=(srci.handle, ready), daemon=True
    )
    thread.start()
    assert ready.wait(timeout=2.0)
    try:
        with SrciClient.connect("127.0.0.1", port, transport=transport) as client:
            assert StatusFlag.SERVO_ON in client.enable().flags
            moving = client.move_joint((0.5, 0.0, 0.0, 0.0, 0.0, 0.0))
            assert moving.job_id == 2
            assert StatusFlag.ESTOP in client.estop().flags
            assert StatusFlag.ESTOP not in client.reset().flags
    finally:
        channel.shutdown()
        arm.stop_ticker()
        thread.join(timeout=2.0)
