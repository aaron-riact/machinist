from __future__ import annotations

import socket
import time
import urllib.request

import pytest

from machinist.core.config import DeviceConfig, SystemConfig
from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.core.world import WorldBuilder
from machinist.devices.machines.mazak_smoothx import (
    HEARTBEAT_ALARM,
    INPUT_SIGNAL_POINTS,
    INPUT_TEXT_FIELDS,
    OUTPUT_SIGNAL_POINTS,
    MazakSmoothXEmulator,
)
from machinist.transport.ethernetip import EtherNetIPScanner, EtherNetIPScannerConfig

from ..conftest import free_port, wait_running


def _make(**options: object) -> MazakSmoothXEmulator:
    defaults: dict[str, object] = {
        "interfaces": ["io"],
        "heartbeat_timeout_seconds": 10.0,
        "heartbeat_interval_seconds": 0.05,
        "door_move_seconds": 0.05,
        "cycle_duration_seconds": 0.05,
        "work_search_seconds": 0.01,
    }
    defaults.update(options)
    return MazakSmoothXEmulator("mazak1", Endpoint("127.0.0.1", 0), EventBus(), defaults)


def test_manual_bit_mapping_matches_manual_offsets() -> None:
    assert INPUT_TEXT_FIELDS[100].offset == 12
    assert INPUT_TEXT_FIELDS[100].length == 32
    assert INPUT_SIGNAL_POINTS[0].byte == 0
    assert INPUT_SIGNAL_POINTS[0].bit == 0
    assert INPUT_SIGNAL_POINTS[101].byte == 44
    assert INPUT_SIGNAL_POINTS[101].bit == 0
    assert INPUT_SIGNAL_POINTS[109].byte == 45
    assert INPUT_SIGNAL_POINTS[109].bit == 0
    assert OUTPUT_SIGNAL_POINTS[107].byte == 45
    assert OUTPUT_SIGNAL_POINTS[107].bit == 1
    assert OUTPUT_SIGNAL_POINTS[108].byte == 45
    assert OUTPUT_SIGNAL_POINTS[108].bit == 2


def test_work_number_search_updates_active_program_and_output_field() -> None:
    device = _make()
    device.set_target_work_number("ABC123")
    device.set_input_bit(101, True)

    device._scan_cycle(now=0.0)
    device._scan_cycle(now=0.02)

    assert device.active_program == "ABC123"
    assert device.io["do101"].value is True
    assert device.output_block[12:18] == b"ABC123"

    device.set_input_bit(101, False)
    device._scan_cycle(now=0.03)

    assert device.io["do101"].value is False


def test_cycle_start_requires_enable_and_triggers_on_falling_edge() -> None:
    device = _make()
    device.set_input_bit(1, True)
    device._scan_cycle(now=0.0)

    assert device.io["do102"].value is True

    device.set_input_bit(102, True)
    device._scan_cycle(now=0.01)
    assert device.io["do103"].value is False
    assert device.state.parts == 0

    device.set_input_bit(102, False)
    device._scan_cycle(now=0.02)
    assert device.io["do103"].value is True
    assert device.state.parts == 0

    device._scan_cycle(now=0.07)
    assert device.io["do103"].value is False
    assert device.io["do104"].value is True
    assert device.state.parts == 1


def test_door_close_requires_robot_clear() -> None:
    device = _make()
    device.set_input_bit(107, True)
    device._scan_cycle(now=0.0)
    device._scan_cycle(now=0.06)

    assert device.state.door("main").open is True
    assert device.io["do107"].value is True
    assert device.io["do108"].value is False

    device.set_input_bit(107, False)
    device._scan_cycle(now=0.07)
    device.set_input_bit(108, True)
    device._scan_cycle(now=0.08)
    device._scan_cycle(now=0.14)

    assert device.state.door("main").open is True
    assert device.io["do108"].value is False

    device.set_input_bit(108, False)
    device.set_input_bit(109, True)
    device._scan_cycle(now=0.15)
    device.set_input_bit(108, True)
    device._scan_cycle(now=0.16)
    device._scan_cycle(now=0.22)

    assert device.state.door("main").open is False
    assert device.io["do107"].value is False
    assert device.io["do108"].value is True


def test_machine_stop_request_stops_door_motion() -> None:
    device = _make()
    device.set_input_bit(107, True)
    device._scan_cycle(now=0.0)
    device.set_input_bit(2, False)
    device._scan_cycle(now=0.01)
    device._scan_cycle(now=0.10)

    assert device.state.door("main").open is False
    assert device.io["do107"].value is False
    assert device.io["do108"].value is False


def test_heartbeat_timeout_raises_alarm_when_echo_does_not_follow() -> None:
    device = _make(heartbeat_timeout_seconds=0.15, heartbeat_interval_seconds=0.05)

    device._scan_cycle(now=0.0)
    assert device.io["do000"].value is True

    device.set_input_bit(0, True)
    device._scan_cycle(now=0.14)
    assert device.alarm_code is None
    assert device.io["do000"].value is False

    device._scan_cycle(now=0.31)
    assert device.alarm_code == HEARTBEAT_ALARM
    assert device.io["do004"].value is True


def test_io_only_device_hides_ethernetip_snapshot() -> None:
    device = _make(interfaces=["io"])
    assert device.ethernetip_snapshot() is None


def test_world_builds_mazak_smoothx_device() -> None:
    world = WorldBuilder().build(
        SystemConfig(devices=(DeviceConfig(name="m1", kind="mazak_smoothx"),))
    )
    assert len(world.devices) == 1
    assert isinstance(world.devices[0], MazakSmoothXEmulator)


def test_default_ethernetip_mode_accepts_incoming_scanner_connection() -> None:
    tcp_port = free_port()
    udp_port = free_port()
    device = MazakSmoothXEmulator(
        "mazak1",
        Endpoint("127.0.0.1", tcp_port),
        EventBus(),
        {
            "ethernetip": {"udp_port": udp_port},
            "heartbeat_timeout_seconds": 1.0,
            "heartbeat_interval_seconds": 0.05,
        },
    )
    scanner = EtherNetIPScanner(
        EtherNetIPScannerConfig(
            host="127.0.0.1",
            port=tcp_port,
            originator_udp_port=free_port(),
            target_udp_port=udp_port,
            output_length=100,
            input_length=100,
            requested_packet_rate_ms=20,
        )
    )
    device.start()
    try:
        wait_running(device)
        for _ in range(20):
            try:
                scanner.open()
                break
            except Exception:
                time.sleep(0.02)
        scanner.write_output_block(b"\x5A\xA5")
        for _ in range(30):
            if device.input_block.startswith(b"\x5A\xA5"):
                break
            time.sleep(0.02)
        assert device.input_block.startswith(b"\x5A\xA5")
    finally:
        scanner.close()
        device.stop()


def test_adapter_mode_keeps_listener_bound_while_idle() -> None:
    tcp_port = free_port()
    udp_port = free_port()
    device = MazakSmoothXEmulator(
        "mazak1",
        Endpoint("127.0.0.1", tcp_port),
        EventBus(),
        {
            "interfaces": ["ethernetip"],
            "ethernetip": {"udp_port": udp_port},
            "heartbeat_timeout_seconds": 0.15,
            "heartbeat_interval_seconds": 0.05,
        },
    )
    device.start()
    try:
        wait_running(device)
        time.sleep(0.4)
        assert device.alarm_code is None
        probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        probe.settimeout(1.0)
        try:
            probe.connect(("127.0.0.1", tcp_port))
        finally:
            probe.close()
    finally:
        device.stop()


def test_scanner_mode_requires_remote_adapter_address() -> None:
    with pytest.raises(ValueError, match="does not listen for inbound EtherNet/IP"):
        _make(
            interfaces=["ethernetip"],
            ethernetip={"mode": "scanner", "host": "0.0.0.0"},
        )


def test_mtconnect_reports_live_machine_state() -> None:
    port = free_port()
    device = _make(mtconnect_port=port)
    device.state.program = "O1000"
    device.state.door("main").set(open=True)
    device.state.parts = 3
    device.state.position.x = 12.5
    device.start()
    try:
        wait_running(device)
        current = urllib.request.urlopen(
            f"http://127.0.0.1:{port}/current", timeout=2
        ).read().decode()
    finally:
        device.stop()

    assert "MTConnectStreams" in current
    assert "<Program dataItemId=\"program\">O1000</Program>" in current
    assert "<DoorState dataItemId=\"door_main\">OPEN</DoorState>" in current
    assert "<PartCount dataItemId=\"parts\">3</PartCount>" in current
