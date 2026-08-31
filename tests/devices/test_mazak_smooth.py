from __future__ import annotations

import socket
import time
import urllib.request

import pytest

from machinist.core.config import DeviceConfig, SystemConfig
from machinist.core.events import EventBus
from machinist.core.io import SignalBank
from machinist.core.types import Endpoint
from machinist.core.world import WorldBuilder
from machinist.devices.machines.mazak_smooth import (
    BLOCK_SIZE,
    HEARTBEAT_ALARM,
    INPUT_SIGNAL_POINTS,
    INPUT_TEXT_FIELDS,
    MTConnectOptions,
    OUTPUT_SIGNAL_POINTS,
    WORK_SEARCH_ALARM,
    MazakSmoothEmulator,
    MazakSmoothOptions,
    _build_ethernetip_transport,
    make_device,
)
from machinist.devices.machines.state import CycleState
from machinist.transport.ethernetip import (
    EtherNetIPAdapterConfig,
    EtherNetIPScanner,
    EtherNetIPScannerConfig,
)
from ..conftest import free_port, wait_running


def _make(**kw: object) -> MazakSmoothEmulator:
    raw_mtconnect_port = kw.pop("mtconnect_port", None)
    mtconnect_opts = MTConnectOptions(port=int(raw_mtconnect_port)) if raw_mtconnect_port is not None else None
    raw_ethernetip = kw.get("ethernetip")
    if isinstance(raw_ethernetip, dict):
        mode = str(raw_ethernetip.get("mode", "adapter")).strip().lower()
        kw["ethernetip_mode"] = mode
        if mode == "adapter":
            kw["ethernetip_adapter_config"] = EtherNetIPAdapterConfig(
                host="127.0.0.1",
                port=0,
                udp_port=int(raw_ethernetip.get("udp_port", 2222)),
                output_length=BLOCK_SIZE,
                input_length=BLOCK_SIZE,
                requested_packet_rate_ms=int(raw_ethernetip.get("requested_packet_rate_ms", 20)),
                o_t_realtime_format=str(raw_ethernetip.get("o_t_realtime_format", "modeless")),
            )
        elif mode == "scanner":
            kw["ethernetip_scanner_config"] = EtherNetIPScannerConfig(
                host=str(raw_ethernetip["host"]).strip(),
                port=int(raw_ethernetip.get("port", 44818)),
                originator_udp_port=int(raw_ethernetip.get("originator_udp_port", 2222)),
                target_udp_port=int(raw_ethernetip.get("target_udp_port", 2222)),
                assembly_object_class=int(raw_ethernetip.get("assembly_object_class", 0x04)),
                configuration_assembly_instance_id=int(
                    raw_ethernetip.get("configuration_assembly_instance_id", 0x01)
                ),
                output_assembly_instance_id=int(raw_ethernetip.get("output_assembly_instance_id", 0x64)),
                input_assembly_instance_id=int(raw_ethernetip.get("input_assembly_instance_id", 0x65)),
                output_length=BLOCK_SIZE,
                input_length=BLOCK_SIZE,
                requested_packet_rate_ms=int(raw_ethernetip.get("requested_packet_rate_ms", 20)),
                o_t_realtime_format=str(raw_ethernetip.get("o_t_realtime_format", "modeless")),
                o_t_connection_type=str(raw_ethernetip.get("o_t_connection_type", "point_to_point")),
                t_o_connection_type=str(raw_ethernetip.get("t_o_connection_type", "point_to_point")),
            )
    opts = MazakSmoothOptions(
        interfaces=kw.pop("interfaces", ["io"]),
        heartbeat_timeout_seconds=kw.pop("heartbeat_timeout_seconds", 10.0),
        heartbeat_interval_seconds=kw.pop("heartbeat_interval_seconds", 0.05),
        door_move_seconds=kw.pop("door_move_seconds", 0.05),
        cycle_duration_seconds=kw.pop("cycle_duration_seconds", 0.05),
        work_search_seconds=kw.pop("work_search_seconds", 0.01),
        mtconnect=mtconnect_opts,
        **kw,
    )
    return make_device("mazak1", Endpoint("127.0.0.1", 0), EventBus(), opts)


def test_manual_bit_mapping_matches_manual_offsets() -> None:
    assert INPUT_TEXT_FIELDS[100].offset == 44
    assert INPUT_TEXT_FIELDS[100].length == 32
    assert INPUT_SIGNAL_POINTS[0].byte == 0
    assert INPUT_SIGNAL_POINTS[0].bit == 0
    assert INPUT_SIGNAL_POINTS[101].byte == 12
    assert INPUT_SIGNAL_POINTS[101].bit == 0
    assert INPUT_SIGNAL_POINTS[109].byte == 13
    assert INPUT_SIGNAL_POINTS[109].bit == 0
    assert OUTPUT_SIGNAL_POINTS[107].byte == 13
    assert OUTPUT_SIGNAL_POINTS[107].bit == 1
    assert OUTPUT_SIGNAL_POINTS[108].byte == 13
    assert OUTPUT_SIGNAL_POINTS[108].bit == 2


def test_work_number_search_updates_active_program_and_output_field() -> None:
    device = _make()
    device.set_input_bit(1, True)
    device.set_target_work_number("ABC123")
    device.set_input_bit(101, True)

    device._scan_cycle(now=0.0)

    assert device.io["do101"].value is False
    assert device.io["do102"].value is True

    device._scan_cycle(now=0.02)

    assert device.active_program == "ABC123"
    assert device.io["do101"].value is True
    assert device.io["do102"].value is False
    assert device.output_block[44:50] == b"ABC123"

    device.set_input_bit(101, False)
    device._scan_cycle(now=0.03)
    device._scan_cycle(now=1.04)

    assert device.io["do101"].value is True
    assert device.io["do102"].value is True


def test_write_input_block_emits_snapshot_event_once_per_change() -> None:
    bus = EventBus()
    events: list[tuple[str, dict[str, object]]] = []
    bus.subscribe(lambda event: events.append((event.kind, event.payload)))
    device = MazakSmoothEmulator(
        "mazak1",
        Endpoint("127.0.0.1", 0),
        bus,
        MazakSmoothOptions(interfaces=["io"]),
        io=SignalBank(owner="mazak1"),
    )
    events.clear()

    device.write_input_block(b"\x01\x02", offset=12)
    device.write_input_block(b"\x01\x02", offset=12)

    snapshots = [payload for kind, payload in events if kind == "snapshot"]
    assert snapshots == [{"interface": "ethernetip", "direction": "input"}]


def test_internal_output_bit_changes_emit_snapshot_event() -> None:
    bus = EventBus()
    events: list[tuple[str, dict[str, object]]] = []
    bus.subscribe(lambda event: events.append((event.kind, event.payload)))
    device = MazakSmoothEmulator(
        "mazak1",
        Endpoint("127.0.0.1", 0),
        bus,
        MazakSmoothOptions(interfaces=["io"]),
        io=SignalBank(owner="mazak1"),
    )
    events.clear()

    device._write_output_bit(107, True)

    snapshots = [payload for kind, payload in events if kind == "snapshot"]
    assert snapshots == [{"interface": "ethernetip", "direction": "output"}]


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
    device.set_input_bit(1, True)  # DO004 is only presented while DI001 is on

    device._scan_cycle(now=0.0)
    assert device.io["do000"].value is True

    device.set_input_bit(0, True)
    device._scan_cycle(now=0.14)
    assert device.alarm_code is None
    assert device.io["do000"].value is False

    device._scan_cycle(now=0.31)
    assert device.alarm_code == HEARTBEAT_ALARM
    assert device.io["do004"].value is True


def test_io_only_device_returns_bare_detail() -> None:
    device = _make(interfaces=["io"])
    detail = device.build_detail()
    assert detail["mode"] == "io"
    assert detail["transport_ready"] is False
    assert detail["input_fields"] == []


def test_world_builds_mazak_smooth_device() -> None:
    world = WorldBuilder().build(
        SystemConfig(devices=(DeviceConfig(name="m1", kind="mazak_smooth"),))
    )
    assert len(world.devices) == 1
    assert isinstance(world.devices[0], MazakSmoothEmulator)


def test_default_ethernetip_mode_accepts_incoming_scanner_connection() -> None:
    tcp_port = free_port()
    udp_port = free_port()
    opts = MazakSmoothOptions(
        ethernetip={"udp_port": udp_port},
        ethernetip_adapter_config=EtherNetIPAdapterConfig(
            host="127.0.0.1", port=tcp_port, udp_port=udp_port,
            output_length=BLOCK_SIZE, input_length=BLOCK_SIZE,
        ),
        heartbeat_timeout_seconds=1.0,
        heartbeat_interval_seconds=0.05,
    )
    device = MazakSmoothEmulator(
        "mazak1",
        Endpoint("127.0.0.1", tcp_port),
        EventBus(),
        opts,
        io=SignalBank(owner="mazak1"),
    )
    device._ethernetip = _build_ethernetip_transport(Endpoint("127.0.0.1", tcp_port), opts)
    scanner = EtherNetIPScanner(
        EtherNetIPScannerConfig(
            host="127.0.0.1",
            port=tcp_port,
            originator_udp_port=free_port(),
            target_udp_port=udp_port,
            output_length=BLOCK_SIZE,
            input_length=BLOCK_SIZE,
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
    opts = MazakSmoothOptions(
        interfaces=["ethernetip"],
        ethernetip={"udp_port": udp_port},
        ethernetip_adapter_config=EtherNetIPAdapterConfig(
            host="127.0.0.1", port=tcp_port, udp_port=udp_port,
            output_length=BLOCK_SIZE, input_length=BLOCK_SIZE,
        ),
        heartbeat_timeout_seconds=0.15,
        heartbeat_interval_seconds=0.05,
    )
    device = MazakSmoothEmulator(
        "mazak1",
        Endpoint("127.0.0.1", tcp_port),
        EventBus(),
        opts,
        io=SignalBank(owner="mazak1"),
    )
    device._ethernetip = _build_ethernetip_transport(Endpoint("127.0.0.1", tcp_port), opts)
    device.start()
    try:
        wait_running(device)
        time.sleep(0.4)
        assert device.alarm_code == HEARTBEAT_ALARM
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


def test_smoothai_variant_side_door_uses_same_di_and_extra_front_door() -> None:
    device = _make(variant="smoothai", front_door=True)
    device.set_input_bit(107, True)
    device._scan_cycle(now=0.0)
    device._scan_cycle(now=2.01)

    assert device.state.door("side").open is True
    assert device.io["do107"].value is True

    device.set_input_bit(107, False)
    device.set_input_bit(108, False)
    device.set_input_bit(109, True)
    device._scan_cycle(now=2.02)
    device.set_input_bit(108, True)
    device._scan_cycle(now=2.03)
    device._scan_cycle(now=4.04)

    assert device.state.door("side").open is False
    assert device.io["do108"].value is True

    device.set_input_bit(110, True)
    device._scan_cycle(now=4.04)
    device._scan_cycle(now=6.05)

    assert device.state.door("front").open is True
    assert device.io["do110"].value is True
    assert device.io["do111"].value is False

    device.set_input_bit(110, False)
    device.set_input_bit(111, True)
    device._scan_cycle(now=6.06)
    device._scan_cycle(now=8.07)

    assert device.state.door("front").open is False
    assert device.io["do110"].value is False
    assert device.io["do111"].value is True


def _settle(device: MazakSmoothEmulator, start: float, duration: float) -> float:
    """Run scan cycles over `duration`, mirroring the heartbeat like a real robot."""
    now, last_hb = start, start
    while now < start + duration:
        if now - last_hb >= 0.5:
            last_hb = now
            device.set_input_bit(0, device._read_output_bit(0))
        device._scan_cycle(now=now)
        now += 0.02
    return now


def test_do104_idles_high_and_clears_on_cycle_start_rising_edge() -> None:
    """mazak6.pcap opens with byte12=0x0B, and DO104 drops 70-163ms after DI102^."""
    device = _make()
    device.set_input_bit(1, True)
    now = _settle(device, 0.0, 0.2)

    assert device.io["do104"].value is True

    device.set_input_bit(102, True)
    device._scan_cycle(now=now)

    assert device.io["do104"].value is False
    assert device.io["do103"].value is False  # cycle still starts on the fall


def test_robot_interface_outputs_are_gated_by_robot_ready() -> None:
    """A DI001 blip drops DO003/DO102/DO104 and they come straight back."""
    device = _make()
    device.set_input_bit(1, True)
    now = _settle(device, 0.0, 0.2)

    assert device.io["do003"].value is True
    assert device.io["do102"].value is True
    assert device.io["do104"].value is True

    device.set_input_bit(1, False)
    device._scan_cycle(now=now)

    assert device.io["do003"].value is False
    assert device.io["do102"].value is False
    assert device.io["do104"].value is False

    device.set_input_bit(1, True)
    device._scan_cycle(now=now + 0.02)

    assert device.io["do003"].value is True
    assert device.io["do102"].value is True
    assert device.io["do104"].value is True


def test_machine_alarm_output_is_gated_by_robot_ready() -> None:
    """mazak3.pcap t=1842-1863s: DO004 drops with DO003 on every DI001 retry."""
    device = _make()
    device.set_input_bit(1, True)
    now = _settle(device, 0.0, 0.2)
    device.inject_alarm(1234, "test")
    device._scan_cycle(now=now)

    assert device.io["do004"].value is True

    device.set_input_bit(1, False)
    device._scan_cycle(now=now + 0.02)

    assert device.io["do004"].value is False
    assert device.alarm_code == 1234  # still alarmed, just not presented

    device.set_input_bit(1, True)
    device._scan_cycle(now=now + 0.04)

    assert device.io["do004"].value is True


def test_idle_control_word_matches_real_smoothai() -> None:
    """Idle T->O assembly must match mazak6.pcap byte-for-byte.

    The real machine idles at byte0=0x0e, byte12=0x0b (DO101 + DO102 + DO104)
    and byte13=0x04 (DO108 only); it never asserts DO106/DO109/DO110/DO111.
    """
    device = _make(variant="smoothai")
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    _settle(device, 0.0, 1.0)

    block = device.output_block
    assert block[0] == 0x0E
    assert block[12] == 0x0B
    assert block[13] == 0x04
    assert device.io["do109"].value is False
    assert device.io["do111"].value is False


def test_front_door_bits_stay_clear_unless_configured() -> None:
    device = _make(variant="smoothai")
    device.set_input_bit(110, True)
    _settle(device, 0.0, 0.2)

    assert device.io["do110"].value is False
    assert device.io["do111"].value is False
    assert device.state.door("front").open is False


def test_door_open_and_close_durations_are_configurable() -> None:
    """A real SmoothAi is asymmetric: 5.44s to open, 4.30s to close."""
    device = _make(door_open_seconds=0.30, door_close_seconds=0.10)
    device.set_input_bit(109, True)
    device.set_input_bit(107, True)
    device._scan_cycle(now=0.0)
    device._scan_cycle(now=0.20)

    assert device.state.door("main").open is False  # still travelling at 0.20s

    device._scan_cycle(now=0.31)
    assert device.state.door("main").open is True

    device.set_input_bit(107, False)
    device._scan_cycle(now=0.32)
    device.set_input_bit(108, True)
    device._scan_cycle(now=0.33)
    device._scan_cycle(now=0.44)

    assert device.state.door("main").open is False


def test_cycle_starts_on_the_real_robot_handshake() -> None:
    """Replays the mazak6.pcap cycle-start handshake (f120282-f120334).

    The program is already loaded, so DO102 must stay ON through the search, and
    DI102 rising in the same frame DI101 falls must still start the cycle.
    """
    device = _make(work_search_seconds=0.5, cycle_duration_seconds=1500.0)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)

    # Load '202' first -- a freshly started emulator has no program, so this
    # first search is a genuine load and does dip DO102.
    device.set_target_work_number("202")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)
    assert device.io["do102"].value is False
    device.set_input_bit(101, False)
    now = _settle(device, now, 1.2)
    assert device.io["do102"].value is True

    # Now the mazak6 case: search the program that is already loaded.
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.io["do101"].value is True
    assert device.io["do102"].value is True
    assert device.output_block[12] == 0x0B

    device.set_input_bit(101, False)
    device.set_input_bit(102, True)
    now = _settle(device, now, 1.94)

    assert device.io["do104"].value is False  # cleared on DI102's rising edge
    assert device.io["do103"].value is False  # but the cycle has not started

    device.set_input_bit(102, False)
    now = _settle(device, now, 0.2)

    assert device.io["do103"].value is True
    assert device.io["do102"].value is False
    assert device.output_block[12] == 0x05


def test_program_changing_search_dips_do102_but_still_honours_the_start() -> None:
    """cyclestart.pcap: a search that swaps programs holds DO102 off for 1.0s.

    A DI102 pulse arriving inside that window must be held, not dropped -- the
    settle window may delay a cycle start but must never lose one.
    """
    device = _make(work_search_seconds=0.5, cycle_duration_seconds=1500.0)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)

    device.set_target_work_number("9")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)
    device.set_input_bit(101, False)
    now = _settle(device, now, 1.2)

    # '6' is a different program: DO102 drops when the search completes.
    device.set_target_work_number("6")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.io["do101"].value is True
    assert device.io["do102"].value is False
    assert device.active_program == "6"

    device.set_input_bit(101, False)
    device.set_input_bit(102, True)
    now = _settle(device, now, 0.3)

    assert device.io["do102"].value is False
    assert device.io["do103"].value is False

    now = _settle(device, now, 1.0)
    assert device.io["do102"].value is True

    device.set_input_bit(102, False)
    now = _settle(device, now, 0.2)

    assert device.io["do103"].value is True


def test_repeat_search_of_the_loaded_program_does_not_dip_do102() -> None:
    """13 no-op searches across mazak6/mazak3/mazak4 never drop DO102."""
    device = _make(work_search_seconds=0.5)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)

    device.set_target_work_number("202")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)
    device.set_input_bit(101, False)
    now = _settle(device, now, 1.2)

    for _ in range(3):
        device.set_input_bit(101, True)
        now = _settle(device, now, 0.6)
        assert device.io["do101"].value is True
        assert device.io["do102"].value is True
        device.set_input_bit(101, False)
        now = _settle(device, now, 0.1)


def test_work_search_settle_seconds_can_disable_the_dip() -> None:
    device = _make(work_search_seconds=0.5, work_search_settle_seconds=0.0)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)

    device.set_target_work_number("202")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.io["do101"].value is True
    assert device.io["do102"].value is True


def test_unavailable_work_number_search_never_finishes() -> None:
    """mazak3.pcap f31399-f31411: DO101 drops, DO004 comes up, DO101 never returns."""
    device = _make(programs=["202"], work_search_seconds=0.5)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)

    device.set_target_work_number("999")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.io["do101"].value is False
    assert device.io["do004"].value is True
    assert device.alarm_code == WORK_SEARCH_ALARM
    assert device.active_program == ""
    assert device.output_block[44:76].rstrip(b"\x00") == b""


def test_di101_is_ignored_while_a_work_search_alarm_is_active() -> None:
    """The robot's two retries at mazak3.pcap t=1826.6/1832.7 get no response."""
    device = _make(programs=["202"], work_search_seconds=0.5)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)
    device.set_target_work_number("999")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)
    assert device.io["do101"].value is False

    for _ in range(2):
        device.set_input_bit(101, False)
        now = _settle(device, now, 0.2)
        device.set_input_bit(101, True)
        now = _settle(device, now, 0.6)
        assert device.io["do101"].value is False
        assert device.active_program == ""


def test_clearing_the_alarm_returns_do101_high() -> None:
    """mazak3.pcap t=3445: DO101 goes high in the same frame DO004 drops."""
    device = _make(programs=["202"], work_search_seconds=0.5)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)
    device.set_target_work_number("999")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)
    assert device.io["do101"].value is False

    device.set_input_bit(101, False)
    device.clear_alarm()
    now = _settle(device, now, 0.1)

    assert device.io["do101"].value is True
    assert device.io["do004"].value is False

    # ...and a valid search works again afterwards
    device.set_target_work_number("202")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.io["do101"].value is True
    assert device.active_program == "202"


def test_failed_work_search_does_not_stop_the_running_program() -> None:
    """DO103 stayed set for 38s after the mazak3.pcap alarm (t=1821 to t=1859)."""
    device = _make(programs=["202"], work_search_seconds=0.5,
                   cycle_duration_seconds=1500.0)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)
    device.set_target_work_number("202")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)
    device.set_input_bit(101, False)
    now = _settle(device, now, 1.2)
    device.set_input_bit(102, True)
    now = _settle(device, now, 0.2)
    device.set_input_bit(102, False)
    now = _settle(device, now, 0.2)
    assert device.io["do103"].value is True

    device.set_target_work_number("999")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.alarm_code == WORK_SEARCH_ALARM
    assert device.io["do103"].value is True
    assert device.state.cycle is CycleState.RUNNING


def test_any_work_number_is_accepted_when_no_library_is_configured() -> None:
    device = _make(work_search_seconds=0.5)
    for number in (1, 2, 109):
        device.set_input_bit(number, True)
    now = _settle(device, 0.0, 0.2)

    device.set_target_work_number("999")
    device.set_input_bit(101, True)
    now = _settle(device, now, 0.6)

    assert device.io["do101"].value is True
    assert device.alarm_code is None
    assert device.active_program == "999"
