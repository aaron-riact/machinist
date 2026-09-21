from __future__ import annotations

import socket

from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.robots.arm import ArmOptions
from machinist.devices.robots.ur import UR_FLANGE_GATEWAY_PORT, URDashboardServer
from machinist.transport.flange_bus import FlangeBus
from machinist.transport.modbus_rtu import framed
from machinist.transport.modbus_rtu_gateway import ModbusRtuGateway

from ..conftest import free_port, wait_running


def _send(host: str, port: int, msg: str) -> str:
    with socket.create_connection((host, port), timeout=2) as s:
        s.sendall(msg.encode() + b"\n")
        return s.recv(256).decode().strip()


def test_dashboard_basic_commands() -> None:
    port = free_port()
    bus = EventBus()
    device = URDashboardServer(
        "ur1", Endpoint("127.0.0.1", port), bus, ArmOptions(), flange=FlangeBus()
    )
    device.start()
    try:
        wait_running(device)
        greeting = _send("127.0.0.1", port, "polyscope")
        assert greeting == "Connected: Universal Robots Dashboard Server"
        assert "RUNNING" in _send("127.0.0.1", port, "robotmode")
        assert "Loading program" in _send("127.0.0.1", port, "load /programs/x")
        # Trigger e-stop via the underlying arm and observe through robotmode.
        device.arm.estop()
        assert "PROTECTIVE_STOP" in _send("127.0.0.1", port, "robotmode")
    finally:
        device.stop()


# --- the tool flange, and the door onto it ----------------------------


def _ur_from_config(**options) -> URDashboardServer:
    """Build through the registry, so the YAML boundary is what is tested."""
    device = default_registry.create(
        "ur_dashboard", "ur", Endpoint("127.0.0.1", free_port()), EventBus(), options
    )
    assert isinstance(device, URDashboardServer)
    return device


def test_a_ur_starts_with_an_empty_flange() -> None:
    assert _ur_from_config().flange.slave_ids == ()


def test_the_passthrough_is_off_until_it_is_asked_for() -> None:
    """A bare UR does not forward the tool line to the network."""
    assert _ur_from_config().flange_gateway_ports == ()


def test_asking_for_the_passthrough_opens_12345() -> None:
    ports = _ur_from_config(flange_gateway_ports=True).flange_gateway_ports

    assert ports == (UR_FLANGE_GATEWAY_PORT,)


def test_the_passthrough_port_can_be_named_outright() -> None:
    assert _ur_from_config(flange_gateway_ports=1502).flange_gateway_ports == (1502,)


def test_a_gripper_on_the_flange_is_read_over_the_passthrough() -> None:
    port = free_port()
    flange = FlangeBus()
    gateway = ModbusRtuGateway(host="127.0.0.1", ports=[port], line=flange)
    device = URDashboardServer(
        "ur_gw", Endpoint("127.0.0.1", free_port()), EventBus(), ArmOptions(),
        flange=flange, gateway=gateway,
    )
    rg = default_registry.create(
        "onrobot_rg", "rg1", Endpoint("127.0.0.1", free_port()), EventBus(),
        {"initial_width_mm": 110.0},
    )
    flange.attach(0x41, rg.register_port)
    device.start()
    try:
        wait_running(device)
        with socket.create_connection(("127.0.0.1", port), timeout=2) as sock:
            sock.sendall(framed(b"\x41\x03\x01\x0b\x00\x01"))
            reply = sock.recv(64)
    finally:
        device.stop()

    assert reply == framed(b"\x41\x03\x02\x04\x4c")
