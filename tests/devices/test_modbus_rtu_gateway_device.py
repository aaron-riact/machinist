"""A bare RTU gateway: a flange line with no arm on the end of it."""

from __future__ import annotations

import socket

import pytest

from machinist.core.config import DeviceConfig, FlangeLink, SystemConfig
from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.core.world import running
from machinist.devices.gateways.modbus_rtu import (
    DEFAULT_GATEWAY_PORT,
    ModbusRtuGatewayDevice,
)
from machinist.transport.modbus_rtu import framed

from ..conftest import free_port, wait_running


def _gateway(port: int, **options) -> ModbusRtuGatewayDevice:
    device = default_registry.create(
        "modbus_rtu_gateway", "bridge", Endpoint("127.0.0.1", port), EventBus(), options
    )
    assert isinstance(device, ModbusRtuGatewayDevice)
    return device


def test_the_device_port_is_the_door_onto_the_line() -> None:
    assert _gateway(1502).flange_gateway_ports == (1502,)


def test_the_default_port_is_the_one_a_controller_answers_on() -> None:
    assert default_registry.default_port("modbus_rtu_gateway") == DEFAULT_GATEWAY_PORT


def test_further_doors_can_be_opened_onto_the_same_line() -> None:
    assert _gateway(1502, extra_ports=[1503]).flange_gateway_ports == (1502, 1503)


def test_a_bare_line_starts_with_nothing_on_it() -> None:
    assert _gateway(1502).flange.slave_ids == ()


def test_an_option_the_gateway_does_not_have_is_refused() -> None:
    with pytest.raises(ValueError, match="robot_type"):
        _gateway(1502, robot_type="cr10a")


def test_a_gripper_wired_onto_the_line_answers_over_tcp() -> None:
    port = free_port()
    device = _gateway(port)
    rg = default_registry.create(
        "onrobot_rg", "rg1", Endpoint("127.0.0.1", free_port()), EventBus(),
        {"initial_width_mm": 110.0},
    )
    device.flange.attach(0x41, rg.register_port)
    device.start()
    try:
        wait_running(device)
        with socket.create_connection(("127.0.0.1", port), timeout=2) as sock:
            sock.sendall(framed(b"\x41\x03\x01\x0b\x00\x01"))
            reply = sock.recv(64)
    finally:
        device.stop()

    assert reply == framed(b"\x41\x03\x02\x04\x4c")


def test_a_scene_wires_grippers_onto_the_gateway_with_flange_links() -> None:
    """flange_links does not care that the master is not an arm."""
    port = free_port()
    config = SystemConfig(
        devices=(
            DeviceConfig(name="bridge", kind="modbus_rtu_gateway", host="127.0.0.1", port=port),
            DeviceConfig(
                name="rg1", kind="onrobot_rg", host="127.0.0.1", port=free_port(),
                options={"initial_width_mm": 110.0},
            ),
            DeviceConfig(
                name="rg2", kind="onrobot_rg", host="127.0.0.1", port=free_port(),
                options={"model": "rg6", "initial_width_mm": 160.0},
            ),
        ),
        flange_links=(
            FlangeLink(master="bridge", slave="rg1", slave_id=0x41),
            FlangeLink(master="bridge", slave="rg2", slave_id=0x42),
        ),
    )

    with running(config) as world:
        for device in world.devices:
            wait_running(device)
        with socket.create_connection(("127.0.0.1", port), timeout=2) as sock:
            sock.sendall(framed(b"\x41\x03\x01\x0b\x00\x01"))
            first = sock.recv(64)
            sock.sendall(framed(b"\x42\x03\x01\x0b\x00\x01"))
            second = sock.recv(64)

    assert first == framed(b"\x41\x03\x02\x04\x4c")
    assert second == framed(b"\x42\x03\x02\x06\x40")
