"""OPC-UA publishing on the generic robot device (requires asyncua)."""

from __future__ import annotations

import socket
import time

import pytest

pytest.importorskip("asyncua")

from asyncua import Client

import machinist.devices  # noqa: F401  (registers device kinds)
from machinist.core.device import Device
from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint

from .conftest import free_port, wait_running


def _wait_port(host: str, port: int, *, timeout: float = 10.0) -> None:
    """Block until ``port`` is accepting TCP connections."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            with socket.create_connection((host, port), timeout=0.5):
                return
        except (ConnectionRefusedError, OSError):
            time.sleep(0.1)
    raise RuntimeError(f"{host}:{port} not listening after {timeout}s")


@pytest.fixture(scope="module")
def opcua_devices(tmp_path_factory: pytest.TempPathFactory) -> tuple[Device, Device]:
    srci_port = free_port()
    robot_opcua_port = free_port()
    robot = default_registry.create(
        "robot",
        "arm-opc",
        Endpoint("127.0.0.1", srci_port),
        EventBus(),
        {"joint_count": 6, "opcua": {"port": robot_opcua_port}},
    )
    robot.start()

    tmp_path = tmp_path_factory.mktemp("opcua_haas")
    mdc_port = free_port()
    haas_opcua_port = free_port()
    haas = default_registry.create(
        "haas_ngc",
        "haas-opc",
        Endpoint("127.0.0.1", mdc_port),
        EventBus(),
        {
            "program_folder": str(tmp_path),
            "doors": ["main"],
            "opcua": {"port": haas_opcua_port},
        },
    )
    haas.start()

    try:
        wait_running(robot, timeout=10.0)
        _wait_port("127.0.0.1", robot_opcua_port)
        wait_running(haas, timeout=10.0)
        _wait_port("127.0.0.1", haas_opcua_port)
    except RuntimeError:
        robot.stop()
        haas.stop()
        raise
    robot._opcua_port = robot_opcua_port  # type: ignore[attr-defined]
    haas._opcua_port = haas_opcua_port  # type: ignore[attr-defined]
    yield robot, haas
    robot.stop()
    haas.stop()


async def test_robot_publishes_state_over_opcua(opcua_devices: tuple[Device, Device]) -> None:
    device = opcua_devices[0]
    async with Client(f"opc.tcp://127.0.0.1:{device._opcua_port}/machinist/server/") as client:
        idx = await client.get_namespace_index("urn:machinist")
        objects = client.nodes.objects
        arm = await objects.get_child([f"{idx}:arm-opc"])
        mode_node = await arm.get_child([f"{idx}:mode"])
        assert await mode_node.read_value() == "idle"
        joints_node = await arm.get_child([f"{idx}:joints"])
        assert isinstance(await joints_node.read_value(), str)


async def test_haas_publishes_state_over_opcua(opcua_devices: tuple[Device, Device]) -> None:
    device = opcua_devices[1]
    async with Client(f"opc.tcp://127.0.0.1:{device._opcua_port}/machinist/server/") as client:
        idx = await client.get_namespace_index("urn:machinist")
        machine = await client.nodes.objects.get_child([f"{idx}:haas-opc"])
        cycle = await machine.get_child([f"{idx}:cycle"])
        assert await cycle.read_value() == "idle"
        door = await machine.get_child([f"{idx}:door_main_open"])
        assert await door.read_value() is False
        x = await machine.get_child([f"{idx}:x"])
        assert await x.read_value() == 0.0
