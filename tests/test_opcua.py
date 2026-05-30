"""OPC-UA publishing on the generic robot device (requires asyncua)."""

from __future__ import annotations

import pytest

pytest.importorskip("asyncua")

from asyncua import Client

import machinist.devices  # noqa: F401  (registers device kinds)
from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint

from .conftest import free_port, wait_running


async def test_robot_publishes_state_over_opcua() -> None:
    srci_port = free_port()
    opcua_port = free_port()
    device = default_registry.create(
        "robot",
        "arm-opc",
        Endpoint("127.0.0.1", srci_port),
        EventBus(),
        {"joint_count": 6, "opcua": {"port": opcua_port}},
    )
    device.start()
    try:
        wait_running(device)
        async with Client(f"opc.tcp://127.0.0.1:{opcua_port}/machinist/server/") as client:
            idx = await client.get_namespace_index("urn:machinist")
            objects = client.nodes.objects
            arm = await objects.get_child([f"{idx}:arm-opc"])
            mode_node = await arm.get_child([f"{idx}:mode"])
            assert await mode_node.read_value() == "idle"
            joints_node = await arm.get_child([f"{idx}:joints"])
            assert isinstance(await joints_node.read_value(), str)
    finally:
        device.stop()


async def test_haas_publishes_state_over_opcua(tmp_path) -> None:  # type: ignore[no-untyped-def]
    mdc_port = free_port()
    opcua_port = free_port()
    device = default_registry.create(
        "haas_ngc",
        "haas-opc",
        Endpoint("127.0.0.1", mdc_port),
        EventBus(),
        {
            "program_folder": str(tmp_path),
            "doors": ["main"],
            "opcua": {"port": opcua_port},
        },
    )
    device.start()
    try:
        wait_running(device)
        async with Client(f"opc.tcp://127.0.0.1:{opcua_port}/machinist/server/") as client:
            idx = await client.get_namespace_index("urn:machinist")
            machine = await client.nodes.objects.get_child([f"{idx}:haas-opc"])
            cycle = await machine.get_child([f"{idx}:cycle"])
            assert await cycle.read_value() == "idle"
            door = await machine.get_child([f"{idx}:door_main_open"])
            assert await door.read_value() is False
    finally:
        device.stop()
