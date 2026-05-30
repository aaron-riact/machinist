"""OPC-UA publishing on the generic robot device (requires asyncua)."""

from __future__ import annotations

import pytest

pytest.importorskip("asyncua")

from asyncua import Client  # noqa: E402

import machinist.devices  # noqa: E402,F401  (registers device kinds)
from machinist.core.events import EventBus  # noqa: E402
from machinist.core.registry import default_registry  # noqa: E402
from machinist.core.types import Endpoint  # noqa: E402

from .conftest import free_port, wait_running  # noqa: E402


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
