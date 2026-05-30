"""The generic protocol-driven robot device, driven over SRCI."""

from __future__ import annotations

import pytest

from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.devices.robots.generic import protocols
from machinist.srci import SrciClient, StatusFlag

import machinist.devices  # noqa: F401  (registers device kinds)

from ..conftest import free_port, wait_running


def _make_robot(port: int, **options: object):
    return default_registry.create(
        "robot",
        "robot1",
        Endpoint("127.0.0.1", port),
        EventBus(),
        {"joint_count": 6, **options},
    )


def test_robot_kind_is_registered() -> None:
    assert "robot" in default_registry.kinds()
    assert "srci" in protocols()


def test_unknown_protocol_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown robot protocol"):
        _make_robot(free_port(), protocol="ge-fanuc-secret")


@pytest.mark.parametrize("transport", ["tcp", "udp"])
def test_robot_serves_srci_over_transport(transport: str) -> None:
    port = free_port()
    device = _make_robot(port, protocol="srci", transport=transport)
    device.start()
    try:
        wait_running(device)
        with SrciClient.connect("127.0.0.1", port, transport=transport) as client:
            assert StatusFlag.SERVO_ON in client.enable().flags
            client.move_joint((0.3, 0.0, 0.0, 0.0, 0.0, 0.0))
            assert StatusFlag.ESTOP in client.estop().flags
    finally:
        device.stop()
