"""SRCI CLI smoke tests against a live generic robot device."""

from __future__ import annotations

import pytest
from typer.testing import CliRunner

import machinist.devices  # noqa: F401  (registers device kinds)
from machinist.core.events import EventBus
from machinist.core.registry import default_registry
from machinist.core.types import Endpoint
from machinist.srci.cli import app

from .conftest import free_port, wait_running

runner = CliRunner()


@pytest.fixture
def robot():  # type: ignore[no-untyped-def]
    port = free_port()
    device = default_registry.create(
        "robot",
        "robot1",
        Endpoint("127.0.0.1", port),
        EventBus(),
        {"joint_count": 6, "protocol": "srci", "transport": "tcp"},
    )
    device.start()
    wait_running(device)
    try:
        yield port
    finally:
        device.stop()


def test_status_prints_table(robot: int) -> None:
    result = runner.invoke(app, ["status", "--port", str(robot)])
    assert result.exit_code == 0
    assert "SRCI status" in result.stdout
    assert "joints" in result.stdout


def test_enable_then_estop(robot: int) -> None:
    assert runner.invoke(app, ["enable", "--port", str(robot)]).exit_code == 0
    result = runner.invoke(app, ["estop", "--port", str(robot)])
    assert result.exit_code == 0
    assert "ESTOP" in result.stdout


def test_movej_accepts_joint_vector(robot: int) -> None:
    result = runner.invoke(
        app, ["movej", "0.1", "0.2", "0.0", "0.0", "0.0", "0.0", "--port", str(robot)]
    )
    assert result.exit_code == 0


def test_movel_requires_six_values(robot: int) -> None:
    result = runner.invoke(app, ["movel", "1.0", "2.0", "--port", str(robot)])
    assert result.exit_code == 1
    assert "6 values" in result.stdout


def test_connect_failure_is_reported() -> None:
    result = runner.invoke(app, ["status", "--port", str(free_port())])
    assert result.exit_code == 1
