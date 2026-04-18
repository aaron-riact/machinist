"""Shared test helpers."""

from __future__ import annotations

import socket

from machinist.core.device import Device


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return int(s.getsockname()[1])


def wait_running(device: Device, *, timeout: float = 2.0) -> None:
    if not device.wait_ready(timeout=timeout):
        raise RuntimeError(f"{device.name} did not become ready")
