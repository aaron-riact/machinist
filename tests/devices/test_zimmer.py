from __future__ import annotations

import json
import urllib.request

from machinist.core.events import EventBus
from machinist.core.types import Endpoint
from machinist.devices.grippers.zimmer_ged6000il import (
    ZimmerGED6000IL,
    ZimmerGED6000ILOptions,
)
from machinist.transport.iolink_http_master import IOLinkHttpMaster

from ..conftest import free_port, wait_running


def test_iolink_http_round_trip() -> None:
    port = free_port()
    device = ZimmerGED6000IL("z1", Endpoint("127.0.0.1", port), EventBus(), ZimmerGED6000ILOptions())
    device._master = IOLinkHttpMaster(host="127.0.0.1", port=port, port_device=device)
    device.start()
    try:
        wait_running(device)
        url = f"http://127.0.0.1:{port}/iolink/port/1/pd"
        with urllib.request.urlopen(url, timeout=2) as r:
            data = json.load(r)
        assert data["diameter_mm"] == 75.0

        req = urllib.request.Request(
            url,
            data=json.dumps({"target_mm": 30.0}).encode(),
            method="POST",
            headers={"Content-Type": "application/json"},
        )
        with urllib.request.urlopen(req, timeout=2) as r:
            assert json.load(r)["status"] == "ok"
    finally:
        device.stop()
