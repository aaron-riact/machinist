from __future__ import annotations

import json
import urllib.error
import urllib.request
from collections.abc import Iterator

import pytest

import machinist.devices  # noqa: F401  (import = device-kind registration)
from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.events import Event
from machinist.core.world import World, WorldBuilder
from machinist.web.server import WebServer, event_to_dict

from .conftest import wait_running


def _world() -> World:
    return WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(
                    name="io1", kind="weidmuller_ur20", options={"inputs": 8, "outputs": 8}
                ),
                DeviceConfig(
                    name="g1", kind="pneumatic_gripper", options={"settle_seconds": 0.01}
                ),
            ),
            io_links=(IOLink(source="io1.o5", target="g1.cmd_open"),),
        )
    )


@pytest.fixture
def server() -> Iterator[WebServer]:
    world = _world()
    world.start()
    for device in world.devices:
        wait_running(device)
    srv = WebServer(world, host="127.0.0.1", port=0)
    srv.start()
    try:
        yield srv
    finally:
        srv.stop()
        world.stop()


def _get(url: str) -> tuple[int, bytes]:
    with urllib.request.urlopen(url, timeout=5) as resp:
        return resp.status, resp.read()


def _post(url: str, payload: dict) -> tuple[int, dict]:
    data = json.dumps(payload).encode()
    req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as resp:
            return resp.status, json.loads(resp.read())
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read())


def test_event_to_dict_is_json_able() -> None:
    frame = event_to_dict(Event(device="io1", kind="rx", payload={"line": "x"}, timestamp=1.0))
    assert frame == {"device": "io1", "kind": "rx", "payload": {"line": "x"}, "timestamp": 1.0}
    json.dumps(frame)  # must not raise


def test_state_endpoint_serves_fleet(server: WebServer) -> None:
    status, body = _get(f"{server.url}/api/state")
    assert status == 200
    snap = json.loads(body)
    names = {d["name"] for d in snap["devices"]}
    assert {"io1", "g1"} <= names


def test_command_endpoint_sets_signal(server: WebServer) -> None:
    status, result = _post(f"{server.url}/api/command", {"command": "set io1.o5 1"})
    assert status == 200
    assert result["ok"] is True
    # The state endpoint should now reflect the driven signal.
    _status, body = _get(f"{server.url}/api/state")
    io1 = next(d for d in json.loads(body)["devices"] if d["name"] == "io1")
    o5 = next(s for s in io1["signals"] if s["name"] == "o5")
    assert o5["value"] is True


def test_command_endpoint_reports_errors(server: WebServer) -> None:
    status, result = _post(f"{server.url}/api/command", {"command": "frobnicate"})
    assert status == 400
    assert result["ok"] is False
    assert "unknown command" in result["message"]


def test_unknown_static_path_is_404(server: WebServer) -> None:
    with pytest.raises(urllib.error.HTTPError) as excinfo:
        _get(f"{server.url}/../server.py")
    assert excinfo.value.code == 404


def test_event_stream_pushes_live_events(server: WebServer) -> None:
    req = urllib.request.Request(f"{server.url}/api/events")
    with urllib.request.urlopen(req, timeout=5) as stream:
        # Trigger an event by driving a signal through the command endpoint.
        _post(f"{server.url}/api/command", {"command": "set io1.o5 1"})
        line = b""
        for _ in range(50):
            chunk = stream.readline()
            if chunk.startswith(b"data:"):
                line = chunk
                break
        assert line.startswith(b"data:")
        frame = json.loads(line[len(b"data:") :].strip())
        assert "device" in frame
        assert "kind" in frame
