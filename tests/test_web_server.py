from __future__ import annotations

import json
import urllib.error
import urllib.request
from collections.abc import Iterator

import pytest

import machinist.devices  # noqa: F401  (import = device-kind registration)
from machinist.core.config import DeviceConfig, IOLink, SystemConfig
from machinist.core.events import Note
from machinist.core.world import World, WorldBuilder
from machinist.web.server import WebServer, changed_views, device_frame, event_to_dict

from .conftest import free_port, wait_running


def _world() -> World:
    return WorldBuilder().build(
        SystemConfig(
            devices=(
                DeviceConfig(
                    name="io1", kind="weidmuller_ur20", port=free_port(),
                    options={"inputs": 8, "outputs": 8},
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
    frame = event_to_dict(Note(device="io1", name="rx", data={"line": "x"}, timestamp=1.0))
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


def _frames(stream, *, want: int = 0, until=None, budget: int = 200) -> list[dict]:
    """Read SSE frames until *want* have arrived or *until* accepts one."""
    frames: list[dict] = []
    for _ in range(budget):
        chunk = stream.readline()
        if not chunk.startswith(b"data:"):
            continue
        frame = json.loads(chunk[len(b"data:") :].strip())
        frames.append(frame)
        if (want and len(frames) >= want) or (until is not None and until(frame)):
            break
    return frames


def test_event_stream_opens_with_a_device_frame_per_device(server: WebServer) -> None:
    req = urllib.request.Request(f"{server.url}/api/events")
    with urllib.request.urlopen(req, timeout=5) as stream:
        frames = _frames(stream, want=2)
    assert [f["kind"] for f in frames] == ["device", "device"]
    assert {f["device"]["name"] for f in frames} == {"io1", "g1"}


def test_event_stream_pushes_a_signal_change_as_log_and_device_frames(server: WebServer) -> None:
    req = urllib.request.Request(f"{server.url}/api/events")
    with urllib.request.urlopen(req, timeout=5) as stream:
        _frames(stream, want=2)  # the opening snapshot
        _post(f"{server.url}/api/command", {"command": "set io1.o5 1"})
        frames = _frames(
            stream, until=lambda f: f["kind"] == "device" and f["device"]["name"] == "io1"
        )
    kinds = {f["kind"] for f in frames}
    assert "signal" in kinds, "the log hears the signal"
    io1 = [f["device"] for f in frames if f["kind"] == "device" and f["device"]["name"] == "io1"]
    assert io1, "the projection re-sent the device that changed"
    assert next(s for s in io1[-1]["signals"] if s["name"] == "o5")["value"] is True


def test_changed_views_names_only_what_moved() -> None:
    from machinist.projection import DeviceView, FleetState

    a = DeviceView(name="a", kind="k", endpoint="e")
    b = DeviceView(name="b", kind="k", endpoint="e")
    before = FleetState.of([a, b])
    after = before.with_device(DeviceView(name="b", kind="k", endpoint="e", fault="x"))
    assert [v.name for v in changed_views(before, after)] == ["b"]
    assert changed_views(FleetState(), before) == [a, b]


def test_device_frame_is_json_able() -> None:
    from machinist.projection import DeviceView

    frame = device_frame(DeviceView(name="a", kind="k", endpoint="e"))
    assert frame["kind"] == "device" and frame["device"]["name"] == "a"
    json.dumps(frame)
