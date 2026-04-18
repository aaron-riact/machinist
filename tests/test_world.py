from __future__ import annotations

from machinist.core.config import SystemConfig
from machinist.core.world import WorldBuilder
from pathlib import Path
import yaml


def test_world_builds_from_yaml(tmp_path: Path) -> None:
    cfg = {
        "devices": [
            {"name": "g1", "kind": "pneumatic_gripper", "options": {"settle_seconds": 0.01}},
            {"name": "io1", "kind": "weidmuller_ur20", "host": "127.0.0.1", "port": 0,
             "options": {"inputs": 4, "outputs": 4}},
        ],
        "io_links": [
            {"source": "io1.o1", "target": "g1.cmd_open"},
        ],
    }
    path = tmp_path / "scene.yaml"
    path.write_text(yaml.safe_dump(cfg))

    from machinist.core.config import load_config
    config = load_config([path])
    assert isinstance(config, SystemConfig)
    world = WorldBuilder().build(config)
    assert {d.name for d in world.devices} == {"g1", "io1"}
    # IO link works without starting servers (we don't bind sockets here).
    world.io_map.bank("io1")["o1"].set(True)
    assert world.devices[0].io["cmd_open"].value is True
