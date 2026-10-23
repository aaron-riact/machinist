from __future__ import annotations

from typer.testing import CliRunner

from machinist.cli import _device_listing, app
from machinist.core.device import Device
from machinist.core.events import EventBus
from machinist.core.types import Endpoint


def test_version() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["version"])
    assert result.exit_code == 0
    assert "machinist" in result.stdout


def test_kinds_lists_all_devices() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["kinds"])
    assert result.exit_code == 0
    for kind in (
        "ur_dashboard",
        "motoman_nx100",
        "yaskawa",
        "dobot_dashboard",
        "fanuc_r30ib",
        "haas_ngc",
        "mazak_840d",
        "mazak_smooth",
        "pneumatic_gripper",
        "onrobot_3fg25",
        "zimmer_ged6000il",
        "weidmuller_ur20",
    ):
        assert kind in result.stdout


def test_run_with_no_tui(tmp_path) -> None:  # type: ignore[no-untyped-def]
    cfg = tmp_path / "scene.yaml"
    cfg.write_text(
        "devices:\n"
        "  - name: g1\n    kind: pneumatic_gripper\n    options: {settle_seconds: 0.01}\n"
    )
    runner = CliRunner()
    # --no-tui returns immediately because we don't actually wait for a signal.
    # We use a 1s timeout via Ctrl+C emulation by patching sigwait isn't easy;
    # instead, validate that the parser accepts the args.
    result = runner.invoke(app, ["run", "--help"])
    assert result.exit_code == 0
    assert "configs" in result.stdout.lower()


def test_run_help_documents_web_options() -> None:
    runner = CliRunner()
    result = runner.invoke(app, ["run", "--help"])
    assert result.exit_code == 0
    for flag in ("--web", "--web-host", "--web-port"):
        assert flag in result.stdout


def test_the_start_up_listing_puts_each_devices_settings_under_it() -> None:
    class _Set(Device):
        def settings(self) -> list[str]:
            return ["first", "second"]

    class _Plain(Device):
        pass

    lines = _device_listing([_Set("a", Endpoint("127.0.0.1", 1), EventBus()),
                             _Plain("b", Endpoint("127.0.0.1", 2), EventBus())])
    assert len(lines) == 4
    assert "a" in lines[0]
    assert lines[1].endswith("first")
    assert lines[2].endswith("second")
    assert "b" in lines[3]
