from __future__ import annotations

from types import SimpleNamespace

from machinist.core.events import Event
from machinist.core.types import DeviceState
from machinist.tui.app import _cmd_ls, _cmd_run, _format_event, _paint_lifecycle


def test_format_event_is_compact_and_deterministic() -> None:
    ev = Event(device="ur1", kind="rx", payload={"line": "power on"}, timestamp=1234.567)
    out = _format_event(ev)
    assert "ur1" in out and "rx" in out and "power on" in out
    assert "         rx" not in out


def test_paint_lifecycle_uses_expected_colours() -> None:
    assert _paint_lifecycle(DeviceState.RUNNING).startswith("[green]")
    assert _paint_lifecycle(DeviceState.FAULTED).startswith("[red]")


class _FakeApp:
    def __init__(self, device) -> None:
        self._selected = device.name
        self._device = device
        self.writes: list[str] = []
        self._log = SimpleNamespace(write=self.writes.append)

    def _lookup(self, name):
        return self._device if (name in (None, self._device.name)) else None


def _fake_device(*, programs=None, run=None):
    return SimpleNamespace(name="haas1", programs=programs, run_program=run)


def test_cmd_ls_lists_programs() -> None:
    programs = SimpleNamespace(list=lambda: ["O0001.nc", "O0002.nc"])
    app = _FakeApp(_fake_device(programs=programs))
    _cmd_ls(app, "")
    assert any("O0001.nc" in w for w in app.writes)


def test_cmd_ls_complains_when_no_library() -> None:
    app = _FakeApp(_fake_device(programs=None))
    _cmd_ls(app, "")
    assert any("no program library" in w for w in app.writes)


def test_cmd_run_dispatches_program_name() -> None:
    called: list[str] = []
    device = _fake_device(
        programs=SimpleNamespace(list=lambda: []),
        run=called.append,
    )
    app = _FakeApp(device)
    _cmd_run(app, "haas1 O0001.nc")
    assert called == ["O0001.nc"]


def test_cmd_run_reports_errors() -> None:
    def boom(_name: str) -> None:
        raise RuntimeError("already running")
    app = _FakeApp(_fake_device(programs=None, run=boom))
    _cmd_run(app, "haas1 X.nc")
    assert any("already running" in w for w in app.writes)
