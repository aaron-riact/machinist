from __future__ import annotations

from machinist.core.events import Event
from machinist.tui.app import _format_event, _paint_lifecycle
from machinist.core.types import DeviceState


def test_format_event_is_compact_and_deterministic() -> None:
    ev = Event(device="ur1", kind="rx", payload={"line": "power on"}, timestamp=1234.567)
    out = _format_event(ev)
    # Should contain all pieces on one line; no wild gaps between
    # device name and kind (regression: old format used '>16' padding
    # which made 3-char names look like they had 13 leading spaces).
    assert "ur1" in out and "rx" in out and "power on" in out
    assert "         rx" not in out  # at most a handful of spaces


def test_paint_lifecycle_uses_expected_colours() -> None:
    assert _paint_lifecycle(DeviceState.RUNNING).startswith("[green]")
    assert _paint_lifecycle(DeviceState.FAULTED).startswith("[red]")
