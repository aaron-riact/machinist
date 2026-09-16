from __future__ import annotations

from pathlib import Path

from machinist.core.events import Event
from machinist.core.programs import ProgramLibrary, ProgramsChanged


def test_refresh_announces_the_listing_only_when_it_changes(tmp_path: Path) -> None:
    events: list[Event] = []
    library = ProgramLibrary(root=tmp_path, owner="mill", publish=events.append)

    assert library.refresh() == ()
    assert library.refresh() == ()
    (tmp_path / "O0001.nc").write_text("M30\n")  # written behind our back, e.g. over SMB
    assert library.refresh() == ("O0001.nc",)

    names = [e.names for e in events if isinstance(e, ProgramsChanged)]
    assert names == [(), ("O0001.nc",)]
    assert all(e.device == "mill" for e in events)


def test_write_announces_at_once(tmp_path: Path) -> None:
    events: list[Event] = []
    library = ProgramLibrary(root=tmp_path, owner="mill", publish=events.append)
    library.write("O0002.nc", "M30\n")
    assert [e.names for e in events if isinstance(e, ProgramsChanged)] == [("O0002.nc",)]
