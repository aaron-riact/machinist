"""The panels of the TUI, each rendering one slice of the fleet projection.

Every widget here takes a value from :mod:`machinist.projection` and paints
it. None of them reach for a device, the bus, or each other; the App hands
them state and listens for the messages they post.
"""

from __future__ import annotations

from textual.app import ComposeResult
from textual.containers import Horizontal, Vertical
from textual.message import Message
from textual.reactive import reactive
from textual.widgets import DataTable, RichLog, Static

from ..core.events import Event
from ..core.panel import Field
from ..projection import DeviceView, FleetState
from .render import detail_header, dot, format_event, io_rows, paint_lifecycle


class DeviceList(DataTable):
    """The fleet, one row per device, keyed by name so the cursor survives repaints."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(cursor_type="row", **kwargs)  # type: ignore[arg-type]
        self._state_col = None

    def _ensure_columns(self) -> None:
        if self._state_col is None:
            _, _, self._state_col = self.add_columns("name", "kind", "state")

    def on_mount(self) -> None:
        self._ensure_columns()

    def show(self, state: FleetState) -> None:
        self._ensure_columns()
        for view in state.devices.values():
            painted = _lifecycle(view)
            if view.name in self.rows:
                self.update_cell(view.name, self._state_col, painted)
            else:
                self.add_row(view.name, view.kind, painted, key=view.name)

    @property
    def highlighted(self) -> str | None:
        """The device under the cursor, if any."""
        if self.row_count == 0:
            return None
        try:
            return str(self.coordinate_to_cell_key(self.cursor_coordinate).row_key.value)
        except Exception:  # noqa: BLE001 - an empty or half-built table has no cell there
            return None


def _lifecycle(view: DeviceView) -> str:
    return paint_lifecycle(view.lifecycle)


class FieldTable(DataTable):
    """Rows of :class:`Field`. Same shape as last time: cells update in place."""

    def __init__(self, label: str, *, with_offset: bool = True, **kwargs: object) -> None:
        super().__init__(cursor_type="row", zebra_stripes=True, cursor_foreground_priority="renderable", **kwargs)  # type: ignore[arg-type]
        self._label = label
        self._with_offset = with_offset
        self._shape: tuple[str, ...] = ()
        self._label_col = None

    def _ensure_columns(self) -> None:
        if self._label_col is None:
            columns = (self._label, "offset", "value") if self._with_offset else (self._label, "value")
            keys = self.add_columns(*columns)
            self._label_col, self._value_col = keys[0], keys[-1]

    def on_mount(self) -> None:
        self._ensure_columns()

    def show(self, fields: tuple[Field, ...]) -> None:
        self._ensure_columns()
        shape = tuple(f"{i}:{f.signal}" for i, f in enumerate(fields))
        if shape != self._shape or self.row_count != len(fields):
            self.clear()
            for key, field in zip(shape, fields, strict=True):
                cells = [_label(field)]
                if self._with_offset:
                    cells.append(field.offset)
                cells.append(field.value)
                self.add_row(*cells, key=key)
            self._shape = shape
            return
        for key, field in zip(shape, fields, strict=True):
            self.update_cell(key, self._label_col, _label(field))
            self.update_cell(key, self._value_col, field.value)


def _label(field: Field):  # type: ignore[no-untyped-def]
    return dot(field) + " " + field.signal + " " + field.name


class ProgramList(DataTable):
    """The program library of the selected device. Select a row to run it."""

    class Run(Message):
        def __init__(self, program: str) -> None:
            super().__init__()
            self.program = program

    def __init__(self, **kwargs: object) -> None:
        super().__init__(cursor_type="row", zebra_stripes=True, **kwargs)  # type: ignore[arg-type]
        self._shown: tuple[str, ...] | None = None
        self._has_columns = False

    def _ensure_columns(self) -> None:
        if not self._has_columns:
            self.add_columns("program")
            self._has_columns = True

    def on_mount(self) -> None:
        self._ensure_columns()

    def show(self, names: tuple[str, ...] | None) -> None:
        self._ensure_columns()
        names = names or ()
        if names == self._shown:
            return
        self._shown = names
        self.clear()
        for name in names:
            self.add_row(name, key=name)

    def on_data_table_row_selected(self, event: DataTable.RowSelected) -> None:
        event.stop()
        self.post_message(self.Run(str(event.row_key.value)))


class DetailPane(Vertical):
    """Everything about the selected device: header (with status), IO tables, programs."""

    view: reactive[DeviceView | None] = reactive(None)
    #: The user's wish (the F key). The list is only ever shown for a device
    #: that has programs, whatever this says.
    show_programs: reactive[bool] = reactive(True)

    def compose(self) -> ComposeResult:
        self.header = Static(id="detail-header")
        yield self.header
        with Horizontal(id="signals-row"):
            self.inputs = FieldTable("input", id="inputs")
            yield self.inputs
            self.outputs = FieldTable("output", id="outputs")
            yield self.outputs
        self.files = ProgramList(id="files")
        yield self.files

    def watch_view(self, view: DeviceView | None) -> None:
        if not hasattr(self, "header"):
            return  # compose has not run yet; on_mount paints the first view
        if view is None:
            self.header.update("[dim]no device selected[/]")
            self.inputs.show(())
            self.outputs.show(())
            self.files.show(None)
            self._place_programs()
            return
        self.header.update(detail_header(view))
        inputs, outputs = io_rows(view)
        self.inputs.show(inputs)
        self.outputs.show(outputs)
        self.files.show(view.programs)
        self._place_programs()

    def watch_show_programs(self, _show: bool) -> None:
        if hasattr(self, "files"):
            self._place_programs()

    def _place_programs(self) -> None:
        has_programs = self.view is not None and self.view.programs is not None
        self.files.display = has_programs and self.show_programs

    def on_mount(self) -> None:
        self.watch_view(self.view)


class EventLogPanel(RichLog):
    """The scrolling event log."""

    def __init__(self, **kwargs: object) -> None:
        super().__init__(wrap=False, max_lines=2000, highlight=False, markup=True, **kwargs)  # type: ignore[arg-type]

    def log_event(self, event: Event) -> None:
        self.write(format_event(event))

    def note(self, markup: str) -> None:
        self.write(markup)
