"""A single-writer cell around a frozen view.

Devices keep their private state in one of these. Reading is a plain
attribute access on an immutable value. Writing goes through :meth:`swap`
or :meth:`update`, which replace the whole view atomically and call
``on_change`` with the new one -- the hook a device uses to announce the
change on the bus. There is no other way to change the view, so a change
that forgets to announce itself cannot be written.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import replace


class StateCell[V]:
    """Holds one frozen dataclass value and announces every replacement."""

    def __init__(self, initial: V, *, on_change: Callable[[V], None] | None = None) -> None:
        self._view = initial
        self._on_change = on_change
        self._lock = threading.Lock()

    @property
    def view(self) -> V:
        return self._view

    def swap(self, change: Callable[[V], V]) -> V:
        """Replace the view with ``change(view)``, atomically. Announces if it differs."""
        with self._lock:
            new = change(self._view)
            changed = new != self._view
            if changed:
                self._view = new
        if changed and self._on_change is not None:
            self._on_change(new)
        return new

    def update(self, **changes: object) -> V:
        """Replace the given fields of the view."""
        return self.swap(lambda view: replace(view, **changes))  # type: ignore[type-var]
