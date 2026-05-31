"""Live web interface for Machinist.

A browser mirror of the Textual TUI: a devices grid, a per-device detail
pane (signals + arm/machine telemetry), a scrolling event log and a command
bar. The backend deliberately leans on the Python standard library only —
``http.server`` for routing and **Server-Sent Events** for the live feed —
so the web UI adds *no* hard runtime dependency, exactly like the rest of
Machinist.

Two layers, kept apart so the interesting logic stays trivially testable:

* :mod:`machinist.web.api`    — pure, IO-free state serialization + command
  dispatch over a :class:`~machinist.core.world.World`.
* :mod:`machinist.web.server` — thin HTTP/SSE wiring over that pure core.
"""

from __future__ import annotations

from .server import WebServer, serve

__all__ = ["WebServer", "serve"]
