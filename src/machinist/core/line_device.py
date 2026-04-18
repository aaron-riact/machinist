"""Convenient :class:`Device` subclass for line-protocol devices.

Subclasses just implement :meth:`handle_line`.
"""

from __future__ import annotations

import threading
from collections.abc import Iterable

from .device import Device
from .events import EventBus
from .types import Endpoint
from ..transport.line_server import LineServer


class LineServerDevice(Device):
    """A device whose external face is a TCP line protocol."""

    #: Wire-line terminator. Override per subclass (e.g. ``"\\r\\n"``).
    TERMINATOR: str = "\n"
    ENCODING: str = "ascii"

    def __init__(self, name: str, endpoint: Endpoint, bus: EventBus) -> None:
        super().__init__(name, endpoint, bus)
        self._server = LineServer(
            endpoint.host,
            endpoint.port,
            handler=self._wrapped_handle,
            terminator=self.TERMINATOR,
            encoding=self.ENCODING,
        )

    def handle_line(self, line: str) -> Iterable[str] | str | None:
        """Handle one received line; subclasses MUST override."""
        raise NotImplementedError

    def _wrapped_handle(self, line: str) -> Iterable[str] | str | None:
        self.emit("rx", line=line)
        try:
            reply = self.handle_line(line)
        except Exception as exc:  # pragma: no cover - exercised in tests
            self.emit("error", message=str(exc), line=line)
            raise
        if reply is not None:
            self.emit("tx", reply=reply if isinstance(reply, str) else list(reply))
        return reply

    def _run(self, stop: threading.Event) -> None:
        ready = threading.Event()
        thread = threading.Thread(
            target=self._server.serve_forever, args=(ready,), daemon=True
        )
        thread.start()
        if not ready.wait(timeout=2.0):
            raise RuntimeError(f"{self.name} server failed to bind")
        self._mark_running()
        stop.wait()
        self._server.shutdown()
        thread.join(timeout=2.0)

    def _shutdown(self) -> None:
        self._server.shutdown()
