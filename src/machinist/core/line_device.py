"""Convenient :class:`Device` subclass for line-protocol devices.

Subclasses either:

* override :meth:`handle_line` for a stateless protocol, **or**
* override :meth:`make_session` to return a fresh per-connection
  :class:`SessionHandler` (useful for protocols with a handshake).
"""

from __future__ import annotations

import threading

from ..transport.framing import Framer, TerminatorFramer
from ..transport.line_server import LineServer, Reply, SessionHandler, stateless
from .device import Device
from .events import EventBus
from .types import Endpoint


class LineServerDevice(Device):
    """A device whose external face is a line-protocol TCP server."""

    #: Per-subclass framing. Override with :class:`TerminatorFramer`,
    #: :class:`ParenFramer`, or any custom :class:`Framer`.
    FRAMER: Framer = TerminatorFramer()

    def __init__(self, name: str, endpoint: Endpoint, bus: EventBus) -> None:
        super().__init__(name, endpoint, bus)
        self._server = LineServer(
            endpoint.host,
            endpoint.port,
            session_factory=self.make_session,
            framer=self.FRAMER,
        )

    # ----- subclass hooks --------------------------------------------

    def handle_line(self, line: str) -> Reply:
        """Handle one received message (stateless default)."""
        raise NotImplementedError

    def make_session(self) -> SessionHandler:
        """Return a per-connection session handler.

        The default implementation defers to :meth:`handle_line`, which
        keeps every message independent. Override for handshakes.
        """
        return stateless(self._wrapped_handle)()

    # ----- internals --------------------------------------------------

    def _wrapped_handle(self, line: str) -> Reply:
        self.emit("rx", line=line)
        try:
            reply = self.handle_line(line)
        except Exception as exc:
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
