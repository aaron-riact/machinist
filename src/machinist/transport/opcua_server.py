"""A tiny read-only OPC-UA server exposing live device state.

OPC-UA is the lingua franca for monitoring industrial equipment, so any
robot or machine should be able to publish its state as browsable nodes.
The wire stack is heavy, so :mod:`asyncua` is an optional dependency,
lazily imported here; importing this module never requires it.

The server is intentionally dumb: a device hands us a mapping of node
name → zero-arg reader, and we poll those readers and write the values
into matching OPC-UA variables on an interval. That keeps every device
ignorant of OPC-UA internals — they expose plain Python callables and
nothing more.

Multiple :class:`OpcUaServer` instances in the same process share a single
underlying ``asyncua.Server`` and event loop so that the expensive
``Server.init()`` call runs once instead of once per device.
"""

from __future__ import annotations

import asyncio
import concurrent.futures
import threading
from collections.abc import Callable, Mapping
from typing import TYPE_CHECKING, Any

NodeReaders = Mapping[str, Callable[[], object]]

DEFAULT_URI = "urn:machinist"

if TYPE_CHECKING:
    from asyncua import Server  # type: ignore[import-untyped]  # noqa: PLC0415

_init_lock = asyncio.Lock()
_shared_loop: asyncio.AbstractEventLoop | None = None
_shared_server: Server | None = None
_shared_idx: int | None = None


_start_lock = threading.Lock()


def _ensure_loop() -> None:
    """Start the shared asyncio event loop in a daemon thread if not running."""
    global _shared_loop
    if _shared_loop is not None:
        return
    with _start_lock:
        if _shared_loop is not None:
            return
        _shared_loop = asyncio.new_event_loop()
        t = threading.Thread(target=_shared_loop.run_forever, daemon=True)
        t.start()


class OpcUaServer:
    """Publish a device's state as OPC-UA variables on a background loop."""

    def __init__(
        self,
        host: str,
        port: int,
        *,
        device_name: str,
        readers: NodeReaders,
        uri: str = DEFAULT_URI,
        interval: float = 0.2,
    ) -> None:
        self._endpoint = f"opc.tcp://{host}:{port}/machinist/server/"
        self._device_name = device_name
        self._readers = dict(readers)
        self._uri = uri
        self._interval = interval
        self._stop = threading.Event()
        self._task: concurrent.futures.Future[None] | None = None

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        _ensure_loop()
        assert _shared_loop is not None
        self._task = asyncio.run_coroutine_threadsafe(
            self._serve(ready), _shared_loop,
        )
        assert self._task is not None
        try:
            self._task.result()
        except asyncio.CancelledError:
            pass

    def shutdown(self) -> None:
        self._stop.set()

    # -----------------------------------------------------------------

    async def _serve(self, ready: threading.Event | None) -> None:
        from asyncua import Server  # type: ignore[import-untyped]  # noqa: PLC0415  (optional dep)

        global _shared_server, _shared_idx

        async with _init_lock:
            if _shared_server is None:
                srv = Server()
                await srv.init()
                srv.set_endpoint(self._endpoint)
                srv.set_server_name("machinist")
                await srv.start()
                idx = await srv.register_namespace(self._uri)
                _shared_idx = idx
                _shared_server = srv

        srv = _shared_server
        idx = _shared_idx
        assert srv is not None
        assert idx is not None

        obj = await srv.nodes.objects.add_object(idx, self._device_name)
        variables = {}
        for name, read in self._readers.items():
            variables[name] = await obj.add_variable(idx, name, _coerce(read()))

        if ready is not None:
            ready.set()

        while not self._stop.is_set():
            for name, read in self._readers.items():
                await variables[name].write_value(_coerce(read()))
            await asyncio.sleep(self._interval)


def _coerce(value: object) -> object:
    """Flatten tuples/lists to a comma-joined string OPC-UA can carry."""
    if isinstance(value, (tuple, list)):
        return ", ".join(f"{v:.4f}" if isinstance(v, float) else str(v) for v in value)
    return value
