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
"""

from __future__ import annotations

import asyncio
import contextlib
import threading
from collections.abc import Callable, Mapping

NodeReaders = Mapping[str, Callable[[], object]]

DEFAULT_URI = "urn:machinist"


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
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop = threading.Event()

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        with contextlib.suppress(asyncio.CancelledError):  # clean shutdown
            asyncio.run(self._serve(ready))

    def shutdown(self) -> None:
        self._stop.set()

    # -----------------------------------------------------------------

    async def _serve(self, ready: threading.Event | None) -> None:
        from asyncua import Server  # type: ignore[import-untyped]  # noqa: PLC0415  (optional dep)

        self._loop = asyncio.get_running_loop()
        server = Server()
        await server.init()
        server.set_endpoint(self._endpoint)
        server.set_server_name(f"machinist:{self._device_name}")
        idx = await server.register_namespace(self._uri)
        obj = await server.nodes.objects.add_object(idx, self._device_name)
        variables = {}
        for name, read in self._readers.items():
            variables[name] = await obj.add_variable(idx, name, _coerce(read()))
        async with server:
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
