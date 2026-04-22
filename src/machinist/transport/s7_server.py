"""Minimal Siemens S7 store and back-end-pluggable server.

The S7 protocol is complex (ISO-on-TCP / COTP / S7 PDU negotiation)
and the complete spec is non-public. Rather than implement it from
scratch, we keep a tiny wire-agnostic store and select a *server
back-end* at runtime:

* ``stub``  (default) — accepts and parks TCP connections so
  integration tests can verify reachability. Use this when no S7
  client is involved.
* ``snap7`` — wraps ``python-snap7``'s ``snap7.server.Server``,
  which speaks real S7 and shares memory with our :class:`S7Store`.
  Requires the native libsnap7 library on the host.

The device code only ever talks to the :class:`S7Store`. Swapping
back-ends is a one-line change in the emulator config.
"""

from __future__ import annotations

import socket
import threading
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Protocol

BitListener = Callable[[bool], None]


@dataclass(slots=True)
class S7Store:
    """Thread-safe sparse storage for S7 data blocks.

    Each data block is a bytearray that grows on demand. Writers can
    flip individual bits; subscribers see the post-write state.
    """

    _blocks: dict[int, bytearray] = field(default_factory=dict)
    _bit_subs: dict[tuple[int, int, int], list[BitListener]] = field(default_factory=dict)
    _lock: threading.Lock = field(default_factory=threading.Lock)

    def _block(self, db: int, *, length: int) -> bytearray:
        block = self._blocks.get(db)
        if block is None:
            block = bytearray(length)
            self._blocks[db] = block
        elif len(block) < length:
            block.extend(b"\x00" * (length - len(block)))
        return block

    def read_byte(self, db: int, byte: int) -> int:
        with self._lock:
            return self._block(db, length=byte + 1)[byte]

    def write_byte(self, db: int, byte: int, value: int) -> None:
        with self._lock:
            self._block(db, length=byte + 1)[byte] = value & 0xFF

    def read_bit(self, db: int, byte: int, bit: int) -> bool:
        return bool(self.read_byte(db, byte) & (1 << bit))

    def write_bit(self, db: int, byte: int, bit: int, value: bool) -> None:
        with self._lock:
            block = self._block(db, length=byte + 1)
            mask = 1 << bit
            current = bool(block[byte] & mask)
            if current == value:
                return
            block[byte] = (block[byte] | mask) if value else (block[byte] & ~mask)
            listeners = list(self._bit_subs.get((db, byte, bit), ()))
        for listener in listeners:
            listener(value)

    def subscribe_bit(self, db: int, byte: int, bit: int, listener: BitListener) -> None:
        with self._lock:
            self._bit_subs.setdefault((db, byte, bit), []).append(listener)


# --- back-end protocol -------------------------------------------------


class S7Backend(Protocol):
    def serve_forever(self, ready: threading.Event | None = None) -> None: ...
    def shutdown(self) -> None: ...


BackendFactory = Callable[[str, int, S7Store], S7Backend]
_BACKENDS: dict[str, BackendFactory] = {}


def register_backend(name: str, factory: BackendFactory) -> None:
    _BACKENDS[name] = factory


# --- stub back-end (default, always available) -------------------------


class _StubBackend:
    """Accept-and-park listener for reachability tests."""

    def __init__(self, host: str, port: int) -> None:
        self._host = host
        self._port = port
        self._sock: socket.socket | None = None
        self._stop = threading.Event()

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind((self._host, self._port))
        self._sock.listen(8)
        self._sock.settimeout(0.25)
        if ready is not None:
            ready.set()
        try:
            while not self._stop.is_set():
                try:
                    client, _ = self._sock.accept()
                except TimeoutError:
                    continue
                except OSError:
                    return
                client.settimeout(0.25)
                threading.Thread(target=self._park, args=(client,), daemon=True).start()
        finally:
            if self._sock is not None:
                self._sock.close()

    def shutdown(self) -> None:
        self._stop.set()
        if self._sock is not None:
            try:
                self._sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def _park(self, client: socket.socket) -> None:
        try:
            while not self._stop.is_set():
                try:
                    if not client.recv(1):
                        return
                except TimeoutError:
                    continue
                except OSError:
                    return
        finally:
            client.close()


register_backend("stub", lambda h, p, _store: _StubBackend(h, p))


# --- snap7 back-end (lazy) ---------------------------------------------


def _snap7_factory(host: str, port: int, store: S7Store) -> S7Backend:
    try:
        import snap7  # type: ignore[import-untyped]
        from snap7.type import Area  # type: ignore[import-untyped]
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "snap7 S7 back-end requires python-snap7 and libsnap7: "
            "`uv pip install python-snap7`"
        ) from exc

    class _Snap7Backend:
        def __init__(self) -> None:
            self._srv = snap7.server.Server()
            # Pre-register any already-populated DBs; hot-growing DBs
            # are registered on demand via the store's lock.
            for db, block in store._blocks.items():
                self._srv.register_area(Area.DB, db, block)

        def serve_forever(self, ready: threading.Event | None = None) -> None:
            self._srv.start_to(host, tcp_port=port)
            if ready is not None:
                ready.set()
            # snap7.Server.start_to spawns its own worker thread;
            # block until shutdown.
            while True:
                import time
                time.sleep(0.1)

        def shutdown(self) -> None:
            self._srv.stop()

    return _Snap7Backend()


register_backend("snap7", _snap7_factory)


# --- public façade -----------------------------------------------------


class S7Server:
    """Pluggable S7 server. Back-end picked via the ``backend`` arg."""

    def __init__(
        self, *, host: str, port: int, store: S7Store, backend: str = "stub",
    ) -> None:
        try:
            factory = _BACKENDS[backend]
        except KeyError as exc:
            raise KeyError(
                f"Unknown S7 back-end {backend!r}. Known: {sorted(_BACKENDS)}"
            ) from exc
        self.store = store
        self._impl = factory(host, port, store)

    def serve_forever(self, ready: threading.Event | None = None) -> None:
        self._impl.serve_forever(ready)

    def shutdown(self) -> None:
        self._impl.shutdown()

