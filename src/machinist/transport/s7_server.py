"""Minimal Siemens S7 store and server scaffolding.

Implementing the full S7 protocol (ISO-on-TCP / COTP / S7 PDU) is out of
scope for the framework's first cut; ``python-snap7`` is the reference
client and the *complete* spec is non-public. We provide:

* :class:`S7Store` — a thread-safe model of S7 data blocks with
  bit-level read/write and a publish/subscribe hook so devices can
  observe writes from the wire.
* :class:`S7Server` — a stub listener that accepts TCP connections on
  port 102 and parks them. Replacing this with a true S7 backend is a
  drop-in change because the *device* code only ever talks to the
  store. This keeps the public surface honest while leaving room for
  a richer implementation (e.g. wrapping the open-source ``snap7``
  library's ``Srv`` class).
"""

from __future__ import annotations

import socket
import threading
from collections.abc import Callable
from dataclasses import dataclass, field

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


class S7Server:
    """Stub S7 listener.

    Accepts and parks TCP connections so that integration tests can
    verify the device is reachable on its endpoint. A future commit can
    replace this with a real S7 protocol speaker without touching the
    device implementations (which talk only to :class:`S7Store`).
    """

    def __init__(self, *, host: str, port: int, store: S7Store) -> None:
        self._host = host
        self._port = port
        self.store = store
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
