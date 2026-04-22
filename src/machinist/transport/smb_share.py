"""SMB share abstraction and back-end registry.

Design goal: let a machine emulator *export* a directory as an SMB
share that ``pysmb`` / ``smbprotocol`` / ``impacket`` / ``aiosmb``
clients can connect to.  **Every** real SMB server implementation in
Python is large and heavy; abstracting the choice keeps the HAAS
emulator code small and lets the user pick the back-end that's in
their environment.

Four back-ends are registered by name (imports are lazy):

* ``impacket`` — impacket.smbserver.SimpleSMBServer.  Supports SMBv1,
  required by older CNC controllers.  *Recommended for HAAS-compatible
  emulation.*
* ``pysmb``    — pysmb's nmbd + smbd (limited client support).
* ``smbprotocol`` — protocol-only; used for smoke tests.
* ``aiosmb``     — asyncio-native; experimental.

Back-ends that can't start (missing library, insufficient privileges,
…) raise :class:`RuntimeError` with a clear hint.
"""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import Protocol


class SmbShare(Protocol):
    """A running SMB share exposing a local directory."""

    def serve_forever(self, ready: threading.Event | None = None) -> None: ...
    def shutdown(self) -> None: ...


@dataclass(slots=True)
class SmbConfig:
    """Shared configuration for all SMB back-ends."""

    host: str
    port: int
    share_name: str
    root: Path
    smb1: bool = True


BackendFactory = Callable[[SmbConfig], SmbShare]
_BACKENDS: dict[str, BackendFactory] = {}


def register_backend(name: str, factory: BackendFactory) -> None:
    _BACKENDS[name] = factory


def build_share(name: str, config: SmbConfig) -> SmbShare:
    try:
        factory = _BACKENDS[name]
    except KeyError as exc:
        raise KeyError(
            f"Unknown SMB back-end {name!r}. Known: {sorted(_BACKENDS)}"
        ) from exc
    return factory(config)


# --- impacket back-end (lazy) ----------------------------------------


def _impacket_factory(config: SmbConfig) -> SmbShare:
    try:
        from impacket import smbserver  # type: ignore[import-untyped]
    except ImportError as exc:  # pragma: no cover
        raise RuntimeError(
            "impacket SMB back-end requires 'impacket': "
            "`uv pip install impacket`"
        ) from exc

    class _ImpacketShare:
        def __init__(self) -> None:
            # SimpleSMBServer binds on construction.
            self._server = smbserver.SimpleSMBServer(
                listenAddress=config.host,
                listenPort=config.port,
            )
            if config.smb1:
                self._server.setSMB2Support(False)
            self._server.addShare(
                config.share_name.upper(), str(config.root), comment="machinist",
            )
            self._server.setSMBChallenge("")  # open access

        def serve_forever(self, ready: threading.Event | None = None) -> None:
            if ready is not None:
                ready.set()
            self._server.start()

        def shutdown(self) -> None:
            self._server.stop()

    return _ImpacketShare()


register_backend("impacket", _impacket_factory)


# --- placeholders (register the names, raise on use) ------------------

for _name, _hint in [
    ("pysmb", "pysmb is client-only; use impacket for hosting."),
    ("smbprotocol", "smbprotocol is client-only; use impacket for hosting."),
    ("aiosmb", "aiosmb is client-only; use impacket for hosting."),
]:
    def _raising(_hint: str = _hint) -> BackendFactory:
        def factory(_cfg: SmbConfig) -> SmbShare:
            raise RuntimeError(_hint)
        return factory
    register_backend(_name, _raising())
