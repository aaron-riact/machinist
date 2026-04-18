"""Address & port allocation.

Devices declare a *desired* :class:`Endpoint`. The allocator hands out an
*actual* endpoint that does not collide with anything previously
allocated. When the user did not pin a host explicitly we are allowed to
walk up the loopback range (``127.0.0.1``, ``127.0.0.2`` …) before giving
up. Otherwise a collision is fatal — the user asked for a specific
interface and we will not silently reassign it.
"""

from __future__ import annotations

from collections.abc import Iterator
from dataclasses import dataclass, field
from ipaddress import IPv4Address, ip_address

from .types import Endpoint


class AddressInUseError(RuntimeError):
    """Raised when a requested non-loopback endpoint is already taken."""


@dataclass(slots=True)
class AddressAllocator:
    """Hand out unique host/port pairs."""

    _used: set[Endpoint] = field(default_factory=set)

    def allocate(self, desired: Endpoint, *, host_was_default: bool) -> Endpoint:
        """Return a free endpoint.

        ``host_was_default`` signals the caller did not pin the host, so
        we are free to bump up the loopback range to dodge a collision.
        """
        if desired not in self._used:
            self._used.add(desired)
            return desired

        if not host_was_default:
            raise AddressInUseError(f"Endpoint {desired} already allocated")

        for candidate in _loopback_walk(start=desired.host):
            endpoint = Endpoint(host=str(candidate), port=desired.port)
            if endpoint not in self._used:
                self._used.add(endpoint)
                return endpoint

        raise AddressInUseError(  # pragma: no cover - effectively unreachable
            f"No free loopback host for port {desired.port}"
        )


def _loopback_walk(*, start: str) -> Iterator[IPv4Address]:
    """Yield successive loopback IPv4 addresses starting after ``start``."""
    base = int(IPv4Address(start))
    for offset in range(1, 1 << 24):
        candidate = ip_address(base + offset)
        if not isinstance(candidate, IPv4Address) or not candidate.is_loopback:
            return
        yield candidate
