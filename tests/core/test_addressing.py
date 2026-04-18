from __future__ import annotations

import pytest

from machinist.core.addressing import AddressAllocator, AddressInUseError
from machinist.core.types import Endpoint


def test_first_allocation_returns_desired() -> None:
    allocator = AddressAllocator()
    endpoint = allocator.allocate(Endpoint("127.0.0.1", 5000), host_was_default=True)
    assert endpoint == Endpoint("127.0.0.1", 5000)


def test_collision_bumps_loopback_when_default() -> None:
    allocator = AddressAllocator()
    allocator.allocate(Endpoint("127.0.0.1", 5000), host_was_default=True)
    bumped = allocator.allocate(Endpoint("127.0.0.1", 5000), host_was_default=True)
    assert bumped == Endpoint("127.0.0.2", 5000)


def test_collision_with_pinned_host_raises() -> None:
    allocator = AddressAllocator()
    allocator.allocate(Endpoint("10.0.0.1", 502), host_was_default=False)
    with pytest.raises(AddressInUseError):
        allocator.allocate(Endpoint("10.0.0.1", 502), host_was_default=False)


def test_walks_past_already_used_loopback() -> None:
    allocator = AddressAllocator()
    allocator.allocate(Endpoint("127.0.0.1", 5000), host_was_default=True)
    allocator.allocate(Endpoint("127.0.0.2", 5000), host_was_default=True)
    third = allocator.allocate(Endpoint("127.0.0.1", 5000), host_was_default=True)
    assert third == Endpoint("127.0.0.3", 5000)
