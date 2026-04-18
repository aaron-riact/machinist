"""Kinematics protocol + a no-op stub back-end.

The stub satisfies the ``Kinematics`` contract without any external
dependency, so unit tests never have to install pinocchio. Real
back-ends live in sibling modules (``pinocchio_backend``, ``pykdl_backend``,
``ik_geo_backend``) and are imported lazily.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

Joints = tuple[float, ...]
Pose = tuple[float, float, float, float, float, float]


class Kinematics(Protocol):
    joint_count: int

    def forward(self, joints: Joints) -> Pose: ...
    def inverse(self, pose: Pose, *, seed: Joints) -> Joints: ...


@dataclass(slots=True)
class NoOpKinematics:
    """Identity-like back-end used when no model is configured.

    ``forward`` returns the first six joint values padded/truncated to a
    pose; ``inverse`` returns the seed unchanged. This is *fine* for
    emulators that don't actually care about kinematic correctness — most
    monitoring/io use-cases never exercise IK.
    """

    joint_count: int = 6

    def forward(self, joints: Joints) -> Pose:
        padded = (*joints, *([0.0] * 6))[:6]
        return (padded[0], padded[1], padded[2], padded[3], padded[4], padded[5])

    def inverse(self, pose: Pose, *, seed: Joints) -> Joints:
        return seed


_BACKENDS: dict[str, Callable[[dict[str, object]], Kinematics]] = {
    "noop": lambda opts: NoOpKinematics(joint_count=int(opts.get("joint_count", 6))),
}


def register_backend(name: str, factory: Callable[[dict[str, object]], Kinematics]) -> None:
    _BACKENDS[name] = factory


def get_backend(name: str, options: dict[str, object] | None = None) -> Kinematics:
    try:
        factory = _BACKENDS[name]
    except KeyError as exc:
        raise KeyError(f"Unknown kinematics back-end {name!r}") from exc
    return factory(options or {})
