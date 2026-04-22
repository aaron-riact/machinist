"""Kinematics protocol and registry.

A :class:`Kinematics` object maps joints ↔ end-effector pose for a
particular robot geometry. The geometry comes from a :class:`RobotModel`
(DH parameters **or** a URDF path). Backends are plug-ins registered by
name; concrete ones live in sibling modules and are imported lazily so
that the :mod:`machinist` core never hard-depends on pinocchio/pykdl/etc.

Typical use from a device::

    kin = build_kinematics({
        "backend": "pinocchio",          # "noop" | "dh" | "pinocchio" | ...
        "urdf":    "models/ur5.urdf",    # either this…
        "dh_params": {                   # …or this
            "a":     [...],
            "d":     [...],
            "alpha": [...],
        },
        "joint_count": 6,
    })
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Protocol

Joints = tuple[float, ...]
Pose = tuple[float, float, float, float, float, float]  # x y z rx ry rz (RPY)


# --- model description ------------------------------------------------


@dataclass(frozen=True, slots=True)
class DHParams:
    """Modified-DH parameters for a serial chain."""

    a: tuple[float, ...]
    d: tuple[float, ...]
    alpha: tuple[float, ...]
    theta_offset: tuple[float, ...] = ()

    def __post_init__(self) -> None:
        n = len(self.a)
        if not (len(self.d) == len(self.alpha) == n):
            raise ValueError("DH param arrays must share length")


@dataclass(frozen=True, slots=True)
class RobotModel:
    """Everything a backend needs to build its internal representation."""

    joint_count: int
    dh: DHParams | None = None
    urdf_path: Path | None = None
    extras: dict[str, Any] = field(default_factory=dict)


# --- kinematics protocol ---------------------------------------------


class Kinematics(Protocol):
    """Forward/inverse kinematics for a specific :class:`RobotModel`."""

    joint_count: int

    def forward(self, joints: Joints) -> Pose: ...
    def inverse(self, pose: Pose, *, seed: Joints) -> Joints: ...


# --- registry ---------------------------------------------------------


BackendFactory = Callable[[RobotModel], Kinematics]

_BACKENDS: dict[str, BackendFactory] = {}


def register_backend(name: str, factory: BackendFactory) -> None:
    """Register a kinematics back-end under ``name``."""
    _BACKENDS[name] = factory


def get_backend(name: str, model: RobotModel) -> Kinematics:
    try:
        factory = _BACKENDS[name]
    except KeyError as exc:
        raise KeyError(
            f"Unknown kinematics back-end {name!r}. Known: {sorted(_BACKENDS)}"
        ) from exc
    return factory(model)


def build_kinematics(options: dict[str, Any]) -> Kinematics:
    """Construct a :class:`Kinematics` from a YAML-style config dict."""
    backend = options.get("backend", "noop")
    model = _model_from_options(options)
    return get_backend(backend, model)


def _model_from_options(options: dict[str, Any]) -> RobotModel:
    dh = options.get("dh_params")
    urdf = options.get("urdf")
    joint_count = int(options.get("joint_count", 6))
    return RobotModel(
        joint_count=joint_count,
        dh=_parse_dh(dh) if dh else None,
        urdf_path=Path(urdf) if urdf else None,
        extras={k: v for k, v in options.items() if k not in {"backend", "dh_params", "urdf", "joint_count"}},
    )


def _parse_dh(dh: dict[str, Any]) -> DHParams:
    return DHParams(
        a=tuple(float(x) for x in dh["a"]),
        d=tuple(float(x) for x in dh["d"]),
        alpha=tuple(float(x) for x in dh["alpha"]),
        theta_offset=tuple(float(x) for x in dh.get("theta_offset", ())),
    )


# --- trivial fallback back-end ---------------------------------------


class NoOpKinematics:
    """Identity-ish fallback for emulators that don't care about kinematics.

    Safe on systems without numpy; always available as the ``"noop"``
    back-end.
    """

    def __init__(self, model: RobotModel) -> None:
        self.joint_count = model.joint_count

    def forward(self, joints: Joints) -> Pose:
        padded = (*joints, *([0.0] * 6))[:6]
        return (padded[0], padded[1], padded[2], padded[3], padded[4], padded[5])

    def inverse(self, pose: Pose, *, seed: Joints) -> Joints:
        return seed


register_backend("noop", NoOpKinematics)


# Register the real back-ends lazily — they each import their heavy
# dependency *inside* their factory so importing this module stays
# zero-cost for deployments that don't use kinematics.
def _lazy(name: str, module: str, attr: str) -> None:
    def factory(model: RobotModel) -> Kinematics:
        import importlib
        mod = importlib.import_module(f"machinist.kinematics.{module}")
        return getattr(mod, attr)(model)
    register_backend(name, factory)


_lazy("dh", "dh_backend", "DHKinematics")
_lazy("pinocchio", "pinocchio_backend", "PinocchioKinematics")
_lazy("pykdl", "pykdl_backend", "PyKDLKinematics")
_lazy("ik-geo", "ikgeo_backend", "IKGeoKinematics")
