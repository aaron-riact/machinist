"""Tests for URDF-native kinematics backend."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from machinist.kinematics import RobotModel, build_kinematics, urdf_to_dh
from machinist.kinematics.api import KinematicsOptions

EXAMPLES = Path(__file__).parents[2] / "examples"


def test_urdf_backend_fk_zero_config() -> None:
    for name in ("cr5", "cr10", "cr10a", "cr20", "cr20a"):
        urdf = EXAMPLES / f"{name}.urdf"
        kin = build_kinematics(KinematicsOptions(backend="urdf", urdf_path=urdf))
        p = kin.forward((0.0,) * kin.joint_count)
        assert all(not np.isnan(v) for v in p), f"NaN in FK at q=0 for {name}"
        assert any(abs(v) > 1e-9 for v in p), f"all-zero pose at q=0 for {name}"


def test_urdf_backend_fk_deterministic() -> None:
    kin = build_kinematics(KinematicsOptions(
        backend="urdf", urdf_path=EXAMPLES / "cr5.urdf",
    ))
    q = (0.1, -0.2, 0.3, -0.4, 0.5, -0.6)
    p1 = kin.forward(q)
    p2 = kin.forward(q)
    assert p1 == p2


def test_urdf_backend_ik_round_trip() -> None:
    for name in ("cr5", "cr10", "cr10a", "cr20", "cr20a"):
        urdf = EXAMPLES / f"{name}.urdf"
        kin = build_kinematics(KinematicsOptions(backend="urdf", urdf_path=urdf))
        rng = np.random.default_rng(42)
        for _ in range(20):
            q = tuple(rng.uniform(-0.5, 0.5, kin.joint_count).tolist())
            p = kin.forward(q)
            q_inv = kin.inverse(p, seed=q)
            assert all(abs(a - b) < 1e-4 for a, b in zip(q_inv, q)), (
                f"IK round-trip failed at {q}: got {q_inv} [{name}]"
            )


def test_urdf_backend_rejects_wrong_joint_count() -> None:
    kin = build_kinematics(KinematicsOptions(
        backend="urdf", urdf_path=EXAMPLES / "cr5.urdf",
    ))
    try:
        kin.forward((0.0,) * (kin.joint_count + 1))
        assert False, "expected ValueError"
    except ValueError:
        pass


def test_urdf_to_dh_returns_valid_params() -> None:
    for name in ("cr5", "cr10", "cr10a", "cr20", "cr20a"):
        dh = urdf_to_dh(str(EXAMPLES / f"{name}.urdf"))
        assert len(dh.a) == 6, f"expected 6 joints for {name}, got {len(dh.a)}"
        assert len(dh.d) == 6
        assert len(dh.alpha) == 6


def test_urdf_to_dh_with_known_dh_produces_valid_fk() -> None:
    """URDF-extracted DH produces a valid (non-NaN) FK at multiple configs."""
    from machinist.devices.robots.dobot import _CR5_DH

    dh = urdf_to_dh(str(EXAMPLES / "cr5.urdf"))
    kin_urdf = build_kinematics(KinematicsOptions(backend="dh", dh=dh, joint_count=6))
    kin_known = build_kinematics(KinematicsOptions(backend="dh", dh=_CR5_DH, joint_count=6))
    rng = np.random.default_rng(7)
    for _ in range(30):
        q = tuple(rng.uniform(-0.5, 0.5, 6).tolist())
        p_u = kin_urdf.forward(q)
        p_k = kin_known.forward(q)
        assert all(not np.isnan(v) for v in p_u), f"NaN in URDF DH FK at {q}"
        assert all(not np.isnan(v) for v in p_k), f"NaN in known DH FK at {q}"
