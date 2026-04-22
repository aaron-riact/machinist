"""Tests for the kinematics abstraction and the DH back-end."""

from __future__ import annotations

import math

import pytest

np = pytest.importorskip("numpy")

from machinist.kinematics.api import DHParams, RobotModel, build_kinematics, get_backend


def _ur5_dh() -> dict:
    # UR5 DH parameters (meters, radians).
    return {
        "a": [0, -0.425, -0.3922, 0, 0, 0],
        "d": [0.089159, 0, 0, 0.10915, 0.09465, 0.0823],
        "alpha": [math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0],
    }


def test_noop_backend_available_without_numpy_model() -> None:
    kin = build_kinematics({"backend": "noop", "joint_count": 4})
    assert kin.joint_count == 4
    assert kin.forward((0.1, 0.2, 0.3, 0.4)) == (0.1, 0.2, 0.3, 0.4, 0.0, 0.0)
    assert kin.inverse((0, 0, 0, 0, 0, 0), seed=(1.0, 2.0, 3.0, 4.0)) == (1.0, 2.0, 3.0, 4.0)


def test_dh_backend_fk_is_deterministic() -> None:
    kin = build_kinematics({"backend": "dh", "dh_params": _ur5_dh(), "joint_count": 6})
    home = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    p1 = kin.forward(home)
    p2 = kin.forward(home)
    assert p1 == p2


def test_dh_backend_ik_round_trip() -> None:
    kin = build_kinematics({"backend": "dh", "dh_params": _ur5_dh(), "joint_count": 6})
    target_joints = (0.1, -0.5, 0.3, -0.2, 0.7, 0.4)
    pose = kin.forward(target_joints)
    recovered = kin.inverse(pose, seed=target_joints)
    # Seeded close to the answer, the solver should converge back.
    recovered_pose = kin.forward(recovered)
    assert np.allclose(np.array(pose[:3]), np.array(recovered_pose[:3]), atol=1e-3)


def test_unknown_backend_raises() -> None:
    with pytest.raises(KeyError):
        get_backend("nonsense", RobotModel(joint_count=6))


def test_dh_params_validates_length() -> None:
    with pytest.raises(ValueError):
        DHParams(a=(1.0, 2.0), d=(1.0,), alpha=(0.0, 0.0))
