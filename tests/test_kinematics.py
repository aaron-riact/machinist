"""Tests for the kinematics abstraction and the DH back-end."""

from __future__ import annotations

import math

import pytest

np = pytest.importorskip("numpy")

from machinist.kinematics.api import DHParams, KinematicsOptions, RobotModel, build_kinematics, get_backend


def _ur5_dh() -> DHParams:
    return DHParams(
        a=(0, -0.425, -0.3922, 0, 0, 0),
        d=(0.089159, 0, 0, 0.10915, 0.09465, 0.0823),
        alpha=(math.pi / 2, 0, 0, math.pi / 2, -math.pi / 2, 0),
    )


def test_noop_backend_available_without_numpy_model() -> None:
    kin = build_kinematics(KinematicsOptions(backend="noop", joint_count=4))
    assert kin.joint_count == 4
    assert kin.forward((0.1, 0.2, 0.3, 0.4)) == (0.1, 0.2, 0.3, 0.4, 0.0, 0.0)
    assert kin.inverse((0, 0, 0, 0, 0, 0), seed=(1.0, 2.0, 3.0, 4.0)) == (1.0, 2.0, 3.0, 4.0)


def test_dh_backend_fk_is_deterministic() -> None:
    kin = build_kinematics(KinematicsOptions(backend="dh", dh=_ur5_dh(), joint_count=6))
    home = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    p1 = kin.forward(home)
    p2 = kin.forward(home)
    assert p1 == p2


def test_backend_is_inferred_from_dh_params() -> None:
    kin = build_kinematics(KinematicsOptions(dh=_ur5_dh(), joint_count=6))
    assert kin.joint_count == 6
    assert any(abs(value) > 0.01 for value in kin.forward((0.0,) * 6)[:3])


def test_dh_backend_ik_round_trip() -> None:
    kin = build_kinematics(KinematicsOptions(backend="dh", dh=_ur5_dh(), joint_count=6))
    target_joints = (0.1, -0.5, 0.3, -0.2, 0.7, 0.4)
    pose = kin.forward(target_joints)
    recovered = kin.inverse(pose, seed=target_joints)
    # Seeded close to the answer, the solver should converge back.
    recovered_pose = kin.forward(recovered)
    assert np.allclose(np.array(pose[:3]), np.array(recovered_pose[:3]), atol=1e-3)


def test_dh_backend_ik_never_winds_up_multiple_turns() -> None:
    """Regression: IK must return joints near the seed, never wound up.

    An unreachable/near-singular target used to make the damped-least-squares
    loop spiral, returning a "solution" thousands of degrees from the current
    pose (e.g. joint 3 at -3608°). Joint-space interpolation to such a target
    swept the TCP wildly across the workspace. The solver now returns the
    equivalent angles nearest the seed, so every joint stays within ±180°.
    """
    kin = build_kinematics(KinematicsOptions(backend="dh", dh=_ur5_dh(), joint_count=6))
    seed = (0.0,) * 6
    # A pose well beyond the UR5's ~0.85 m reach — deliberately unreachable.
    unreachable = (2.0, 2.0, 2.0, 0.0, 0.0, 0.0)
    sol = kin.inverse(unreachable, seed=seed)
    assert all(abs(j - s) <= math.pi + 1e-9 for j, s in zip(sol, seed)), sol


def test_dh_backend_ik_returns_shortest_equivalent_from_seed() -> None:
    """A reachable target seeded from a wound-up pose comes back near the seed."""
    kin = build_kinematics(KinematicsOptions(backend="dh", dh=_ur5_dh(), joint_count=6))
    target_joints = (0.2, -0.4, 0.6, -0.3, 0.5, -0.2)
    pose = kin.forward(target_joints)
    # Seed one joint offset by a full turn: the solver must not return a
    # multi-turn answer, and FK must still match the target pose.
    seed = (0.2 + 2 * math.pi, -0.4, 0.6, -0.3, 0.5, -0.2)
    recovered = kin.inverse(pose, seed=seed)
    assert all(abs(j - s) <= math.pi + 1e-9 for j, s in zip(recovered, seed)), recovered
    recovered_pose = kin.forward(recovered)
    assert np.allclose(np.array(pose[:3]), np.array(recovered_pose[:3]), atol=1e-3)


def _cr20a_dh() -> DHParams:
    return DHParams(
        a=(0.0, 0.0, -0.8252, -0.746, 0.0, 0.0),
        d=(0.23, 0.0, 0.0468, 0.1288, 0.1288, 0.1365),
        alpha=(0.0, math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2),
        theta_offset=(0.0, -math.pi / 2, 0.0, -math.pi / 2, 0.0, 0.0),
    )


def test_dh_backend_ik_escapes_local_minimum() -> None:
    """A reachable target that single-seed DLS gets stuck on is still solved.

    From the home seed the damped-least-squares descent lands in a local
    minimum ~60 mm short of this (reachable) CR20A pose. inverse() must retry
    from alternate seeds and converge. Taken from a real MovL that failed.
    """
    kin = build_kinematics(KinematicsOptions(backend="dh", dh=_cr20a_dh(), joint_count=6))
    target = (
        -1.1308256, 1.0210019, 0.3669737,
        math.radians(159.0254), math.radians(-0.3968), math.radians(-89.3969),
    )
    sol = kin.inverse(target, seed=(0.0,) * 6)
    fk = kin.forward(sol)
    pos_err = float(np.linalg.norm(np.array(fk[:3]) - np.array(target[:3])))
    assert pos_err < 1e-3, f"IK left {pos_err * 1000:.1f} mm of position error"
    # Deterministic despite the random restarts (fixed RNG seed).
    assert kin.inverse(target, seed=(0.0,) * 6) == sol


def test_unknown_backend_raises() -> None:
    with pytest.raises(KeyError):
        get_backend("nonsense", RobotModel(joint_count=6))


def test_dh_params_validates_length() -> None:
    with pytest.raises(ValueError):
        DHParams(a=(1.0, 2.0), d=(1.0,), alpha=(0.0, 0.0))
