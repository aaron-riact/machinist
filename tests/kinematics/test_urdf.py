"""Tests for URDF-native kinematics backend."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

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


def test_urdf_backend_falls_back_when_pinocchio_not_available() -> None:
    kin = build_kinematics(KinematicsOptions(urdf_path=EXAMPLES / "cr5.urdf"))
    assert type(kin).__name__ == "UrdfKinematics"


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


_TWO_LINK = """<robot name="two">
  <link name="base"/><link name="a"/><link name="b"/><link name="b_up"/><link name="tip"/>
  <joint name="j1" type="revolute"><parent link="base"/><child link="a"/>
    <origin xyz="0 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="j2" type="revolute"><parent link="a"/><child link="b"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/></joint>
  <joint name="riser" type="fixed"><parent link="b"/><child link="b_up"/>
    <origin xyz="0 0 0.5" rpy="0 0 0"/></joint>
  <joint name="flange" type="fixed"><parent link="b_up"/><child link="tip"/>
    <origin xyz="0.25 0 0" rpy="0 1.5707963 0"/></joint>
</robot>
"""


def test_urdf_fixed_joints_after_the_last_moving_one_form_the_tip(tmp_path: Path) -> None:
    urdf = tmp_path / "two.urdf"
    urdf.write_text(_TWO_LINK)
    kin = build_kinematics(KinematicsOptions(backend="urdf", urdf_path=urdf))
    x, y, z, rx, ry, rz = kin.forward((0.0, 0.0))
    assert (x, y, z) == pytest.approx((1.25, 0.0, 0.5))
    assert (rx, ry, rz) == pytest.approx((0.0, np.pi / 2, 0.0), abs=1e-6)


def test_urdf_a_fixed_joint_between_moving_ones_is_kept(tmp_path: Path) -> None:
    """With j2 fixed and the riser moving, j2's offset now lies between two moving joints."""
    urdf = tmp_path / "mid.urdf"
    urdf.write_text(
        _TWO_LINK.replace('name="j2" type="revolute"', 'name="j2" type="fixed"')
        .replace('name="riser" type="fixed"', 'name="riser" type="revolute"')
    )
    kin = build_kinematics(KinematicsOptions(backend="urdf", urdf_path=urdf))
    x, y, z, *_ = kin.forward((0.0, 0.0))
    assert (x, y, z) == pytest.approx((1.25, 0.0, 0.5))
