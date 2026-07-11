"""URDF-native kinematics back-end.

Parses a serial-chain URDF and evaluates FK by composing joint
transforms directly — no DH conversion needed.  IK uses the same
damped least-squares Jacobian iteration as the DH back-end.

Only revolute (and continuous) joints are used; fixed joints are folded
into the preceding link transform.  The chain is traversed in URDF order
(root → first child → …).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from .api import Joints, Kinematics, Pose, RobotModel
from .dh_backend import _mat_to_pose, _pose_to_mat, _se3_error


@dataclass(slots=True)
class UrdfKinematics(Kinematics):
    """Forward/inverse kinematics for a serial-chain URDF."""

    joint_count: int
    _fixed: list[NDArray[np.float64]]
    _axes: list[NDArray[np.float64]]

    def __init__(self, model: RobotModel) -> None:
        if model.urdf_path is None:
            raise ValueError("UrdfKinematics requires model.urdf_path to be set")
        joints = _parse_chain(model.urdf_path)
        self.joint_count = len(joints)
        self._fixed = [j["fixed"] for j in joints]
        self._axes = [j["axis"] for j in joints]

    def forward(self, joints: Joints) -> Pose:
        if len(joints) != self.joint_count:
            raise ValueError(f"expected {self.joint_count} joints, got {len(joints)}")
        T = np.eye(4)
        for fixed, ax, q in zip(self._fixed, self._axes, joints, strict=False):
            T = T @ fixed @ _rot(ax, q)
        return _mat_to_pose(T)

    def inverse(
        self, pose: Pose, *, seed: Joints,
        max_iter: int = 200, tolerance: float = 1e-4, damping: float = 0.05,
    ) -> Joints:
        target = _pose_to_mat(pose)
        q = np.array(seed, dtype=float)
        if q.size != self.joint_count:
            raise ValueError(f"seed length {q.size} != joint_count {self.joint_count}")

        for _ in range(max_iter):
            current = self._fk_matrix(q)
            err = _se3_error(target, current)
            if np.linalg.norm(err) < tolerance:
                break
            J = self._numerical_jacobian(q, current)
            JJt = J @ J.T + (damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)
            q = q + dq
        return tuple(q.tolist())

    def _fk_matrix(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        T = np.eye(4)
        for fixed, ax, qi in zip(self._fixed, self._axes, q, strict=False):
            T = T @ fixed @ _rot(ax, qi)
        return T

    def _numerical_jacobian(
        self, q: NDArray[np.float64], current: NDArray[np.float64],
        *, eps: float = 1e-6,
    ) -> NDArray[np.float64]:
        J = np.zeros((6, self.joint_count))
        for i in range(self.joint_count):
            q_plus = q.copy(); q_plus[i] += eps
            perturbed = self._fk_matrix(q_plus)
            J[:, i] = _se3_error(perturbed, current) / eps
        return J


# --- URDF parsing ---------------------------------------------------------

def _parse_chain(urdf_path: str | Path) -> list[dict]:
    """Extract revolute/continuous joints in traversal order from a URDF file."""
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    moving_types = {"revolute", "continuous"}
    children: dict[str, list] = {}
    for joint in root.findall("joint"):
        if joint.get("type", "revolute") not in moving_types:
            continue
        parent = joint.find("parent").get("link")
        info = _parse_joint(joint)
        children.setdefault(parent, []).append(info)

    moving_parents = set()
    moving_children = set()
    for joint in root.findall("joint"):
        if joint.get("type", "revolute") not in moving_types:
            continue
        moving_parents.add(joint.find("parent").get("link"))
        moving_children.add(joint.find("child").get("link"))
    base = (moving_parents - moving_children).pop() if (moving_parents - moving_children) else "base_link"

    chain: list[dict] = []
    _traverse(base, children, chain)
    return chain


def _traverse(link: str, children: dict[str, list], out: list[dict]) -> None:
    for j in children.get(link, []):
        out.append(j)
        _traverse(j["child"], children, out)


def _parse_joint(elem: ET.Element) -> dict:
    origin = elem.find("origin")
    xyz = [float(x) for x in (origin.get("xyz", "0 0 0").split())] if origin is not None else [0.0, 0.0, 0.0]
    rpy = [float(r) for r in (origin.get("rpy", "0 0 0").split())] if origin is not None else [0.0, 0.0, 0.0]
    axis_elem = elem.find("axis")
    axis = [float(a) for a in (axis_elem.get("xyz", "0 0 1").split())] if axis_elem is not None else [0.0, 0.0, 1.0]

    fixed = _compose_transform(np.array(xyz, dtype=float), np.array(rpy, dtype=float))
    ax = np.array(axis, dtype=float)
    ax = ax / np.linalg.norm(ax)

    return {
        "name": elem.get("name"),
        "child": elem.find("child").get("link"),
        "fixed": fixed,
        "axis": ax,
    }


# --- helpers ---------------------------------------------------------------


def _compose_transform(xyz: NDArray[np.float64], rpy: NDArray[np.float64]) -> NDArray[np.float64]:
    """Build a 4×4 transform from xyz translation and ZYX (RPY) rotation."""
    T = np.eye(4)
    T[:3, 3] = xyz
    cr, sr = math.cos(rpy[0]), math.sin(rpy[0])
    cp, sp = math.cos(rpy[1]), math.sin(rpy[1])
    cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
    T[:3, :3] = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr],
    ])
    return T


def _rot(axis: NDArray[np.float64], angle: float) -> NDArray[np.float64]:
    """Rotation matrix about *axis* by *angle* (Rodrigues formula)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    return np.array([
        [c + x*x*(1-c),   x*y*(1-c) - z*s, x*z*(1-c) + y*s, 0.0],
        [y*x*(1-c) + z*s, c + y*y*(1-c),   y*z*(1-c) - x*s, 0.0],
        [z*x*(1-c) - y*s, z*y*(1-c) + x*s, c + z*z*(1-c),   0.0],
        [0.0,             0.0,             0.0,              1.0],
    ], dtype=float)
