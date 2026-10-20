"""URDF-native kinematics back-end.

Parses a serial-chain URDF and evaluates FK by composing joint
transforms directly — no DH conversion needed.  IK uses the same
damped least-squares Jacobian iteration as the DH back-end.

The chain runs from the URDF's root link to a tip link: the one the
caller names, or the URDF's only end link. Only revolute (and continuous)
joints move; a fixed joint on the way is folded into the transform of the
moving joint after it, and fixed joints after the last moving one are the
tip's offset (a flange or tool).
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Any
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from .api import Joints, Kinematics, Pose, RobotModel
from .units import Meters, Radians
from .dh_backend import _mat_to_pose, pose_to_mat, _se3_error


@dataclass(slots=True)
class UrdfKinematics(Kinematics):
    """Forward/inverse kinematics for a serial-chain URDF."""

    joint_count: int
    _fixed: list[NDArray[np.float64]]
    _axes: list[NDArray[np.float64]]
    _tip: NDArray[np.float64]

    def __init__(self, model: RobotModel) -> None:
        if model.urdf_path is None:
            raise ValueError("UrdfKinematics requires model.urdf_path to be set")
        joints, self._tip = _walk_chain(model.urdf_path, tip_link=model.tip_link)
        self.joint_count = len(joints)
        self._fixed = [j["fixed"] for j in joints]
        self._axes = [j["axis"] for j in joints]

    def forward(self, joints: Joints) -> Pose:
        if len(joints) != self.joint_count:
            raise ValueError(f"expected {self.joint_count} joints, got {len(joints)}")
        return _mat_to_pose(self._fk_matrix(np.array(joints, dtype=float)))

    def inverse(
        self, pose: Pose, *, seed: Joints,
        max_iter: int = 200, tolerance: float = 1e-4, damping: float = 0.05,
    ) -> Joints:
        target = pose_to_mat(pose)
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
        return tuple(Radians(v) for v in q.tolist())

    def _fk_matrix(self, q: NDArray[np.float64]) -> NDArray[np.float64]:
        T = np.eye(4)
        for fixed, ax, qi in zip(self._fixed, self._axes, q, strict=False):
            T = T @ fixed @ _rot(ax, qi)
        return T @ self._tip

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

_MOVING_TYPES = frozenset({"revolute", "continuous"})
_FIXED_TYPE = "fixed"


def _parse_chain(urdf_path: str | Path, *, tip_link: str | None = None) -> list[dict]:
    """The moving joints from the root to the tip link, in order."""
    return _walk_chain(urdf_path, tip_link=tip_link)[0]


def _walk_chain(
    urdf_path: str | Path, *, tip_link: str | None = None
) -> tuple[list[dict], NDArray[np.float64]]:
    """The moving joints from the root to *tip_link*, and the offset after the last of them.

    A link has one parent joint, so the path from the tip back to the root is
    unique. Each moving joint's ``fixed`` transform includes the fixed joints
    before it on the path.
    """
    root = ET.parse(urdf_path).getroot()
    joints = [_parse_joint(elem) for elem in root.findall("joint")]
    links = [str(elem.get("name")) for elem in root.findall("link")]
    tip = tip_link if tip_link is not None else _end_link(urdf_path, links, joints)
    if tip not in links:
        raise ValueError(f"{urdf_path} has no link {tip!r}")

    parent_joint = {j["child"]: j for j in joints}
    path: list[dict] = []
    link = tip
    while link in parent_joint:
        path.append(parent_joint[link])
        link = parent_joint[link]["parent"]

    for joint in path:
        if joint["type"] not in _MOVING_TYPES | {_FIXED_TYPE} or joint["mimic"]:
            kind = "mimic" if joint["mimic"] else joint["type"]
            raise ValueError(
                f"{urdf_path}: joint {joint['name']!r} on the way to {tip!r} is {kind}, "
                "which this backend cannot model"
            )

    chain: list[dict] = []
    pending = np.eye(4)
    for joint in reversed(path):
        if joint["type"] in _MOVING_TYPES:
            chain.append({**joint, "fixed": pending @ joint["fixed"]})
            pending = np.eye(4)
        else:
            pending = pending @ joint["fixed"]
    return chain, pending


def _end_link(urdf_path: str | Path, links: list[str], joints: list[dict[str, Any]]) -> str:
    """The URDF's only end link: one that a joint leads to and none leaves."""
    parents = {j["parent"] for j in joints}
    children = {j["child"] for j in joints}
    ends = [link for link in links if link in children and link not in parents]
    if len(ends) != 1:
        raise ValueError(
            f"{urdf_path} ends in {len(ends)} links ({', '.join(ends)}); "
            "name the tool's link as tip_link"
        )
    return ends[0]


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
        "type": elem.get("type", "revolute"),
        "mimic": elem.find("mimic") is not None,
        "parent": elem.find("parent").get("link"),
        "child": elem.find("child").get("link"),
        "fixed": fixed,
        "axis": ax,
    }


# --- URDF → DH conversion (analytical) ------------------------------------


def urdf_to_dh(
    urdf_path: str, *, joint_count: int | None = None, tip_link: str | None = None
) -> DHParams:
    r"""Extract modified-Denavit–Hartenberg parameters from a serial-chain URDF.

    The extraction follows the common-normal convention: the *z*\ :sub:`i`
    axis is the joint axis, and *x*\ :sub:`i` is the common normal between
    *z*\ :sub:`i` and *z*\ :sub:`i+1`.

    Parameters
    ----------
    urdf_path:
        Path to the URDF file.
    joint_count:
        Number of moving joints to extract (default: all revolute/continuous
        joints from the root to the tip link).
    tip_link:
        The link the chain ends at (default: the URDF's only end link).

    Returns
    -------
    DHParams
        Named tuple with ``(a, d, alpha, theta_offset)``, each a tuple of
        *n* floats.

    """
    from pathlib import Path

    from .api import DHParams  # noqa: F811
    from .dh_backend import DHKinematics  # noqa: F811
    from .dh_backend import _dh_matrix, _mat_to_pose, pose_to_mat  # noqa: F401, F811

    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # collect homogeneous transforms for each joint frame in the chain
    joints = _parse_chain(urdf_path, tip_link=tip_link)
    if joint_count is not None:
        joints = joints[:joint_count]
    n = len(joints)

    # series of T matrices: T_i_prev->i (from origin of joint i to joint i+1)
    T_chain: list[NDArray[np.float64]] = []
    T_total = np.eye(4)
    for j in joints:
        T_total = T_total @ j["fixed"]
        T_chain.append(T_total.copy())

    a = np.zeros(n)
    d = np.zeros(n)
    alpha = np.zeros(n)
    theta_offset = np.zeros(n)

    # We compute the modified-DH parameters by aligning frames.
    # For joint n (last), we need the axis of the "next" frame.
    # We pick an arbitrary v_n that is not parallel to z_n.
    for i in range(n):
        # z axes of current and next frames
        z_i = T_chain[i][:3, 2]
        if i + 1 < n:
            z_next = T_chain[i + 1][:3, 2]
            o_next = T_chain[i + 1][:3, 3]
        else:
            # For the last joint, project a virtual next frame by extending
            # a fixed distance along the wrist direction.
            z_next = z_i
            o_next = T_chain[i][:3, 3] + 0.5 * z_i

        o_i = T_chain[i][:3, 3]

        # common normal (x_i axis)
        common = np.cross(z_i, z_next)
        norm = np.linalg.norm(common)
        if norm < 1e-12:
            # parallel axes — pick x_i perpendicular to z_i
            if abs(z_i[0]) < 0.9:
                common = np.cross(z_i, np.array([1.0, 0.0, 0.0]))
            else:
                common = np.cross(z_i, np.array([0.0, 1.0, 0.0]))
            common = common / np.linalg.norm(common)
        else:
            common = common / norm

        # find point where common normal pierces z_i and z_next
        # (closest-point between two skew lines)
        w = o_next - o_i
        z_i_z_next = np.dot(z_i, z_next)
        denom = 1.0 - z_i_z_next ** 2
        if abs(denom) < 1e-12:
            t_i = 0.0
        else:
            t_i = (np.dot(w, z_i) - z_i_z_next * np.dot(w, z_next)) / denom
        p_i = o_i + t_i * z_i  # foot on z_i

        # a_i = signed distance along common normal (always >= 0 if we use norm)
        a_i_vec = o_next - p_i
        a_i = np.dot(a_i_vec, common)
        d_i = np.dot(o_i - (o_next - a_i_vec), z_i)  # actually this reads weird
        # simpler: d_i is distance from x_i to x_{i-1} along z_{i-1}
        # In modified DH: d_i = translation along z_{i-1}
        # We'll use the standard definition: d_i = (o_i - o_{i-1}) · z_{i-1}
        # But for the first joint, there's no previous joint.
        # Let's use the known correct formula:
        if i == 0:
            d_i = 0.0
        else:
            d_i = np.dot(o_i - o_prev, z_prev)
        a[i] = abs(a_i)  # a_i is always positive by convention
        d[i] = d_i

        # alpha_i = angle from z_i to z_next about x_i
        sin_alpha = np.dot(np.cross(z_i, z_next), common)
        cos_alpha = np.dot(z_i, z_next)
        alpha[i] = math.atan2(sin_alpha, cos_alpha)

        # theta_offset = angle from x_{i-1} to x_i about z_i
        if i == 0:
            # no previous x axis; pick arbitrary reference
            x_prev = np.array([1.0, 0.0, 0.0])
            # project x_prev onto plane perpendicular to z_i
            x_prev_proj = x_prev - np.dot(x_prev, z_i) * z_i
            if np.linalg.norm(x_prev_proj) < 1e-12:
                x_prev_proj = np.array([0.0, 1.0, 0.0]) - np.dot(np.array([0.0, 1.0, 0.0]), z_i) * z_i
            x_prev_proj = x_prev_proj / np.linalg.norm(x_prev_proj)
        else:
            x_prev = common_prev
            # project previous x_i onto plane perpendicular to z_i
            x_prev_proj = x_prev - np.dot(x_prev, z_i) * z_i
            if np.linalg.norm(x_prev_proj) < 1e-12:
                x_prev_proj = np.array([1.0, 0.0, 0.0]) - np.dot(np.array([1.0, 0.0, 0.0]), z_i) * z_i
            x_prev_proj = x_prev_proj / np.linalg.norm(x_prev_proj)

        # project current x_i onto perpendicular to z_i
        x_i_proj = common - np.dot(common, z_i) * z_i
        if np.linalg.norm(x_i_proj) < 1e-12:
            x_i_proj = np.array([1.0, 0.0, 0.0]) - np.dot(np.array([1.0, 0.0, 0.0]), z_i) * z_i
        x_i_proj = x_i_proj / np.linalg.norm(x_i_proj)

        sin_theta = np.dot(np.cross(x_prev_proj, x_i_proj), z_i)
        cos_theta = np.dot(x_prev_proj, x_i_proj)
        theta_offset[i] = math.atan2(sin_theta, cos_theta)

        z_prev = z_i
        o_prev = o_i
        common_prev = common

    return DHParams(
        a=tuple(a.tolist()),
        d=tuple(d.tolist()),
        alpha=tuple(alpha.tolist()),
        theta_offset=tuple(theta_offset.tolist()),
    )


# --- helpers ---------------------------------------------------------------

from pathlib import Path

from .api import DHParams  # noqa: F811 (re-import for urdf_to_dh above)
from .dh_backend import DHKinematics  # noqa: F811
from .dh_backend import _dh_matrix, _mat_to_pose, pose_to_mat, _se3_error  # noqa: F401, F811


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
