"""Pure-NumPy DH kinematics back-end.

Forward kinematics uses the standard DH transform per joint and
composes the chain. Inverse kinematics uses damped-least-squares
Jacobian iteration seeded with the caller's current joints — fast and
dependency-free, adequate for emulator 'tell me where the TCP is' use
cases.

Mathematical conventions:
* Modified (proximal) DH parameters: ``a_i, d_i, alpha_i``.
* Joint value ``q_i`` is added to ``theta_offset_i`` to get the actual
  rotation about Z for the i-th joint.
* Pose format: ``(x, y, z, rx, ry, rz)`` with RPY in radians.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from .api import DHParams, Joints, Kinematics, Pose, RobotModel
from .units import Meters, Radians


@dataclass(slots=True)
class DHKinematics(Kinematics):
    """Numerical FK/IK for a DH-described serial chain."""

    joint_count: int
    dh: DHParams

    def __init__(self, model: RobotModel) -> None:
        if model.dh is None:
            raise ValueError("DHKinematics requires model.dh to be set")
        self.joint_count = model.joint_count
        self.dh = model.dh

    # ----- forward ---------------------------------------------------

    def forward(self, joints: Joints) -> Pose:
        T = self._fk_matrix(joints)
        return _mat_to_pose(T)

    def _fk_matrix(self, joints: Joints) -> NDArray[np.float64]:
        if len(joints) != self.joint_count:
            raise ValueError(f"expected {self.joint_count} joints, got {len(joints)}")
        T = np.eye(4)
        offsets = self.dh.theta_offset or (0.0,) * self.joint_count
        for a, d, alpha, theta_off, q in zip(
            self.dh.a, self.dh.d, self.dh.alpha, offsets, joints, strict=False
        ):
            T = T @ _dh_matrix(a, d, alpha, q + theta_off)
        return T

    def jacobian(self, joints: Joints) -> NDArray[np.float64]:
        q = np.array(joints, dtype=float)
        current = self._fk_matrix(joints)
        return self._numerical_jacobian(q, current)

    def ik_step(self, target: NDArray[np.float64], joints: Joints, *, damping: float = 0.05) -> Joints:
        """Single damped-least-squares IK step — fast, approximate, no iteration."""
        q = np.array(joints, dtype=float)
        current = self._fk_matrix(joints)
        err = _se3_error(target, current)
        J = self._numerical_jacobian(q, current)
        JJt = J @ J.T + damping ** 2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJt, err)
        return tuple(Radians(v) for v in (q + dq).tolist())

    # ----- inverse (damped least-squares Jacobian) --------------------

    def inverse(
        self, pose: Pose, *, seed: Joints,
        max_iter: int = 200, tolerance: float = 1e-4, damping: float = 0.05,
        max_step: float = 0.4,
    ) -> Joints:
        target = pose_to_mat(pose)
        q = np.array(seed, dtype=float)
        if q.size != self.joint_count:
            raise ValueError(f"seed length {q.size} != joint_count {self.joint_count}")

        # Keep the closest iterate seen, not merely the last one: near a
        # singular / unreachable target the damped-least-squares step can wander
        # *away* after getting close, and returning the final (worse) iterate is
        # what produced multi-turn "solutions" in the emulator.
        best_q = q.copy()
        best_norm = math.inf
        for _ in range(max_iter):
            current = self._fk_matrix(tuple(q.tolist()))
            err = _se3_error(target, current)
            norm = float(np.linalg.norm(err))
            if norm < best_norm:
                best_norm = norm
                best_q = q.copy()
            if norm < tolerance:
                break
            J = self._numerical_jacobian(q, current)
            # Damped least squares
            JJt = J @ J.T + (damping ** 2) * np.eye(6)
            dq = J.T @ np.linalg.solve(JJt, err)
            # Cap the step so a near-singular Jacobian can't fling the joints
            # through several revolutions in a single iteration.
            step = float(np.linalg.norm(dq))
            if step > max_step:
                dq *= max_step / step
            q = q + dq

        # Express the solution as the joint angles nearest the seed (each is
        # equivalent modulo 2π, so the pose is identical) so a result never
        # winds up multiple turns from the current pose — which would make
        # joint-space interpolation sweep wildly across the workspace.
        seed_arr = np.array(seed, dtype=float)
        result = seed_arr + _wrap_to_pi(best_q - seed_arr)
        return tuple(Radians(v) for v in result.tolist())

    def velocity_step(
        self, joints: Joints, twist: NDArray[np.float64],
        *, damping: float = 0.01, max_iter: int = 4,
    ) -> Joints:
        """FK-verified velocity jog via SVD-damped pseudoinverse.

        Each iteration computes a fresh Jacobian at the current joint
        position, takes an SVD-DLS step, then measures the actual FK
        displacement to determine the residual.  2-3 iterations converge
        even for 40mm steps on 6-DOF arms.
        """
        q = np.array(joints, dtype=float)
        start_T = self._fk_matrix(joints)
        remaining = np.asarray(twist, dtype=float).copy()
        for _ in range(max_iter):
            J = self._numerical_jacobian(q, self._fk_matrix(tuple(q.tolist())))
            U, S, Vt = np.linalg.svd(J, full_matrices=False)
            S_damped = S / (S * S + damping * damping)
            J_pinv = Vt.T @ np.diag(S_damped) @ U.T
            dq = J_pinv @ remaining
            q = q + dq
            actual = _se3_error(self._fk_matrix(tuple(q.tolist())), start_T)
            remaining = twist - actual
            if np.linalg.norm(remaining) < 1e-8:
                break
        if np.linalg.norm(remaining) > max(1e-3, 0.05 * np.linalg.norm(twist)):
            return joints
        return tuple(Radians(v) for v in q.tolist())

    def _numerical_jacobian(
        self, q: NDArray[np.float64], current: NDArray[np.float64],
        *, eps: float = 1e-6,
    ) -> NDArray[np.float64]:
        J = np.zeros((6, self.joint_count))
        for i in range(self.joint_count):
            q_plus = q.copy(); q_plus[i] += eps
            perturbed = self._fk_matrix(tuple(q_plus.tolist()))
            J[:, i] = _se3_error(perturbed, current) / eps
        return J


# --- math helpers -----------------------------------------------------


def _wrap_to_pi(x: NDArray[np.float64]) -> NDArray[np.float64]:
    """Wrap each angle (radians) to the equivalent value in ``(-π, π]``."""
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _dh_matrix(a: float, d: float, alpha: float, theta: float) -> NDArray[np.float64]:
    """Modified-DH transform: Rot_x(α) · Trans_x(a) · Rot_z(θ) · Trans_z(d)."""
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    return np.array([
        [ct,      -st,      0.0,  a],
        [st * ca,  ct * ca, -sa, -sa * d],
        [st * sa,  ct * sa,  ca,  ca * d],
        [0.0,      0.0,      0.0, 1.0],
    ], dtype=float)


def _mat_to_pose(T: NDArray[np.float64]) -> Pose:
    x, y, z = T[0, 3], T[1, 3], T[2, 3]
    # ZYX RPY: rz = atan2(r21, r11), ry = atan2(-r31, sqrt(r32²+r33²)), rx = atan2(r32, r33)
    rz = math.atan2(T[1, 0], T[0, 0])
    ry = math.atan2(-T[2, 0], math.hypot(T[2, 1], T[2, 2]))
    rx = math.atan2(T[2, 1], T[2, 2])
    return (Meters(float(x)), Meters(float(y)), Meters(float(z)),
            Radians(rx), Radians(ry), Radians(rz))


def pose_to_mat(pose: Pose) -> NDArray[np.float64]:
    x, y, z, rx, ry, rz = pose
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = np.array([
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
        [-sy,     cy * sx,                 cx * cy],
    ])
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [x, y, z]
    return T


def _se3_error(target: NDArray[np.float64], current: NDArray[np.float64]) -> NDArray[np.float64]:
    """6-vector position + rotation-vector error, current → target."""
    pos_err = target[:3, 3] - current[:3, 3]
    R_err = target[:3, :3] @ current[:3, :3].T
    # Rotation-vector (axis * angle) from rotation matrix
    cos_a = max(min((np.trace(R_err) - 1.0) / 2.0, 1.0), -1.0)
    angle = math.acos(cos_a)
    if angle < 1e-9:
        rot_err = np.zeros(3)
    else:
        rot_err = angle / (2.0 * math.sin(angle)) * np.array([
            R_err[2, 1] - R_err[1, 2],
            R_err[0, 2] - R_err[2, 0],
            R_err[1, 0] - R_err[0, 1],
        ])
    return np.concatenate([pos_err, rot_err])
