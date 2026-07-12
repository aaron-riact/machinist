"""Pinocchio kinematics back-end.

Builds the model either from a URDF (preferred) or from DH parameters
via a short synthetic URDF. Uses Pinocchio's damped-least-squares
numerical IK (same algorithm tupleo uses), which handles real 6-DOF
arms robustly.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .api import Joints, Kinematics, Pose, RobotModel
from .units import Meters, Radians


@dataclass(slots=True)
class PinocchioKinematics(Kinematics):
    joint_count: int

    def __init__(self, model: RobotModel) -> None:
        try:
            import pinocchio as pin
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "pinocchio back-end requires 'pin-py': "
                "`uv pip install pin` (or `uv pip install -e .[kinematics]`)"
            ) from exc

        self._pin = pin
        if model.urdf_path is not None:
            self._model = pin.buildModelFromUrdf(str(model.urdf_path))
        elif model.dh is not None:
            self._model = _model_from_dh(pin, model)
        else:
            raise ValueError("PinocchioKinematics needs a urdf_path or DH params")
        self._data = self._model.createData()
        self._ee_frame = self._model.nframes - 1
        self.joint_count = self._model.nq

    # ----- forward ---------------------------------------------------

    def forward(self, joints: Joints) -> Pose:
        pin = self._pin
        q = np.array(joints, dtype=float)
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        oMf = self._data.oMf[self._ee_frame]
        return _se3_to_pose(oMf)

    # ----- inverse ---------------------------------------------------

    def inverse(
        self, pose: Pose, *, seed: Joints,
        max_iter: int = 1000, tolerance: float = 1e-4, step: float = 0.1,
    ) -> Joints:
        pin = self._pin
        oMdes = _pose_to_se3(pin, pose)
        q = np.array(seed, dtype=float)
        for _ in range(max_iter):
            pin.forwardKinematics(self._model, self._data, q)
            pin.updateFramePlacements(self._model, self._data)
            err = pin.log6(self._data.oMf[self._ee_frame].inverse() * oMdes).vector
            if np.linalg.norm(err) < tolerance:
                break
            J = pin.computeFrameJacobian(
                self._model, self._data, q, self._ee_frame, pin.ReferenceFrame.LOCAL,
            )
            dq = np.linalg.lstsq(J, err, rcond=None)[0]
            q = pin.integrate(self._model, q, dq * step)
        return tuple(Radians(v) for v in q.tolist())


# ----- helpers -------------------------------------------------------


def _model_from_dh(pin, model: RobotModel):  # type: ignore[no-untyped-def]
    """Compose a serial revolute chain from modified-DH parameters."""
    dh = model.dh
    assert dh is not None
    pmodel = pin.Model()
    parent = 0
    offsets = dh.theta_offset or (0.0,) * model.joint_count
    for i, (a, d, alpha, off) in enumerate(
        zip(dh.a, dh.d, dh.alpha, offsets, strict=False)
    ):
        # DH link → SE3 placement for the joint origin.
        placement = _dh_se3(pin, a, d, alpha, off)
        jid = pmodel.addJoint(
            parent, pin.JointModelRZ(), placement, f"joint_{i}",
        )
        pmodel.appendBodyToJoint(jid, pin.Inertia.Identity(), pin.SE3.Identity())
        parent = jid
    pmodel.addFrame(pin.Frame("tool0", parent, 0, pin.SE3.Identity(), pin.FrameType.OP_FRAME))
    return pmodel


def _dh_se3(pin, a: float, d: float, alpha: float, theta: float):  # type: ignore[no-untyped-def]
    ca, sa = math.cos(alpha), math.sin(alpha)
    ct, st = math.cos(theta), math.sin(theta)
    R = np.array([
        [ct,      -st,      0.0],
        [st * ca,  ct * ca, -sa],
        [st * sa,  ct * sa,  ca],
    ])
    t = np.array([a, -sa * d, ca * d])
    return pin.SE3(R, t)


def _se3_to_pose(se3) -> Pose:  # type: ignore[no-untyped-def]
    R = se3.rotation
    t = se3.translation
    rz = math.atan2(R[1, 0], R[0, 0])
    ry = math.atan2(-R[2, 0], math.hypot(R[2, 1], R[2, 2]))
    rx = math.atan2(R[2, 1], R[2, 2])
    return (Meters(float(t[0])), Meters(float(t[1])), Meters(float(t[2])),
            Radians(rx), Radians(ry), Radians(rz))


def _pose_to_se3(pin, pose: Pose):  # type: ignore[no-untyped-def]
    x, y, z, rx, ry, rz = pose
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    R = np.array([
        [cy * cz, cz * sx * sy - cx * sz, cx * cz * sy + sx * sz],
        [cy * sz, cx * cz + sx * sy * sz, -cz * sx + cx * sy * sz],
        [-sy,     cy * sx,                 cx * cy],
    ])
    return pin.SE3(R, np.array([x, y, z]))
