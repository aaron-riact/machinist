"""ik-geo kinematics back-end.

ik-geo solves closed-form IK for 6-DOF arms whose geometry matches a
specific type (spherical wrist, intersecting axes, …). Where it
applies, it's dramatically faster and more deterministic than
iterative solvers.

We accept a ``robot_type`` in ``model.extras`` to pick the ik-geo
sub-solver (e.g. ``"ur5"``, ``"kuka_r800"``). Forward kinematics falls
back to the DH back-end so the two remain consistent.
"""

from __future__ import annotations

from dataclasses import dataclass

from .api import Joints, Kinematics, Pose, RobotModel


@dataclass(slots=True)
class IKGeoKinematics(Kinematics):
    joint_count: int

    def __init__(self, model: RobotModel) -> None:
        try:
            import ik_geo  # type: ignore[import-untyped]
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "ik-geo back-end requires ik_geo: `uv pip install ik-geo`"
            ) from exc

        robot_type = model.robot_type or "general"
        self._ik_geo = ik_geo
        self._robot = ik_geo.Robot(robot_type)
        if model.dh is None:
            raise ValueError("IKGeoKinematics currently requires DH params for FK")
        # Re-use the DH backend for FK so the two solvers stay in sync.
        from .dh_backend import DHKinematics
        self._fk = DHKinematics(model)
        self.joint_count = model.joint_count

    def forward(self, joints: Joints) -> Pose:
        return self._fk.forward(joints)

    def inverse(self, pose: Pose, *, seed: Joints) -> Joints:
        # ik_geo expects a 3x3 rotation matrix + translation; we
        # reuse helpers from the DH module to convert RPY↔matrix.
        import numpy as np
        from .dh_backend import _pose_to_mat
        T = _pose_to_mat(pose)
        R = T[:3, :3]
        t = T[:3, 3]
        solutions = self._robot.ik(R, t)
        if not solutions:
            return seed
        # Return the solution closest to the seed.
        best = min(solutions, key=lambda q: np.linalg.norm(np.array(q) - np.array(seed)))
        return tuple(best)
