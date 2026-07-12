"""PyKDL kinematics back-end — thin wrapper around orocos_kdl.

Only the interface layer lives here; the heavy math is delegated to
the ``PyKDL`` library. If the library isn't present, ``__init__``
raises :class:`ImportError` with a clear hint — the DH back-end remains
available as a numpy-only fallback.
"""

from __future__ import annotations

from dataclasses import dataclass

from .api import Joints, Kinematics, Pose, RobotModel
from .units import Meters, Radians


@dataclass(slots=True)
class PyKDLKinematics(Kinematics):
    joint_count: int

    def __init__(self, model: RobotModel) -> None:
        try:
            import PyKDL  # type: ignore[import-untyped]
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "pykdl back-end requires PyKDL: `uv pip install PyKDL`"
            ) from exc

        if model.dh is None:
            raise ValueError("PyKDLKinematics currently requires DH parameters")

        chain = PyKDL.Chain()
        for a, d, alpha in zip(model.dh.a, model.dh.d, model.dh.alpha, strict=False):
            segment = PyKDL.Segment(
                PyKDL.Joint(PyKDL.Joint.RotZ),
                PyKDL.Frame.DH(a, alpha, d, 0.0),
            )
            chain.addSegment(segment)

        self._PyKDL = PyKDL
        self._chain = chain
        self._fk = PyKDL.ChainFkSolverPos_recursive(chain)
        self._ik = PyKDL.ChainIkSolverPos_LMA(chain)
        self.joint_count = model.joint_count

    # ----- forward ---------------------------------------------------

    def forward(self, joints: Joints) -> Pose:
        PyKDL = self._PyKDL
        q = PyKDL.JntArray(self.joint_count)
        for i, v in enumerate(joints):
            q[i] = v
        f = PyKDL.Frame()
        self._fk.JntToCart(q, f)
        return _frame_to_pose(f)

    # ----- inverse ---------------------------------------------------

    def inverse(self, pose: Pose, *, seed: Joints) -> Joints:
        PyKDL = self._PyKDL
        q_init = PyKDL.JntArray(self.joint_count)
        for i, v in enumerate(seed):
            q_init[i] = v
        target = _pose_to_frame(PyKDL, pose)
        q_out = PyKDL.JntArray(self.joint_count)
        self._ik.CartToJnt(q_init, target, q_out)
        return tuple(Radians(q_out[i]) for i in range(self.joint_count))


# ----- helpers -------------------------------------------------------


def _frame_to_pose(f) -> Pose:  # type: ignore[no-untyped-def]
    rx, ry, rz = f.M.GetRPY()
    return (Meters(f.p.x()), Meters(f.p.y()), Meters(f.p.z()),
            Radians(rx), Radians(ry), Radians(rz))


def _pose_to_frame(PyKDL, pose: Pose):  # type: ignore[no-untyped-def]
    x, y, z, rx, ry, rz = pose
    return PyKDL.Frame(PyKDL.Rotation.RPY(rx, ry, rz), PyKDL.Vector(x, y, z))
