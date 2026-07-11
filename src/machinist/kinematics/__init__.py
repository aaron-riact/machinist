"""Pluggable kinematics back-ends.

We expose a single :class:`Kinematics` Protocol with three method
contracts: ``forward``, ``inverse`` and ``joint_count``. Concrete
back-ends live next to this file and are selected by name in YAML, e.g.

    options:
      kinematics: pinocchio   # or pykdl, ik_geo, noop

Heavy back-ends (pinocchio etc.) are loaded lazily so the framework
remains importable without them installed.
"""

from .api import (  # noqa: F401
    DHParams,
    Joints,
    Kinematics,
    NoOpKinematics,
    Pose,
    RobotModel,
    build_kinematics,
    get_backend,
    register_backend,
)
from .urdf_backend import UrdfKinematics  # noqa: F401
