from __future__ import annotations

from machinist.devices.robots.arm import RobotArm
from machinist.kinematics.units import Radians


def test_forward_gives_the_pose_at_other_joints_without_moving() -> None:
    arm = RobotArm()
    joints = tuple(Radians(j) for j in (0.1, 0.2, 0.3, 0.4, 0.5, 0.6))
    assert arm.forward(joints) == joints  # no kinematics: the pose is the joints
    assert arm.state.view.joints == (0.0,) * 6
