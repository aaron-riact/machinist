from __future__ import annotations

from pathlib import Path

import pytest

from machinist.devices.robots.arm import ArmOptions, arm_from_options

EXAMPLES = Path(__file__).parents[2] / "examples"


def test_a_scene_names_the_tip_of_a_urdf_that_ends_in_several_links() -> None:
    arm = arm_from_options(ArmOptions(urdf=str(EXAMPLES / "cr5.urdf"), tip_link="gripper"))
    assert arm.state.view.pose


def test_a_urdf_that_ends_in_several_links_needs_a_tip() -> None:
    with pytest.raises(ValueError, match="name the tool's link as tip_link"):
        arm_from_options(ArmOptions(urdf=str(EXAMPLES / "cr5.urdf")))
