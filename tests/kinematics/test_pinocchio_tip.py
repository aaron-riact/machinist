"""Pinocchio and the URDF backend agree on where a URDF's chain ends."""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from machinist.kinematics import build_kinematics
from machinist.kinematics.api import KinematicsOptions

pytest.importorskip("pinocchio")

EXAMPLES = Path(__file__).parents[2] / "examples"


@pytest.mark.parametrize(("name", "tip"), [("yr_up50n", None), ("cr10", None), ("cr5", "Link6")])
def test_pinocchio_and_urdf_backends_compute_the_same_pose(name: str, tip: str | None) -> None:
    def kinematics(backend: str):  # type: ignore[no-untyped-def]
        return build_kinematics(KinematicsOptions(
            backend=backend, urdf_path=EXAMPLES / f"{name}.urdf", tip_link=tip,
        ))

    pin, urdf = kinematics("pinocchio"), kinematics("urdf")
    rng = np.random.default_rng(5)
    for _ in range(20):
        q = tuple(rng.uniform(-1.5, 1.5, 6).tolist())
        assert pin.forward(q) == pytest.approx(urdf.forward(q), abs=1e-9)


def test_pinocchio_needs_a_tip_for_a_urdf_that_ends_in_several_links() -> None:
    with pytest.raises(ValueError, match="name the tool's link as tip_link"):
        build_kinematics(KinematicsOptions(backend="pinocchio", urdf_path=EXAMPLES / "cr5.urdf"))


def test_pinocchio_tip_link_must_be_a_link() -> None:
    with pytest.raises(ValueError, match="has no link 'nope'"):
        build_kinematics(KinematicsOptions(
            backend="pinocchio", urdf_path=EXAMPLES / "cr5.urdf", tip_link="nope",
        ))
