"""The cell around the devices: the parts in it, and what a mounted sensor sees.

A device such as a laser only reacts to a distance. The scene works that distance
out: it knows where the parts stand, where each sensor is mounted, and where the
device it is mounted on has moved to. A sensor's beam leaves its mount point along
the mount frame's +Z axis.

Everything here is in metres, in the cell frame: the base frame of the cell's arms.
"""

from __future__ import annotations

import math
from collections.abc import Iterable
from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

from ..kinematics.dh_backend import pose_to_mat
from ..kinematics.units import Meters, Radians
from .capabilities import SensesDistance
from .config import MountConfig, PartsConfig

Vector = NDArray[np.float64]
Transform = NDArray[np.float64]

#: Below this a beam counts as running along a surface, not through it.
_PARALLEL = 1e-12


@dataclass(frozen=True, slots=True)
class Cylinder:
    """An upright cylinder: its bottom face's centre, radius and height."""

    bottom: Vector
    radius: float
    height: float

    def hit(self, origin: Vector, direction: Vector) -> float | None:
        """How far along the unit *direction* from *origin* the beam meets the
        cylinder; 0 from inside it, None when it misses."""
        top = self.bottom[2] + self.height
        if self._contains(origin, top):
            return 0.0
        hits = []
        offset = origin[:2] - self.bottom[:2]
        a = float(direction[:2] @ direction[:2])
        if a > _PARALLEL:
            b = 2 * float(offset @ direction[:2])
            c = float(offset @ offset) - self.radius**2
            discriminant = b * b - 4 * a * c
            if discriminant >= 0:
                root = math.sqrt(discriminant)
                for t in ((-b - root) / (2 * a), (-b + root) / (2 * a)):
                    z = origin[2] + t * direction[2]
                    if t >= 0 and self.bottom[2] <= z <= top:
                        hits.append(t)
        if abs(direction[2]) > _PARALLEL:
            for z in (self.bottom[2], top):
                t = (z - origin[2]) / direction[2]
                point = origin[:2] + t * direction[:2] - self.bottom[:2]
                if t >= 0 and float(point @ point) <= self.radius**2:
                    hits.append(t)
        return min(hits, default=None)

    def _contains(self, point: Vector, top: float) -> bool:
        offset = point[:2] - self.bottom[:2]
        return (self.bottom[2] < point[2] < top
                and float(offset @ offset) < self.radius**2)


@dataclass(frozen=True, slots=True)
class Box:
    """A box standing on its bottom face: that face's centre, its half sizes along
    its own X and Y, its height, and how far it is turned about Z."""

    bottom: Vector
    half_x: float
    half_y: float
    height: float
    rz: float

    def hit(self, origin: Vector, direction: Vector) -> float | None:
        """How far along the unit *direction* from *origin* the beam meets the
        box; 0 from inside it, None when it misses."""
        turn = np.array([[math.cos(self.rz), math.sin(self.rz), 0],
                         [-math.sin(self.rz), math.cos(self.rz), 0],
                         [0, 0, 1]])
        local_origin = turn @ (origin - self.bottom)
        local_direction = turn @ direction
        low = np.array([-self.half_x, -self.half_y, 0.0])
        high = np.array([self.half_x, self.half_y, self.height])
        near, far = -math.inf, math.inf
        for axis in range(3):
            if abs(local_direction[axis]) < _PARALLEL:
                if not low[axis] <= local_origin[axis] <= high[axis]:
                    return None
                continue
            a = (low[axis] - local_origin[axis]) / local_direction[axis]
            b = (high[axis] - local_origin[axis]) / local_direction[axis]
            near, far = max(near, min(a, b)), min(far, max(a, b))
        if near > far or far < 0:
            return None
        return max(near, 0.0)


Solid = Cylinder | Box


def solids_of(parts: PartsConfig) -> list[Solid]:
    """Each part of a parts entry, as a solid in the cell."""
    solids: list[Solid] = []
    for bottom in parts.bottoms_mm():
        at = np.array(bottom) / 1000
        if parts.cylinder is not None:
            solids.append(Cylinder(at, parts.cylinder.diameter_mm / 2000,
                                   parts.cylinder.height_mm / 1000))
        elif parts.box is not None:
            x, y, z = (size / 1000 for size in parts.box.size_mm)
            solids.append(Box(at, x / 2, y / 2, z, math.radians(parts.rz_deg)))
    return solids


@dataclass(frozen=True, slots=True)
class Scene:
    """The solids in the cell, which beams meet."""

    solids: tuple[Solid, ...]

    @classmethod
    def of(cls, parts: Iterable[PartsConfig]) -> Scene:
        return cls(tuple(solid for entry in parts for solid in solids_of(entry)))

    def distance_along(self, beam: Transform) -> float | None:
        """How far the beam that leaves *beam*'s origin along its +Z axis runs before
        it meets a solid; None when it meets none."""
        origin, direction = beam[:3, 3], beam[:3, 2]
        hits = (solid.hit(origin, direction) for solid in self.solids)
        return min((hit for hit in hits if hit is not None), default=None)


@dataclass(frozen=True, slots=True)
class MountedSensor:
    """A sensor at *offset* on what carries it, seeing the scene."""

    sensor: SensesDistance
    offset: Transform
    scene: Scene

    def see_from(self, carrier: Transform) -> None:
        """What carries the sensor is at *carrier*: hand the sensor its distance."""
        self.sensor.sense_distance(self.scene.distance_along(carrier @ self.offset))


def mount_offset(mount: MountConfig) -> Transform:
    """A mount's offset as a transform."""
    x, y, z, rx, ry, rz = mount.at_mm_deg
    return pose_to_mat((Meters(x / 1000), Meters(y / 1000), Meters(z / 1000),
                        Radians(math.radians(rx)), Radians(math.radians(ry)),
                        Radians(math.radians(rz))))
