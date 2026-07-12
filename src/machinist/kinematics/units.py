"""NewType wrappers for dimensional unit safety.

Every value crossing between the wire protocol (mm, degrees) and the
internal physics model (m, radians) is tagged with its unit.  The type
checker then prevents passing degrees where radians are expected
without an explicit conversion call.

These are zero-cost at runtime — ``NewType`` is a ``float`` under the
hood.  There is no runtime enforcement; rely on ``mypy --strict``.
"""

from __future__ import annotations

import math
from typing import NewType

Radians = NewType("Radians", float)
Degrees = NewType("Degrees", float)
Meters = NewType("Meters", float)
Mm = NewType("Mm", float)


def deg2rad(d: Degrees) -> Radians:
    return Radians(math.radians(d))


def rad2deg(r: Radians) -> Degrees:
    return Degrees(math.degrees(r))


def mm2m(mm: Mm) -> Meters:
    return Meters(mm * 1e-3)


def m2mm(m: Meters) -> Mm:
    return Mm(m * 1e3)
