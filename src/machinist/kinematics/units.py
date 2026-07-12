"""Unit type aliases for self-documenting function signatures.

Each alias is just ``float`` — zero cost at runtime.  The point is
readability: ``def movej(target: tuple[Radians, ...])`` tells you what
unit is expected without needing a comment.
"""

Radians = float
Degrees = float
Meters = float
Mm = float
