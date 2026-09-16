"""Typed device options: the boundary where a YAML ``options`` block becomes an object.

Every device declares an :class:`Options` subclass. The registry parses the
raw mapping into it once, when the device is created, so a factory and a
constructor only ever see typed, validated, defaulted values. Defaults live
on the fields; unknown keys are errors; nested blocks are nested models.
"""

from __future__ import annotations

from collections.abc import Mapping
from typing import Any, TypeVar

from pydantic import BaseModel, ConfigDict, ValidationError

O = TypeVar("O", bound="Options")


class Options(BaseModel):
    """Base for a device's ``options`` block. Frozen; extra keys are rejected."""

    model_config = ConfigDict(frozen=True, extra="forbid")


class OptionsError(ValueError):
    """A device's options block does not match its schema."""


def parse_options(cls: type[O], raw: Mapping[str, Any], *, kind: str) -> O:
    """Parse *raw* into *cls*, naming the device *kind* in any complaint."""
    try:
        return cls.model_validate(dict(raw))
    except ValidationError as exc:
        problems = "; ".join(
            f"{'.'.join(str(p) for p in err['loc']) or '(options)'}: {err['msg']}"
            for err in exc.errors()
        )
        raise OptionsError(f"{kind}: bad options: {problems}") from exc
