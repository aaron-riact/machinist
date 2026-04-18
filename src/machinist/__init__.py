"""Machinist: a composable framework for emulating industrial machines."""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("machinist")
except PackageNotFoundError:  # pragma: no cover - editable install fallback
    __version__ = "0.0.0"

__all__ = ["__version__"]
def main() -> None:
    print("Hello from machinist!")
