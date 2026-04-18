"""Machinist CLI.

Run with::

    machinist run scene.yaml [scene2.yaml ...] [--no-tui] [--device kind=name=host:port[:opt=val]]

The CLI orchestrates configuration loading, world building, optional
TUI launch, and graceful shutdown on Ctrl-C.
"""

from __future__ import annotations

import signal
import threading
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console

from . import __version__, devices  # noqa: F401  (devices import = registration)
from .core.config import DEFAULT_HOST, DeviceConfig, SystemConfig, load_config
from .core.registry import default_registry
from .core.world import WorldBuilder

app = typer.Typer(help="Machinist - emulate fleets of industrial machines.")
console = Console()


@app.command()
def version() -> None:
    """Print the package version and exit."""
    console.print(f"machinist {__version__}")


@app.command()
def kinds() -> None:
    """List all registered device kinds."""
    for kind in default_registry.kinds():
        console.print(f"  • [bold cyan]{kind}[/]  default port [yellow]{default_registry.default_port(kind)}[/]")


@app.command()
def run(
    configs: Annotated[list[Path], typer.Argument(help="One or more YAML config files.")],
    device: Annotated[
        list[str] | None,
        typer.Option(
            "--device",
            "-d",
            help=(
                "Inline device override of the form "
                "'kind:name[:host[:port]]'. Repeatable."
            ),
        ),
    ] = None,
    tui: Annotated[bool, typer.Option(help="Launch the Textual UI.")] = True,
) -> None:
    """Start a fleet of emulated devices from one or more YAML files."""
    config = _build_config(configs, inline=device or [])
    world = WorldBuilder().build(config)
    console.print(f"[bold green]Starting[/] {len(world.devices)} device(s):")
    for d in world.devices:
        console.print(
            f"  [cyan]{d.name:>20}[/]  {d.kind:<24} [magenta]{d.endpoint}[/]"
        )
    world.start()

    stop_signal = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_signal.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_signal.set())

    try:
        if tui:
            from .tui.app import MachinistApp
            MachinistApp(world).run()
        else:
            stop_signal.wait()
    finally:
        console.print("[yellow]Shutting down…[/]")
        world.stop()


# ---------------------------------------------------------------------


def _build_config(paths: list[Path], *, inline: list[str]) -> SystemConfig:
    base = load_config(paths) if paths else SystemConfig()
    if not inline:
        return base
    extra = tuple(_parse_device_spec(spec) for spec in inline)
    return SystemConfig(devices=(*base.devices, *extra), io_links=base.io_links)


def _parse_device_spec(spec: str) -> DeviceConfig:
    """Parse 'kind:name[:host[:port]]' into a :class:`DeviceConfig`."""
    parts = spec.split(":")
    if len(parts) < 2:
        raise typer.BadParameter(f"--device must be 'kind:name[:host[:port]]', got {spec!r}")
    kind, name, *rest = parts
    host: str | None = None
    port: int | None = None
    if rest:
        host = rest[0] or DEFAULT_HOST
    if len(rest) > 1:
        port = int(rest[1])
    return DeviceConfig(name=name, kind=kind, host=host, port=port)


if __name__ == "__main__":  # pragma: no cover
    app()
