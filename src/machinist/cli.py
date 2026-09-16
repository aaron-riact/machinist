"""Machinist CLI.

Run with::

    machinist run scene.yaml [scene2.yaml ...] [--no-tui] [--device kind=name=host:port[:opt=val]]

The CLI orchestrates configuration loading, world building, optional
TUI launch, and graceful shutdown on Ctrl-C.
"""

from __future__ import annotations

import os
import signal
import sys
import threading
from pathlib import Path
from typing import Annotated

import typer
from rich.console import Console
from rich.table import Table

from . import __version__, devices  # noqa: F401  (devices import = registration)
from .core.config import DEFAULT_HOST, DeviceConfig, SystemConfig, load_config
from .core.eventlog import EventLog
from .core.registry import default_registry
from .core.world import WorldBuilder
from .projection import Projection
from .web.server import WebServer

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
def urdf_to_dh(
    urdf_path: Annotated[Path, typer.Argument(help="Path to a URDF file.")],
) -> None:
    """Convert a URDF to modified-DH parameters."""
    from .kinematics import urdf_to_dh as _convert

    dh = _convert(str(urdf_path))
    table = Table(title=f"DH Parameters — {urdf_path.name}")
    table.add_column("i", style="dim")
    table.add_column("a", justify="right")
    table.add_column("d", justify="right")
    table.add_column("alpha", justify="right")
    table.add_column("theta_offset", justify="right")
    for i in range(len(dh.a)):
        table.add_row(
            str(i),
            f"{dh.a[i]:11.6f}",
            f"{dh.d[i]:11.6f}",
            f"{dh.alpha[i]:11.6f}",
            f"{dh.theta_offset[i] if dh.theta_offset else 0:11.6f}",
        )
    console.print(table)


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
    web: Annotated[bool, typer.Option(help="Serve the live web UI.")] = False,
    log_stderr: Annotated[bool, typer.Option("--log-stderr", help="Print received commands to stderr (use --no-tui to avoid TUI interference).")] = False,
    web_host: Annotated[str, typer.Option(help="Host the web UI binds to.")] = "127.0.0.1",
    web_port: Annotated[int, typer.Option(help="Port the web UI binds to.")] = 8080,
    event_log: Annotated[
        Path | None, typer.Option("--event-log", help="Append every event to this JSON Lines file.")
    ] = None,
    check_projection: Annotated[
        bool,
        typer.Option(
            "--check-projection",
            help="Every second, compare the UI projection with the devices and report drift.",
        ),
    ] = False,
) -> None:
    """Start a fleet of emulated devices from one or more YAML files."""
    if log_stderr:
        os.environ["MACHINIST_LOG_STDERR"] = "1"
    config = _build_config(configs, inline=device or [])
    world = WorldBuilder().build(config)
    recorder = None
    if event_log is not None:
        recorder = EventLog(event_log)
        recorder.attach(world.bus)
        console.print(f"[bold green]Event log[/] {event_log}")
    checker = _ProjectionChecker(world) if check_projection else None
    console.print(f"[bold green]Starting[/] {len(world.devices)} device(s):")
    for d in world.devices:
        console.print(
            f"  [cyan]{d.name:>20}[/]  {d.kind:<24} [magenta]{d.endpoint}[/]"
        )
    world.start()

    web_server = None
    if web:
        web_server = WebServer(world, host=web_host, port=web_port)
        web_server.start()
        console.print(f"[bold green]Web UI[/] on [link={web_server.url}]{web_server.url}[/]")

    stop_signal = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop_signal.set())
    signal.signal(signal.SIGTERM, lambda *_: stop_signal.set())
    if checker is not None:
        checker.start(stop_signal)

    try:
        if tui:
            from .tui.app import MachinistApp
            MachinistApp(world).run()
        else:
            stop_signal.wait()
    finally:
        console.print("[yellow]Shutting down…[/]")
        stop_signal.set()
        if web_server is not None:
            web_server.stop()
        world.stop()
        if checker is not None:
            checker.close()
        if recorder is not None:
            recorder.close()


class _ProjectionChecker:
    """Once a second, compare a projection with the devices and print any drift."""

    def __init__(self, world) -> None:  # type: ignore[no-untyped-def]
        self._world = world
        self._projection = Projection(world)
        self._thread: threading.Thread | None = None

    def start(self, stop: threading.Event) -> None:
        def loop() -> None:
            while not stop.wait(1.0):
                for problem in self._projection.verify(self._world):
                    print(f"[projection drift] {problem}", file=sys.stderr, flush=True)

        self._thread = threading.Thread(target=loop, name="machinist-projection-check", daemon=True)
        self._thread.start()

    def close(self) -> None:
        self._projection.close()


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
