"""A small CLI built on :class:`~machinist.srci.client.SrciClient`.

Doubles as a worked example of driving the client from another project.
Exposed as the ``srci`` console script::

    srci status --host 127.0.0.1 --port 15001
    srci movej 0.1 0.2 0.3 0.0 0.0 0.0 --speed 0.5
    srci estop
"""

from __future__ import annotations

from typing import Annotated

import typer
from rich.console import Console
from rich.table import Table

from .client import SrciClient
from .codec import StatusFlag, StatusTelegram

app = typer.Typer(help="Talk to an SRCI robot over a pluggable transport.")
console = Console()

HostOpt = Annotated[str, typer.Option(help="Robot host.")]
PortOpt = Annotated[int, typer.Option(help="Robot port.")]
TransportOpt = Annotated[str, typer.Option(help="Transport: tcp | udp.")]
SpeedOpt = Annotated[float, typer.Option(help="Speed fraction 0..1.")]


def _client(host: str, port: int, transport: str) -> SrciClient:
    try:
        return SrciClient.connect(host, port, transport=transport)
    except (OSError, ValueError) as exc:
        console.print(f"[red]connect failed[/]: {exc}")
        raise typer.Exit(code=1) from exc


def _render(status: StatusTelegram) -> None:
    table = Table(show_header=False, box=None)
    flags = ", ".join(f.name for f in StatusFlag if f and f in status.flags) or "NONE"
    table.add_row("job", str(status.job_id))
    table.add_row("flags", flags)
    table.add_row("active", status.active_function.name)
    table.add_row("error", str(status.error_code))
    table.add_row("joints", "  ".join(f"{j:+.4f}" for j in status.joints))
    table.add_row("pose", "  ".join(f"{p:+.4f}" for p in status.pose))
    colour = "red" if StatusFlag.ERROR in status.flags else "green"
    console.print(f"[{colour}]●[/] SRCI status")
    console.print(table)


def _run(host: str, port: int, transport: str, action) -> None:  # type: ignore[no-untyped-def]
    with _client(host, port, transport) as client:
        _render(action(client))


@app.command()
def status(
    host: HostOpt = "127.0.0.1", port: PortOpt = 15001, transport: TransportOpt = "tcp"
) -> None:
    """Poll and print the robot's current status."""
    _run(host, port, transport, lambda c: c.read_status())


@app.command()
def enable(
    host: HostOpt = "127.0.0.1", port: PortOpt = 15001, transport: TransportOpt = "tcp"
) -> None:
    """Energise the servos."""
    _run(host, port, transport, lambda c: c.enable())


@app.command()
def disable(
    host: HostOpt = "127.0.0.1", port: PortOpt = 15001, transport: TransportOpt = "tcp"
) -> None:
    """De-energise the servos."""
    _run(host, port, transport, lambda c: c.disable())


@app.command()
def estop(
    host: HostOpt = "127.0.0.1", port: PortOpt = 15001, transport: TransportOpt = "tcp"
) -> None:
    """Engage the e-stop."""
    _run(host, port, transport, lambda c: c.estop())


@app.command()
def reset(
    host: HostOpt = "127.0.0.1", port: PortOpt = 15001, transport: TransportOpt = "tcp"
) -> None:
    """Clear the e-stop / fault."""
    _run(host, port, transport, lambda c: c.reset())


@app.command()
def movej(
    joints: Annotated[list[float], typer.Argument(help="Target joint angles (radians).")],
    host: HostOpt = "127.0.0.1",
    port: PortOpt = 15001,
    transport: TransportOpt = "tcp",
    speed: SpeedOpt = 1.0,
) -> None:
    """Move to an absolute joint target."""
    _run(host, port, transport, lambda c: c.move_joint(tuple(joints), speed=speed))


@app.command()
def movel(
    pose: Annotated[list[float], typer.Argument(help="Target pose x y z rx ry rz.")],
    host: HostOpt = "127.0.0.1",
    port: PortOpt = 15001,
    transport: TransportOpt = "tcp",
    speed: SpeedOpt = 1.0,
) -> None:
    """Move linearly to a Cartesian pose."""
    if len(pose) != 6:
        console.print("[red]pose needs exactly 6 values: x y z rx ry rz[/]")
        raise typer.Exit(code=1)
    target = (pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
    _run(host, port, transport, lambda c: c.move_linear(target, speed=speed))


if __name__ == "__main__":  # pragma: no cover
    app()
