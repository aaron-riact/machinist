"""Pure, IO-free serialization and command dispatch for the web UI.

Everything here operates on an in-memory :class:`~machinist.core.world.World`
and returns plain JSON-able ``dict``/``list`` structures (or accepts a command
string and mutates the world). There is deliberately **no** HTTP, sockets or
threading in this module: that keeps it unit-testable in microseconds, mirroring
how :mod:`machinist.tui.app`'s helpers are tested.

The serialization intentionally produces the same facts the TUI renders —
lifecycle, IO grouped by direction, robot-arm snapshot, CNC machine state — so
the browser and the terminal stay feature-equivalent.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any

from ..core.world import World
from ..devices.robots.dobot import PROTECTIVE_STOP_MODE_BY_NAME, EnableFailure


def snapshot_world(world: World) -> dict[str, Any]:
    """Serialize the whole fleet into a JSON-able snapshot."""
    return {"devices": [snapshot_device(d) for d in world.devices]}


def snapshot_device(device: Any) -> dict[str, Any]:
    """Serialize a single device, including any arm/machine/IO it exposes."""
    snap: dict[str, Any] = {
        "name": device.name,
        "kind": device.kind,
        "endpoint": str(device.endpoint),
        "lifecycle": str(device.lifecycle),
    }
    detail = device.build_detail()
    if detail is not None:
        snap["signals"] = detail.get("signals", [])
        mode = detail.get("mode", "")
        if mode == "modbus":
            snap["modbus"] = detail
        else:
            snap["ethernetip"] = detail
    arm = _arm(getattr(device, "arm", None))
    if arm is not None:
        snap["arm"] = arm
    machine = _machine(getattr(device, "state", None))
    if machine is not None:
        snap["machine"] = machine
    programs = getattr(device, "programs", None)
    if programs is not None and hasattr(programs, "list"):
        snap["programs"] = list(programs.list())
    return snap


def _arm(arm: Any) -> dict[str, Any] | None:
    if arm is None:
        return None
    s = arm.state.snapshot()
    return {
        "mode": str(s.mode),
        "servo_on": s.servo_on,
        "estopped": s.estopped,
        "faulted": s.faulted,
        "moving": s.moving,
        "command": s.current_command,
        "speed_fraction": s.speed_fraction,
        "joints": list(s.joints),
        "pose": list(s.pose),
    }


def _machine(state: Any) -> dict[str, Any] | None:
    if state is None or not hasattr(state, "cycle"):
        return None
    program = state.program.splitlines()[0] if state.program else ""
    return {
        "cycle": str(state.cycle),
        "program": program,
        "spindle_rpm": state.spindle_rpm,
        "feed": state.feed,
        "tool": state.tool,
        "parts": state.parts,
        "position": {
            "x": state.position.x,
            "y": state.position.y,
            "z": state.position.z,
        },
        "doors": {name: door.open for name, door in state.doors.items()},
        "chucks": {name: chuck.open for name, chuck in state.chucks.items()},
    }


# --- command dispatch ---------------------------------------------------


class CommandError(Exception):
    """Raised when a command string cannot be applied to the world."""


def dispatch_command(world: World, line: str) -> dict[str, Any]:
    """Apply a command string to ``world`` and return a JSON-able result.

    The verbs mirror the TUI command bar so muscle memory transfers between
    the terminal and the browser: ``estop``/``reset``/``servo``/``set``/
    ``ls``/``run``/``help``. The result always carries an ``ok`` flag and a
    human-readable ``message`` (and, for ``ls``, a ``programs`` list).
    """
    line = line.strip()
    if not line:
        raise CommandError("empty command")
    verb, _, rest = line.partition(" ")
    handler = _COMMANDS.get(verb)
    if handler is None:
        raise CommandError(f"unknown command: {verb}")
    return handler(world, rest.strip())


def _lookup(world: World, name: str) -> Any:
    device = next((d for d in world.devices if d.name == name), None)
    if device is None:
        raise CommandError(f"unknown device: {name!r}")
    return device


def _arm_of(world: World, name: str) -> Any:
    arm = getattr(_lookup(world, name), "arm", None)
    if arm is None:
        raise CommandError(f"{name!r} has no arm")
    return arm


def _cmd_estop(world: World, rest: str) -> dict[str, Any]:
    _arm_of(world, rest).estop()
    return _ok(f"e-stop engaged on {rest}")


def _cmd_reset(world: World, rest: str) -> dict[str, Any]:
    _arm_of(world, rest).reset()
    return _ok(f"reset {rest}")


def _cmd_servo(world: World, rest: str) -> dict[str, Any]:
    name, _, value = rest.partition(" ")
    on = value.strip() in ("1", "true", "on")
    _arm_of(world, name).set_servo(on)
    return _ok(f"servo {'on' if on else 'off'} for {name}")


def _cmd_set(world: World, rest: str) -> dict[str, Any]:
    target, _, value = rest.partition(" ")
    on = value.strip() in ("1", "true", "on")
    try:
        world.io_map.signal(target).set(on)
    except (KeyError, ValueError) as exc:
        raise CommandError(str(exc)) from exc
    return _ok(f"set {target} = {on}")


def _cmd_ls(world: World, rest: str) -> dict[str, Any]:
    programs = getattr(_lookup(world, rest), "programs", None)
    if programs is None or not hasattr(programs, "list"):
        raise CommandError(f"{rest!r} has no program library")
    names = list(programs.list())
    return {"ok": True, "message": f"{rest}: {', '.join(names) or '(empty)'}", "programs": names}


def _cmd_run(world: World, rest: str) -> dict[str, Any]:
    target, _, program = rest.partition(" ")
    program = program.strip()
    run_program = getattr(_lookup(world, target), "run_program", None)
    if run_program is None:
        raise CommandError(f"{target!r} cannot run programs")
    try:
        run_program(program)
    except (FileNotFoundError, RuntimeError) as exc:
        raise CommandError(str(exc)) from exc
    return _ok(f"started {program} on {target}")


@dataclass(frozen=True, slots=True)
class _PstopArgs:
    """A parsed ``pstop`` command line."""

    device: str
    clear: bool = False
    robot_mode: str = "collision"
    controller_ids: tuple[int, ...] = ()
    sticky: bool = False


def _parse_pstop(rest: str) -> _PstopArgs:
    device, _, tail = rest.partition(" ")
    if not device:
        raise CommandError("usage: pstop <device> [clear|collision|error|disabled] [ids=…] [sticky]")

    clear = False
    robot_mode = "collision"
    controller_ids: tuple[int, ...] = ()
    sticky = False
    for token in tail.split():
        if token == "clear":
            clear = True
        elif token == "sticky":
            sticky = True
        elif token in PROTECTIVE_STOP_MODE_BY_NAME:
            robot_mode = token
        elif token.startswith("ids="):
            try:
                controller_ids = tuple(int(c) for c in token[4:].split(",") if c)
            except ValueError as exc:
                raise CommandError(f"bad alarm ids: {token[4:]!r}") from exc
        else:
            raise CommandError(f"unknown pstop option: {token!r}")
    return _PstopArgs(device, clear, robot_mode, controller_ids, sticky)


def _faultable(world: World, name: str) -> Any:
    device = _lookup(world, name)
    if not hasattr(device, "inject_protective_stop"):
        raise CommandError(f"{name!r} cannot be put into a protective stop")
    return device


def _cmd_pstop(world: World, rest: str) -> dict[str, Any]:
    args = _parse_pstop(rest)
    device = _faultable(world, args.device)
    if args.clear:
        device.clear_protective_stop()
        return _ok(f"protective stop cleared on {args.device}")
    device.inject_protective_stop(
        robot_mode=PROTECTIVE_STOP_MODE_BY_NAME[args.robot_mode],
        controller_ids=args.controller_ids,
        sticky=args.sticky,
    )
    ids = ",".join(str(c) for c in args.controller_ids) or "none"
    sticky = ", sticky" if args.sticky else ""
    return _ok(f"protective stop {args.robot_mode} on {args.device} (alarms {ids}{sticky})")


def _cmd_failenable(world: World, rest: str) -> dict[str, Any]:
    name, _, value = rest.partition(" ")
    value = value.strip()
    device = _faultable(world, name)
    if value in ("off", ""):
        device.set_enable_failure(None)
        return _ok(f"{name} enables normally again")
    try:
        failure = EnableFailure(value)
    except ValueError as exc:
        raise CommandError(f"usage: failenable <device> stuck|error|off") from exc
    device.set_enable_failure(failure)
    return _ok(f"{name} will fail to enable ({failure.value})")


def _cmd_help(_world: World, _rest: str) -> dict[str, Any]:
    return _ok(
        "commands: estop <device> | reset <device> | servo <device> on|off | "
        "set <device.signal> 0|1 | ls <device> | run <device> <program> | "
        "pstop <device> [clear|collision|error|disabled] [ids=17,116] [sticky] | "
        "failenable <device> stuck|error|off"
    )


def _ok(message: str) -> dict[str, Any]:
    return {"ok": True, "message": message}


_COMMANDS = {
    "estop": _cmd_estop,
    "reset": _cmd_reset,
    "servo": _cmd_servo,
    "set": _cmd_set,
    "ls": _cmd_ls,
    "run": _cmd_run,
    "pstop": _cmd_pstop,
    "failenable": _cmd_failenable,
    "help": _cmd_help,
}
