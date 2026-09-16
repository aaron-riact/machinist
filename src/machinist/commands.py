"""The one command surface, shared by the TUI command bar and the web endpoint.

A command is a line of text: a verb, usually a device name, then arguments.
:meth:`Commands.dispatch` parses it, finds the device, checks it has the
capability the verb needs, applies the change, and returns a
:class:`CommandResult` to show. Anything wrong is a :class:`CommandError`
with a message meant for a person.

The TUI passes the currently selected device as *selected*, so a verb may
leave the device name out; the web has no selection and must name it.
Effects reach the UIs the same way everything else does: as events.
"""

from __future__ import annotations

from dataclasses import dataclass

from .core.capabilities import HasPrograms
from .core.device import Device
from .core.world import World
from .devices.robots.arm import HasArm, RobotArm
from .devices.robots.dobot import PROTECTIVE_STOP_MODE_BY_NAME, DobotDashboard, EnableFailure

HELP = (
    "commands: estop <device> | reset <device> | servo <device> on|off | "
    "set <device.signal> 0|1 | ls <device> | run <device> <program> | "
    "pstop <device> [clear|collision|error|disabled] [ids=17,116] [sticky] | "
    "failenable <device> stuck|error|off | help"
)

_TRUE = ("1", "true", "on")


class CommandError(Exception):
    """A command line that cannot be applied, with a message for the person who typed it."""


@dataclass(frozen=True, slots=True)
class CommandResult:
    """What a successful command has to say."""

    message: str
    programs: tuple[str, ...] | None = None


@dataclass(frozen=True, slots=True)
class _PstopArgs:
    device: str
    clear: bool = False
    robot_mode: str = "collision"
    controller_ids: tuple[int, ...] = ()
    sticky: bool = False


class Commands:
    """Apply command lines to a :class:`World`."""

    def __init__(self, world: World) -> None:
        self._world = world

    def dispatch(self, line: str, *, selected: str | None = None) -> CommandResult:
        line = line.strip()
        if not line:
            raise CommandError("empty command")
        verb, _, rest = line.partition(" ")
        rest = rest.strip()
        match verb:
            case "help":
                return CommandResult(HELP)
            case "estop":
                return self.estop(self._name(rest, selected))
            case "reset":
                return self.reset(self._name(rest, selected))
            case "servo":
                name, _, value = rest.partition(" ")
                return self.servo(self._name(name, selected), value.strip() in _TRUE)
            case "set":
                target, _, value = rest.partition(" ")
                return self.set_signal(target, value.strip() in _TRUE)
            case "ls":
                return self.ls(self._name(rest, selected))
            case "run":
                name, _, program = rest.partition(" ")
                return self.run(self._name(name, selected), program.strip())
            case "pstop":
                return self.pstop(_parse_pstop(rest, selected))
            case "failenable":
                name, _, value = rest.partition(" ")
                return self.failenable(self._name(name, selected), value.strip())
            case _:
                raise CommandError(f"unknown command: {verb}")

    # ----- verbs --------------------------------------------------------

    def estop(self, name: str) -> CommandResult:
        self._arm(name).estop()
        return CommandResult(f"e-stop engaged on {name}")

    def reset(self, name: str) -> CommandResult:
        self._arm(name).reset()
        return CommandResult(f"reset {name}")

    def servo(self, name: str, on: bool) -> CommandResult:
        self._arm(name).set_servo(on)
        return CommandResult(f"servo {'on' if on else 'off'} for {name}")

    def set_signal(self, target: str, on: bool) -> CommandResult:
        try:
            self._world.io_map.signal(target).set(on)
        except (KeyError, ValueError) as exc:
            raise CommandError(str(exc)) from exc
        return CommandResult(f"set {target} = {on}")

    def ls(self, name: str) -> CommandResult:
        names = tuple(self._programs(name).programs.list())
        return CommandResult(f"{name}: {', '.join(names) or '(empty)'}", programs=names)

    def run(self, name: str, program: str) -> CommandResult:
        if not program:
            raise CommandError("usage: run <device> <program>")
        device = self._programs(name, verb="run programs")
        try:
            device.run_program(program)
        except (FileNotFoundError, RuntimeError) as exc:
            raise CommandError(str(exc)) from exc
        return CommandResult(f"started {program} on {name}")

    def pstop(self, args: _PstopArgs) -> CommandResult:
        device = self._faultable(args.device)
        if args.clear:
            device.clear_protective_stop()
            return CommandResult(f"protective stop cleared on {args.device}")
        device.inject_protective_stop(
            robot_mode=PROTECTIVE_STOP_MODE_BY_NAME[args.robot_mode],
            controller_ids=args.controller_ids,
            sticky=args.sticky,
        )
        ids = ",".join(str(c) for c in args.controller_ids) or "none"
        sticky = ", sticky" if args.sticky else ""
        return CommandResult(
            f"protective stop {args.robot_mode} on {args.device} (alarms {ids}{sticky})"
        )

    def failenable(self, name: str, value: str) -> CommandResult:
        device = self._faultable(name)
        if value in ("off", ""):
            device.set_enable_failure(None)
            return CommandResult(f"{name} enables normally again")
        try:
            failure = EnableFailure(value)
        except ValueError as exc:
            raise CommandError("usage: failenable <device> stuck|error|off") from exc
        device.set_enable_failure(failure)
        return CommandResult(f"{name} will fail to enable ({failure.value})")

    # ----- lookups ------------------------------------------------------

    @staticmethod
    def _name(given: str, selected: str | None) -> str:
        name = given.strip() or (selected or "")
        if not name:
            raise CommandError("which device? name one, or select it first")
        return name

    def _device(self, name: str) -> Device:
        device = next((d for d in self._world.devices if d.name == name), None)
        if device is None:
            raise CommandError(f"unknown device: {name!r}")
        return device

    def _arm(self, name: str) -> RobotArm:
        device = self._device(name)
        if not isinstance(device, HasArm):
            raise CommandError(f"{name!r} has no arm")
        return device.arm

    def _programs(self, name: str, *, verb: str | None = None) -> HasPrograms:
        device = self._device(name)
        if not isinstance(device, HasPrograms):
            complaint = f"cannot {verb}" if verb else "has no program library"
            raise CommandError(f"{name!r} {complaint}")
        return device

    def _faultable(self, name: str) -> DobotDashboard:
        """The fault-injection verbs speak Dobot's vocabulary (mode codes, alarm ids)."""
        device = self._device(name)
        if not isinstance(device, DobotDashboard):
            raise CommandError(f"{name!r} cannot be put into a protective stop")
        return device


def _parse_pstop(rest: str, selected: str | None) -> _PstopArgs:
    device, _, tail = rest.partition(" ")
    if _is_pstop_option(device):  # no device named: the first token is already an option
        device, tail = "", rest
    device = Commands._name(device, selected)
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


def _is_pstop_option(token: str) -> bool:
    return token in ("clear", "sticky") or token in PROTECTIVE_STOP_MODE_BY_NAME or token.startswith("ids=")
