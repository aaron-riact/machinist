"""Server-side SRCI: turn command telegrams into robot-arm motion.

:class:`SrciServer` is the bridge between the wire protocol and the
shared :class:`~machinist.devices.robots.arm.RobotArm` physics. It is
deliberately transport-free: feed it a request frame, get a response
frame. A device wraps it with a :class:`MessageServer`; tests can drive
it with raw bytes.
"""

from __future__ import annotations

from typing import Protocol

from ..kinematics.api import Joints, Pose
from ..kinematics.units import Meters, Radians
from .codec import CommandTelegram, Function, StatusFlag, StatusTelegram

_MOVE_DURATION = 1.0


class _ArmSnapshot(Protocol):
    joints: tuple[float, ...]
    pose: tuple[float, float, float, float, float, float]
    servo_on: bool
    current_command: str | None
    moving: bool
    estopped: bool
    faulted: bool


class _ArmState(Protocol):
    def snapshot(self) -> _ArmSnapshot: ...


class SrciArm(Protocol):
    state: _ArmState

    def set_servo(self, on: bool) -> None: ...

    def estop(self) -> None: ...

    def reset(self) -> None: ...

    def movej(self, target: tuple[float, ...], *, duration: float = 1.0) -> None: ...

    def movel(
        self,
        target_pose: tuple[float, float, float, float, float, float],
        *,
        duration: float = 1.0,
    ) -> None: ...


class SrciServer:
    """Execute SRCI command frames against a :class:`RobotArm`."""

    def __init__(self, arm: SrciArm) -> None:
        self._arm = arm

    def handle(self, frame: bytes) -> bytes:
        """Decode one command frame and return one status frame."""
        try:
            command = CommandTelegram.decode(frame)
        except ValueError:
            return self._status(job_id=0, error_code=1).encode()
        error = self._apply(command)
        return self._status(job_id=command.job_id, error_code=error).encode()

    # -----------------------------------------------------------------

    def _apply(self, command: CommandTelegram) -> int:
        try:
            self._dispatch(command)
        except (RuntimeError, ValueError):
            return 2
        return 0

    def _dispatch(self, command: CommandTelegram) -> None:
        fn = command.function
        if fn is Function.ENABLE:
            self._arm.set_servo(True)
        elif fn is Function.DISABLE:
            self._arm.set_servo(False)
        elif fn is Function.STOP:
            self._arm.estop()
        elif fn is Function.RESET:
            self._arm.reset()
        elif fn is Function.MOVE_JOINT:
            target: Joints = tuple(Radians(v) for v in command.args)
            self._arm.movej(target, duration=_MOVE_DURATION / max(command.speed, 1e-3))
        elif fn is Function.MOVE_LINEAR:
            p = command.pose
            target_pose: Pose = (Meters(p[0]), Meters(p[1]), Meters(p[2]),
                                 Radians(p[3]), Radians(p[4]), Radians(p[5]))
            self._arm.movel(target_pose, duration=_MOVE_DURATION / max(command.speed, 1e-3))
        # NOP / READ_STATUS: just report current state.

    def _status(self, *, job_id: int, error_code: int) -> StatusTelegram:
        s = self._arm.state.snapshot()
        flags = StatusFlag.NONE
        if s.servo_on:
            flags |= StatusFlag.SERVO_ON
        if s.moving:
            flags |= StatusFlag.BUSY
        else:
            flags |= StatusFlag.DONE
        if s.estopped:
            flags |= StatusFlag.ESTOP
        if error_code or s.faulted:
            flags |= StatusFlag.ERROR
        active = _COMMAND_TO_FUNCTION.get(s.current_command, Function.NOP)
        return StatusTelegram(
            job_id=job_id,
            flags=flags,
            active_function=active,
            joints=s.joints,
            pose=s.pose,
            error_code=error_code,
        )


_COMMAND_TO_FUNCTION: dict[str | None, Function] = {
    "movej": Function.MOVE_JOINT,
    "movel": Function.MOVE_LINEAR,
}
