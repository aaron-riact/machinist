"""Shared robot-arm state machine.

All four robots (UR, Motoman, Dobot, Fanuc) speak different wire
protocols but share the same physics: joint positions, a TCP pose,
servo-on/off, e-stop, and the ability to execute a *movement command*
(``movej`` or ``movel``) that gradually transitions the state. This
module owns that physics so the per-vendor modules can stay focused on
parsing/formatting their wire protocol.
"""

from __future__ import annotations

import math
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import StrEnum, auto

from ...kinematics.api import Kinematics, NoOpKinematics, RobotModel

JOINT_COUNT_DEFAULT = 6
Pose = tuple[float, float, float, float, float, float]  # x,y,z,rx,ry,rz
Joints = tuple[float, ...]


class ArmMode(StrEnum):
    IDLE = auto()
    MOVING = auto()
    ESTOPPED = auto()
    FAULTED = auto()


@dataclass(slots=True)
class _Move:
    target: Joints
    duration: float
    started_at: float
    started_joints: Joints
    kind: str  # "movej" or "movel"


@dataclass(slots=True)
class ArmState:
    """Mutable, lock-protected robot state."""

    joints: Joints = (0.0,) * JOINT_COUNT_DEFAULT
    pose: Pose = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    mode: ArmMode = ArmMode.IDLE
    servo_on: bool = True
    program_running: bool = False
    speed_fraction: float = 1.0
    _lock: threading.Lock = field(default_factory=threading.Lock)
    _move: _Move | None = None

    def snapshot(self) -> "ArmStateView":
        with self._lock:
            return ArmStateView(
                joints=self.joints,
                pose=self.pose,
                mode=self.mode,
                servo_on=self.servo_on,
                program_running=self.program_running,
                speed_fraction=self.speed_fraction,
                current_command=self._move.kind if self._move else None,
            )


@dataclass(frozen=True, slots=True)
class ArmStateView:
    """Immutable point-in-time snapshot of :class:`ArmState`."""

    joints: Joints
    pose: Pose
    mode: ArmMode
    servo_on: bool
    program_running: bool
    speed_fraction: float
    current_command: str | None = None


class RobotArm:
    """Common robot-arm physics. Wire protocols compose this."""

    def __init__(
        self,
        *,
        joint_count: int = JOINT_COUNT_DEFAULT,
        kinematics: Kinematics | None = None,
    ) -> None:
        self.state = ArmState(joints=(0.0,) * joint_count)
        self._kinematics: Kinematics = kinematics or NoOpKinematics(
            RobotModel(joint_count=joint_count)
        )
        self._tick_thread: threading.Thread | None = None
        self._stop = threading.Event()

    # ----- commands ---------------------------------------------------

    def estop(self) -> None:
        with self.state._lock:
            self.state.mode = ArmMode.ESTOPPED
            self.state._move = None

    def reset(self) -> None:
        with self.state._lock:
            if self.state.mode is ArmMode.ESTOPPED:
                self.state.mode = ArmMode.IDLE

    def set_servo(self, on: bool) -> None:
        with self.state._lock:
            self.state.servo_on = on

    def movej(self, target: Joints, *, duration: float = 1.0) -> None:
        self._begin_move(target, duration=duration, kind="movej")

    def movel(self, target_pose: Pose, *, duration: float = 1.0) -> None:
        target_joints = self._kinematics.inverse(target_pose, seed=self.state.joints)
        self._begin_move(target_joints, duration=duration, kind="movel")

    # ----- background tick -------------------------------------------

    def start_ticker(self) -> None:
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._tick_thread.start()

    def stop_ticker(self) -> None:
        self._stop.set()
        if self._tick_thread is not None:
            self._tick_thread.join(timeout=1.0)

    # -----------------------------------------------------------------

    def _begin_move(self, target: Joints, *, duration: float, kind: str) -> None:
        with self.state._lock:
            if self.state.mode is ArmMode.ESTOPPED or not self.state.servo_on:
                raise RuntimeError(f"cannot move: mode={self.state.mode} servo={self.state.servo_on}")
            self.state._move = _Move(
                target=target,
                duration=max(duration, 1e-3) / max(self.state.speed_fraction, 1e-3),
                started_at=time.monotonic(),
                started_joints=self.state.joints,
                kind=kind,
            )
            self.state.mode = ArmMode.MOVING

    def _tick_loop(self) -> None:
        while not self._stop.wait(0.02):
            self._tick()

    def _tick(self) -> None:
        s = self.state
        with s._lock:
            move = s._move
            if move is None:
                return
            elapsed = time.monotonic() - move.started_at
            t = min(elapsed / move.duration, 1.0)
            s.joints = tuple(
                _lerp(a, b, t) for a, b in zip(move.started_joints, move.target, strict=False)
            )
            s.pose = self._kinematics.forward(s.joints)
            if t >= 1.0:
                s._move = None
                s.mode = ArmMode.IDLE


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def joints_almost_equal(a: Joints, b: Joints, *, tol: float = 1e-6) -> bool:
    return len(a) == len(b) and all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b, strict=True))


def arm_from_options(options: dict) -> RobotArm:
    """Build a :class:`RobotArm` from a device's YAML ``options``.

    Reads ``joint_count`` and an optional ``kinematics`` sub-dict
    ({backend, urdf, dh_params, …}). Keeping this helper here means
    every wire-protocol adapter (UR, Motoman, Dobot, Fanuc) shares
    identical config semantics with zero duplication.
    """
    from ...kinematics.api import build_kinematics
    joint_count = int(options.get("joint_count", 6))
    kin_opts = options.get("kinematics") or {"backend": "noop"}
    kin_opts.setdefault("joint_count", joint_count)
    return RobotArm(joint_count=joint_count, kinematics=build_kinematics(kin_opts))


def arm_readers(arm: RobotArm) -> dict[str, Callable[[], object]]:
    """Zero-arg readers exposing arm state (e.g. for an OPC-UA server).

    Each value is read fresh from a snapshot, so callers — like the
    OPC-UA publisher — stay oblivious to the arm's locking.
    """
    return {
        "mode": lambda: str(arm.state.snapshot().mode),
        "servo_on": lambda: arm.state.snapshot().servo_on,
        "estopped": lambda: arm.state.snapshot().mode is ArmMode.ESTOPPED,
        "moving": lambda: arm.state.snapshot().mode is ArmMode.MOVING,
        "command": lambda: arm.state.snapshot().current_command or "none",
        "joints": lambda: arm.state.snapshot().joints,
        "pose": lambda: arm.state.snapshot().pose,
    }
