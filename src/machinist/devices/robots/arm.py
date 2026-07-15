"""Shared robot-arm state machine.

All four robots (UR, Motoman, Dobot, Fanuc) speak different wire
protocols but share the same physics: joint positions, a TCP pose,
servo-on/off, e-stop, and the ability to execute a *movement command*
(``movej`` or ``movel``) that gradually transitions the state. This
module owns that physics so the per-vendor modules can stay focused on
parsing/formatting their wire protocol.
"""

from __future__ import annotations

import json
import math
import os
import sys
import threading
import time
from collections.abc import Callable
from dataclasses import dataclass, field
from enum import StrEnum, auto
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from ...kinematics.api import DHParams, Joints, Kinematics, KinematicsOptions, NoOpKinematics, Pose, RobotModel
from ...kinematics.units import Meters, Radians


JOINT_COUNT_DEFAULT = 6


@dataclass(frozen=True, slots=True)
class ArmOptions:
    """Typed schema for robot-arm YAML ``options`` section.

    Supports both forms::

        # All kinematics keys at the top level
        joint_count: 6
        backend: dh
        dh_params:
          a: [0, 0, …]

        # Kinematics nested under the ``kinematics`` key
        joint_count: 6
        kinematics:
          backend: dh
          dh_params:
            a: [0, 0, …]
    """

    joint_count: int = JOINT_COUNT_DEFAULT
    kinematics: KinematicsOptions | None = None
    backend: str | None = None
    dh_params: DHParams | None = None
    urdf: str | None = None


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
    seq: int = 0
    requested_pose: Pose | None = None  # cartesian goal for movel (before IK)


@dataclass(slots=True)
class ArmState:
    """Mutable, lock-protected robot state."""

    joints: Joints = (Radians(0.0),) * JOINT_COUNT_DEFAULT
    pose: Pose = (Meters(0.0), Meters(0.0), Meters(0.0), Radians(0.0), Radians(0.0), Radians(0.0))
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

    @property
    def moving(self) -> bool:
        return self.mode is ArmMode.MOVING

    @property
    def estopped(self) -> bool:
        return self.mode is ArmMode.ESTOPPED

    @property
    def faulted(self) -> bool:
        return self.mode is ArmMode.FAULTED


def _joints_deg(joints: Joints) -> list[float]:
    return [round(math.degrees(j), 4) for j in joints]


def _pose_mm_deg(pose: Pose) -> list[float]:
    """Pose in emulator SI units → mm + degrees (the Dobot/teach-pendant convention)."""
    return [
        round(pose[0] * 1e3, 4), round(pose[1] * 1e3, 4), round(pose[2] * 1e3, 4),
        round(math.degrees(pose[3]), 4), round(math.degrees(pose[4]), 4), round(math.degrees(pose[5]), 4),
    ]


class MoveLogger:
    """Records movement commands and their per-tick effects as JSONL for later analysis.

    Every record is one JSON object per line with a monotonic clock (``mono``),
    wall clock (``wall``), the arm ``name``, an ``event`` tag, and a per-move
    ``seq`` id so ticks can be grouped back to the command that issued them.
    Joints are logged in **degrees** and poses in **mm + degrees** so the data
    lines up with what a teach pendant / the Dobot wire protocol reports.

    Events:

    - ``move_start`` — a ``movej``/``movel`` began: the requested target, the
      joint solution chosen (for ``movel`` this is the IK result — the place
      where "questionable kinematic choices" get made), and the forward-kinematics
      pose of that solution so IK error is visible.
    - ``progress`` — one per physics tick (~50 Hz): interpolation fraction ``t``,
      elapsed seconds, and the live joints/pose.
    - ``move_end`` — the move reached ``t=1``.
    - ``jog`` — an instantaneous cartesian jog (``relmovltool``): the twist applied
      and the before/after state.
    """

    def __init__(self, path: str | Path, *, name: str = "arm", to_stderr: bool = True) -> None:
        self.name = name
        self._to_stderr = to_stderr
        self._lock = threading.Lock()
        self._path = Path(path)
        self._path.parent.mkdir(parents=True, exist_ok=True)
        self._f = self._path.open("a", buffering=1)

    def _base(self, event: str, seq: int, kind: str) -> dict[str, object]:
        return {
            "mono": round(time.monotonic(), 6),
            "wall": round(time.time(), 6),
            "arm": self.name,
            "event": event,
            "seq": seq,
            "kind": kind,
        }

    def _write(self, rec: dict[str, object]) -> None:
        line = json.dumps(rec)
        with self._lock:
            self._f.write(line + "\n")

    def move_start(
        self,
        *,
        seq: int,
        kind: str,
        duration: float,
        start_joints: Joints,
        start_pose: Pose,
        target_joints: Joints,
        target_fk_pose: Pose,
        requested_pose: Pose | None,
    ) -> None:
        rec = self._base("move_start", seq, kind)
        rec["duration"] = round(duration, 6)
        rec["start_joints"] = _joints_deg(start_joints)
        rec["start_pose"] = _pose_mm_deg(start_pose)
        rec["target_joints"] = _joints_deg(target_joints)
        rec["target_fk_pose"] = _pose_mm_deg(target_fk_pose)
        if requested_pose is not None:
            req = _pose_mm_deg(requested_pose)
            fk = rec["target_fk_pose"]
            rec["requested_pose"] = req
            rec["ik_pos_err_mm"] = round(math.dist(req[:3], fk[:3]), 4)
        self._write(rec)
        if self._to_stderr:
            extra = f" req_pose={rec['requested_pose']} ik_err={rec['ik_pos_err_mm']}mm" if requested_pose is not None else ""
            print(
                f"[move/{self.name}] start #{seq} {kind} dur={duration:.3f}s "
                f"target_j={rec['target_joints']}{extra}",
                file=sys.stderr, flush=True,
            )

    def progress(self, *, seq: int, kind: str, t: float, elapsed: float, joints: Joints, pose: Pose) -> None:
        rec = self._base("progress", seq, kind)
        rec["t"] = round(t, 6)
        rec["elapsed"] = round(elapsed, 6)
        rec["joints"] = _joints_deg(joints)
        rec["pose"] = _pose_mm_deg(pose)
        self._write(rec)

    def move_end(self, *, seq: int, kind: str, joints: Joints, pose: Pose) -> None:
        rec = self._base("move_end", seq, kind)
        rec["joints"] = _joints_deg(joints)
        rec["pose"] = _pose_mm_deg(pose)
        self._write(rec)
        if self._to_stderr:
            print(
                f"[move/{self.name}] end   #{seq} {kind} joints={rec['joints']} pose={rec['pose']}",
                file=sys.stderr, flush=True,
            )

    def jog(
        self,
        *,
        seq: int,
        start_joints: Joints,
        start_pose: Pose,
        joints: Joints,
        pose: Pose,
        twist: NDArray[np.float64],
    ) -> None:
        rec = self._base("jog", seq, "jog")
        rec["start_joints"] = _joints_deg(start_joints)
        rec["start_pose"] = _pose_mm_deg(start_pose)
        rec["joints"] = _joints_deg(joints)
        rec["pose"] = _pose_mm_deg(pose)
        rec["twist"] = [round(float(x), 6) for x in twist]
        self._write(rec)
        if self._to_stderr:
            print(
                f"[move/{self.name}] jog   #{seq} twist={rec['twist']} -> joints={rec['joints']}",
                file=sys.stderr, flush=True,
            )

    def close(self) -> None:
        with self._lock:
            if not self._f.closed:
                self._f.close()


def move_logger_from_env(name: str) -> MoveLogger | None:
    """Build a :class:`MoveLogger` if ``MACHINIST_MOVE_LOG`` is set, else ``None``.

    The env var is a path. If it points at (or is) a directory, the log is
    written to ``<dir>/<name>.moves.jsonl``; otherwise it is used verbatim as
    the file path. Set ``MACHINIST_MOVE_LOG_QUIET=1`` to suppress the concise
    per-move stderr summaries (the JSONL file is still written).
    """
    raw = os.environ.get("MACHINIST_MOVE_LOG")
    if not raw:
        return None
    path = Path(raw)
    if path.is_dir() or raw.endswith(os.sep):
        path = path / f"{name}.moves.jsonl"
    to_stderr = not os.environ.get("MACHINIST_MOVE_LOG_QUIET")
    return MoveLogger(path, name=name, to_stderr=to_stderr)


class RobotArm:
    """Common robot-arm physics. Wire protocols compose this."""

    def __init__(
        self,
        *,
        joint_count: int = JOINT_COUNT_DEFAULT,
        kinematics: Kinematics | None = None,
    ) -> None:
        home_joints: Joints = (Radians(0.0),) * joint_count
        self._kinematics: Kinematics = kinematics or NoOpKinematics(
            RobotModel(joint_count=joint_count)
        )
        self.state = ArmState(
            joints=home_joints,
            pose=self._kinematics.forward(home_joints),
        )
        self._tick_thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.move_logger: MoveLogger | None = None
        self._move_seq = 0

    # ----- commands ---------------------------------------------------

    def estop(self) -> None:
        with self.state._lock:
            self.state.mode = ArmMode.ESTOPPED
            self.state._move = None

    def reset(self) -> None:
        with self.state._lock:
            if self.state.mode is ArmMode.ESTOPPED:
                self.state.mode = ArmMode.IDLE

    def stop(self) -> None:
        with self.state._lock:
            self.state._move = None
            self.state.mode = ArmMode.IDLE

    def set_servo(self, on: bool) -> None:
        with self.state._lock:
            self.state.servo_on = on

    def set_speed_factor(self, fraction: float) -> None:
        with self.state._lock:
            self.state.speed_fraction = fraction

    def movej(self, target: Joints, *, duration: float = 1.0) -> None:
        self._begin_move(target, duration=duration, kind="movej")

    def movel(self, target_pose: Pose, *, duration: float = 1.0) -> None:
        target_joints = self._kinematics.inverse(target_pose, seed=self.state.joints)
        self._begin_move(target_joints, duration=duration, kind="movel", requested_pose=target_pose)

    def jog_cartesian(self, twist: NDArray[np.float64], *, dt: float = 1.0, damping: float = 0.02) -> None:
        """Single-step velocity-based cartesian jog via SVD Jacobian pseudoinverse.

        Updates arm state in-place — no interpolation delay.
        The *twist* is a 6-vector (m/s + rad/s) in the **flange** world frame.
        """
        entry: tuple[int, Joints, Pose, Joints, Pose] | None = None
        with self.state._lock:
            start_joints = self.state.joints
            start_pose = self.state.pose
            new_joints = self._kinematics.velocity_step(self.state.joints, twist * dt, damping=damping)
            self.state.joints = new_joints
            self.state.pose = self._kinematics.forward(new_joints)
            self.state._move = None
            self.state.mode = ArmMode.IDLE
            if self.move_logger is not None:
                self._move_seq += 1
                entry = (self._move_seq, start_joints, start_pose, new_joints, self.state.pose)
        if entry is not None and self.move_logger is not None:
            seq, sj, sp, nj, np_ = entry
            self.move_logger.jog(seq=seq, start_joints=sj, start_pose=sp, joints=nj, pose=np_, twist=twist * dt)

    # ----- background tick -------------------------------------------

    def start_ticker(self) -> None:
        self._tick_thread = threading.Thread(target=self._tick_loop, daemon=True)
        self._tick_thread.start()

    def stop_ticker(self) -> None:
        self._stop.set()
        if self._tick_thread is not None:
            self._tick_thread.join(timeout=1.0)
        if self.move_logger is not None:
            self.move_logger.close()

    # -----------------------------------------------------------------

    def _begin_move(self, target: Joints, *, duration: float, kind: str, requested_pose: Pose | None = None) -> None:
        start: tuple[int, float, Joints, Pose] | None = None
        with self.state._lock:
            if self.state.mode is ArmMode.ESTOPPED or not self.state.servo_on:
                raise RuntimeError(f"cannot move: mode={self.state.mode} servo={self.state.servo_on}")
            self._move_seq += 1
            scaled = max(duration, 1e-3) / max(self.state.speed_fraction, 1e-3)
            self.state._move = _Move(
                target=target,
                duration=scaled,
                started_at=time.monotonic(),
                started_joints=self.state.joints,
                kind=kind,
                seq=self._move_seq,
                requested_pose=requested_pose,
            )
            self.state.mode = ArmMode.MOVING
            if self.move_logger is not None:
                start = (self._move_seq, scaled, self.state.joints, self.state.pose)
        if start is not None and self.move_logger is not None:
            seq, scaled, start_joints, start_pose = start
            self.move_logger.move_start(
                seq=seq, kind=kind, duration=scaled,
                start_joints=start_joints, start_pose=start_pose,
                target_joints=target, target_fk_pose=self._kinematics.forward(target),
                requested_pose=requested_pose,
            )

    def _tick_loop(self) -> None:
        while not self._stop.wait(0.02):
            self._tick()

    def _tick(self) -> None:
        s = self.state
        entry: tuple[int, str, float, float, Joints, Pose, bool] | None = None
        with s._lock:
            move = s._move
            if move is None:
                return
            elapsed = time.monotonic() - move.started_at
            t = min(elapsed / move.duration, 1.0)
            s.joints = tuple(
                Radians(_lerp(a, b, t)) for a, b in zip(move.started_joints, move.target, strict=False)
            )
            s.pose = self._kinematics.forward(s.joints)
            done = t >= 1.0
            if self.move_logger is not None:
                entry = (move.seq, move.kind, t, elapsed, s.joints, s.pose, done)
            if done:
                s._move = None
                s.mode = ArmMode.IDLE
        if entry is not None and self.move_logger is not None:
            seq, kind, t, elapsed, joints, pose, done = entry
            self.move_logger.progress(seq=seq, kind=kind, t=t, elapsed=elapsed, joints=joints, pose=pose)
            if done:
                self.move_logger.move_end(seq=seq, kind=kind, joints=joints, pose=pose)


def _lerp(a: float, b: float, t: float) -> float:
    return a + (b - a) * t


def joints_almost_equal(a: Joints, b: Joints, *, tol: float = 1e-6) -> bool:
    return len(a) == len(b) and all(math.isclose(x, y, abs_tol=tol) for x, y in zip(a, b, strict=True))


def arm_from_options(options: ArmOptions, *, name: str = "arm") -> RobotArm:
    """Build a :class:`RobotArm` from typed arm options.

    If ``MACHINIST_MOVE_LOG`` is set in the environment, the arm is attached to
    a :class:`MoveLogger` (see :func:`move_logger_from_env`) that records every
    movement command and its per-tick effects for later analysis.
    """
    from ...kinematics.api import build_kinematics

    kin_opts = _kinematics_options(options)
    arm = RobotArm(joint_count=kin_opts.joint_count, kinematics=build_kinematics(kin_opts))
    arm.move_logger = move_logger_from_env(name)
    return arm


def _kinematics_options(options: ArmOptions) -> KinematicsOptions:
    if options.kinematics is not None:
        return options.kinematics
    return KinematicsOptions(
        joint_count=options.joint_count,
        backend=options.backend,
        dh=options.dh_params,
        urdf_path=Path(options.urdf) if options.urdf else None,
    )


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
