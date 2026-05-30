"""Binary telegrams for the Standard Robot Command Interface.

Two fixed-magic, versioned frames make up the protocol:

* :class:`CommandTelegram` — controller → robot. Carries a monotonically
  increasing *job id* (so replies can be correlated), a :class:`Function`
  selector, a speed fraction, and a variable-length ``float64`` argument
  vector (joint targets for ``MOVE_JOINT``, an x/y/z/rx/ry/rz pose for
  ``MOVE_LINEAR``; empty for the rest).
* :class:`StatusTelegram` — robot → controller. Echoes the job id, packs
  the handshake/state into a :class:`StatusFlag` bitfield with an error
  code, names the function the robot is currently executing, and reports
  the live joint vector and TCP pose.

Layouts are explicit ``struct`` formats with a magic + version prefix so
a stray or mismatched frame is rejected rather than misread. Everything
here is pure bytes ↔ value; no sockets, no robot, no threads.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag

MAGIC = 0x53524349  # "SRCI"
VERSION = 1

Pose = tuple[float, float, float, float, float, float]


class Function(IntEnum):
    """The command a controller asks the robot to perform."""

    NOP = 0
    ENABLE = 1  # servos on
    DISABLE = 2  # servos off
    MOVE_JOINT = 3  # args = joint targets (radians)
    MOVE_LINEAR = 4  # args = x,y,z,rx,ry,rz pose
    STOP = 5  # e-stop
    RESET = 6  # clear e-stop / fault
    READ_STATUS = 7  # no-op poll


class StatusFlag(IntFlag):
    """Robot handshake/state bits in a :class:`StatusTelegram`."""

    NONE = 0
    BUSY = 1 << 0  # a move is in progress
    DONE = 1 << 1  # last commanded job finished
    ERROR = 1 << 2  # error_code is meaningful
    ESTOP = 1 << 3  # e-stop engaged
    SERVO_ON = 1 << 4  # servos energised


_CMD_HEAD = struct.Struct(">IBIBfH")  # magic, version, job_id, function, speed, argc
_STATUS_HEAD = struct.Struct(">IBIBHBH")  # magic, version, job_id, flags, err, fn, jointc
_F64 = struct.Struct(">d")
_POSE = struct.Struct(">6d")


def _pack_floats(values: tuple[float, ...]) -> bytes:
    return b"".join(_F64.pack(v) for v in values)


def _unpack_floats(blob: bytes, count: int) -> tuple[float, ...]:
    if len(blob) < count * _F64.size:
        raise ValueError("truncated float vector")
    return tuple(_F64.unpack_from(blob, i * _F64.size)[0] for i in range(count))


@dataclass(frozen=True, slots=True)
class CommandTelegram:
    """Controller → robot command frame."""

    job_id: int
    function: Function
    args: tuple[float, ...] = ()
    speed: float = 1.0

    def encode(self) -> bytes:
        head = _CMD_HEAD.pack(
            MAGIC, VERSION, self.job_id, int(self.function), self.speed, len(self.args)
        )
        return head + _pack_floats(self.args)

    @classmethod
    def decode(cls, frame: bytes) -> "CommandTelegram":
        try:
            magic, version, job_id, function, speed, argc = _CMD_HEAD.unpack_from(frame)
        except struct.error as exc:
            raise ValueError("not an SRCI frame (too short)") from exc
        _check(magic, version)
        args = _unpack_floats(frame[_CMD_HEAD.size :], argc)
        return cls(job_id=job_id, function=Function(function), args=args, speed=speed)

    @property
    def pose(self) -> Pose:
        """Interpret ``args`` as a 6-DOF pose (raises if wrong length)."""
        if len(self.args) != 6:
            raise ValueError(f"expected 6 pose values, got {len(self.args)}")
        return self.args  # type: ignore[return-value]


@dataclass(frozen=True, slots=True)
class StatusTelegram:
    """Robot → controller status frame."""

    job_id: int
    flags: StatusFlag
    active_function: Function
    joints: tuple[float, ...]
    pose: Pose
    error_code: int = 0

    def encode(self) -> bytes:
        head = _STATUS_HEAD.pack(
            MAGIC,
            VERSION,
            self.job_id,
            int(self.flags),
            self.error_code,
            int(self.active_function),
            len(self.joints),
        )
        return head + _pack_floats(self.joints) + _POSE.pack(*self.pose)

    @classmethod
    def decode(cls, frame: bytes) -> "StatusTelegram":
        try:
            magic, version, job_id, flags, err, fn, jointc = _STATUS_HEAD.unpack_from(frame)
        except struct.error as exc:
            raise ValueError("not an SRCI frame (too short)") from exc
        _check(magic, version)
        body = frame[_STATUS_HEAD.size :]
        joints = _unpack_floats(body, jointc)
        pose_off = jointc * _F64.size
        pose = _POSE.unpack_from(body, pose_off)
        return cls(
            job_id=job_id,
            flags=StatusFlag(flags),
            active_function=Function(fn),
            joints=joints,
            pose=pose,
            error_code=err,
        )


def _check(magic: int, version: int) -> None:
    if magic != MAGIC:
        raise ValueError(f"not an SRCI frame (magic={magic:#x})")
    if version != VERSION:
        raise ValueError(f"unsupported SRCI version {version}")
