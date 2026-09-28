"""Local stand-ins for the litearm-core public types, with no third-party deps.

The standalone simulation must keep working with nothing but ``pybullet`` and
``numpy``, so ``litearm-core`` cannot be a hard dependency.  But the types and
exceptions the simulation hands back have to stay *structurally identical* to
the real SDK's — otherwise "swap ``PyBulletArm`` for the real ``Arm``" is a
lie: ``except litearm_core.MotionTimeoutError`` would not catch a simulation
failure, and ``state.value.joints[i].q`` would not be the same expression.

``_compat`` is the single place that decides which implementation is in play
(real SDK when importable, this module otherwise).  The two are kept in step by
``tests/test_api_parity.py``, which compares field names and signatures
against the real SDK whenever it is importable.

Aligned with: litearm-core 2.1.0.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Generic, List, Sequence, Tuple, TypeVar

__all__ = [
    "JointState", "RobotState", "Msg", "CartPlan",
    "LiteArmError", "NotConnectedError", "TransportError", "FirmwareMismatchError",
    "InvalidCommandError", "MotorFaultError", "MotionTimeoutError", "IKError",
    "CommandRejectedError", "UnsupportedByFirmwareError", "CartesianPlanError",
    "MotionSupersededError", "CartReplyLostError", "ArmIsInDfuError",
    "MODE_NAMES", "FLAG_NAMES", "FLAG_ENABLED_BIT", "MAX_JOINTS",
    "as_pose", "rpy_to_mat", "mat_to_rpy", "is_rotation",
]


# ── Protocol constants (mirrors of litearm_core._protocol) ─────────────────────

#: Firmware motion modes.  The simulation uses the same names for the same
#: states so ``state.mode_name`` reads the same on both sides.
MODE_NAMES = {0: "INIT", 1: "MOVE_J", 2: "MOVE_P", 3: "MOVE_JS",
              4: "MOVE_MIT", 5: "MIT_ALL", 6: "EMERGENCY", 7: "ZERO_G"}
#: Safety flag names.  Bit 9 (enabled) and bit 10 (cartesian busy) are
#: deliberately absent: those are read through properties, never as faults.
FLAG_NAMES = {0: "FAULT", 1: "WD_TRIPPED", 2: "FB_STALE",
              3: "TEMP_WARN", 4: "POS_VIOL", 5: "OVERSPEED"}
#: flags bit9 = motors enabled.
FLAG_ENABLED_BIT = 9
#: Upper bound on joints representable in the joint_fault bitmap.
MAX_JOINTS = 16


# ── State types ───────────────────────────────────────────────────────────────

@dataclass
class JointState:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    t_mos: float = 0.0
    t_coil: float = 0.0
    err: int = 0


@dataclass
class RobotState:
    mode: int = 0
    mode_name: str = "INIT"
    flags: int = 0
    flag_names: List[str] = field(default_factory=list)
    seq: int = 0
    joints: List[JointState] = field(default_factory=list)
    joint_fault: int = 0

    @property
    def n(self) -> int:
        return len(self.joints)

    @property
    def enabled(self) -> bool:
        """flags bit9.  False on the real arm means "not reported" for old
        firmware; the simulation always reports it truthfully."""
        return bool(self.flags & (1 << FLAG_ENABLED_BIT))

    @property
    def cart_busy(self) -> bool:
        """flags bit10 — a cartesian plan is being played."""
        return bool(self.flags & (1 << 10))

    @property
    def fault_axes(self) -> List[int]:
        """Zero-based indices of the joints the firmware dropped."""
        return [i for i in range(MAX_JOINTS) if self.joint_fault & (1 << i)]

    @property
    def q(self) -> List[float]:
        return [j.q for j in self.joints]

    @property
    def dq(self) -> List[float]:
        return [j.dq for j in self.joints]

    @property
    def tau(self) -> List[float]:
        return [j.tau for j in self.joints]

    @property
    def drop_hold_inferred(self) -> bool:
        """Inferred, never read (the firmware does not report it).

        Follows the upstream shorthand: a joint fault implies a possible
        drop-hold latch, ``joint_fault == 0`` implies there is none.
        """
        return bool(self.joint_fault)

    @property
    def faulted(self) -> bool:
        """FAULT flag / EMERGENCY, or any axis dropped by the firmware."""
        return bool(self.flags & 1) or self.mode == 6 or self.joint_fault != 0

    @property
    def fault_detail(self) -> str:
        """Human-readable fault description (safe flags + dropped axes)."""
        parts = []
        if self.flag_names:
            parts.append("flags=" + ",".join(self.flag_names))
        if self.joint_fault:
            parts.append("断轴=" + ",".join(f"J{a + 1}" for a in self.fault_axes))
        return " ".join(parts) if parts else "无故障位"


_T = TypeVar("_T")


@dataclass(frozen=True)
class Msg(Generic[_T]):
    """One frame's value plus how that frame type has been arriving.

    ``value`` is the getter's own return value (``None`` when no frame was
    available), ``hz`` the average arrival rate of that frame type this
    session, ``timestamp`` the local time of its most recent frame.
    """
    value: _T
    hz: float
    timestamp: float


@dataclass
class CartPlan:
    """Result of a cartesian command.

    ``ok`` only means "planned and accepted" — never "the arm stopped on
    target".  Read ``settled`` (only meaningful with ``wait=True``) and, to
    know where the arm really is, read the TCP back.
    """
    ok: bool = False
    err: int = 0
    n_wp: int = 0
    plan_us: int = 0
    started_busy: bool = False
    settled: bool = False
    q_final: List[float] = field(default_factory=list)
    settle_err_rad: float = 0.0

    #: SDK-invented ``err`` slot meaning "outcome unknown" (see
    #: ``CartReplyLostError``); no producer today.
    ERR_REPLY_LOST = -1


# ── Errors ────────────────────────────────────────────────────────────────────

class LiteArmError(Exception):
    """Base class for every error this package (and litearm-core) raises."""


class NotConnectedError(LiteArmError):
    """An entry point that needs a live link was called before ``connect()``."""


class TransportError(LiteArmError):
    """The link itself failed: open/read/write/CRC, or nothing to talk to."""


class FirmwareMismatchError(LiteArmError):
    """The firmware does not match the expected version convention."""


class InvalidCommandError(LiteArmError):
    """Bad parameters: speed range, arity, pose shape, out-of-range values."""


class MotorFaultError(LiteArmError):
    """A fault / emergency / dropped axis was observed while moving."""


class MotionTimeoutError(LiteArmError):
    """The motion did not arrive, or no reply came, within the timeout."""


class IKError(LiteArmError):
    """Inverse kinematics found no solution for the requested pose."""


class CommandRejectedError(LiteArmError):
    """The firmware answered ``ERR{cmd,code}`` with a non-zero code."""

    def __init__(self, message: str = "", cmd: int = 0, code: int = 0) -> None:
        super().__init__(message)
        self.cmd = cmd
        self.code = code


class UnsupportedByFirmwareError(CommandRejectedError):
    """``ERR{cmd,0x00}`` — this firmware does not implement that command."""


class CartesianPlanError(LiteArmError):
    """A cartesian plan was rejected (error 2/3/4).

    Deliberately *not* a subclass of ``InvalidCommandError``.
    """


class MotionSupersededError(LiteArmError):
    """A cartesian plan was cancelled by another motion (``err=5``)."""


class CartReplyLostError(LiteArmError):
    """The cartesian result reply was lost — the outcome is unknown."""


class ArmIsInDfuError(LiteArmError):
    """The arm is in the ROM bootloader; a new ``Arm`` is required."""


# ── Pose helpers (ZYX intrinsic, matching the firmware convention) ─────────────

Mat3 = List[List[float]]
Vec3 = List[float]
Pose = Tuple[Vec3, Mat3]


def rpy_to_mat(rpy: Sequence[float]) -> Mat3:
    """``(roll, pitch, yaw)`` -> ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``."""
    r, p, y = (float(v) for v in rpy)
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]]


def mat_to_rpy(R: Sequence[Sequence[float]]) -> Vec3:
    """``R -> (roll, pitch, yaw)``, forcing ``yaw=0`` at gimbal lock."""
    p = math.atan2(-R[2][0], math.hypot(R[0][0], R[1][0]))
    if abs(abs(p) - math.pi / 2) < 1e-6:
        return [math.atan2(-R[1][2], R[1][1]), p, 0.0]
    return [math.atan2(R[2][1], R[2][2]), p, math.atan2(R[1][0], R[0][0])]


def is_rotation(R: Sequence[Sequence[float]], atol: float = 1e-5) -> bool:
    """True when ``R`` is a valid SO(3) member (orthonormal, ``det≈+1``)."""
    for i in range(3):
        for j in range(3):
            got = sum(R[k][i] * R[k][j] for k in range(3))
            if abs(got - (1.0 if i == j else 0.0)) > atol:
                return False
    det = (R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1])
           - R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0])
           + R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0]))
    return det > 0.999


def _is_sequence(x) -> bool:
    return isinstance(x, (list, tuple)) and not isinstance(x, (str, bytes))


def as_pose(pose) -> Pose:
    """Normalize a caller-supplied pose into ``(position[3], R[3x3])``.

    Accepts, in this order: a ``(pos[3], R[3x3])`` pair, six scalars
    ``[x, y, z, roll, pitch, yaw]``, or a 4x4 homogeneous matrix.  A flat
    9-element rotation matrix is rejected — a pose without a position is not a
    pose.  Anything else raises ``InvalidCommandError`` naming the shape that
    actually arrived.
    """
    if not _is_sequence(pose):
        raise InvalidCommandError(
            f"pose 需为 (pos[3], R[3x3]) 或 [x,y,z,r,p,y]; 收到 {type(pose).__name__}")

    items = list(pose)
    # Form 1: (pos[3], R[3x3])
    if len(items) == 2 and _is_sequence(items[0]) and _is_sequence(items[1]):
        p, R = items
        if len(list(p)) != 3:
            raise InvalidCommandError(f"pose 位置需 3 个分量, 收到 {len(list(p))}")
        Rl = [list(row) for row in R]
        if len(Rl) != 3 or any(len(row) != 3 for row in Rl):
            raise InvalidCommandError(
                f"pose 旋转需 3x3, 收到 {len(Rl)}x{len(Rl[0]) if Rl else 0}")
        Rf = [[float(v) for v in row] for row in Rl]
        if not is_rotation(Rf):
            raise InvalidCommandError("pose 旋转矩阵不是有效 SO(3) (正交且 det≈+1)")
        return [float(v) for v in p], Rf

    # Form 2: 6-vector xyz+rpy
    if len(items) == 6 and not any(_is_sequence(v) for v in items):
        v = [float(x) for x in items]
        return v[:3], rpy_to_mat(v[3:])

    # Form 3: 4x4 homogeneous
    if len(items) == 4 and all(_is_sequence(r) and len(list(r)) == 4 for r in items):
        M = [[float(v) for v in row] for row in items]
        Rf = [row[:3] for row in M[:3]]
        if not is_rotation(Rf):
            raise InvalidCommandError("pose 4x4 的旋转块不是有效 SO(3)")
        return [M[0][3], M[1][3], M[2][3]], Rf

    raise InvalidCommandError(
        f"pose 形状无法识别: len={len(items)}; 支持 (pos[3], R[3x3]) / "
        f"[x,y,z,r,p,y] / 4x4 齐次矩阵")
