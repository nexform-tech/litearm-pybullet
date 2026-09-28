"""Tests for the litearm-core compatibility layer (_compat + _fallback).

These run in both environments — with the real SDK installed and without.  The
point of `_compat` is that the two are interchangeable, so every assertion below
is about the *public* surface it exposes, never about which module won.  Where
the two could drift (field names, signatures, constants) that is
`test_api_parity.py`'s job; here we pin behaviour.

Nothing here touches PyBullet or a serial port.
"""
from __future__ import annotations

import dataclasses

import pytest

from litearm_pybullet import _compat, _fallback


# ── The decision point ────────────────────────────────────────────────────────

def test_decision_point_is_self_consistent():
    """HAS_LITEARM_CORE must agree with what _compat actually hands out."""
    assert isinstance(_compat.HAS_LITEARM_CORE, bool)
    if _compat.HAS_LITEARM_CORE:
        assert _compat.SDK_IMPORT_ERROR is None
        assert _compat.litearm_core is not None
        assert _compat.Arm is _compat.litearm_core.Arm
        # The real classes, not the local copies.
        assert _compat.Msg is not _fallback.Msg
        assert _compat.MotionTimeoutError is not _fallback.MotionTimeoutError
    else:
        assert _compat.SDK_IMPORT_ERROR is not None
        assert _compat.litearm_core is None
        assert _compat.Arm is None
        assert _compat.Msg is _fallback.Msg
        assert _compat.RobotState is _fallback.RobotState


def test_compat_exports_are_complete():
    """Everything in __all__ resolves; nothing is left as a dangling alias."""
    for name in _compat.__all__:
        assert hasattr(_compat, name), f"_compat.{name} missing"


def test_fallback_needs_no_third_party():
    """The standalone path must import with stdlib only.

    `_fallback` is what makes "simulate before the arm arrives" true, so it may
    not quietly acquire a numpy/pyserial dependency — or start importing the
    real SDK, which would defeat the whole point of the fallback.
    """
    import ast

    with open(_fallback.__file__, encoding="utf-8") as fh:
        tree = ast.parse(fh.read())

    imported = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported.update(a.name.split(".")[0] for a in node.names)
        elif isinstance(node, ast.ImportFrom):
            # Relative imports (level > 0) are this package's own modules.
            if node.level == 0 and node.module:
                imported.add(node.module.split(".")[0])

    assert imported <= {"__future__", "math", "dataclasses", "typing"}, imported


# ── State types ───────────────────────────────────────────────────────────────

def test_joint_state_defaults():
    j = _compat.JointState()
    assert (j.q, j.dq, j.tau, j.t_mos, j.t_coil, j.err) == (0.0,) * 5 + (0,)
    assert _compat.JointState(q=1.0).q == 1.0


def test_robot_state_joint_accessors():
    st = _compat.RobotState(joints=[_compat.JointState(q=0.1, dq=0.2, tau=0.3),
                                    _compat.JointState(q=0.4, dq=0.5, tau=0.6)])
    assert st.n == 2
    assert st.q == pytest.approx([0.1, 0.4])
    assert st.dq == pytest.approx([0.2, 0.5])
    assert st.tau == pytest.approx([0.3, 0.6])
    assert _compat.RobotState().n == 0


def test_robot_state_flag_bits():
    """bit9 = enabled, bit10 = cartesian busy — read via properties only."""
    st = _compat.RobotState(flags=(1 << 9) | (1 << 10))
    assert st.enabled is True
    assert st.cart_busy is True
    assert st.flag_names == []          # neither bit is a *safety* flag
    assert _compat.RobotState(flags=0).enabled is False
    assert _compat.RobotState(flags=0).cart_busy is False


def test_robot_state_fault_axes_and_detail():
    st = _compat.RobotState(joint_fault=(1 << 1) | (1 << 3),
                            flag_names=["FAULT"], flags=1)
    assert st.fault_axes == [1, 3]
    assert st.faulted is True
    assert st.drop_hold_inferred is True
    assert st.fault_detail == "flags=FAULT 断轴=J2,J4"


def test_robot_state_clean_is_not_faulted():
    st = _compat.RobotState()
    assert st.faulted is False
    assert st.fault_axes == []
    assert st.drop_hold_inferred is False
    assert st.fault_detail == "无故障位"


def test_robot_state_emergency_mode_is_faulted():
    """mode 6 = EMERGENCY faults even with no flag bits set."""
    assert _compat.RobotState(mode=6, mode_name="EMERGENCY").faulted is True


def test_msg_envelope_is_frozen():
    m = _compat.Msg(value=7, hz=100.0, timestamp=12.5)
    assert (m.value, m.hz, m.timestamp) == (7, 100.0, 12.5)
    with pytest.raises(dataclasses.FrozenInstanceError):
        m.value = 8
    # The "no frame arrived" shape every reading getter can return.
    assert _compat.Msg(None, 0.0, 0.0).value is None


def test_cart_plan_defaults():
    p = _compat.CartPlan()
    assert (p.ok, p.err, p.n_wp, p.plan_us) == (False, 0, 0, 0)
    assert (p.started_busy, p.settled) == (False, False)
    assert p.q_final == []
    assert p.settle_err_rad == 0.0
    assert _compat.CartPlan.ERR_REPLY_LOST == -1
    # Mutable default must not be shared between instances.
    a, b = _compat.CartPlan(), _compat.CartPlan()
    a.q_final.append(0.0)
    assert b.q_final == []


# ── Error hierarchy ───────────────────────────────────────────────────────────

ALL_ERRORS = [
    "NotConnectedError", "TransportError", "FirmwareMismatchError",
    "InvalidCommandError", "MotorFaultError", "MotionTimeoutError", "IKError",
    "CommandRejectedError", "UnsupportedByFirmwareError", "CartesianPlanError",
    "MotionSupersededError", "CartReplyLostError", "ArmIsInDfuError",
]


def test_every_error_derives_from_lite_arm_error():
    """One `except` clause has to be able to catch the whole family."""
    assert issubclass(_compat.LiteArmError, Exception)
    for name in ALL_ERRORS:
        cls = getattr(_compat, name)
        assert issubclass(cls, _compat.LiteArmError), name


def test_error_separation_that_matters():
    """The distinctions callers branch on must survive the trip.

    A superseded cartesian plan is normal operation, not a failure, and a plan
    rejection is not a bad argument — collapsing either into a parent class
    turns a normal takeover into an error path.
    """
    assert issubclass(_compat.UnsupportedByFirmwareError, _compat.CommandRejectedError)
    assert not issubclass(_compat.CartesianPlanError, _compat.CommandRejectedError)
    assert not issubclass(_compat.CartesianPlanError, _compat.InvalidCommandError)
    assert not issubclass(_compat.MotionSupersededError, _compat.CartesianPlanError)
    assert not issubclass(_compat.ArmIsInDfuError, _compat.NotConnectedError)


def test_command_rejected_carries_cmd_and_code():
    e = _compat.CommandRejectedError("nope", cmd=0x01, code=0x03)
    assert (e.cmd, e.code) == (0x01, 0x03)
    assert "nope" in str(e)
    # Defaults keep `raise X("msg")` working.
    assert (_compat.CommandRejectedError("x").cmd,
            _compat.CommandRejectedError("x").code) == (0, 0)


# ── Pose helpers ──────────────────────────────────────────────────────────────

def test_rpy_round_trip():
    for rpy in ([0.0, 0.0, 0.0], [0.3, -0.7, 1.1], [0.0, 0.4, 0.0]):
        R = _compat.rpy_to_mat(rpy)
        assert _compat.is_rotation(R)
        assert _compat.mat_to_rpy(R) == pytest.approx(rpy, abs=1e-9)


def test_rpy_to_mat_is_zyx_intrinsic():
    """R = Rz @ Ry @ Rx — getting this backwards silently mirrors every motion."""
    import math
    R = _compat.rpy_to_mat([0.0, 0.0, math.pi / 2])
    # +90 deg about Z maps x -> y.
    assert R[0] == pytest.approx([0.0, -1.0, 0.0], abs=1e-12)
    assert R[1] == pytest.approx([1.0, 0.0, 0.0], abs=1e-12)


def test_mat_to_rpy_forces_yaw_zero_at_gimbal_lock():
    import math
    R = _compat.rpy_to_mat([0.4, math.pi / 2, 0.0])
    r, p, y = _compat.mat_to_rpy(R)
    assert p == pytest.approx(math.pi / 2)
    assert y == 0.0
    assert _compat.mat_to_rpy(R) == pytest.approx([r, p, 0.0])


def test_is_rotation_rejects_bad_matrices():
    assert _compat.is_rotation([[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) is True
    # Scaled -> not orthonormal.
    assert _compat.is_rotation([[2.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) is False
    # Reflection -> det = -1.
    assert _compat.is_rotation([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]) is False


def _assert_mat_close(R, expected, abs_=1e-9):
    assert len(R) == len(expected)
    for row, exp_row in zip(R, expected):
        assert row == pytest.approx(exp_row, abs=abs_)


IDENTITY = [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]


def test_as_pose_accepts_three_forms():
    pos, R = _compat.as_pose(([1.0, 2.0, 3.0], IDENTITY))
    assert pos == [1.0, 2.0, 3.0]
    assert R == IDENTITY

    pos6, R6 = _compat.as_pose([1.0, 2.0, 3.0, 0.0, 0.0, 0.0])
    assert pos6 == [1.0, 2.0, 3.0]
    _assert_mat_close(R6, IDENTITY)

    posM, RM = _compat.as_pose([[1.0, 0, 0, 4.0], [0, 1.0, 0, 5.0],
                                [0, 0, 1.0, 6.0], [0, 0, 0, 1.0]])
    assert posM == [4.0, 5.0, 6.0]
    assert RM == IDENTITY


def test_as_pose_normalizes_to_plain_lists():
    """Downstream code indexes R[0][0]; numpy rows would break `==` comparisons."""
    pos, R = _compat.as_pose([[1.0, 0, 0, 4.0], [0, 1.0, 0, 5.0],
                              [0, 0, 1.0, 6.0], [0, 0, 0, 1.0]])
    assert isinstance(pos, list) and isinstance(R, list)
    assert all(isinstance(row, list) for row in R)


def test_as_pose_six_vector_becomes_rotation():
    """The 6-vector form is xyz+rpy, not a flat rotation matrix."""
    _, R = _compat.as_pose([0.0, 0.0, 0.0, 0.0, 0.0, 1.5707963267948966])
    assert R[0][0] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("bad", [
    [1.0, 0, 0, 0, 1.0, 0, 0, 0, 1.0],          # flat 9: a rotation, but no position
    [1.0, 2.0, 3.0],                             # bare position
    [1.0, 2.0, 3.0, 4.0, 5.0],                   # 5 components
    ([1.0, 2.0], [[1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]),  # short position
    ([1.0, 2.0, 3.0], [[1.0, 0], [0, 1.0]]),     # non-3x3 rotation
    "not a pose",
    None,
])
def test_as_pose_rejects_bad_shapes(bad):
    with pytest.raises(_compat.InvalidCommandError):
        _compat.as_pose(bad)


def test_as_pose_rejects_non_so3_rotation():
    with pytest.raises(_compat.InvalidCommandError):
        _compat.as_pose(([1.0, 2.0, 3.0],
                         [[2.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0]]))


def test_as_pose_error_names_the_shape():
    """`pose 非法` alone leaves the caller guessing between the three forms."""
    with pytest.raises(_compat.InvalidCommandError) as exc:
        _compat.as_pose([1.0, 2.0, 3.0])
    assert "len=3" in str(exc.value)


# ── Protocol tables ───────────────────────────────────────────────────────────

def test_cart_start_tolerances():
    """move_c's start check: the numbers are part of the contract, not a knob."""
    assert _compat.CART_START_POS_TOL == 0.006
    assert _compat.CART_START_RPY_TOL == 0.03


def test_mode_and_flag_tables():
    assert _compat.MODE_NAMES[0] == "INIT"
    assert _compat.MODE_NAMES[6] == "EMERGENCY"
    assert _compat.MODE_NAMES[7] == "ZERO_G"
    # Only the six safety flags are named: bit9/bit10 are properties, never faults.
    assert sorted(_compat.FLAG_NAMES) == [0, 1, 2, 3, 4, 5]
    assert _compat.FLAG_NAMES[0] == "FAULT"
    assert _compat.FLAG_ENABLED_BIT == 9
    assert _compat.MAX_JOINTS == 16
