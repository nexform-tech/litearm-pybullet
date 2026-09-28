"""The one place that decides where this package's SDK-shaped names come from.

Two modes:

* ``litearm-python`` is importable -> re-export *its* types, errors and pose
  helpers, so ``litearm_pybullet.Msg`` **is** ``litearm.arm.Msg`` and
  ``except litearm.MotionTimeoutError`` catches a simulation failure.
* It is not (the standalone case: no pyserial, no hardware) -> take the
  structurally identical local copies from :mod:`._fallback`.

Nothing else in the package may import ``litearm`` or ``_fallback``
directly; everything comes through here, so ``HAS_LITEARM`` tells the
truth about which implementation a process is running.

Why the local copies exist at all: ``PyBulletArm`` has to work with only
``pybullet`` + ``numpy`` installed.  That is the whole point of a simulator you
can run before the arm arrives.
"""
from __future__ import annotations

from typing import Any, Optional

__all__ = [
    "HAS_LITEARM", "SDK_IMPORT_ERROR", "litearm", "Arm",
    "JointState", "RobotState", "Msg", "CartPlan",
    "LiteArmError", "NotConnectedError", "TransportError", "FirmwareMismatchError",
    "InvalidCommandError", "MotorFaultError", "MotionTimeoutError", "IKError",
    "CommandRejectedError", "UnsupportedByFirmwareError", "CartesianPlanError",
    "MotionSupersededError", "CartReplyLostError", "ArmIsInDfuError",
    "ForkedSessionError", "TeleopBusyError", "TeleopLockedError",
    "MODE_NAMES", "FLAG_NAMES", "FLAG_ENABLED_BIT", "MAX_JOINTS",
    "CART_START_POS_TOL", "CART_START_RPY_TOL",
    "as_pose", "rpy_to_mat", "mat_to_rpy", "is_rotation",
]

#: True when the real SDK is being used, False when the local copies are.
HAS_LITEARM = False
#: Why the real SDK is not in play (``None`` when it is).  Kept for diagnostics:
#: "not installed" and "installed but broken" both land in the fallback, and a
#: silent fallback is exactly the kind of thing that wastes an afternoon.
SDK_IMPORT_ERROR: Optional[BaseException] = None

#: The SDK module itself, or ``None`` in the standalone case.  For code that
#: needs something outside this re-export list (e.g. ``litearm.testing``).
litearm: Any = None
#: ``litearm.Arm``, or ``None``.  Real-arm code must check ``HAS_LITEARM``.
Arm: Any = None

try:  # pragma: no cover - exercised on both branches across environments
    import litearm as _sdk
    from litearm import _protocol as _protocol
    from litearm._rot import as_pose, is_rotation, mat_to_rpy, rpy_to_mat
    from litearm.arm import Msg
    from litearm.cart import CartPlan
    from litearm.cart import (
        CART_START_POS_TOL as CART_START_POS_TOL,
        CART_START_RPY_TOL as CART_START_RPY_TOL,
    )
    from litearm.errors import (  # noqa: F401
        ArmIsInDfuError,
        CartesianPlanError,
        CartReplyLostError,
        CommandRejectedError,
        FirmwareMismatchError,
        ForkedSessionError,
        IKError,
        InvalidCommandError,
        LiteArmError,
        MotionSupersededError,
        MotionTimeoutError,
        MotorFaultError,
        NotConnectedError,
        TeleopBusyError,
        TeleopLockedError,
        TransportError,
        UnsupportedByFirmwareError,
    )
    from litearm.state import JointState, RobotState
except ImportError as _exc:
    SDK_IMPORT_ERROR = _exc
    from ._fallback import (  # noqa: F401
        CART_START_POS_TOL,
        CART_START_RPY_TOL,
        ArmIsInDfuError,
        CartesianPlanError,
        CartReplyLostError,
        CommandRejectedError,
        FirmwareMismatchError,
        ForkedSessionError,
        IKError,
        InvalidCommandError,
        JointState,
        LiteArmError,
        MotionSupersededError,
        MotionTimeoutError,
        MotorFaultError,
        Msg,
        NotConnectedError,
        RobotState,
        TeleopBusyError,
        TeleopLockedError,
        TransportError,
        UnsupportedByFirmwareError,
        as_pose,
        is_rotation,
        mat_to_rpy,
        rpy_to_mat,
    )
    from ._fallback import CartPlan  # noqa: F401
    from ._fallback import (
        FLAG_ENABLED_BIT,
        FLAG_NAMES,
        MAX_JOINTS,
        MODE_NAMES,
    )

    Arm = None
else:
    litearm = _sdk
    Arm = _sdk.Arm
    HAS_LITEARM = True
    # Mode/flag tables live in the SDK's private _protocol (they are not part of
    # its public surface), so they are copied from there rather than re-exported
    # by a public name.  tests/test_api_parity.py pins them against _fallback's.
    MODE_NAMES = _protocol.MODE_NAMES
    FLAG_NAMES = _protocol.FLAG_NAMES
    FLAG_ENABLED_BIT = _protocol.FLAG_ENABLED_BIT
    MAX_JOINTS = _protocol.MAX_JOINTS
