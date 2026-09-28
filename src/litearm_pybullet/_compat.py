"""The one place that decides where this package's SDK-shaped names come from.

Two modes:

* ``litearm-core`` is importable -> re-export *its* types, errors and pose
  helpers, so ``litearm_pybullet.Msg`` **is** ``litearm_core.arm.Msg`` and
  ``except litearm_core.MotionTimeoutError`` catches a simulation failure.
* It is not (the standalone case: no pyserial, no hardware) -> take the
  structurally identical local copies from :mod:`._fallback`.

Nothing else in the package may import ``litearm_core`` or ``_fallback``
directly; everything comes through here, so ``HAS_LITEARM_CORE`` tells the
truth about which implementation a process is running.

Why the local copies exist at all: ``PyBulletArm`` has to work with only
``pybullet`` + ``numpy`` installed.  That is the whole point of a simulator you
can run before the arm arrives.
"""
from __future__ import annotations

from typing import Any, Optional

__all__ = [
    "HAS_LITEARM_CORE", "SDK_IMPORT_ERROR", "litearm_core", "Arm",
    "JointState", "RobotState", "Msg", "CartPlan",
    "LiteArmError", "NotConnectedError", "TransportError", "FirmwareMismatchError",
    "InvalidCommandError", "MotorFaultError", "MotionTimeoutError", "IKError",
    "CommandRejectedError", "UnsupportedByFirmwareError", "CartesianPlanError",
    "MotionSupersededError", "CartReplyLostError", "ArmIsInDfuError",
    "MODE_NAMES", "FLAG_NAMES", "FLAG_ENABLED_BIT", "MAX_JOINTS",
    "as_pose", "rpy_to_mat", "mat_to_rpy", "is_rotation",
]

#: True when the real SDK is being used, False when the local copies are.
HAS_LITEARM_CORE = False
#: Why the real SDK is not in play (``None`` when it is).  Kept for diagnostics:
#: "not installed" and "installed but broken" both land in the fallback, and a
#: silent fallback is exactly the kind of thing that wastes an afternoon.
SDK_IMPORT_ERROR: Optional[BaseException] = None

#: The SDK module itself, or ``None`` in the standalone case.  For code that
#: needs something outside this re-export list (e.g. ``litearm_core.testing``).
litearm_core: Any = None
#: ``litearm_core.Arm``, or ``None``.  Real-arm code must check ``HAS_LITEARM_CORE``.
Arm: Any = None

try:  # pragma: no cover - exercised on both branches across environments
    import litearm_core as _sdk
    from litearm_core import _protocol as _protocol
    from litearm_core._rot import as_pose, is_rotation, mat_to_rpy, rpy_to_mat
    from litearm_core.arm import Msg
    from litearm_core.cart import CartPlan
    from litearm_core.errors import (  # noqa: F401
        ArmIsInDfuError,
        CartesianPlanError,
        CartReplyLostError,
        CommandRejectedError,
        FirmwareMismatchError,
        IKError,
        InvalidCommandError,
        LiteArmError,
        MotionSupersededError,
        MotionTimeoutError,
        MotorFaultError,
        NotConnectedError,
        TransportError,
        UnsupportedByFirmwareError,
    )
    from litearm_core.state import JointState, RobotState
except ImportError as _exc:
    SDK_IMPORT_ERROR = _exc
    from ._fallback import (  # noqa: F401
        ArmIsInDfuError,
        CartesianPlanError,
        CartReplyLostError,
        CommandRejectedError,
        FirmwareMismatchError,
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
    litearm_core = _sdk
    Arm = _sdk.Arm
    HAS_LITEARM_CORE = True
    # Mode/flag tables live in the SDK's private _protocol (they are not part of
    # its public surface), so they are copied from there rather than re-exported
    # by a public name.  tests/test_api_parity.py pins them against _fallback's.
    MODE_NAMES = _protocol.MODE_NAMES
    FLAG_NAMES = _protocol.FLAG_NAMES
    FLAG_ENABLED_BIT = _protocol.FLAG_ENABLED_BIT
    MAX_JOINTS = _protocol.MAX_JOINTS
