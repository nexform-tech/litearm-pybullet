"""LiteArm SDK stub — routes to real SDK when available.

When litearm-python is installed (via pip install litearm-pybullet[mirror]),
this module re-exports it transparently.  Otherwise it provides only the
trajectory types needed for record_trajectory/play_trajectory.

The stub is intentional: PyBulletArm standalone mode does not require any
real-arm communication, so we avoid pulling in heavy dependencies (zenoh,
protobuf) for the basic simulation use case.
"""
from .types import JointTrajectory, TrajectoryFrame

# Try to re-export the real litearm SDK if available
try:
    import litearm as _real_litearm

    # Re-export everything from the real SDK
    Arm = _real_litearm.Arm
    DeviceManager = getattr(_real_litearm, "DeviceManager", None)
    __all__ = ["Arm", "JointTrajectory", "TrajectoryFrame"]
except ImportError:
    # Standalone mode — no real SDK available
    Arm = None  # type: ignore
    DeviceManager = None
    __all__ = ["JointTrajectory", "TrajectoryFrame"]