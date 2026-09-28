"""litearm-pybullet: LiteArm 七轴机械臂 PyBullet 仿真。

与 litearm-python API 完全兼容，支持三种模式：

1. 独立仿真：替换 litearm.Arm，无需连接真实机械臂
2. 镜像模式：仿真跟随真实机械臂状态同步运动
3. 双控模式：同时向真实机械臂和仿真发送指令

Usage::

    from litearm_pybullet import PyBulletArm, DualArm, MirrorMode

    # 模式 1: 独立仿真
    with PyBulletArm(render=True) as arm:
        arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)
        state = arm.get_state()

    # 模式 2: 双控
    dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
    dual.start()
    dual.movej([0.0]*7, speed=0.2)  # 实臂 + 仿真同时运动
    dual.close()

    # 模式 3: 镜像
    import litearm
    real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
    sim = PyBulletArm(render=True)
    sim.start()
    sim.mirror_from(real)  # 仿真跟随实臂
"""

__version__ = "0.1.0"

from ._compat import HAS_LITEARM_CORE, litearm_core
from .arm import PyBulletArm
from .mirror import DualArm, MirrorMode
from .trajectory import JointTrajectory, TrajectoryFrame

__all__ = [
    "__version__",
    "PyBulletArm",
    "DualArm",
    "MirrorMode",
    "JointTrajectory",
    "TrajectoryFrame",
    # The real-arm SDK, when it is installed — None otherwise. Its absence is
    # the normal case for a simulation-only install, which is why it is here as
    # a name that can be None rather than an ImportError.
    "HAS_LITEARM_CORE",
    "litearm_core",
]