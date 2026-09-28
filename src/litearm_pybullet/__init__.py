"""litearm-pybullet: LiteArm 七轴机械臂 PyBullet 仿真。

仿真侧与 `litearm-core` 同形：31 个方法名、形参、默认值一一对齐，
读接口同样返回 `Msg` 信封（值在 `.value`），运动接口同样返回
`RobotState` / `CartPlan` 并以抛异常报错。所以真机代码里把
`litearm_core.Arm(port=...)` 换成 `PyBulletArm(render=True)`，其余不动。

此外还有 66 个 SDK 没有的能力（`fk`、`plan_*`、轨迹录制回放、手爪/设备
罐头接口等）标注为**仿真独有**，以及 7 个 0.1 时代的老名字
（`movel/movec/movep/get_tcp_pose/zero_gravity/request_stop/clear_stop`）
保留为纯转发别名并会发 DeprecationWarning。

支持三种模式：

1. 独立仿真：替换 litearm_core.Arm，无需连接真实机械臂
2. 镜像模式：仿真跟随真实机械臂状态同步运动
3. 双控模式：同时向真实机械臂和仿真发送指令

Usage::

    from litearm_pybullet import PyBulletArm, DualArm, MirrorMode

    # 模式 1: 独立仿真
    sim = PyBulletArm(render=True).connect()
    sim.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)
    state = sim.get_state().value

    # 模式 2: 双控 —— 真臂走 USB CDC，没有 server/endpoint 那一层
    from litearm_pybullet import litearm_core   # 装了 litearm-core 才不是 None
    dual = DualArm(real_port=None, render=True)  # None = 自动找 CDC 串口
    dual.start()
    dual.enable()                    # 使能是显式的：构造不会给实臂上电
    dual.movej([0.0] * 7, speed=0.2)  # 实臂 + 仿真同时运动
    dual.close()

    # 模式 3: 镜像
    real = litearm_core.Arm(port=None).connect()
    sim = PyBulletArm(render=True).connect()
    sim.mirror_from(real)            # 仿真跟随实臂
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