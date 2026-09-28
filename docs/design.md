# litearm-pybullet 设计文档

## 项目定位

`litearm-pybullet` 是 LiteArm 七轴机械臂的 PyBullet 仿真，API 与 `litearm-core` SDK 逐项对齐（31 个同名方法、同形参、同返回类型），支持三种模式：

| 模式 | 说明 | 需要真机 |
|------|------|----------|
| 独立仿真 | `PyBulletArm` 直接替换 `litearm_core.Arm`，无需硬件 | ❌ |
| 镜像模式 | 仿真跟随真实机械臂状态同步运动 | ✅ |
| 双控模式 | 同时向真实机械臂和仿真发送指令 | ✅ |

## 架构

```
┌─────────────────────────────────────────────────┐
│                  你的 Python 程序                │
│                                                   │
│   arm = PyBulletArm()  ← 可替换为 litearm_core.Arm │
│   arm.movej(...)                                  │
│   arm.get_state()                                 │
└──────────┬────────────────────┬─────────────────┘
           │                    │
    ┌──────▼──────┐      ┌─────▼──────────┐
    │ 独立仿真     │      │ 双控 / 镜像     │
    │             │      │                │
    │ PyBullet    │      │ PyBullet +     │
    │ 物理引擎    │      │ litearm-core   │
    │             │      │ (USB CDC)      │
    │ PID 控制器  │      │                │
    │ FK/IK      │      │ 实臂 + 仿真    │
    │             │      │ 同时运动        │
    └─────────────┘      └────────────────┘
```

## 代码结构

与 `litearm-mujoco` 完全对齐：

```
litearm-pybullet/
├── pyproject.toml
├── README.md
├── .gitignore
├── .markdownlint.json
├── docs/
│   ├── design.md
│   ├── DEVELOPER_GUIDE.md
│   └── DEVELOPER_GUIDE_zh-CN.md
├── examples/
│   ├── 01_hello_sim.py
│   ├── 02_movej_sim.py
│   ├── 03_trajectory.py
│   ├── 04_mirror_real.py
│   └── 05_dual_control.py
├── src/
│   └── litearm_pybullet/
│       ├── __init__.py
│       ├── arm.py              # PyBulletArm 主类
│       ├── controller.py       # JointPIDController + TrajectoryGenerator
│       ├── kinematics.py       # PyBullet 内置 FK/IK + plan_*
│       ├── mirror.py           # DualArm + MirrorMode
│       ├── trajectory.py       # JointTrajectory / TrajectoryFrame（仿真独有）
│       ├── _compat.py          # 唯一决策点：装了 litearm_core 就用真的
│       ├── _fallback.py        # 没装时的结构等价副本（字段/签名对齐 2.1.0）
│       └── assets/
│           ├── litearm.urdf
│           └── meshes/
└── tests/
    ├── test_api_parity.py      # 对着真 SDK 钉住签名与字段
    ├── test_compat_fallback.py
    ├── test_mirror.py
    ├── test_pybullet_arm.py
    └── test_trajectory.py
```

## 模块职责

### arm.py — PyBulletArm

主类，31 个方法与 `litearm_core.Arm` 同名同形。核心能力：

- **生命周期**：`connect()`（`start()` 是裸转发别名）/ `close()` / `disconnect()` / context manager
- **状态读取**：`get_state(refresh=)` / `get_status_now()` / `get_tcp()`（都返回 `Msg` 信封）；仿真独有 `fk()` / `ik()`（`ik` 与 SDK 同形）
- **运动控制**：`movej()` / `movej_sync()` / `move_p()` / `move_l()` / `move_c()` / `move_path()` / `home()` / `park()`，返回 `RobotState` / `CartPlan`
- **急停/使能**：`emergency_stop()` / `reset()` / `clear_faults()` / `enable()` / `disable()`
- **参数**：`set_speed()` / `set_motion_mode()` / `set_payload()`；仿真独有 `set_gains()` / `get_gains()`
- **仿真设备**：`device()` / `hand` / `devices` 等模拟外设（仿真独有）
- **镜像模式**：`mirror_from()` / `stop_mirroring()`（仿真独有）
- **轨迹**：`record_trajectory()` / `play_trajectory()` 等（仿真独有）

内部使用 PyBullet DIRECT/GUI 客户端，后台物理线程以 500 Hz（`dt=0.002`）运行扭矩控制（PD + 前馈）。

### controller.py — PID 控制器 + 轨迹生成

与 litearm-mujoco 的 controller.py 完全对齐：

- `JointPIDController`：关节空间 PD + 前馈（重力/科氏补偿），对齐真机 MIT 模式增益
- `TrajectoryGenerator`：梯形速度曲线 + minimum jerk 轨迹

### kinematics.py — 运动学

纯计算，不依赖物理线程。复用 PyBullet 内置 `getLinkState`（FK）和 `calculateInverseKinematics`（IK 初值），加一步 LM 精修。

- `fk(q)` / `ik(pos, R, q_seed)`
- `plan_movel()` / `plan_movec()` / `plan_movep()`

### mirror.py — 双控 + 镜像

- `DualArm`：同时持有 `litearm_core.Arm` 和 `PyBulletArm`，运动命令同时发两边
- `MirrorMode`：后台线程持续拉真机状态 → 同步到仿真

镜像必须按 `rate_hz`（默认 50 Hz）**限频**，并且每次都用 `get_state(refresh=True)`：
litearm-core 没有后台读线程，`refresh=False` 回的只是本调用方上次读到的那帧，
而 500 Hz 每个物理步都发一次串口请求会打死链路。

### _compat.py / _fallback.py — 类型来源决策

`_compat.py` 是唯一决策点：能 `import litearm_core` 就 re-export 它的
`Msg` / `RobotState` / `CartPlan` / 异常 / `as_pose` / `rpy_to_mat` / `mat_to_rpy`，
否则从 `_fallback.py` 取结构等价副本。这样 `except litearm_core.MotionTimeoutError`
在纯仿真环境里也接得住——"PyBulletArm 可替换真臂"这句话才是真的。
事实由 `litearm_pybullet.HAS_LITEARM_CORE` 暴露，字段/签名由
`tests/test_api_parity.py` 在装了 SDK 的一侧钉住。

### trajectory.py — 轨迹类型

`JointTrajectory` / `TrajectoryFrame` 是**仿真独有**的数据类型（SDK 里没有轨迹对象），
由 `record_trajectory()` / `replay_trajectory()` 使用。

## 依赖策略

```
基础安装: pybullet>=3.2.5, numpy>=1.21
mirror extra: litearm-core  (真机通信，USB CDC)
```

`litearm-core` 还没上 PyPI。`[mirror]` extra 里写的就是这个名字，但它在 PyPI 上
解析不到，所以文档里要写清"从源码装"：`pip install -e ../litearm-core`。

## 物理引擎参数

- 仿真步长：2ms（500Hz），高增益 PD 稳定
- PD 增益：对齐真机（Kp: 260/260/150/150/50/50/50，Kd: 5/5/4/5/2.5/2.5/2.5）
- 关节限位：与真机一致
- 重力：9.81 m/s²
- URDF 惯量补丁：最小质量 0.5kg，最小惯量 0.01 kg·m²

## API 对齐对照

完整对照表见 [README](../README_zh-CN.md#api-对照)。这里只给三层的划分：

**1:1 对齐面（31 个）** —— 与 `litearm_core.Arm` 同名、同形参、同默认值、同返回类型、同异常：

```
connect/disconnect/close, enable/disable,
movej, movej_sync, move_p, move_l, move_c, move_path, home, park, ik,
get_state, get_status_now, get_tcp,
set_speed, set_motion_mode, set_payload,
zero_g/zero_g_start/zero_g_stop/zero_g_active/zero_g_error,
emergency_stop, reset, clear_faults,
move_js, send_mit, send_mit_all
```

**仿真独有面（58 个）** —— SDK 没有对应物：`fk`、`plan_movel/movec/movep`、
轨迹录制/回放（`record_trajectory`、`play_trajectory`、`replay_*`、`save/list/delete_trajectory`）、
镜像（`mirror_from`、`stop_mirroring`、`mirroring`、`mirror_error`）、
阻抗（`hold`、`joint_impedance`、`joint_follow`、`cartesian_impedance`、`set_gains`/`get_gains`）、
手爪/设备罐头（`device`、`hand`、`devices`、`enter_teleop`…）、
配置/日志/服务（`get_joint_limits`、`get_logs`、`restart_service`…）、`n` / `firmware` / `port`。

**已废弃老名（7 个）** —— 纯转发别名，各发一条 `DeprecationWarning`：
`movel→move_l`、`movec→move_c`、`movep→move_path`、`get_tcp_pose→get_tcp`、
`zero_gravity→zero_g`、`request_stop→emergency_stop`、`clear_stop→reset`。