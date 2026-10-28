# litearm-pybullet 开发者指南

> 基于 litearm-pybullet v0.1.0，真机后端已迁移到 `litearm-python` 2.1.0。
> 如果你在移植针对老的 Zenoh 版 `litearm-python` 0.1.0 写的代码，先看
> [从 0.1 迁移](#从-01litearm-python-时代迁移) 那一节。

## 项目结构

```
litearm-pybullet/
├── pyproject.toml
├── setup.cfg
├── README.md
├── .gitignore
├── src/
│   └── litearm_pybullet/
│       ├── __init__.py         # 导出 PyBulletArm, DualArm, MirrorMode, HAS_LITEARM
│       ├── arm.py              # 核心：PyBulletArm 类（与 litearm.Arm 对齐）
│       ├── controller.py       # PID 关节控制器 + 梯形速度轨迹生成器
│       ├── kinematics.py       # 正逆运动学（FK/IK）+ 路径规划
│       ├── mirror.py           # DualArm（双控）+ MirrorMode（镜像）
│       ├── trajectory.py       # JointTrajectory / TrajectoryFrame（仿真独有）
│       ├── _compat.py          # 唯一决策点：装了真 SDK 就用真的
│       ├── _fallback.py        # 没装时的结构等价副本（对齐 2.1.0 的字段与类名）
│       └── assets/
│           ├── litearm.urdf    # 七轴机械臂 URDF 模型
│           └── meshes/         # 真实 STL 网格（9 个连杆）
├── examples/
│   ├── 01_hello_sim.py         # 独立仿真 + 读状态
│   ├── 02_movej_sim.py         # 仿真运动 + FK/IK
│   ├── 03_trajectory.py        # 轨迹录制与回放
│   ├── 04_mirror_real.py       # 镜像模式（仿真跟随实臂）
│   └── 05_dual_control.py      # 双控模式（同时控制实臂+仿真）
├── tests/
│   ├── __init__.py
│   ├── test_api_parity.py      # 对着真 SDK 钉住签名与字段
│   ├── test_compat_fallback.py
│   ├── test_mirror.py
│   ├── test_pybullet_arm.py
│   └── test_trajectory.py
└── docs/
    ├── design.md
    ├── DEVELOPER_GUIDE.md
    └── DEVELOPER_GUIDE_zh-CN.md
```

## 核心架构

```
┌──────────────────────────────────────────────────┐
│                 你的 Python 程序                   │
│                                                    │
│  arm = PyBulletArm(render=True)  ← 可替换为 litearm.Arm │
│  arm.movej(...) / arm.get_state() / arm.close()    │
└──────────┬──────────────────┬────────────────────┘
           │                  │
    ┌──────▼──────┐    ┌─────▼──────────────┐
    │ 独立仿真     │    │ 双控 / 镜像         │
    │             │    │                    │
    │ PyBullet    │    │ PyBullet +         │
    │ 物理引擎    │    │ litearm-python       │
    │ PID 控制器  │    │ (USB CDC)          │
    └─────────────┘    └────────────────────┘
```

没有 server 层：`litearm-python` 直连固件的 USB CDC 协议。整个技术栈里没有
`endpoint`、没有 `arm_id`、也没有 Zenoh。

### 模块职责

| 模块 | 职责 |
|------|------|
| `arm.py` | 主类 `PyBulletArm`，31 个方法与 `litearm.Arm` 1:1 对齐，后台线程运行物理仿真 |
| `controller.py` | `JointPIDController` 位置控制器 + `TrajectoryGenerator` 梯形速度轨迹生成 |
| `kinematics.py` | `Kinematics` 类：FK（正运动学）、IK（阻尼最小二乘 + Levenberg-Marquardt 精化）+ 路径规划（直线/圆弧/多航点） |
| `mirror.py` | `DualArm`（同时控制实臂+仿真）、`MirrorMode`（仿真跟随实臂） |
| `_compat.py` | 决定 `Msg`/`RobotState`/`CartPlan`/异常/位姿辅助函数从哪来——能 import 到真 SDK 就用真的，否则用 `_fallback.py` |
| `_fallback.py` | 本地结构等价物，类名与字段对齐 `litearm` 2.1.0，由 `tests/test_api_parity.py` 钉住 |
| `litearm.urdf` | URDF 模型：7 个铰链关节、真实 STL 网格几何、力矩电机 |

### 数据流

```
arm.movej(q_target, speed=0.5)
        │
        ▼
┌───────────────────┐
│ 轨迹生成器         │  ← TrajectoryGenerator.linear_trajectory()
│ 梯形速度剖面       │     生成关节空间路径
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ PID 控制器         │  ← JointPIDController.set_target(q_des)
│ 设置目标位置       │
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ 后台仿真线程       │  ← _sim_loop() @ 500Hz
│ compute() → tau   │    重力/科氏前馈 + PID 计算力矩 → stepSimulation() 推进物理
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ 到位置判定         │  ← _arrive()：|q−target| < q_tol 且 |dq| < dq_tol
│ 返回 RobotState    │     连续 arrive_frames 帧才算到；否则抛异常
└───────────────────┘
```

## 设计的核心原则

### 1. API 对齐

PyBulletArm 有 31 个方法与 `litearm.Arm` 同名、同形参、同默认值、同返回类型、
同异常。用户只需将 `litearm.Arm(port=...)` 替换为 `PyBulletArm(render=True)`
即可在仿真环境中运行相同的控制代码。

```
connect/disconnect/close, enable/disable,
movej, movej_sync, move_p, move_l, move_c, move_path, home, park, ik,
get_state, get_status_now, get_tcp,
set_speed, set_motion_mode, set_payload,
zero_g/zero_g_start/zero_g_stop/zero_g_active/zero_g_error,
emergency_stop, reset, clear_faults,
move_js, send_mit, send_mit_all
```

除此之外，58 个名字是**仿真独有**（SDK 里没有对应物：`fk`、`plan_*`、轨迹录制回放、
镜像、阻抗模式、手爪/设备代理、配置/日志/服务罐头接口、`n`/`firmware`/`port`），
另有 7 个是**已废弃的 0.1 老名**，每个都只转发到一个新方法。

`_compat.py` 的重点是**类型同一性**：装了 `litearm-python` 时，仿真 import 的是**它的**
`Msg`、`RobotState`、`CartPlan` 和异常类，所以 `except litearm.MotionTimeoutError`
也能接住仿真里的失败。没装时由 `_fallback.py` 提供结构完全一致的本地副本。
`litearm_pybullet.HAS_LITEARM` 告诉你当前是哪一种。

### 2. 三种操作模式

| 模式 | 类 | 创建方式 | 需要实臂 |
|------|-----|----------|----------|
| 独立仿真 | `PyBulletArm` | `PyBulletArm(render=True)` | 否 |
| 镜像模式 | `PyBulletArm` + `mirror_from()` | `sim.mirror_from(real)` | 是 |
| 双控模式 | `DualArm` | `DualArm(real_port=...)` | 是 |

### 3. 线程安全

- 仿真循环在后台线程运行，通过 `self._lock` 保护数据访问
- `get_state()` 返回状态缓存的快照，不会阻塞仿真循环
- 运动学计算使用实时的 PyBullet client（关节状态重置的作用域限定在 FK/IK 调用内）
- 镜像在它自己的守护线程里跑：读实臂、写关节位置。**镜像运行期间不要再给同一个
  实臂对象发命令**——litearm-python 会把另一个读者算作外来帧。先调 `stop_mirroring()`。

### 4. 零依赖独立模式

独立仿真模式只需 `pybullet>=3.2.5` 和 `numpy>=1.21`。镜像/双控模式需要
`litearm-pybullet[mirror]` extra，它对 `litearm-python` 的依赖是可选且**延迟加载**的
——不主动要求镜像时，仿真绝不会去 import 它。

## URDF 模型 (litearm.urdf)

- 7 个铰链关节，力矩电机驱动
- 运动学链（连杆位置/姿态、关节轴/限位）与真实 LiteArm URDF 完全对齐
- 零位 TCP = [0, 0, 0.814] m，与真机一致
- 质量/惯量/质心取自 URDF（总质量约 2.93 kg）
- 视觉/碰撞几何为真实 STL 网格（9 个 mesh，随包分发）
- 远端连杆添加最小惯量/质量填充（`_MIN_INERTIA=0.01`, `_MIN_MASS=0.5`），确保高增益 PD 控制稳定
- 电机使用 `TORQUE_CONTROL` 模式（直接力矩控制），通过 `VELOCITY_CONTROL + zero force` 初始化以禁用 PyBullet 内置电机控制

## 控制器参数

默认增益与真机 MIT 跟随模式对齐：

| 关节 | kp (Nm/rad) | kd (Nm·s/rad) |
|------|-------------|----------------|
| 1-2 | 260 | 5 |
| 3-4 | 150 | 4-5 |
| 5-7 | 50 | 2.5 |

可通过 `set_gains()` 运行时调整（仿真独有），通过 `get_gains()` 查询当前值。

到位置判定由四个构造参数决定，名字与 SDK 一致：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `q_tol` | `0.03` | 关节位置容差 [rad] |
| `dq_tol` | `0.10` | 关节速度容差 [rad/s] |
| `arrive_frames` | `3` | 连续多少帧都在两个容差内才算"到了" |
| `move_timeout` | `15.0` | 超过这么多秒抛 `MotionTimeoutError` |

## 环境要求与安装

### 环境要求

- Python >= 3.10
- pybullet >= 3.2.5
- numpy >= 1.21

### 可选依赖（镜像/双控模式）

- `litearm-python` —— **没有上 PyPI**。`[mirror]` extra 里写的就是这个名字，但它在
  PyPI 上解析不到，所以要从源码装：`pip install -e ../litearm-python`

### 安装

```bash
# 独立仿真模式（仅核心依赖）
pip install litearm-pybullet

# 镜像/双控模式（需要真机通信）
pip install -e ../litearm-python     # 先装这个：没上 PyPI
pip install "litearm-pybullet[mirror]"
```

从源码安装：

```bash
git clone https://gitee.com/xxx/litearm-pybullet.git
cd litearm-pybullet
pip install -e ".[dev]"
```

## 快速开始

### 独立仿真（无需真实机械臂）

```python
from litearm_pybullet import PyBulletArm

# 创建仿真机械臂并打开可视化窗口，connect() 返回 self
arm = PyBulletArm(render=True).connect()

# 查询状态：Msg 信封，值在 .value 里
state = arm.get_state().value
print("关节角:", state.q)
print("状态机:", state.mode_name)

# 关节空间运动：`speed` 是 0..1 的分数倍率，不是百分比
arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

# 笛卡尔直线运动：pose = xyz + rpy
x, y, z, r, p, yw = arm.get_tcp().value
arm.move_l([x, y, z - 0.1, r, p, yw], speed=0.2)

arm.close()
```

## 连接管理

PyBulletArm 支持三种连接管理方式：

### 方式一：显式 connect/close

```python
arm = PyBulletArm(render=True).connect()
try:
    arm.movej([0.0] * 7, speed=0.3)
finally:
    arm.close()
```

`connect(port=None)` 启动后台仿真线程并返回 `self`。`port` 只为兼容
`litearm.Arm(port=...)` 的签名而存在，仿真这边没有串口可开，会被忽略。
`connect()` 是幂等的。`start()` 是它的老名字，仍然可用，但不建议新代码再用。
`disconnect()` 是 `close()` 的别名，对齐 SDK 的命名。

### 方式二：上下文管理器（推荐）

```python
with PyBulletArm(render=True) as arm:
    arm.movej([0.0] * 7, speed=0.3)
    state = arm.get_state().value
```

### 方式三：自动启动（首次调用运动方法时自动启动）

```python
arm = PyBulletArm(render=True)
arm.movej([0.0] * 7, speed=0.3)  # 自动启动后台线程
arm.close()
```

注意：`connect()` 会启动后台仿真线程（500Hz）。`close()` 会停止线程、停止镜像、
断开 PyBullet 连接并关闭可视化窗口；第二次调用不做任何事（强幂等）。

## API 参考

以下所有方法均为 `PyBulletArm` 的实例方法。带 ⭐ 的与 `litearm.Arm` 1:1 对齐；
带 🔷 的是**仿真独有**；带 ⚠️ 的是**已废弃的 0.1 老名**（纯转发别名，各发一条
`DeprecationWarning`）。

### 运动控制

| 方法 | 说明 | 阻塞 |
|------|------|------|
| ⭐ `movej(q, speed=1.0)` | 关节空间点到点运动，各轴各自跑梯形速度剖面 | 是 |
| ⭐ `movej_sync(q, speed=1.0)` | 同上，但各轴共用一个时间基准（minimum jerk），同起同停 | 是 |
| ⭐ `move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03)` | 笛卡尔目标的关节空间点到点（先 IK 再 movej） | 是 |
| ⭐ `move_l(pose, speed=1.0, wait=True)` | 笛卡尔直线运动，返回 `CartPlan` | 是 |
| ⭐ `move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True)` | 笛卡尔圆弧运动，返回 `CartPlan` | 是 |
| ⭐ `move_path(poses, speed=1.0, wait=True)` | 笛卡尔多航点路径运动，返回 `CartPlan` | 是 |
| ⭐ `home(*, timeout=None)` | 回固件 home 位（低速 0.10），返回 `RobotState` | 是 |
| ⭐ `park()` | 放松到 park 位姿，返回 `None` | 是 |
| 🔷 `recover_joint_limits(speed=0.05, inset_rad=0.0)` | 缓慢将超限关节恢复到安全边界，返回 `RobotState` | 是 |
| 🔷 `replay_joint_path(q_path, speed=1.0, goto_start=True)` | 回放关节位置序列，返回 `RobotState` | 是 |
| 🔷 `replay_trajectory(traj_q, speed=1.0, goto_start=True)` | 回放 JointTrajectory 或路径，返回 `RobotState` | 是 |
| 🔷 `replay_timed_trajectory(traj_q, traj_t, speed=1.0)` | 按录制时间轴回放轨迹，返回 `RobotState` | 是 |
| 🔷 `play_trajectory(trajectory, speed=1.0)` | 从文件或对象加载并回放轨迹，返回 `RobotState` | 是 |
| 🔷 `hold(kp_scale=3.0)` | 保持当前位置（增加刚度），返回 `RobotState` | 是 |
| ⭐ `zero_g(period=0.04)` / `zero_g_start` / `zero_g_stop` | 零重力/自由拖拽模式；两种写法都可用 | 否 |
| 🔷 `joint_impedance(q_des, K, B)` | 关节空间阻抗控制（简化实现），返回 `RobotState` | 否 |
| 🔷 `cartesian_impedance(q_des, K_cart, B_cart)` | 笛卡尔空间阻抗控制（简化实现），返回 `RobotState` | 否 |
| 🔷 `joint_follow(K=None, B=None, speed_limit=None)` | 跟随外部目标源（兼容空实现），返回 `RobotState` | 否 |
| ⚠️ `movel(pose, speed)` | → `move_l(pose, speed)` | 是 |
| ⚠️ `movec(via, goal, speed)` | → `move_c(pose_start, via, goal, speed)`（别名用实测 TCP 补起点） | 是 |
| ⚠️ `movep(poses, speed)` | → `move_path(poses, speed)` | 是 |
| ⚠️ `zero_gravity()` | → `zero_g()` | 否 |

**参数说明：**

- `q`：7 元素关节角列表（rad）
- `pose`：见下面的[位姿格式](#位姿格式)一节
- `speed`：**0..1 的分数倍率**（不是百分比）。0.0 合法，表示"不动"。
  百分比是 `set_speed(percent)`，那是另一个旋钮（0..100 全局限速），两者相乘。
- **没有 `settle_s`**：到位置判定取代了它。返回值是 `RobotState`/`CartPlan` 而不是
  `bool`，失败抛异常（`MotionTimeoutError`、`IKError`、`MotorFaultError`、
  `CartesianPlanError`、`InvalidCommandError`）。
- `CartPlan.ok` 只说明"规划并下发成功"，**不代表"停在目标上"**。
  `ok=True, settled=False` 表示这次规划被后续命令顶掉了。

### 状态读取

三个读接口都返回 `Msg` 信封（`litearm-python` 2.0 起如此）：负载在 `.value`，
`.hz` 是实测帧到达频率，`.timestamp` 是最近一帧的本地 `time.monotonic()`。
还没有帧时 `value` 为 `None`。

| 方法 | 返回值 | 说明 |
|------|--------|------|
| ⭐ `get_state(refresh=False, timeout=0.5)` | `Msg[RobotState]` | 最近一帧状态 |
| ⭐ `get_status_now(timeout=0.5)` | `Msg[RobotState]` | 强制取一帧新的 |
| ⭐ `get_tcp(timeout=0.6)` | `Msg[(x,y,z,r,p,y)]` | 当前末端位姿 |
| 🔷 `n` / `firmware` / `port` | `int` / `str` / `None` | 关节数 / 固件串（`"PyBulletSim-7J"`，**不是**真实固件版本）/ 串口 |

关于 `refresh`：SDK 没有后台读线程，`refresh=False` 回的是**本调用方上次读到的那帧**，
可能已经陈旧。仿真这边总是有新鲜帧，两种取值都会返回它；但镜像/对齐场景下请统一
传 `refresh=True`，这样同一份代码在两种后端上语义一致。

**`RobotState` 字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `q` | `list[float]` | 7 个关节角（rad） |
| `dq` | `list[float]` | 7 个关节速度（rad/s） |
| `tau` | `list[float]` | 7 个关节力矩（Nm） |
| `mode` / `mode_name` | `int` / `str` | 状态机编号与名字（`"idle"` / `"moving"` / `"estop"` / `"zero_g"` …） |
| `enabled` / `faulted` / `cart_busy` | `bool` | 使能、故障、笛卡尔在途 |
| `errs` | `list[int]` | 误差码 |
| `temps` | `list[tuple]` | 电机温度 |
| `watchdog` | `dict` | 看门狗状态 |
| `robot_serial` | `str` | 序列号 |
| `config_checksum_sha256` | `str` | 配置校验和 |

### 急停/使能

| 方法 | 说明 |
|------|------|
| ⭐ `emergency_stop()` | 紧急停止：立即置零力矩，不是平滑减速 |
| ⭐ `reset()` | 清除急停、恢复默认限速，继续正常操作 |
| ⭐ `enable(attempts=12)` | 使能电机并保持当前姿态 |
| ⭐ `disable()` | 禁能电机（机械臂将在重力下坠落） |
| ⭐ `clear_faults()` | 清除电机故障，返回 `None`（与 SDK 一致；0.1 版本返回 `[]`） |
| ⚠️ `request_stop()` | → `emergency_stop()` |
| ⚠️ `clear_stop()` | → `reset()` |

零重力模式下**所有动作命令都会抛** `InvalidCommandError`（SDK 的守卫，仿真同样执行）；
`emergency_stop()`、`disable()`、`zero_g_stop()` 这些"卸力"命令仍然可用。

### 参数调优

| 方法 | 说明 |
|------|------|
| ⭐ `set_speed(percent)` | 设置 0..100 的全局限速（与运动方法上的 `speed=` 相乘） |
| ⭐ `set_motion_mode(mode)` | 按编号设置固件运动模式 |
| ⭐ `set_payload(mass, com=(0,0,0))` | 设置末端负载，**返回 `None`**；仿真中重力/惯量来自 URDF，不受影响 |
| ⭐ `move_js(q, dq=None, tau_ff=None)` | 流式关节命令（自定义控制器直写） |
| ⭐ `send_mit(idx, q, dq, kp, kd, tau)` / `send_mit_all(q, dq, kp, kd, tau)` | MIT 模式流式命令 |
| 🔷 `get_gains()` / `set_gains(kp=None, kd=None)` | 获取/设置 PD 增益（真机的增益在固件里） |
| 🔷 `get_payload()` | 获取末端负载信息 |
| 🔷 `set_installation(base_rpy, gravity)` / `get_installation()` | 设置/获取安装姿态与重力方向（仿真空操作） |
| 🔷 `set_joint_positions(q)` | 绕过控制器直接写入关节位置（镜像用它同步姿态；真机做不到） |

### 外设（模拟实现，仿真独有）

| 方法 | 说明 |
|------|------|
| `device(device_id)` | 获取模拟外设代理 |
| `devices` | 模拟外设管理器（支持 `arm.devices["hand_0"]` 语法） |
| `hand` | 向后兼容的手部属性，等价于 `arm.device("hand_0")` |
| `list_device_types()` | 列出支持的外设类型（sim_hand / sim_gripper） |
| `connect_device(category, subtype, device_id, ...)` | 连接模拟外设 |
| `disconnect_device(device_id)` | 断开模拟外设 |
| `get_active_device(device_id)` | 获取外设激活状态 |

模拟外设的 `_SimDevice` 代理支持 `open()`, `close()`, `set_gesture()`, `set_force()`, `get_state()`, `finger_move()`, `set_speed()`, `set_torque()`, `set_width()`, `get_width()`, `get_joints()`, `get_buttons()` 等方法，均返回固定值。

### 系统

| 方法 | 说明 |
|------|------|
| `get_system_stats()` | 系统状态（仿真返回固定值） |
| `get_logs(page, size, search)` | 日志查询（仿真返回空列表） |
| `restart_service()` | 重启服务（仿真返回 ok） |
| `get_joint_limits()` | 获取关节限位 |
| `set_joint_limits(limits)` | 设置关节限位（仿真空操作） |
| `get_zero_offsets()` | 获取零位偏移 |
| `set_zero_offsets(offsets)` | 设置零位偏移（仿真空操作） |
| `get_end_effector()` | 获取末端执行器配置 |
| `set_end_effector(config)` | 设置末端执行器（仿真空操作） |
| `get_cartesian_limits()` | 获取笛卡尔空间限位 |
| `set_cartesian_limits(limits)` | 设置笛卡尔空间限位（仿真空操作） |
| `get_collision_config()` | 获取碰撞检测配置 |
| `set_collision_config(config)` | 设置碰撞检测配置（仿真空操作） |

### 轨迹（仿真独有）

| 方法 | 说明 |
|------|------|
| `record_trajectory(output, duration_s, sample_rate_hz, ...)` | 录制仿真运动轨迹，返回 `JointTrajectory` |
| `start_recording()` | 开始录制（兼容空实现） |
| `stop_recording()` | 停止录制（兼容空实现） |
| `discard_recording()` | 丢弃录制（兼容空实现） |
| `get_recording_state()` | 录制状态查询（兼容空实现） |
| `get_playback_state()` | 回放状态查询（兼容空实现） |
| `list_trajectories()` | 轨迹列表（仿真返回空列表） |
| `save_trajectory(id, name, points, duration)` | 保存轨迹（兼容空实现） |
| `delete_trajectory(id)` | 删除轨迹（兼容空实现） |

`JointTrajectory` / `TrajectoryFrame` 是**仿真独有**的数据类型：`litearm-python` 里
根本没有"轨迹对象"这回事。

### 运动学计算（纯计算，不推进仿真）

| 方法 | 说明 |
|------|------|
| 🔷 `fk(q)` | 正运动学：关节角 → `(position, rotation_matrix)`。SDK 只报固件当前位姿的 FK |
| ⭐ `ik(pose, q_seed=None, timeout=3.0)` | 逆运动学：位姿 → `q[7]`，无解**抛 `IKError`**（不再返回 `ok` 标志） |
| 🔷 `plan_movel(q_start, pose_goal)` | 规划笛卡尔直线路径（吃 `(position, R)` 二元组） |
| 🔷 `plan_movec(q_start, pose_via, pose_goal)` | 规划笛卡尔圆弧路径 |
| 🔷 `plan_movep(q_start, poses_goal)` | 规划笛卡尔多航点路径 |

`ik()` 的 `q_seed` 默认取当前关节构型（真机也是从实测状态起解）。`timeout` 只为
签名兼容而存在：求解是本地的，只会失败，不会超时。

### 设备管理 / 遥操（仿真独有）

| 方法 | 说明 |
|------|------|
| `enter_teleop(mode, **params)` | 进入遥操模式（兼容空实现） |
| `exit_teleop()` | 退出遥操模式（兼容空实现） |
| `get_teleop_status()` | 遥操状态查询（兼容空实现） |

### 镜像模式

| 方法 | 说明 |
|------|------|
| `mirror_from(real_arm, rate_hz=50.0)` | 启动镜像：仿真跟随实臂关节状态 |
| `stop_mirroring()` | 停止镜像并 join 线程（最多 2 秒），幂等 |
| `mirroring` / `mirror_error` | 镜像线程是否在跑 / 失败原因（健康时为 `None`） |

**`rate_hz` 是真实负载，不是显示偏好**：每一帧都是一次串口往返，而 litearm-python
没有后台读线程，按 500 Hz 物理步逐帧请求会打死链路。默认 50 Hz 是刻意的。

镜像里读的是 `get_state(refresh=True)`——`refresh=False` 会一直回同一帧，
"镜像永远显示同一个姿态"正是那种你看不见的故障。镜像线程里的异常不会抛到你的
线程里，而是记在 `mirror_error` 上；掉一帧不会终止镜像。

镜像运行期间**不要**自己给同一个实臂对象发命令，两边会互相偷帧，先 `stop_mirroring()`。

## 三种模式

### 模式一：独立仿真

在无真实机械臂的情况下运行仿真，用于算法开发、测试和验证。

```python
from litearm_pybullet import PyBulletArm
from litearm_pybullet._compat import mat_to_rpy

with PyBulletArm(render=True) as arm:
    # 关节空间运动
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

    # 读取状态
    state = arm.get_state().value
    print(f"关节角: {[round(x, 3) for x in state.q]}")

    # 正逆运动学：ik 吃 6 向量，fk 返回 (pos, R) 二元组
    pos, R = arm.fk([0.0] * 7)
    q_sol = arm.ik(list(pos) + mat_to_rpy(R))

    # 笛卡尔直线运动
    arm.move_l([pos[0], pos[1], pos[2] - 0.1] + mat_to_rpy(R), speed=0.2)

    # 轨迹录制与回放
    traj = arm.record_trajectory(duration_s=3.0, sample_rate_hz=100.0)
    traj.save("trajectories/demo.json")
    arm.replay_trajectory(traj, speed=1.0)
```

### 模式二：镜像模式

仿真跟随真实机械臂的状态同步运动，用于可视化监控和数字孪生。

**前提条件：**

- 机械臂已通过 USB 接上（CDC 串口）
- 客户端已装 `litearm-python`（没上 PyPI，从源码装：`pip install -e ../litearm-python`）

```python
import time

import litearm as pa

from litearm_pybullet import PyBulletArm

# 连接真实机械臂（port=None 表示自动查找唯一的 STM32 CDC 设备）
real = pa.Arm(port=None).connect()

# 创建仿真
sim = PyBulletArm(render=True).connect()

# 开始镜像：仿真跟随实臂运动
sim.mirror_from(real, rate_hz=50.0)

# 在实臂上执行操作（拖动/运动），观察仿真同步
try:
    while True:
        r_msg = real.get_state(refresh=True)
        s_msg = sim.get_state()
        if r_msg.value is None:
            print(f"实臂无回帧 (mirror_error={sim.mirror_error})", end="\r")
            continue
        q_real, q_sim = r_msg.value.q, s_msg.value.q
        err = max(abs(q_real[i] - q_sim[i]) for i in range(min(len(q_real), len(q_sim))))
        print(f"镜像误差: {err:.4f} rad", end="\r")
        time.sleep(1)
except KeyboardInterrupt:
    pass
finally:
    sim.stop_mirroring()
    sim.close()
    real.close()
```

也可以使用 `MirrorMode` 类（等效封装）：

```python
import litearm as pa

from litearm_pybullet import PyBulletArm, MirrorMode

real = pa.Arm(port=None).connect()
sim = PyBulletArm(render=True).connect()

mirror = MirrorMode(real, sim, rate_hz=50.0)
mirror.start()

# ... 仿真跟随实臂 ...
print(mirror.running, mirror.error)

mirror.stop()
sim.close()
real.close()
```

### 模式三：双控模式

同时向真实机械臂和仿真发送相同的运动指令，两个臂同步运动。

**前提条件：** 同镜像模式。

```python
from litearm_pybullet import DualArm

dual = DualArm(
    real_port=None,     # None = 自动查找 CDC 串口
    render=True,
    mirror_first=True,  # 先同步实臂姿态到仿真，再开始运动
)
dual.start()

# 使能是**显式**的：构造函数不会给实体机械臂上电
dual.enable()

# 双控运动：返回 (实臂结果, 仿真结果)，两边都是 RobotState
real_state, sim_state = dual.movej(
    [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0],
    speed=0.2,
)
print(f"实臂: mode={real_state.mode_name}  仿真: mode={sim_state.mode_name}")

# 笛卡尔运动：返回 (CartPlan, CartPlan)
x, y, z, r, p, yw = dual.get_sim_tcp().value
dual.move_l([x, y, z - 0.1, r, p, yw], speed=0.2)

# 急停
dual.emergency_stop()

# 关闭
dual.close()
```

**构造参数：**

| 参数 | 说明 |
|------|------|
| `real_port` | 实臂串口；`None` 表示自动查找唯一的 STM32 CDC 设备 |
| `sim_model_path` | URDF 模型路径 |
| `render` | 是否打开 PyBullet 窗口（默认 `True`） |
| `mirror_first` | `True`（默认）时 `start()` 会等第一帧镜像到位才返回，仿真从实臂姿态开始；`False` 时立即返回，镜像在后台追赶 |
| `mirror_rate_hz` | 镜像轮询频率（默认 50.0） |

**属性与方法：**

| 名称 | 说明 |
|------|------|
| `dual.start()` | 启动仿真与镜像线程，返回 `self` |
| `dual.movej()` / `movej_sync()` / `move_p()` | 同时下发，返回 `(RobotState, RobotState)` |
| `dual.move_l()` / `move_c()` / `move_path()` | 同时下发，返回 `(CartPlan, CartPlan)` |
| `dual.get_real_state(refresh=True)` | 实臂状态 `Msg` |
| `dual.get_sim_state()` | 仿真状态 `Msg` |
| `dual.get_tcp()` / `dual.get_sim_tcp()` | 实臂 / 仿真的末端位姿 `Msg` |
| `dual.enable(attempts=12)` / `disable()` | 使能/禁能两边 |
| `dual.emergency_stop()` / `reset()` | 急停 / 清除急停（两边） |
| `dual.close()` | 关闭两边，幂等 |
| `dual.real` / `dual.sim` | 底层的两个臂对象 |
| `dual.mirroring` / `dual.mirror_error` | 镜像线程状态 |

两边**都会尝试**：实臂抛异常时仿真命令照样执行，而你看到的是实臂那个异常。
实臂永远不会被隐式使能——`enable()` 是 review 时能指出来的那一行。

## 位姿格式

`get_tcp()` 和所有运动方法都用 SDK 的 6 元素形式：

```python
pose = [x, y, z, roll, pitch, yaw]   # 米 与 弧度
```

由 SDK 同一个归一化函数（`as_pose`）处理的三种写法：

```python
[x, y, z, roll, pitch, yaw]              # 固件形式 —— get_tcp() 返回的就是它
([x, y, z], [[r00, r01, r02],            # 位置 + 3x3 行主序旋转矩阵
             [r10, r11, r12],
             [r20, r21, r22]])
[[r00, r01, r02, x],                     # 4x4 齐次矩阵
 [r10, r11, r12, y],
 [r20, r21, r22, z],
 [0,   0,   0,   1]]
```

**扁平 9 元素旋转矩阵不接受**——它没有位置。任何不合法的写法都会抛
`InvalidCommandError`，并在消息里给出**实际收到的形状**。

`fk()` 返回的是 `(position, R)` 二元组，`plan_*` 吃的也是同一个二元组——它们是仿真
自己的接口，不是 SDK 的。

```python
pos, R = arm.fk([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0])
path = arm.plan_movel([0.0]*7, (pos, R))
```

万向锁附近 roll 与 yaw 不唯一。固件和本包都在那里强制 `yaw = 0`，但两侧的**锁带
宽度不同**，所以 rpy 分量可以差很远却描述同一个旋转。要自己校验位姿，请比较
**旋转**而不是分量。

## 安全注意事项

1. **使能是显式的**：`PyBulletArm` 和 `DualArm` 都不会隐式给实体机械臂上电。
   `DualArm.start()` 只起仿真和镜像线程；让实臂动起来的是你写的 `dual.enable()`，
   review 时看得见。

2. **仿真无物理碰撞检测**：PyBulletArm 不会对工作空间边界、桌面、障碍物进行碰撞检测。请确保运动路径在安全范围内。

3. **关节限位保护**：代码内置了关节限位（`Q_MIN` / `Q_MAX`），IK 求解会裁剪到限位内。但直接传入的 `movej` 目标值不会自动裁剪，请确保目标值在限位范围内。

4. **禁能后重力坠落**：调用 `disable()` 后机械臂将在重力下坠落。在 PyBullet 窗口中观察，不要在实际环境中混淆。

5. **双控/镜像模式的安全**：在双控模式下，仿真和实臂同时运动。确保实臂周围无障碍物，运动范围安全。建议先用低速（`speed=0.1`）测试。

6. **急停机制**：`emergency_stop()` 会立即切断所有电机力矩（置零力矩），是急停而不是平滑减速。`reset()` 之后需重新调用 `enable()` 恢复控制。零重力模式下这三个"卸力"命令仍然可用。

7. **运动是阻塞的**：所有运动方法（`movej`、`movej_sync`、`move_p`、`move_l`、`move_c`、`move_path`、`replay_*`）都阻塞调用线程直到到位或失败。"到位"是指关节在 `q_tol`/`dq_tol` 内连续保持 `arrive_frames` 帧；始终做不到就抛 `MotionTimeoutError`，而不是返回一个假值。

8. **仿真与实臂差异**：仿真使用理想化 PID 控制器，不考虑摩擦、关节柔度、传感器噪声、电机饱和等物理效应。仿真中验证通过的轨迹在实际机械臂上仍需谨慎测试。

9. **控制器参数**：默认 PD 增益与真机对齐，但仿真中的动力学响应可能与真机有所不同。如需精确复现真机行为，可通过 `set_gains()` 调整增益。

10. **仿真不是实时安全的**：仿真循环以固定的 500Hz 步长运行，但实时同步是近似的。不要把仿真时序用于安全攸关的实时保证。

## 常见问题

### Q: 导入时报错 `ModuleNotFoundError: No module named 'litearm_pybullet'`

A: 确保已安装 litearm-pybullet：

```bash
pip install litearm-pybullet
```

如果是开发模式，使用可编辑安装：

```bash
pip install -e .
```

### Q: URDF 模型找不到

A: 确保 URDF 文件在正确位置。PyBulletArm 会按以下顺序查找：
1. 显式传入的 `model_path` 参数
2. 包内 `src/litearm_pybullet/assets/litearm.urdf`
3. 当前工作目录下的 `src/litearm_pybullet/assets/litearm.urdf`
4. 当前工作目录下的 `assets/litearm.urdf`

如果以上路径均不存在，请显式传入 `model_path` 参数。

### Q: PyBullet 窗口没有显示

A: 确保 `render=True` 在构造时传入。如果已传入但窗口仍不显示，检查：
- 是否在无图形界面的服务器上运行（需要 X11 转发或虚拟显示）
- 是否安装了 PyBullet 的 GUI 支持

### Q: 仿真运动速度不对

A: 检查以下几点：
- `speed` 参数取值范围 0..1（分数倍率，不是百分比），默认 1.0 对应 `max_velocity=3.0 rad/s`
- 是否动了全局限速：`set_speed(percent)` 是 0..100 的另一个旋钮，会和 `speed` 相乘
- 构造时可传入 `max_velocity` 调整最大速度
- 物理时间步长 `dt` 默认为 0.002s（500Hz），过大的步长会导致仿真不稳定

### Q: 仿真机械臂抖动或发散

A: 可能原因：
- PD 增益过高导致数值不稳定，尝试降低增益：`arm.set_gains(kp=[100]*7, kd=[3]*7)`
- 物理时间步长过大，尝试减小 `dt`：`PyBulletArm(render=True, dt=0.001)`
- 目标位置超出关节限位，检查目标值是否在 `Q_MIN` ~ `Q_MAX` 范围内

### Q: IK 求解失败

A: `ik()` 现在**抛 `IKError`**，不再返回 `(q, False)`。可能原因：
- 目标位姿在机械臂工作空间之外
- 目标位姿在奇异点附近
- 尝试传入不同的 `q_seed` 初始猜测值

装了 `litearm-python` 时 `IKError` 继承它的异常体系，所以一条 `except` 能同时覆盖
仿真与实机。

### Q: `movej` 抛了 `MotionTimeoutError`

A: 在 `move_timeout` 秒内，关节始终没有做到"位置误差小于 `q_tol` 且速度小于
`dq_tol`，连续 `arrive_frames` 帧"。常见原因：目标超出关节限位、臂被顶住了、
或者 `speed` 太低没跑完。这里**没有 `settle_s` 可以调**——请调构造参数里的
`move_timeout` 或两个容差。

### Q: 双控/镜像模式报错说没装 litearm-python

A: `litearm-python` 没有上 PyPI，要从源码装：

```bash
pip install -e ../litearm-python
```

独立仿真不需要它；`litearm_pybullet.HAS_LITEARM` 能告诉你当前有没有 import 到。

### Q: 轨迹录制不包含实际运动数据

A: `record_trajectory()` 在录制期间会关掉位置环，使机械臂在自由状态下运动。录制的是自由运动轨迹，而非受控运动。如需录制受控运动的轨迹，请在录制前调用 `movej()` 等运动方法，或在录制时手动控制。

### Q: 仿真中的外设（夹爪/手）不工作

A: 外设 API 为模拟实现，所有方法返回固定值（`open()` 返回 `True`，`get_state()` 返回 `{"connected": True}` 等）。如需真实外设控制，请使用真实机械臂。

### Q: 仿真看起来卡住了

A: 确认调过 `connect()`（或用了上下文管理器）。检查是否调过 `emergency_stop()` 或
`disable()`。如果用 `movej`，看 `get_state().value.mode_name` 确认仿真循环在跑。
如果在镜像，先看 `mirror_error`——串口断了是记在那里，而不是抛异常。

## 依赖

### 核心依赖

- `pybullet>=3.2.5`
- `numpy>=1.21`

### 可选依赖（镜像/双控模式）

- `litearm-python`（USB CDC 直连固件；**没上 PyPI**，用 `pip install -e ../litearm-python`）

## 已知限制

- 控制器为关节空间 PD + 重力/科氏前馈，未包含完整计算力矩前馈与摩擦前馈
- 仿真没有串口链路，`connect(port=...)` 会忽略它的参数
- 灵巧手/夹爪/示教板为模拟代理，返回固定值
- 系统管理/日志/遥操为 API 兼容空实现
- 无碰撞检测（PyBullet 内置碰撞检测未启用）
- 轨迹录制会关掉位置环以便手动拖动，与真机的录制行为不同
- 笛卡尔运动（`move_l`/`move_c`/`move_path`）在接近奇异构型时可能失败
- 仿真不包含传感器噪声、电机饱和、关节柔度等物理效应
- 仿真不是实时安全的，时序是近似的

## 从 0.1（litearm-python 时代）迁移

`litearm-python` 这个名字底下是两个只共享 `import` 一行的 SDK：0.1.0 是 Zenoh
客户端，对的是一个独立的 `litearm-server` 进程，用 `endpoint=`/`arm_id=` 寻址；
2.1 是薄客户端，直接走 USB CDC 说固件协议，构造参数是 `port=`。本仓真机后端
用的是 2.1。

| 0.1（server 时代） | 2.1（USB CDC） |
|---|---|
| `litearm-python` 0.1.0，`import litearm` | `litearm-python` 2.1，还是 `import litearm` |
| `litearm.Arm(endpoint="tcp/host:7447", arm_id="armA")` | `litearm.Arm(port="/dev/ttyACM0")` |
| `DualArm(real_endpoint=..., real_arm_id=...)` | `DualArm(real_port=...)` |
| `arm.start()` | `arm.connect()`（别名还在，但不建议继续用） |
| `state = arm.get_state(); state["q"]` | `arm.get_state().value.q` |
| `pos, R = arm.get_tcp_pose()` | `tcp = arm.get_tcp().value`（是 rpy，不是矩阵） |
| `q, ok = arm.ik(pos, R)` | `q = arm.ik(pose)`——失败抛 `IKError`，不再返回 `ok=False` |
| `arm.movej(q, speed, settle_s=2.0)` | `arm.movej(q, speed)`——返回 `RobotState`，等到位才回 |
| `arm.movel(pose, speed)` | `arm.move_l(pose, speed)` → `CartPlan` |
| `arm.movec(via, goal, speed)` | `arm.move_c(pose_start, via, goal, speed)` |
| `arm.movep(poses, speed)` | `arm.move_path(poses, speed)` |
| `arm.request_stop()` / `clear_stop()` | `arm.emergency_stop()` / `arm.reset()` |
| `arm.zero_gravity()` | `arm.zero_g()` |

这七个老名字仍然存在，是纯转发别名（各发一条 `DeprecationWarning`），所以 0.1 的
代码跑得起来——但应该改掉。

**`settle_s` 已删除**，运动方法返回 `RobotState`/`CartPlan` 而不是 `bool`，失败一律
抛异常（`MotionTimeoutError`、`IKError`、`InvalidCommandError` 等）。到位置判定是
`|q − target| < q_tol (0.03)` 且 `|dq| < dq_tol (0.10)` 连续 `arrive_frames (3)` 帧。

## License

Proprietary
