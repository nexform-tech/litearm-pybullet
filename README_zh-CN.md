# litearm-pybullet

LiteArm 七轴机械臂 PyBullet 仿真。与 [`litearm-core`](https://github.com/nexform-tech/litearm-core)
SDK 逐项对齐——把 `litearm_core.Arm(port=...)` 换成 `PyBulletArm(render=True)`，
同一份控制代码在仿真和实机上都跑得起来。也支持仿真与实体机械臂同时运动。

## 特点

- 🔄 **与 litearm-core 1:1 对齐**：31 个方法名、形参、默认值、`Msg` 信封、
  `RobotState`/`CartPlan` 返回类型全部一致，`PyBulletArm` 可直接替换 `Arm`
- 🖥️ **三种模式**：独立仿真 / 镜像跟随 / 双控同步
- 🎮 **可视化**：PyBullet 原生渲染，实时观察机械臂运动
- 🧪 **无硬件测试**：不连真实机械臂也能开发和测试运动逻辑
- 🐍 **纯 Python**：零编译，pip install 即用

## 安装

```bash
# 独立仿真（无需真机通信）
pip install litearm-pybullet

# 镜像/双控模式（需要真机通信）
pip install "litearm-pybullet[mirror]"
```

或从源码安装：

```bash
git clone https://gitee.com/xxx/litearm-pybullet.git
cd litearm-pybullet
pip install -e ".[dev]"
```

**镜像/双控模式依赖 `litearm-core`，它还没有上 PyPI。** `[mirror]` extra 里写的就是
这个名字，但它在 PyPI 上解析不到，所以要先从源码装一次：

```bash
pip install -e ../litearm-core      # 同级目录的源码
pip install -e ".[mirror]"          # 或者干脆 PYTHONPATH=../litearm-core/src
```

独立仿真不需要它：`litearm_pybullet.HAS_LITEARM_CORE` 告诉你真 SDK 有没有 import 成功，
独立仿真永远不依赖它。

## 快速开始

### 模式 1：独立仿真

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:            # 等价于 connect()
    # 关节运动。`speed` 是 0..1 的分数倍率，不是百分比
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # 笛卡尔直线运动。pose = xyz + rpy
    x, y, z, r, p, yw = arm.get_tcp().value
    arm.move_l([x, y, z - 0.1, r, p, yw], speed=0.1)

    # 读取状态。Msg 信封，值在 .value 里
    state = arm.get_state().value
    print(state.q)
```

### 模式 2：镜像模式 — 仿真跟随实臂

```python
from litearm_pybullet import PyBulletArm, litearm_core   # 需要装 litearm-core

# 连接真实机械臂。走 USB CDC，没有 server / endpoint 这一层
real = litearm_core.Arm(port=None).connect()             # None = 自动查找

# 创建仿真并启动镜像
sim = PyBulletArm(render=True).connect()
sim.mirror_from(real)   # 仿真跟随实臂；出错不抛异常，记在 sim.mirror_error 里

# 此时在实臂上做任何操作，仿真都会实时跟随
```

### 模式 3：双控模式 — 同时控制

```python
from litearm_pybullet import DualArm

dual = DualArm(real_port=None, render=True)   # None = 自动查找 CDC 串口
dual.start()
dual.enable()   # 使能是显式的：构造函数不会给实体机械臂上电

# 一条命令，实臂和仿真同时运动！
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

dual.close()
```

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
    │ FK/IK      │      │  实臂 + 仿真   │
    │             │      │  同时运动      │
    └─────────────┘      └────────────────┘
```

## 示例

| 样例 | 说明 | 需要实臂 |
|------|------|----------|
| `01_hello_sim.py` | 创建仿真 + 读状态 | ❌ |
| `02_movej_sim.py` | 关节/笛卡尔运动 + FK/IK | ❌ |
| `03_trajectory.py` | 轨迹录制与回放 | ❌ |
| `04_mirror_real.py` | 仿真镜像跟随实臂 | ✅ |
| `05_dual_control.py` | 同时控制实臂和仿真 | ✅ |

```bash
python3 examples/01_hello_sim.py
python3 examples/02_movej_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --port /dev/ttyACM0
python3 examples/05_dual_control.py --port /dev/ttyACM0
```

## API 对照

分三层。第一层是承诺：名字、形参、返回类型、异常全一样——由
`tests/test_api_parity.py` 对着真实安装的 SDK 钉住。第二层是仿真独有的能力，
SDK 里没有对应物。第三层是 0.1 时代的老名字，保留给老调用方，正在退场。

### 与 `litearm_core.Arm` 1:1（31 个）

| `litearm_core.Arm` | `PyBulletArm` | 说明 |
|---|---|---|
| `Arm(port=...)` | `PyBulletArm(render=True)` | 构造；仿真这边多一个 `render`，连接走 `connect(port=None)` |
| `connect(port=None)` / `disconnect()` / `close()` | 同 | |
| `enable(attempts=12)` / `disable()` | 同 | |
| `movej(q, speed=1.0)` | 同 | → `RobotState`，超时抛异常 |
| `movej_sync(q, speed=1.0)` | 同 | → `RobotState` |
| `move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03)` | 同 | → `RobotState` |
| `move_l(pose, speed=1.0, wait=True)` | 同 | → `CartPlan` |
| `move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True)` | 同 | → `CartPlan` |
| `move_path(poses, speed=1.0, wait=True)` | 同 | → `CartPlan` |
| `home(*, timeout=None)` / `park()` | 同 | |
| `ik(pose, q_seed=None, timeout=3.0)` | 同 | → `List[float]`，无解抛 `IKError` |
| `get_state(refresh=False, timeout=0.5)` | 同 | → `Msg[RobotState]` |
| `get_status_now(timeout=0.5)` | 同 | → `Msg[RobotState]` |
| `get_tcp(timeout=0.6)` | 同 | → `Msg[(x,y,z,r,p,y)]` |
| `set_speed(percent)` | 同 | 0..100 全局限速——和运动方法上的 `speed=` 不是同一个旋钮 |
| `set_motion_mode(mode)` | 同 | |
| `zero_g(period=0.04)` / `zero_g_start` / `zero_g_stop` / `zero_g_active` / `zero_g_error` | 同 | |
| `emergency_stop()` / `reset()` / `clear_faults()` | 同 | `clear_faults()` 返回 `None`，与 SDK 一致 |
| `move_js(q, dq=None, tau_ff=None)` / `send_mit(...)` / `send_mit_all(...)` | 同 | |
| `set_payload(mass, com=(0,0,0))` | 同 | 返回 `None`；仿真的重力来自 URDF |

### 仿真独有（58 个）

SDK 里没有对应物，是仿真额外提供的能力。值得知道的几组：

- **运动学** —— `fk(q)`（任意构型的正解；SDK 只报固件自己反馈的 FK）
- **只规划不执行** —— `plan_movel`、`plan_movec`、`plan_movep`
- **轨迹** —— `record_trajectory`、`play_trajectory`、`replay_trajectory`、
  `replay_joint_path`、`replay_timed_trajectory`、`save_trajectory`、`list_trajectories`、
  `delete_trajectory`
- **镜像** —— `mirror_from(real_arm)`、`stop_mirroring()`、`mirroring`、`mirror_error`
- **阻抗/保持** —— `hold()`、`set_gains`/`get_gains`、`joint_impedance`、`joint_follow`、
  `cartesian_impedance`、`set_joint_positions`
- **手爪与设备** —— `device()`、`hand`、`devices`、`list_device_types`、`get_active_device`、
  `connect_device()`、`disconnect_device()`、`enter_teleop()`、`exit_teleop()`、`get_teleop_status`
- **限位、配置、日志、服务** —— `get_joint_limits`/`set_joint_limits`、
  `get_cartesian_limits`/`set_cartesian_limits`、`get_collision_config`/`set_collision_config`、
  `get_zero_offsets`/`set_zero_offsets`、`get_end_effector`/`set_end_effector`、
  `get_installation`/`set_installation`、`get_payload`、`get_system_stats`、`get_logs`、
  `restart_service`、`recover_joint_limits`，以及录制状态那四个罐头接口
  （`start_recording`/`stop_recording`/`discard_recording`/`get_recording_state`）
- **身份信息** —— `n`（关节数）、`firmware`（`"PyBulletSim-7J"`，形状兼容的假固件串，
  不是真实固件版本）、`port`

### 已废弃的 0.1 老名（7 个）

纯转发别名——每个都发一条 `DeprecationWarning`，并且只调用一个新方法，所以不会漂移。

| 老名字 | 换用 |
|---|---|
| `movel(pose, speed)` | `move_l(pose, speed)` |
| `movec(via, goal)` | `move_c(pose_start, via, goal)`（别名会用实测 TCP 补上起点） |
| `movep(poses)` | `move_path(poses)` |
| `get_tcp_pose()` | `get_tcp()`（别名用 `rpy_to_mat` 还原 `(pos, R)` 矩阵） |
| `zero_gravity()` | `zero_g()` |
| `request_stop()` | `emergency_stop()` |
| `clear_stop()` | `reset()` |

`start()` 也还在，是 `connect()` 的裸转发。

## 从 0.1（litearm-python 时代）迁移

真机后端原来是 Zenoh 版的 `litearm-python` 0.1.0，现在换成 `litearm-core`
（USB CDC 直连固件），仿真侧 API 跟着对齐。迁移基本是机械替换：

| 0.1 | 现在 |
|---|---|
| `import litearm` | `import litearm_core` |
| `litearm.Arm(endpoint="tcp/host:7447", arm_id="armA")` | `litearm_core.Arm(port="/dev/ttyACM0")` |
| `DualArm(real_endpoint=..., real_arm_id=...)` | `DualArm(real_port=...)` |
| `arm.start()` | `arm.connect()`（别名还在，但不建议继续用） |
| `state = arm.get_state(); state["q"]` | `arm.get_state().value.q` |
| `pos, R = arm.get_tcp_pose()` | `tcp = arm.get_tcp().value`（是 rpy，不是矩阵） |
| `q, ok = arm.ik(pos, R)` | `q = arm.ik(pose)`——失败抛 `IKError`，不再返回 `ok=False` |
| `arm.movej(q, speed, settle_s=2.0)` | `arm.movej(q, speed)`——返回 `RobotState`，等到位才回 |
| `arm.movel(pose, speed)` | `arm.move_l(pose, speed)` → `CartPlan` |
| `arm.request_stop()` / `clear_stop()` | `arm.emergency_stop()` / `arm.reset()` |

**`settle_s` 已删除**，运动方法返回 `RobotState`/`CartPlan` 而不是 `bool`，
失败一律抛异常（`MotionTimeoutError`、`IKError`、`InvalidCommandError` 等）。
到位置判定是 `|q − target| < q_tol (0.03)` 且 `|dq| < dq_tol (0.10)` 连续
`arrive_frames (3)` 帧，这三个都是构造参数。

## 开发

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

仓库自带 dev container，预装 Python、PyBullet、测试工具，并带仿真窗口的浏览器
视图。在 VS Code 安装 Dev Containers 扩展后：F1 → **Dev Containers: Reopen in
Container**。详见 [.devcontainer/README.md](.devcontainer/README.md)。

## License

Proprietary

---

[English](README.md) | [开发者指南](docs/DEVELOPER_GUIDE_zh-CN.md) | [设计文档](docs/design.md)