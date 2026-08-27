# litearm-pybullet 开发者指南

> 基于 litearm-pybullet v0.1.0

## 项目结构

```
litearm-pybullet/
├── pyproject.toml
├── setup.cfg
├── README.md
├── .gitignore
├── src/
│   └── litearm_pybullet/
│       ├── __init__.py         # 导出 PyBulletArm, DualArm, MirrorMode
│       ├── arm.py              # 核心：PyBulletArm 类（API 与 litearm.Arm 兼容）
│       ├── controller.py       # PID 关节控制器 + 梯形速度轨迹生成器
│       ├── kinematics.py       # 正逆运动学（FK/IK）+ 路径规划
│       ├── mirror.py           # DualArm（双控）+ MirrorMode（镜像）
│       ├── _litearm/           # 精简版 vendored SDK（轨迹类型）
│       │   ├── __init__.py
│       │   └── types.py        # JointTrajectory / TrajectoryFrame
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
│   └── test_pybullet_arm.py
└── docs/
    └── DEVELOPER_GUIDE.zh-CN.md
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
    │ PyBullet    │    │ PyBullet + Zenoh   │
    │ 物理引擎    │    │ → litearm-server   │
    │ PID 控制器  │    │ → 真实机械臂        │
    └─────────────┘    └────────────────────┘
```

### 模块职责

| 模块 | 职责 |
|------|------|
| `arm.py` | 主类 `PyBulletArm`，与 `litearm.Arm` API 完全兼容，后台线程运行物理仿真 |
| `controller.py` | `JointPIDController` 位置控制器 + `TrajectoryGenerator` 梯形速度轨迹生成 |
| `kinematics.py` | `Kinematics` 类：FK（正运动学）、IK（阻尼最小二乘 + Levenberg-Marquardt 精化）+ 路径规划（直线/圆弧/多航点） |
| `mirror.py` | `DualArm`（同时控制实臂+仿真）、`MirrorMode`（仿真跟随实臂） |
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
│ 状态缓存           │  ← _state_cache (每步更新)
│ get_state() 读取   │
└───────────────────┘
```

## 设计的核心原则

### 1. API 兼容性

PyBulletArm 实现了 litearm.Arm 的完整 API 接口。用户只需将 `litearm.Arm(endpoint=...)` 替换为 `PyBulletArm(render=True)` 即可在仿真环境中运行相同的控制代码。

### 2. 三种操作模式

| 模式 | 类 | 创建方式 | 需要实臂 |
|------|-----|----------|----------|
| 独立仿真 | `PyBulletArm` | `PyBulletArm(render=True)` | 否 |
| 镜像模式 | `PyBulletArm` + `mirror_from()` | `sim.mirror_from(real)` | 是 |
| 双控模式 | `DualArm` | `DualArm(real_endpoint=...)` | 是 |

### 3. 线程安全

- 仿真循环在后台线程运行，通过 `self._lock` 保护数据访问
- `get_state()` 返回状态缓存的快照，不会阻塞仿真循环
- 双控模式下，实臂和仿真的运动命令在独立线程中并行执行

### 4. 零依赖独立模式

独立仿真模式只需 `pybullet>=3.2.5` 和 `numpy>=1.21`。镜像/双控模式需要额外安装 `litearm-pybullet[mirror]`，它们对 `litearm-python` 的依赖是可选且延迟加载的。

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

可通过 `set_gains()` 运行时调整，通过 `get_gains()` 查询当前值。

## 环境要求与安装

### 环境要求

- Python >= 3.10
- pybullet >= 3.2.5
- numpy >= 1.21

### 安装

```bash
# 独立仿真模式（仅核心依赖）
pip install litearm-pybullet

# 镜像/双控模式（含 litearm-python，用于连接真实机械臂）
pip install "litearm-pybullet[mirror]"
```

## 快速开始

### 独立仿真（无需真实机械臂）

```python
from litearm_pybullet import PyBulletArm

# 创建仿真机械臂并打开可视化窗口
arm = PyBulletArm(render=True)
arm.start()

# 查询状态
state = arm.get_state()
print("关节角:", state["q"])
print("状态机:", state["state"])

# 关节空间运动：移动到舒展构型
arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

# 笛卡尔直线运动：沿 Z 轴下降 0.1m
pos, R = arm.get_tcp_pose()
pos_target = [pos[0], pos[1], pos[2] - 0.1]
arm.movel([pos_target, R], speed=0.2)

arm.close()
```

## 连接管理

PyBulletArm 支持三种连接管理方式：

### 方式一：显式 start/close

```python
arm = PyBulletArm(render=True)
arm.start()
try:
    arm.movej([0.0] * 7, speed=0.3)
finally:
    arm.close()
```

### 方式二：上下文管理器（推荐）

```python
with PyBulletArm(render=True) as arm:
    arm.movej([0.0] * 7, speed=0.3)
    state = arm.get_state()
```

### 方式三：自动启动（首次调用运动方法时自动启动）

```python
arm = PyBulletArm(render=True)
arm.movej([0.0] * 7, speed=0.3)  # 自动调用 start()
arm.close()
```

注意：`start()` 会启动后台仿真线程（500Hz）。`close()` 会停止线程、断开 PyBullet 连接并关闭可视化窗口。

## API 参考

以下所有方法均为 `PyBulletArm` 的实例方法，与 `litearm.Arm` API 完全兼容。

### 运动控制

| 方法 | 说明 | 阻塞 |
|------|------|------|
| `movej(q_target, speed=1.0, settle_s=1.0)` | 关节空间点到点运动，梯形速度剖面 | 是 |
| `movel(pose_goal, speed=1.0, settle_s=0.8)` | 笛卡尔直线运动 | 是 |
| `movec(pose_via, pose_goal, speed=1.0, settle_s=0.8)` | 笛卡尔圆弧运动（经过中间点） | 是 |
| `movep(poses_goal, speed=1.0, settle_s=0.8)` | 笛卡尔多航点路径运动 | 是 |
| `replay_joint_path(q_path, speed=1.0, goto_start=True)` | 回放关节位置序列 | 是 |
| `replay_trajectory(traj_q, speed=1.0, goto_start=True)` | 回放 JointTrajectory 或路径 | 是 |
| `replay_timed_trajectory(traj_q, traj_t, speed=1.0)` | 按录制时间轴回放轨迹 | 是 |
| `play_trajectory(trajectory, speed=1.0)` | 从文件或对象加载并回放轨迹 | 是 |
| `hold(kp_scale=3.0)` | 保持当前位置（增加刚度） | 是 |
| `zero_gravity(duration_s=None)` | 零重力/自由拖拽模式 | 否 |
| `joint_impedance(q_des, K, B)` | 关节空间阻抗控制（简化实现） | 否 |
| `cartesian_impedance(q_des, K_cart, B_cart)` | 笛卡尔空间阻抗控制（简化实现） | 否 |
| `joint_follow(K=None, B=None, speed_limit=None)` | 跟随外部目标源（兼容空实现） | 否 |
| `recover_joint_limits(speed=0.05, inset_rad=0.0)` | 缓慢将超限关节恢复到安全边界 | 是 |

**参数说明：**

- `q_target`：7 元素关节角列表（rad）
- `pose_goal` / `pose_via`：`(position, rotation_matrix)` 二元组，position 为 `[x, y, z]`（m），rotation_matrix 为 `3x3` 列表
- `speed`：速度倍率（0.0 ~ 1.0+，默认 1.0）
- `settle_s`：运动完成后的稳定等待时间（秒）
- `goto_start`：回放前是否先移动到起始位置

### 状态读取

| 方法 | 返回值 | 说明 |
|------|--------|------|
| `get_state(refresh=False)` | `dict` 或 `None` | 获取完整机器人状态 |
| `get_tcp_pose()` | `(pos, R)` | 获取当前末端位姿 |

**状态字典字段：**

| 字段 | 类型 | 说明 |
|------|------|------|
| `q` | `list[float]` | 7 个关节角（rad） |
| `dq` | `list[float]` | 7 个关节速度（rad/s） |
| `tau` | `list[float]` | 7 个关节力矩（Nm），仿真恒为 0 |
| `fault` | `list` | 故障列表，仿真恒为空 |
| `errs` | `list[int]` | 误差码，仿真恒为 0 |
| `temps` | `list[tuple]` | 电机温度，仿真恒为 25°C |
| `state` | `str` | 状态机："ready" / "stopping" / "disconnected" |
| `feedback` | `dict` | 反馈质量信息（仿真模拟） |
| `watchdog` | `dict` | 看门狗状态（仿真模拟） |
| `robot_serial` | `str` | 序列号，仿真恒为 "PYBULLET-SIM-001" |
| `config_checksum_sha256` | `str` | 配置校验和，仿真恒为 "sim" |

### 急停/使能

| 方法 | 说明 |
|------|------|
| `request_stop()` | 紧急停止：置零力矩，停止仿真运动 |
| `clear_stop()` | 清除急停状态 |
| `enable()` | 使能电机并保持当前姿态 |
| `disable()` | 禁能电机（机械臂将在重力下坠落） |

### 参数调优

| 方法 | 说明 |
|------|------|
| `set_gains(kp=None, kd=None)` | 设置 PD 控制器增益，返回当前增益 |
| `get_gains()` | 获取当前 PD 增益 |
| `clear_faults()` | 清除电机故障（仿真空操作，返回空列表） |
| `set_payload(mass, com)` | 设置末端负载质量与质心（仿真空操作） |
| `get_payload()` | 获取末端负载信息 |
| `set_installation(base_rpy, gravity)` | 设置安装姿态与重力方向（仿真空操作） |
| `get_installation()` | 获取安装信息 |

### 外设（模拟实现）

| 方法 | 说明 |
|------|------|
| `device(device_id)` | 获取模拟外设代理 |
| `devices` | 模拟外设管理器（支持 `arm.devices["hand_0"]` 语法） |
| `hand` | 向后兼容的手部属性 |
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

### 轨迹

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

### 运动学计算（纯计算，不推进仿真）

| 方法 | 说明 |
|------|------|
| `fk(q)` | 正运动学：关节角 → `(position, rotation_matrix)` |
| `ik(pos_d, R_d, q_seed=None)` | 逆运动学：位姿 → `(q_solution, success)` |
| `plan_movel(q_start, pose_goal)` | 规划笛卡尔直线路径 |
| `plan_movec(q_start, pose_via, pose_goal)` | 规划笛卡尔圆弧路径 |
| `plan_movep(q_start, poses_goal)` | 规划笛卡尔多航点路径 |

### 设备管理

| 方法 | 说明 |
|------|------|
| `enter_teleop(mode, **params)` | 进入遥操模式（兼容空实现） |
| `exit_teleop()` | 退出遥操模式（兼容空实现） |
| `get_teleop_status()` | 遥操状态查询（兼容空实现） |

### 镜像模式

| 方法 | 说明 |
|------|------|
| `mirror_from(real_arm, rate_hz=50.0)` | 启动镜像：仿真跟随实臂关节状态 |
| `stop_mirroring()` | 停止镜像 |
| `set_joint_positions(q)` | 直接设置关节位置（用于镜像模式同步） |

## 三种模式

### 模式一：独立仿真

在无真实机械臂的情况下运行仿真，用于算法开发、测试和验证。

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:
    # 关节空间运动
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

    # 读取状态
    state = arm.get_state()
    print(f"关节角: {[round(x, 3) for x in state['q']]}")

    # 正逆运动学
    pos, R = arm.fk([0.0] * 7)
    q_sol, success = arm.ik(pos, R)

    # 笛卡尔直线运动
    pos_target = [pos[0], pos[1], pos[2] - 0.1]
    arm.movel([pos_target, R], speed=0.2)

    # 轨迹录制与回放
    traj = arm.record_trajectory(duration_s=3.0, sample_rate_hz=100.0)
    traj.save("trajectories/demo.json")
    arm.replay_trajectory(traj, speed=1.0)
```

### 模式二：镜像模式

仿真跟随真实机械臂的状态同步运动，用于可视化监控和数字孪生。

**前提条件：**
- 机械臂控制器上 litearm-server 已启动
- 客户端与控制器网络互通
- 客户端已安装 `litearm-pybullet[mirror]`

```python
from litearm_pybullet import PyBulletArm, litearm

# 连接真实机械臂
real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

# 创建仿真
sim = PyBulletArm(render=True)
sim.start()

# 将仿真初始化为实臂当前姿态
real_state = real.get_state()
sim.set_joint_positions(real_state["q"])

# 开始镜像：仿真跟随实臂运动
sim.mirror_from(real, rate_hz=50.0)

# 在实臂上执行操作（拖动/运动），观察仿真同步
try:
    while True:
        r_state = real.get_state()
        s_state = sim.get_state()
        err = max(abs(r_state["q"][i] - s_state["q"][i]) for i in range(7))
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
from litearm_pybullet import PyBulletArm, MirrorMode, litearm

real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
sim = PyBulletArm(render=True)
sim.start()

mirror = MirrorMode(real, sim, rate_hz=50.0)
mirror.start()

# ... 仿真跟随实臂 ...

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
    real_endpoint="tcp/192.168.31.139:7447",
    render=True,
    mirror_first=True,  # 先同步实臂姿态到仿真，再开始运动
)
dual.start()

# 双控运动：实臂和仿真同时执行
real_ok, sim_ok = dual.movej(
    [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0],
    speed=0.2,
)
print(f"实臂: {'OK' if real_ok else 'FAIL'}, 仿真: {'OK' if sim_ok else 'FAIL'}")

# 笛卡尔运动
pos, R = dual.get_tcp_pose()
target = [[pos[0], pos[1], pos[2] - 0.1], R]
real_ok, sim_ok = dual.movel(target, speed=0.2)

# 急停
dual.request_stop()

# 关闭
dual.close()
```

DualArm 支持的属性：

| 属性 | 说明 |
|------|------|
| `dual.real` | 真实机械臂实例（litearm.Arm） |
| `dual.sim` | 仿真机械臂实例（PyBulletArm） |

DualArm 支持的方法：`movej()`, `movel()`, `movec()`, `movep()`, `get_real_state()`, `get_sim_state()`, `get_tcp_pose()`, `get_sim_tcp_pose()`, `request_stop()`, `clear_stop()`, `close()`。

## 位姿格式

位姿使用纯 Python list 表示，无需导入任何类型：

```python
# 关节角：7 元素列表（rad）
q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]

# 笛卡尔位姿：(position, rotation_matrix) 二元组
# position: [x, y, z] 米
# rotation_matrix: 3x3 列表
pos = [0.3, 0.0, 0.6]
R = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
]
pose = [pos, R]

# 使用
arm.movej(q, speed=0.3)
arm.movel(pose, speed=0.2)
arm.movec(pose_via, pose, speed=0.2)

# 末端位姿获取
pos, R = arm.get_tcp_pose()
# pos = [0.3, 0.0, 0.638] 之类
# R = [[...], [...], [...]]  3x3 旋转矩阵
```

## 安全注意事项

1. **仿真无物理碰撞检测**：PyBulletArm 不会对工作空间边界、桌面、障碍物进行碰撞检测。请确保运动路径在安全范围内。

2. **关节限位保护**：代码内置了关节限位（`Q_MIN` / `Q_MAX`），IK 求解会裁剪到限位内。但直接传入的 `movej` 目标值不会自动裁剪，请确保目标值在限位范围内。

3. **禁能后重力坠落**：调用 `disable()` 后机械臂将在重力下坠落。在 PyBullet 窗口中观察，不要在实际环境中混淆。

4. **双控/镜像模式的安全**：在双控模式下，仿真和实臂同时运动。确保实臂周围无障碍物，运动范围安全。建议先用低速（`speed=0.1`）测试。

5. **急停机制**：`request_stop()` 会立即停止仿真运动（置零力矩）。`clear_stop()` 后需重新调用 `enable()` 恢复控制。

6. **仿真与实臂差异**：仿真使用理想化 PID 控制器，不考虑摩擦、关节柔度、传感器噪声、电机饱和等物理效应。仿真中验证通过的轨迹在实际机械臂上仍需谨慎测试。

7. **控制器参数**：默认 PD 增益与真机对齐，但仿真中的动力学响应可能与真机有所不同。如需精确复现真机行为，可通过 `set_gains()` 调整增益。

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
- `speed` 参数取值范围 0.0~1.0+，默认 1.0 对应 `max_velocity=3.0 rad/s`
- 构造时可传入 `max_velocity` 调整最大速度
- 物理时间步长 `dt` 默认为 0.002s（500Hz），过大的步长会导致仿真不稳定

### Q: 仿真机械臂抖动或发散

A: 可能原因：
- PD 增益过高导致数值不稳定，尝试降低增益：`arm.set_gains(kp=[100]*7, kd=[3]*7)`
- 物理时间步长过大，尝试减小 `dt`：`PyBulletArm(render=True, dt=0.001)`
- 目标位置超出关节限位，检查目标值是否在 `Q_MIN` ~ `Q_MAX` 范围内

### Q: IK 求解失败

A: `ik()` 返回 `(q, False)` 表示求解失败。可能原因：
- 目标位姿在机械臂工作空间之外
- 目标位姿在奇异点附近
- 尝试传入不同的 `q_seed` 初始猜测值

### Q: 双控/镜像模式报错 `ImportError: ... requires litearm-pybullet[mirror]`

A: 需要安装 litearm-python 依赖：

```bash
pip install "litearm-pybullet[mirror]"
```

### Q: 轨迹录制不包含实际运动数据

A: `record_trajectory()` 在录制期间会禁用控制器（`_enabled = False`），使机械臂在重力下自由运动。录制的是自由运动轨迹，而非受控运动。如需录制受控运动的轨迹，请在录制前调用 `movej()` 等运动方法，或在录制时手动控制。

### Q: 仿真中的外设（夹爪/手）不工作

A: 外设 API 为模拟实现，所有方法返回固定值（`open()` 返回 `True`，`get_state()` 返回 `{"connected": True}` 等）。如需真实外设控制，请使用真实机械臂。

## 依赖

### 核心依赖
- `pybullet>=3.2.5`
- `numpy>=1.21`

### 可选依赖（镜像/双控模式）
- `litearm-python`（含 zenoh 通信栈）

## 已知限制

- 控制器为关节空间 PD + 重力/科氏前馈，未包含完整计算力矩前馈与摩擦前馈
- 灵巧手/夹爪/示教板为模拟代理，返回固定值
- 系统管理/日志/遥操为 API 兼容空实现
- 无碰撞检测（PyBullet 内置碰撞检测未启用）
- 笛卡尔运动（movel/movec/movep）在接近奇异构型时可能失败
- 仿真不包含传感器噪声、电机饱和、关节柔度等物理效应

## License

Proprietary