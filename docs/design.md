# litearm-pybullet 设计文档

## 项目定位

`litearm-pybullet` 是 LiteArm 七轴机械臂的 PyBullet 仿真，API 与 `litearm-python` SDK 完全兼容，支持三种模式：

| 模式 | 说明 | 需要真机 |
|------|------|----------|
| 独立仿真 | `PyBulletArm` 直接替换 `litearm.Arm`，无需硬件 | ❌ |
| 镜像模式 | 仿真跟随真实机械臂状态同步运动 | ✅ |
| 双控模式 | 同时向真实机械臂和仿真发送指令 | ✅ |

## 架构

```
┌─────────────────────────────────────────────────┐
│                  你的 Python 程序                │
│                                                   │
│   arm = PyBulletArm()  ← 可替换为 litearm.Arm    │
│   arm.movej(...)                                  │
│   arm.get_state()                                 │
└──────────┬────────────────────┬─────────────────┘
           │                    │
    ┌──────▼──────┐      ┌─────▼──────────┐
    │ 独立仿真     │      │ 双控 / 镜像     │
    │             │      │                │
    │ PyBullet    │      │ PyBullet +     │
    │ 物理引擎    │      │ litearm-python │
    │             │      │                │
    │ PID 控制器  │      │ 实臂 + 仿真    │
    │ FK/IK      │      │ 同时运动        │
    │             │      │                │
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
│   └── DEVELOPER_GUIDE.md
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
│       ├── _litearm/           # 内置 SDK stub（轨迹类型）
│       │   ├── __init__.py
│       │   └── types.py
│       └── assets/
│           ├── litearm.urdf
│           └── meshes/
└── tests/
    └── test_pybullet_arm.py
```

## 模块职责

### arm.py — PyBulletArm

主类，API 与 `litearm.Arm` 完全兼容。核心能力：

- **生命周期**：`start()` / `close()` / context manager
- **状态读取**：`get_state()` / `get_tcp_pose()` / `fk()` / `ik()`
- **运动控制**：`movej()` / `movel()` / `movec()` / `movep()` / `replay_joint_path()` / `replay_trajectory()` 等
- **急停/使能**：`request_stop()` / `clear_stop()` / `enable()` / `disable()`
- **参数**：`set_gains()` / `get_gains()` / `set_payload()` 等
- **仿真设备**：`device()` / `hand` / `devices` 等模拟外设
- **镜像模式**：`mirror_from()` / `stop_mirroring()`
- **轨迹**：`record_trajectory()` / `play_trajectory()` 等

内部使用 PyBullet DIRECT/GUI 客户端，后台物理线程以 1kHz 运行扭矩控制（PD + 前馈）。

### controller.py — PID 控制器 + 轨迹生成

与 litearm-mujoco 的 controller.py 完全对齐：

- `JointPIDController`：关节空间 PD + 前馈（重力/科氏补偿），对齐真机 MIT 模式增益
- `TrajectoryGenerator`：梯形速度曲线 + minimum jerk 轨迹

### kinematics.py — 运动学

纯计算，不依赖物理线程。复用 PyBullet 内置 `getLinkState`（FK）和 `calculateInverseKinematics`（IK 初值），加一步 LM 精修。

- `fk(q)` / `ik(pos, R, q_seed)`
- `plan_movel()` / `plan_movec()` / `plan_movep()`

### mirror.py — 双控 + 镜像

- `DualArm`：同时持有 `litearm.Arm` 和 `PyBulletArm`，运动命令同时发两边
- `MirrorMode`：后台线程持续拉真机状态 → 同步到仿真

### _litearm/ — SDK stub

当 `litearm-python` 未安装时，提供 `JointTrajectory` / `TrajectoryFrame` 类型用于轨迹录制/回放。当安装了 `litearm-python` 时透明转发。

## 依赖策略

```
基础安装: pybullet>=3.2.5, numpy>=1.21
mirror extra: litearm-python  (真机通信)
```

## 物理引擎参数

- 仿真步长：1ms（1000Hz），高增益 PD 稳定
- PD 增益：对齐真机（Kp: 260/260/150/150/50/50/50，Kd: 5/5/4/5/2.5/2.5/2.5）
- 关节限位：与真机一致
- 重力：9.81 m/s²
- URDF 惯量补丁：最小质量 0.5kg，最小惯量 0.01 kg·m²

## API 对齐对照

| litearm.Arm | PyBulletArm | 说明 |
|-------------|-------------|------|
| `Arm(endpoint=...)` | `PyBulletArm(render=True)` | 创建实例 |
| `movej(q, speed)` | `movej(q, speed)` | ✅ 相同 |
| `movel(pose, speed)` | `movel(pose, speed)` | ✅ 相同 |
| `movec(via, goal)` | `movec(via, goal)` | ✅ 相同 |
| `movep(poses)` | `movep(poses)` | ✅ 相同 |
| `get_state()` | `get_state()` | ✅ 相同 |
| `get_tcp_pose()` | `get_tcp_pose()` | ✅ 相同 |
| `fk(q)` | `fk(q)` | ✅ 相同 |
| `ik(pos, R)` | `ik(pos, R)` | ✅ 相同 |
| `request_stop()` | `request_stop()` | ✅ 相同 |
| `enable/disable()` | `enable/disable()` | ✅ 相同 |
| `set_gains(kp, kd)` | `set_gains(kp, kd)` | ✅ 相同 |
| `device("hand_0")` | `device("hand_0")` | ✅ 模拟 |
| `arm.hand.open()` | `arm.hand.open()` | ✅ 模拟 |