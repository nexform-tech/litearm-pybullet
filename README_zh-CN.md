# litearm-pybullet

LiteArm 七轴机械臂 PyBullet 仿真。API 与 litearm-python SDK 完全兼容（已内置），
支持仿真与实体机械臂同时运动。

## 特点

- 🔄 **API 兼容**：与 litearm-python 相同的接口，`PyBulletArm` 可直接替换 `Arm`
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

## 快速开始

### 模式 1：独立仿真

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:
    # 关节运动
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # 笛卡尔直线运动
    pos, R = arm.get_tcp_pose()
    arm.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.1)

    # 读取状态
    state = arm.get_state()
    print(state["q"])
```

### 模式 2：镜像模式 — 仿真跟随实臂

```python
from litearm_pybullet import litearm, PyBulletArm

# 连接真实机械臂
real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

# 创建仿真并启动镜像
sim = PyBulletArm(render=True)
sim.start()
sim.mirror_from(real)  # 仿真跟随实臂同步运动

# 此时在实臂上做任何操作，仿真都会实时跟随
```

### 模式 3：双控模式 — 同时控制

```python
from litearm_pybullet import DualArm

dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
dual.start()

# 一条命令，实臂和仿真同时运动！
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

dual.close()
```

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
    │ PyBullet    │      │ PyBullet+Zenoh │
    │ 物理引擎    │      │ → litearm-     │
    │             │      │   server       │
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
python3 examples/04_mirror_real.py --endpoint tcp/192.168.31.139:7447
python3 examples/05_dual_control.py --endpoint tcp/192.168.31.139:7447
```

## API 对照

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

## 开发

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## License

Proprietary