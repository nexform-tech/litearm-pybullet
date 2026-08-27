# litearm-pybullet

Official PyBullet-based simulation environment for the **LiteArm 7-DOF robotic arm**.
Fully API-compatible with `litearm-python` — swap `Arm` with `PyBulletArm` and your
control code runs identically in simulation and on hardware.

## Features

- 🔄 **Drop-in API compatibility** — Same interface as `litearm-python`. `PyBulletArm` replaces `Arm` directly.
- 🖥️ **Three operating modes** — Standalone simulation / Mirror tracking / Dual control
- 🎮 **Native PyBullet rendering** — Real-time visualization of arm motion
- 🧪 **No hardware required** — Develop and test motion logic without a physical arm
- 🐍 **Pure Python** — Zero compilation. `pip install` and go.

## Installation

```bash
# Standalone simulation (no hardware needed)
pip install litearm-pybullet

# Mirror / Dual control mode (requires hardware connectivity)
pip install "litearm-pybullet[mirror]"
```

Or from source:

```bash
git clone https://github.com/nexform-tech/litearm-pybullet.git
cd litearm-pybullet
pip install -e ".[dev]"
```

## Quick Start

### Mode 1 — Standalone Simulation

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:
    # Joint-space motion
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # Cartesian straight-line motion
    pos, R = arm.get_tcp_pose()
    arm.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.1)

    # Read state
    state = arm.get_state()
    print(state["q"])
```

### Mode 2 — Mirror Mode (sim follows real arm)

```python
from litearm_pybullet import litearm, PyBulletArm

# Connect to real arm
real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

# Create simulation and start mirroring
sim = PyBulletArm(render=True)
sim.start()
sim.mirror_from(real)  # sim tracks real arm in real time
```

### Mode 3 — Dual Control (control both simultaneously)

```python
from litearm_pybullet import DualArm

dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
dual.start()

# One command — both arms move!
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

dual.close()
```

## Architecture

```
┌─────────────────────────────────────────────────┐
│               Your Python Program                │
│                                                   │
│   arm = PyBulletArm()  ← can replace litearm.Arm │
│   arm.movej(...)                                  │
│   arm.get_state()                                 │
└──────────┬────────────────────┬─────────────────┘
           │                    │
    ┌──────▼──────┐      ┌─────▼──────────┐
    │ Standalone   │      │ Dual / Mirror   │
    │ Simulation   │      │                 │
    │              │      │ PyBullet +      │
    │ PyBullet     │      │ litearm-python  │
    │ physics      │      │                 │
    │ engine       │      │ Real + Sim      │
    │ PID ctrl     │      │ together        │
    │ FK/IK        │      │                 │
    └──────────────┘      └─────────────────┘
```

## Examples

| Example | Description | Needs real arm |
|---------|-------------|:---:|
| `01_hello_sim.py` | Create simulation + read state | ❌ |
| `02_movej_sim.py` | Joint & Cartesian motion + FK/IK | ❌ |
| `03_trajectory.py` | Record & replay trajectories | ❌ |
| `04_mirror_real.py` | Sim mirrors real arm | ✅ |
| `05_dual_control.py` | Control both arms simultaneously | ✅ |

```bash
python3 examples/01_hello_sim.py
python3 examples/02_movej_sim.py
python3 examples/03_trajectory.py
python3 examples/04_mirror_real.py --endpoint tcp/192.168.31.139:7447
python3 examples/05_dual_control.py --endpoint tcp/192.168.31.139:7447
```

## API Reference

| litearm.Arm | PyBulletArm | Notes |
|-------------|-------------|-------|
| `Arm(endpoint=...)` | `PyBulletArm(render=True)` | Constructor |
| `movej(q, speed)` | `movej(q, speed)` | ✅ Identical |
| `movel(pose, speed)` | `movel(pose, speed)` | ✅ Identical |
| `movec(via, goal)` | `movec(via, goal)` | ✅ Identical |
| `movep(poses)` | `movep(poses)` | ✅ Identical |
| `get_state()` | `get_state()` | ✅ Identical |
| `get_tcp_pose()` | `get_tcp_pose()` | ✅ Identical |
| `fk(q)` | `fk(q)` | ✅ Identical |
| `ik(pos, R)` | `ik(pos, R)` | ✅ Identical |
| `replay_joint_path(path)` | `replay_joint_path(path)` | ✅ Identical |
| `replay_trajectory(traj)` | `replay_trajectory(traj)` | ✅ Identical |
| `record_trajectory()` | `record_trajectory()` | ✅ Simulated |
| `hold()` / `zero_gravity()` | `hold()` / `zero_gravity()` | ✅ Identical |
| `request_stop()` | `request_stop()` | ✅ Identical |
| `enable/disable()` | `enable/disable()` | ✅ Identical |
| `set_gains(kp, kd)` | `set_gains(kp, kd)` | ✅ Identical |
| `device("hand_0")` | `device("hand_0")` | ✅ Simulated proxy |
| `arm.hand.open()` | `arm.hand.open()` | ✅ Simulated proxy |

## Development

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## License

Proprietary

---

[中文文档](README_zh-CN.md) | [Developer Guide](docs/DEVELOPER_GUIDE.md)