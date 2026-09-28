# litearm-pybullet

Official PyBullet-based simulation environment for the **LiteArm 7-DOF robotic arm**.
Aligned 1:1 with the [`litearm-python`](https://github.com/nexform-tech/litearm-python) SDK —
swap `litearm.Arm(port=...)` with `PyBulletArm(render=True)` and your control code
runs the same way in simulation and on hardware.

## Features

- 🔄 **1:1 API alignment with `litearm-python`** — the same 31 method names, parameters, defaults, `Msg` envelope and `RobotState`/`CartPlan` return types. `PyBulletArm` replaces `Arm` directly.
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

**Mirror/dual mode needs `litearm-python`, which is not on PyPI.** The `[mirror]`
extra names it as a dependency, but that name does not resolve against PyPI yet,
so install it from a source checkout first:

```bash
pip install -e ../litearm-python      # sibling checkout
pip install -e ".[mirror]"          # or just use PYTHONPATH=../litearm-python/src
```

Standalone simulation needs neither: `litearm_pybullet.HAS_LITEARM` tells you
whether the real SDK was importable, and standalone mode never requires it.

## Quick Start

### Mode 1 — Standalone Simulation

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:            # connect() is implicit
    # Joint-space motion. `speed` is a 0..1 fractional multiplier, not a percent
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # Cartesian straight-line motion. pose = xyz + rpy
    x, y, z, r, p, yw = arm.get_tcp().value
    arm.move_l([x, y, z - 0.1, r, p, yw], speed=0.1)

    # Read state — Msg envelope, the value is in .value
    state = arm.get_state().value
    print(state.q)
```

### Mode 2 — Mirror Mode (sim follows real arm)

```python
from litearm_pybullet import PyBulletArm, litearm   # needs litearm-python installed

# Connect to the real arm over USB CDC — no server, no endpoint
real = litearm.Arm(port=None).connect()             # None = auto-detect

# Create simulation and start mirroring
sim = PyBulletArm(render=True).connect()
sim.mirror_from(real)   # sim tracks real arm; failures land in sim.mirror_error
```

### Mode 3 — Dual Control (control both simultaneously)

```python
from litearm_pybullet import DualArm

dual = DualArm(real_port=None, render=True)   # None = auto-detect the CDC port
dual.start()
dual.enable()   # explicit: the constructor never powers the physical arm

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
    │ PyBullet     │      │ litearm-python    │
    │ physics      │      │ (USB CDC)       │
    │ engine       │      │                 │
    │ PID ctrl     │      │ Real + Sim      │
    │ FK/IK        │      │ together        │
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
python3 examples/04_mirror_real.py --port /dev/ttyACM0
python3 examples/05_dual_control.py --port /dev/ttyACM0
```

## API Reference

Three tiers. The first is the promise: same names, same parameters, same
return types, same exceptions — verified by `tests/test_api_parity.py` against
the installed SDK. The second is simulation-only, which the SDK has no
equivalent for. The third is kept for 0.1-era callers and is on its way out.

### 1:1 with `litearm.Arm` (31 names)

| `litearm.Arm` | `PyBulletArm` | Notes |
|---|---|---|
| `Arm(port=...)` | `PyBulletArm(render=True)` | Constructor; the sim takes `render`, and `connect(port=None)` |
| `connect(port=None)` / `disconnect()` / `close()` | same |  |
| `enable(attempts=12)` / `disable()` | same | |
| `movej(q, speed=1.0)` | same | → `RobotState`, raises on timeout |
| `movej_sync(q, speed=1.0)` | same | → `RobotState` |
| `move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03)` | same | → `RobotState` |
| `move_l(pose, speed=1.0, wait=True)` | same | → `CartPlan` |
| `move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True)` | same | → `CartPlan` |
| `move_path(poses, speed=1.0, wait=True)` | same | → `CartPlan` |
| `home(*, timeout=None)` / `park()` | same | |
| `ik(pose, q_seed=None, timeout=3.0)` | same | → `List[float]`, raises `IKError` |
| `get_state(refresh=False, timeout=0.5)` | same | → `Msg[RobotState]` |
| `get_status_now(timeout=0.5)` | same | → `Msg[RobotState]` |
| `get_tcp(timeout=0.6)` | same | → `Msg[(x,y,z,r,p,y)]` |
| `set_speed(percent)` | same | 0..100 global governor — not the same knob as `speed=` |
| `set_motion_mode(mode)` | same | |
| `zero_g(period=0.04)` / `zero_g_start` / `zero_g_stop` / `zero_g_active` / `zero_g_error` | same | |
| `emergency_stop()` / `reset()` / `clear_faults()` | same | `clear_faults()` returns `None`, like the SDK |
| `move_js(q, dq=None, tau_ff=None)` / `send_mit(...)` / `send_mit_all(...)` | same | |
| `set_payload(mass, com=(0,0,0))` | same | returns `None`; the simulation takes gravity from the URDF |

### Simulation-only (58 names)

No SDK counterpart — these are what the simulation adds on top. The ones worth
knowing:

- **Kinematics** — `fk(q)` (arbitrary configuration; the SDK only reports the firmware's own FK)
- **Planning without executing** — `plan_movel`, `plan_movec`, `plan_movep`
- **Trajectories** — `record_trajectory`, `play_trajectory`, `replay_trajectory`,
  `replay_joint_path`, `replay_timed_trajectory`, `save_trajectory`, `list_trajectories`,
  `delete_trajectory`
- **Mirroring** — `mirror_from(real_arm)`, `stop_mirroring()`, `mirroring`, `mirror_error`
- **Impedance / hold** — `hold()`, `set_gains`/`get_gains`, `joint_impedance`, `joint_follow`,
  `cartesian_impedance`, `set_joint_positions`
- **Hand & devices** — `device()`, `hand`, `devices`, `list_device_types`, `get_active_device`,
  `connect_device()`, `disconnect_device()`, `enter_teleop()`, `exit_teleop()`, `get_teleop_status`
- **Limits, config, logs, services** — `get_joint_limits`/`set_joint_limits`,
  `get_cartesian_limits`/`set_cartesian_limits`, `get_collision_config`/`set_collision_config`,
  `get_zero_offsets`/`set_zero_offsets`, `get_end_effector`/`set_end_effector`,
  `get_installation`/`set_installation`, `get_payload`, `get_system_stats`, `get_logs`,
  `restart_service`, `recover_joint_limits`, and the recording-state stub quartet
  (`start_recording`/`stop_recording`/`discard_recording`/`get_recording_state`)
- **Identity** — `n` (joint count), `firmware` (`"PyBulletSim-7J"` — a shape-compatible
  banner, not real firmware), `port`

### Deprecated 0.1 names (7)

Pure forwarding aliases — each emits a `DeprecationWarning` and calls exactly one
new method, so they cannot drift.

| Deprecated | Use instead |
|---|---|
| `movel(pose, speed)` | `move_l(pose, speed)` |
| `movec(via, goal)` | `move_c(pose_start, via, goal)` (the alias fills in the measured TCP) |
| `movep(poses)` | `move_path(poses)` |
| `get_tcp_pose()` | `get_tcp()` (the alias rebuilds the `(pos, R)` matrix via `rpy_to_mat`) |
| `zero_gravity()` | `zero_g()` |
| `request_stop()` | `emergency_stop()` |
| `clear_stop()` | `reset()` |

`start()` is also still there as a bare forward to `connect()`.

## Migrating from 0.1 (the `litearm-python` era)

The distribution name `litearm-python` covers two SDKs that share only the import
line. 0.1.0 was a Zenoh client: it talked to a separate `litearm-server` process and
was addressed with `endpoint=`/`arm_id=`. 2.1 is a thin client that speaks the
firmware's protocol over USB CDC, and its constructor takes `port=`. The real-arm
backend here uses 2.1 and the simulation API follows it, so porting is mostly
mechanical:

| 0.1 (server-era) | 2.1 (USB CDC) |
|---|---|
| `litearm-python` 0.1.0, `import litearm` | `litearm-python` 2.1, the same `import litearm` |
| `litearm.Arm(endpoint="tcp/host:7447", arm_id="armA")` | `litearm.Arm(port="/dev/ttyACM0")` |
| `DualArm(real_endpoint=..., real_arm_id=...)` | `DualArm(real_port=...)` |
| `arm.start()` | `arm.connect()` (still an alias, but deprecated in spirit) |
| `state = arm.get_state(); state["q"]` | `arm.get_state().value.q` |
| `pos, R = arm.get_tcp_pose()` | `tcp = arm.get_tcp().value` (rpy, not a matrix) |
| `q, ok = arm.ik(pos, R)` | `q = arm.ik(pose)` — raises `IKError` instead of returning `ok=False` |
| `arm.movej(q, speed, settle_s=2.0)` | `arm.movej(q, speed)` — returns `RobotState`, waits for arrival |
| `arm.movel(pose, speed)` | `arm.move_l(pose, speed)` → `CartPlan` |
| `arm.request_stop()` / `clear_stop()` | `arm.emergency_stop()` / `arm.reset()` |

**`settle_s` is gone** and motion methods return `RobotState`/`CartPlan` rather than
`bool`; failures raise (`MotionTimeoutError`, `IKError`, `InvalidCommandError`, …).
Arrival is decided by `|q − target| < q_tol (0.03)` and `|dq| < dq_tol (0.10)` over
`arrive_frames (3)` consecutive frames, all constructor arguments.

## Development

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## License

Proprietary

---

[中文文档](README_zh-CN.md) | [Developer Guide](docs/DEVELOPER_GUIDE.md)