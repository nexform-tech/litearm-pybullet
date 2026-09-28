# litearm-pybullet Developer Guide

> Based on litearm-pybullet v0.1.0, real-arm backend migrated to `litearm-python` 2.1.0.
> If you are porting code written against the old Zenoh-based `litearm-python` 0.1.0, read
> [Migrating from 0.1](#migrating-from-01-the-litearm-python-era) first.

## Project Structure

```
litearm-pybullet/
├── pyproject.toml
├── README.md
├── .gitignore
├── src/
│   └── litearm_pybullet/
│       ├── __init__.py         # Exports PyBulletArm, DualArm, MirrorMode, HAS_LITEARM
│       ├── arm.py              # Core: PyBulletArm class (aligned with litearm.Arm)
│       ├── controller.py       # PID joint controller + trajectory generator
│       ├── kinematics.py       # Forward/inverse kinematics (FK/IK) + path planning
│       ├── mirror.py           # DualArm (dual control) + MirrorMode (mirroring)
│       ├── trajectory.py       # JointTrajectory / TrajectoryFrame (simulation-only)
│       ├── _compat.py          # Single decision point: real SDK if importable
│       ├── _fallback.py        # Structural stand-ins, field/name-aligned with 2.1.0
│       └── assets/
│           ├── litearm.urdf    # 7-DOF arm URDF model
│           └── meshes/         # Real STL meshes (9 links)
├── examples/
│   ├── 01_hello_sim.py         # Standalone simulation + read state
│   ├── 02_movej_sim.py         # Joint/Cartesian motion + FK/IK
│   ├── 03_trajectory.py        # Trajectory recording and playback
│   ├── 04_mirror_real.py       # Mirror mode (sim follows real arm)
│   └── 05_dual_control.py      # Dual control (simultaneous real + sim)
├── tests/
│   ├── __init__.py
│   ├── test_api_parity.py      # Pins signatures/fields against the real SDK
│   ├── test_compat_fallback.py
│   ├── test_mirror.py
│   ├── test_pybullet_arm.py
│   └── test_trajectory.py
└── docs/
    ├── design.md
    ├── DEVELOPER_GUIDE.md
    └── DEVELOPER_GUIDE_zh-CN.md
```

## Core Architecture

```
┌──────────────────────────────────────────────────┐
│                 Your Python Program               │
│                                                    │
│  arm = PyBulletArm(render=True)  ← swap for litearm.Arm │
│  arm.movej(...) / arm.get_state() / arm.close()    │
└──────────┬──────────────────┬────────────────────┘
           │                  │
    ┌──────▼──────┐    ┌─────▼──────────────┐
    │ Standalone   │    │ Mirror / Dual      │
    │             │    │                    │
    │ PyBullet    │    │ PyBullet +         │
    │ Physics     │    │ litearm-python       │
    │ PID Ctrl    │    │ (USB CDC)          │
    └─────────────┘    └────────────────────┘
```

There is no server tier: `litearm-python` speaks the firmware protocol directly over
USB CDC. There is no `endpoint`, no `arm_id`, and no Zenoh anywhere in the stack.

### Module Responsibilities

| Module | Responsibility |
|--------|---------------|
| `arm.py` | Main class `PyBulletArm`, 31 methods aligned 1:1 with `litearm.Arm`, background thread running physics simulation |
| `controller.py` | `JointPIDController` position controller + `TrajectoryGenerator` trapezoidal velocity trajectory generation |
| `kinematics.py` | `Kinematics` class: FK (forward kinematics), IK (damped least-squares with Levenberg-Marquardt refinement) + path planning (straight line / circular arc / multi-waypoint) |
| `mirror.py` | `DualArm` (simultaneous control of real + sim), `MirrorMode` (sim tracks real arm) |
| `_compat.py` | Decides where `Msg`/`RobotState`/`CartPlan`/exceptions/pose helpers come from — the real SDK if it imports, `_fallback.py` otherwise |
| `_fallback.py` | Local structural equivalents, name- and field-aligned with `litearm` 2.1.0, pinned by `tests/test_api_parity.py` |
| `litearm.urdf` | URDF model: 7 joints, real STL mesh geometry, torque motors |

### Data Flow

```
arm.movej(q_target, speed=0.5)
        │
        ▼
┌───────────────────┐
│ Trajectory         │  ← TrajectoryGenerator.linear_trajectory()
│ Generator          │     Generate joint-space path
│ Trapezoidal        │
│ velocity profile   │
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ PID Controller     │  ← JointPIDController.set_target(q_des)
│ Set target         │
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ Background         │  ← _sim_loop() @ 500 Hz
│ Simulation Thread  │     PID computes torque → setJointMotorControlArray
│                    │     + gravity/Coriolis feed-forward → stepSimulation()
└───────┬───────────┘
        │
        ▼
┌───────────────────┐
│ Arrival Check      │  ← _arrive(): |q−target| < q_tol and |dq| < dq_tol
│ return RobotState  │     for arrive_frames consecutive frames; else raise
└───────────────────┘
```

## Core Design Principles

### 1. API Alignment

`PyBulletArm` implements 31 methods with the same names, parameters, defaults, return
types and exceptions as `litearm.Arm`. Users only need to replace
`litearm.Arm(port=...)` with `PyBulletArm(render=True)` to run the same control
code in simulation.

```
connect/disconnect/close, enable/disable,
movej, movej_sync, move_p, move_l, move_c, move_path, home, park, ik,
get_state, get_status_now, get_tcp,
set_speed, set_motion_mode, set_payload,
zero_g/zero_g_start/zero_g_stop/zero_g_active/zero_g_error,
emergency_stop, reset, clear_faults,
move_js, send_mit, send_mit_all
```

Beyond those, 58 names are **simulation-only** (no SDK counterpart: `fk`, `plan_*`,
trajectory recording/playback, mirroring, impedance modes, the hand/device proxies,
the config/log/service accessors, `n`/`firmware`/`port`), and 7 names are
**deprecated 0.1 aliases** that forward to a single new method each.

Type identity is the point of `_compat.py`: if `litearm-python` is installed, the sim
imports *its* `Msg`, `RobotState`, `CartPlan` and exception classes, so
`except litearm.MotionTimeoutError` catches a simulation failure too. Without it,
`_fallback.py` provides structurally identical local copies. `litearm_pybullet.HAS_LITEARM`
tells you which happened.

### 2. Three Operating Modes

| Mode | Class | Creation | Requires Real Arm |
|------|-------|----------|-------------------|
| Standalone | `PyBulletArm` | `PyBulletArm(render=True)` | No |
| Mirror | `PyBulletArm` + `mirror_from()` | `sim.mirror_from(real)` | Yes |
| Dual Control | `DualArm` | `DualArm(real_port=...)` | Yes |

### 3. Thread Safety

- The simulation loop runs in a background thread, with data access protected by `self._lock`
- Kinematics computations use the live PyBullet client (joint state resets are scoped to the FK/IK call)
- `get_state()` returns a snapshot of the state cache, does not block the simulation loop
- Mirroring runs in its own daemon thread. It reads the real arm and writes joint
  positions; do not send commands to the same real arm object while it runs
  (litearm-python counts the competing reader as a foreign frame). Call
  `stop_mirroring()` first.

### 4. Zero-Dependency Standalone Mode

Standalone simulation mode requires only `pybullet>=3.2.5` and `numpy>=1.21`. Mirror/dual
modes require the `litearm-pybullet[mirror]` extra, whose dependency on `litearm-python`
is optional and imported lazily — the sim never imports it unless you ask for mirroring.

## URDF Model (litearm.urdf)

- 7 revolute joints, torque motor driven
- Kinematic chain (link positions/quaternions, joint axes/ranges) aligned with the real LiteArm URDF
- Zero-position TCP = [0, 0, 0.814] m, matching the real arm
- Mass/inertia/COM from URDF (total mass approximately 2.93 kg)
- Visual/collision geometry uses real STL meshes (9 meshes, distributed with the package)
- Motors use `VELOCITY_CONTROL` with zero force (enabling direct torque control via `TORQUE_CONTROL`)
- Minimum mass/inertia patch applied for stable high-gain PD control

## Controller Parameters

Default gains are aligned with the real arm MIT follow-mode gains:

| Joint | kp (Nm/rad) | kd (Nm.s/rad) |
|-------|-------------|----------------|
| 1-2   | 260         | 5              |
| 3-4   | 150         | 4-5            |
| 5-7   | 50          | 2.5            |

Arrival is decided by four constructor arguments, all matching the SDK's names:

| Argument | Default | Meaning |
|----------|---------|---------|
| `q_tol` | `0.03` | Joint-space position tolerance [rad] |
| `dq_tol` | `0.10` | Joint-space velocity tolerance [rad/s] |
| `arrive_frames` | `3` | Consecutive frames inside both tolerances before "arrived" |
| `move_timeout` | `15.0` | Seconds before `MotionTimeoutError` |

## Requirements and Installation

### Requirements

- `pybullet >= 3.2.5`
- `numpy >= 1.21`
- `Python >= 3.10`

### Optional Dependencies (mirror/dual mode)

- `litearm-python` — **not on PyPI.** The `[mirror]` extra names it, but that name does
  not resolve against PyPI, so install from a source checkout:
  `pip install -e ../litearm-python`

### Installation

```bash
# Standalone simulation (no real-arm communication needed)
pip install litearm-pybullet

# Mirror/dual mode (requires real-arm communication)
pip install -e ../litearm-python     # first: not on PyPI
pip install "litearm-pybullet[mirror]"
```

Install from source:

```bash
git clone https://gitee.com/xxx/litearm-pybullet.git
cd litearm-pybullet
pip install -e ".[dev]"
```

## Quick Start

### Standalone Simulation

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:
    # Joint-space motion. `speed` is a 0..1 fraction, not a percentage
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # Cartesian straight-line motion. pose = xyz + rpy
    x, y, z, r, p, yw = arm.get_tcp().value
    arm.move_l([x, y, z - 0.1, r, p, yw], speed=0.1)

    # Read state — Msg envelope, the payload is in .value
    state = arm.get_state().value
    print(state.q)
```

## Connection Management

### connect(port=None)

Starts the background simulation thread and returns `self`. The simulation loop runs
at 500 Hz (1/dt where dt=0.002). `port` is accepted for signature compatibility with
`litearm.Arm(port=...)` and ignored — there is nothing to open. Idempotent:
calling it twice does no extra work.

```python
arm = PyBulletArm(render=True).connect()
```

### start()

Legacy alias for `connect()`, kept for 0.1 callers. New code should use `connect()`.

### close()

Stops the simulation, stops mirroring if active, and disconnects the PyBullet client.
Idempotent in the strong sense: the second call does nothing at all.

```python
arm.close()
```

### disconnect()

Alias for `close()`, matching the SDK's name.

### Context Manager

`PyBulletArm` supports the `with` statement for automatic cleanup:

```python
with PyBulletArm(render=True) as arm:
    arm.movej([0.0]*7, speed=0.3)
# arm.close() called automatically
```

## API Reference

### 4.1 Computation

Methods that perform pure computation without stepping the simulation.

#### fk(q)

Forward kinematics: compute TCP pose from joint angles. **Simulation-only** —
`litearm` has no FK for an arbitrary configuration, because the firmware only
reports the TCP of the pose it is actually in.

**Parameters:**

- `q` (List[float]): Joint angles [rad], length 7.

**Returns:** `(position, rotation_matrix)` where position is `[px, py, pz]` and rotation_matrix is a 3x3 row-major matrix.

```python
pos, R = arm.fk([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0])
```

#### ik(pose, q_seed=None, timeout=3.0)

Inverse kinematics. Same signature as `litearm.Arm.ik`: takes a pose, returns
`q[7]`, and **raises `IKError`** when the pose is unreachable — there is no `ok` flag.
Uses PyBullet's built-in IK as an initial seed, then refines with Levenberg-Marquardt.

**Parameters:**

- `pose` (Sequence[float]): The target pose. Accepts `[x, y, z, roll, pitch, yaw]`
  (the form `get_tcp()` returns and the firmware uses), a `(position[3], R[3x3])` pair,
  or a 4x4 homogeneous matrix. A flat 9-element rotation is rejected — it carries no
  position. Anything malformed raises `InvalidCommandError`.
- `q_seed` (Optional[Sequence[float]]): Initial guess (default: the current joint configuration).
- `timeout` (float): Accepted for signature compatibility; unused — the solver is local
  and can only fail, not time out.

**Returns:** `List[float]` — the joint solution. **Raises** `IKError` on failure.

```python
q_sol = arm.ik([0.5, 0.0, 0.4, 0.0, 1.5708, 0.0])
```

#### plan_movel(q_start, pose_goal)

Plan a straight-line Cartesian path from a start configuration to a goal pose.
**Simulation-only** (`litearm` plans internally, inside `move_l`).

**Parameters:**

- `q_start` (List[float]): Start joint angles.
- `pose_goal` (Tuple): `(position, rotation_matrix)` target pose.

**Returns:** `List[List[float]]` — list of joint waypoints.

```python
path = arm.plan_movel(current_q, (target_pos, target_R))
```

Note the planners take the `(position, R)` pair, not the 6-vector: they are the
simulation's own helpers, and `fk()` produces exactly what they consume.

#### plan_movec(q_start, pose_via, pose_goal)

Plan a circular-arc Cartesian path through a via-point. Simulation-only.

**Parameters:**

- `q_start` (List[float]): Start joint angles.
- `pose_via` (Tuple): `(position, rotation_matrix)` via-point pose.
- `pose_goal` (Tuple): `(position, rotation_matrix)` goal pose.

**Returns:** `List[List[float]]` — list of joint waypoints.

```python
path = arm.plan_movec(q_start, pose_via, pose_goal)
```

#### plan_movep(q_start, poses_goal)

Plan a multi-waypoint Cartesian path. Simulation-only.

**Parameters:**

- `q_start` (List[float]): Start joint angles.
- `poses_goal` (List[Tuple]): List of `(position, rotation_matrix)` waypoint poses.

**Returns:** `List[List[float]]` — list of joint waypoints.

```python
path = arm.plan_movep(q_start, [pose1, pose2, pose3])
```

### 4.2 Motion Control

All motion methods are blocking — they return when the arm has arrived, and raise if it
never does. They do **not** return a boolean.

`speed` on every motion call is a 0..1 fractional multiplier of the trajectory rate
(0.0 is legal and means "don't move"; clamped internally so it cannot divide by zero).
It is not a percentage — percentages are `set_speed(percent)`, a separate global
governor. The two multiply.

#### movej(q, speed=1.0)

Joint-space point-to-point motion; each axis runs its own trapezoidal profile, so the
intermediate TCP path is not predictable.

**Parameters:**

- `q` (Sequence[float]): Target joint angles [rad].
- `speed` (float): Fractional speed multiplier, 0..1 (default 1.0).

**Returns:** `RobotState` — the state it arrived in.
**Raises:** `MotionTimeoutError` if it never arrives within `move_timeout`;
`MotorFaultError` if the arm faults or stops first.

```python
arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)
```

#### movej_sync(q, speed=1.0)

Same target semantics as `movej`, but all axes share one time base
(`minimum_jerk_trajectory`), so they start and finish together.

**Returns:** `RobotState`

#### move_p(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03)

Joint-space point-to-point to a Cartesian target: IK to a configuration, then `movej`
there. Tolerances are in metres and radians.

**Returns:** `RobotState`

#### move_l(pose, speed=1.0, wait=True)

Straight-line Cartesian motion. The path is planned as IK waypoints and each is played
in turn.

**Parameters:**

- `pose`: Target pose, in any form `ik()` accepts.
- `speed` (float): Fractional speed multiplier.
- `wait` (bool): Block until the plan settles (default `True`).

**Returns:** `CartPlan` with `ok`, `err`, `n_wp`, `plan_us`, `started_busy`, `settled`,
`q_final`, `settle_err_rad`.
**`ok` means the plan was built and issued — never "stopped on target."**
`ok=True, settled=False` means the plan was superseded by a later command.

```python
x, y, z, r, p, yw = arm.get_tcp().value
plan = arm.move_l([x, y, z - 0.1, r, p, yw], speed=0.2)
print(plan.ok, plan.n_wp, plan.settled)
```

#### move_c(pose_start, pose_via, pose_goal, speed=1.0, wait=True)

Circular arc through a via-point. Three explicit poses, as the SDK takes them.

**Returns:** `CartPlan`

```python
plan = arm.move_c(start_pose, via_pose, goal_pose, speed=0.2)
```

#### move_path(poses, speed=1.0, wait=True)

Multi-waypoint Cartesian path. Takes a list of poses.

**Returns:** `CartPlan`

```python
plan = arm.move_path([pose1, pose2, pose3], speed=0.2)
```

#### home(*, timeout=None)

Move to the firmware's home pose at the hard-coded low safety speed (0.10). `timeout`
defaults to `move_timeout`.

**Returns:** `RobotState`

#### park()

Relax to the park pose. Moves at low speed and returns once the pose is commanded.
Simulation-realistic but not a firmware park.

**Returns:** `None`

#### recover_joint_limits(speed=0.05, inset_rad=0.0, **kwargs)

Slowly return out-of-limit joints to safe boundaries. Simulation-only.

**Returns:** `RobotState`

#### replay_joint_path(q_path, speed=1.0, goto_start=True, goto_speed=0.3, max_cycles=None, **kwargs)

Replay a sequence of joint configurations. Simulation-only.

**Parameters:**

- `q_path` (List[List[float]]): List of joint positions.
- `speed` (float): Playback speed multiplier.
- `goto_start` (bool): Move to the first waypoint before replaying.
- `goto_speed` (float): Speed for the `goto_start` move.

**Returns:** `RobotState`

```python
arm.replay_joint_path(path, speed=0.5)
```

#### replay_trajectory(traj_q, speed=1.0, goto_start=True, goto_speed=0.3, max_cycles=None, check_singularity=True, **kwargs)

Replay a `JointTrajectory` or path. Accepts `JointTrajectory` objects, dicts with
`frames`, or a raw `List[List[float]]`. Simulation-only.

**Returns:** `RobotState`

```python
traj = arm.record_trajectory(duration_s=3.0)
arm.replay_trajectory(traj, speed=1.0)
```

#### replay_timed_trajectory(traj_q, traj_t, speed=1.0, goto_start=True, goto_speed=0.3, simplify_tolerance_rad=0.01, max_cycles=None, **kwargs)

Replay a measured trajectory on its recorded time axis. Simulation-only.

**Parameters:**

- `traj_q` (List[List[float]]): Joint positions over time.
- `traj_t` (List[float]): Timestamps for each frame.
- `speed` (float): Playback speed multiplier.

**Returns:** `RobotState`

#### play_trajectory(trajectory, speed=1.0, goto_start=True, goto_speed=0.3, verify_robot=True, simplify_tolerance_rad=0.01, max_cycles=None, **kwargs)

Load and replay a saved trajectory. Accepts a file path (string), a `JointTrajectory`
object, or a dict. Simulation-only.

**Returns:** `RobotState`

```python
arm.play_trajectory("trajectories/my_traj.json", speed=1.0)
```

#### record_trajectory(output="trajectories", duration_s=None, sample_rate_hz=100.0, filter_alpha=0.15, name=None, **kwargs)

Record a trajectory by sampling the arm. Simulation-only; the SDK has no trajectory
object at all. The position loop is down while sampling, so what you record is the arm
moving freely.

**Parameters:**

- `output` (str): Output directory or file path (used by `traj.save`, not by recording).
- `duration_s` (Optional[float]): Recording duration (default: 5.0).
- `sample_rate_hz` (float): Sampling rate (Hz).
- `name` (Optional[str]): Trajectory name.

**Returns:** `JointTrajectory`

```python
traj = arm.record_trajectory(duration_s=3.0, name="my_traj")
traj.save("trajectories/my_traj.json")
```

#### hold(kp_scale=3.0, max_cycles=None, **kwargs)

Hold the current position with increased stiffness. Simulation-only.

**Returns:** `RobotState`

```python
arm.hold()
```

#### zero_g(period=0.04) / zero_g_start(period=0.04) / zero_g_stop(raise_on_lost=False)

Zero-gravity (free-drag) mode. Both spellings work, as on the real arm:

```python
with arm.zero_g():      # exits the mode on block exit
    ...
arm.zero_g(); ...; arm.zero_g_stop()
```

`zero_g_active` is the boolean property, `zero_g_error` the last error, if any.
While zero-g is held, **every action command raises** `InvalidCommandError` — the SDK's
guard, and the simulation enforces it too. Drain-power commands (`emergency_stop`,
`disable`, `zero_g_stop`) stay reachable.

#### joint_impedance(q_des, K, B, tau_max=None, engage_sec=0.3, max_cycles=None, **kwargs)

Joint-space impedance control (simplified — sets the target and holds). Simulation-only.

**Returns:** `RobotState`

#### cartesian_impedance(q_des, K_cart, B_cart, v_des=None, tau_max=None, engage_sec=0.3, max_cycles=None, sigma_min_thresh=None, max_ori_err=None, measured_overspeed_factor=None, vel_max=None, **kwargs)

Cartesian-space impedance control (simplified — sets the target and holds). Simulation-only.

**Returns:** `RobotState`

#### joint_follow(K=None, B=None, speed_limit=None, accel_limit=None, engage_sec=0.3, max_cycles=None, duration_s=None, q_des=None, **kwargs)

Follow an external target provider (stub). Simulation-only.

**Returns:** `RobotState`

### 4.3 State Reading

All three readers return a `Msg` envelope, exactly as `litearm-python` 2.0+ does. The
payload is in `.value`; `.hz` is the measured frame arrival rate and `.timestamp` is
the local `time.monotonic()` of the most recent frame. `value` is `None` when there is
no frame yet.

#### get_state(refresh=False, timeout=0.5)

Get the latest robot state.

**Parameters:**

- `refresh` (bool): In the SDK, `refresh=False` returns the last frame *this caller*
  read — litearm-python has no background reader thread, so a `False` read can be stale.
  The simulation always has a fresh frame and returns it either way; pass `refresh=True`
  when you mean "right now" so the code reads the same against both backends.
- `timeout` (float): Accepted for compatibility; the simulation always answers.

**Returns:** `Msg[RobotState]`. `RobotState` carries `q`, `dq`, `tau`, `mode`,
`mode_name`, `enabled`, `faulted`, `cart_busy`, `errs`, `temps`, `watchdog`,
`robot_serial`, `config_checksum_sha256`.

```python
state = arm.get_state().value
print(state.q)            # Joint positions [rad]
print(state.dq)           # Joint velocities [rad/s]
print(state.tau)          # Joint torques [Nm]
print(state.mode_name)    # "idle", "moving", "estop", "zero_g", ...
print(state.enabled, state.faulted)
print(arm.get_state().hz) # Frame arrival rate
```

#### get_status_now(timeout=0.5)

Force a fresh read. Same return type as `get_state()`.

#### get_tcp(timeout=0.6)

Get the current TCP pose as `[x, y, z, roll, pitch, yaw]`.

**Returns:** `Msg[tuple]` — 6 floats, position in metres and rotation in radians.
`Msg.value` is `None` if there is no frame.

```python
x, y, z, r, p, yw = arm.get_tcp().value
```

### 4.4 Emergency Stop / Enable

#### emergency_stop()

Cut motor torques immediately — an emergency stop, not a graceful deceleration. Also
reachable while zero-g is held.

```python
arm.emergency_stop()
```

#### reset()

Clear the stop condition, restore the speed governor to its default, and resume normal
operation.

```python
arm.reset()
```

#### enable(attempts=12)

Enable motors and hold the current pose. `attempts` is the retry count, as in the SDK.

```python
arm.enable()
```

#### disable()

Disable motors. **Warning:** the arm will drop under gravity.

```python
arm.disable()
```

#### clear_faults()

Clear motor faults. Returns `None`, like the SDK (the 0.1 version returned `[]`).

```python
arm.clear_faults()
```

### 4.5 Parameters

#### set_speed(percent)

Set the global speed governor, 0..100. This multiplies with the per-call `speed`
fraction:

```python
arm.set_speed(50)                     # half the configured rate, globally
arm.movej([0.0]*7, speed=0.5)         # and half again for this move
```

`reset()` restores the governor to its default.

#### set_motion_mode(mode)

Set the firmware motion mode by index. `mode_name` on `RobotState` reports the
resulting mode as a string.

#### set_payload(mass, com=(0.0, 0.0, 0.0))

Set the payload parameters, exactly as the SDK takes them. **Returns `None`** — it does
not report back what was stored. In the simulation this does not change the physics:
gravity and inertia come from the URDF.

```python
arm.set_payload(0.5, com=(0.0, 0.0, 0.05))
```

#### get_payload()

Get the current payload settings. Simulation-only.

**Returns:** `dict` with `mass` and `com`.

#### set_gains(kp=None, kd=None)

Set the PD controller gains. Simulation-only — the SDK's gain lives in the firmware.

**Parameters:**

- `kp` (Optional[Any]): Position gain (scalar or list of 7).
- `kd` (Optional[Any]): Velocity gain (scalar or list of 7).

**Returns:** `dict` with `kp` and `kd` lists.

```python
arm.set_gains(kp=[300]*7, kd=[10]*7)
```

#### get_gains()

Get the current PD gains. Simulation-only.

**Returns:** `dict` with `kp` and `kd` lists.

```python
gains = arm.get_gains()
print(gains["kp"])
```

#### set_installation(base_rpy=None, gravity=None)

Set installation parameters (stored for API compatibility). Simulation-only.

**Returns:** `dict`

#### get_installation()

Get installation parameters. Simulation-only.

**Returns:** `dict` with `base_rpy` and `gravity`.

#### get_joint_limits()

Get joint position limits. Simulation-only.

**Returns:** `dict` with keys `joint0` through `joint6`, each with `min` and `max`.

```python
limits = arm.get_joint_limits()
print(limits["joint0"]["min"], limits["joint0"]["max"])
```

#### set_joint_limits(limits)

Set joint position limits (stored for API compatibility). Simulation-only.

**Returns:** `dict`

#### get_zero_offsets() / set_zero_offsets(offsets)

Get/set zero offsets. Simulation-only.

**Returns:** `dict` with `offsets` (list of 7 zeros) / `dict`

#### get_end_effector() / set_end_effector(config)

Get/set the end-effector configuration. Simulation-only. `get_end_effector()` returns
`type = "none"`.

**Returns:** `dict`

#### get_cartesian_limits() / set_cartesian_limits(limits)

Get/set the Cartesian workspace limits. Simulation-only.

**Returns:** `dict` with `x`, `y`, `z` ranges

#### get_collision_config() / set_collision_config(config)

Get/set the collision detection configuration. Simulation-only. `get_collision_config()`
returns an empty dict.

**Returns:** `dict`

#### set_joint_positions(q)

Teleport the simulation's joints directly, bypassing the controller. Simulation-only:
the real arm cannot be teleported. Used by mirroring to place the arm at the real pose.

**Returns:** `None`

#### n / firmware / port

Identity properties. `n` is the joint count (the SDK's name for it), `firmware` is a
shape-compatible banner string (`"PyBulletSim-7J"`) — **not** a real firmware version —
and `port` is the serial port the simulation was configured with (`None`).

#### move_js(q, dq=None, tau_ff=None) / send_mit(idx, q, dq, kp, kd, tau) / send_mit_all(q, dq, kp, kd, tau)

Streaming joint commands, same signatures as the SDK. Useful for custom controllers
that want to write torques directly instead of going through `movej`.

### 4.6 Peripheral Devices

Simulated device proxies that return canned success responses. Simulation-only.

#### device(device_id)

Get a simulated device proxy by ID.

**Parameters:**

- `device_id` (str): Device identifier (e.g. `"hand_0"`, `"gripper_0"`).

**Returns:** `_SimDevice` — a proxy with methods: `open()`, `close()`, `set_gesture()`, `list_gestures()`, `set_force()`, `get_state()`, `get_status()`, `get_info()`, `connect()`, `disconnect()`, `clear_faults()`, `finger_move()`, `set_speed()`, `set_torque()`, `set_width()`, `get_width()`, `get_joints()`, `get_buttons()`.

```python
hand = arm.device("hand_0")
hand.open()
hand.set_gesture("pinch")
```

#### devices

Device manager (simulated). Supports `arm.devices["hand_0"]` syntax.

```python
arm.devices["hand_0"].open()
```

#### hand

Backward-compatible hand property. Equivalent to `arm.device("hand_0")`.

```python
arm.hand.open()
arm.hand.close()
```

### 4.7 System / Settings

Stubs kept for API compatibility with the real arm's system management. Simulation-only.

#### get_system_stats()

**Returns:** `dict` with `cpu`, `memory`, `board_temp`, `uptime`.

#### get_logs(page=1, size=50, search="")

**Returns:** `dict` with `logs`, `total`, `page`, `size`.

#### restart_service()

**Returns:** `dict` with `status`.

### 4.8 Trajectory Management

Simulation-only stubs.

#### start_recording() / stop_recording() / discard_recording() / get_recording_state()

**Returns:** `dict` with `status: "recording"` / `"stopped"` / `"discarded"` / `recording: False`.

#### list_trajectories()

**Returns:** `dict` with `trajectories: []`.

#### save_trajectory(id, name, points, duration=None)

**Returns:** `dict` with `id`, `name`, `saved`.

#### delete_trajectory(id)

**Returns:** `dict` with `id`, `deleted`.

#### get_playback_state()

**Returns:** `dict` with `playing: False`.

### 4.9 Device Management

Simulation-only stubs.

#### list_device_types()

**Returns:** `List[dict]` — simulated hand and gripper.

```python
types = arm.list_device_types()
# [{"category": "hand", "subtype": "sim_hand", "label": "Simulated Hand"},
#  {"category": "gripper", "subtype": "sim_gripper", "label": "Simulated Gripper"}]
```

#### connect_device(category, subtype, device_id="end_0", can_iface="", config=None)

**Returns:** `dict` with `device_id`, `connected`.

#### disconnect_device(device_id="end_0")

**Returns:** `dict` with `device_id`, `disconnected`.

#### get_active_device(device_id="end_0")

**Returns:** `dict` with `device_id`, `active`.

### 4.10 Teleop

Simulation-only stubs.

#### enter_teleop(mode, **params)

**Returns:** `dict` with `mode`, `active: True`.

#### exit_teleop()

**Returns:** `dict` with `active: False`.

#### get_teleop_status()

**Returns:** `dict` with `active: False`, `mode: "none"`.

### 4.11 Mirror Mode

#### mirror_from(real_arm, rate_hz=50.0)

Start mirroring the state of a real arm into this simulation. A daemon thread polls
`real_arm` and puts each frame's joint vector into the simulation, so the arm on screen
follows the physical one.

**Parameters:**

- `real_arm` (Any): Anything answering `get_state(refresh=True)` with a `Msg` whose
  `value.q` is the joint vector — normally a connected `litearm.Arm`.
- `rate_hz` (float): How often to poll. **Every frame is a serial round trip**, so this
  is real load on the wire, not a display preference. The default 50 Hz is deliberate;
  litearm-python has no background reader, and polling at the 500 Hz physics rate would
  swamp the link.

The read is `refresh=True` on purpose: `refresh=False` would hand back the last frame
*this caller* read, and a mirror that reports the same pose forever is exactly the
failure you cannot see.

Failures do not raise into your thread — they are recorded and readable through
`mirror_error` (`None` while healthy) with `mirroring` reporting whether the thread is
live. A dropped frame does not end the mirror.

Do not send commands to `real_arm` yourself while mirroring: the two readers steal each
other's frames. Call `stop_mirroring()` first.

```python
import litearm as pa
real = pa.Arm(port="/dev/ttyACM0").connect()
sim = PyBulletArm(render=True).connect()
sim.mirror_from(real)

# ... move the real arm by hand, teleop, or another program ...
# sim follows. Check sim.mirror_error if it seems stuck.
```

#### stop_mirroring()

Stop mirroring and join the thread (at most 2 s). Idempotent.

```python
sim.stop_mirroring()
```

### 4.12 DualArm

Control both a real arm and a simulated arm simultaneously. Sends the same motion
commands to both.

```python
from litearm_pybullet import DualArm

dual = DualArm(real_port="/dev/ttyACM0", render=True)
dual.start()
dual.enable()                     # explicit — the constructor never powers the arm

# Both arms execute the same motion
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

state = dual.get_sim_state().value
tcp = dual.get_sim_tcp().value

dual.close()
```

**Constructor parameters:**

- `real_port` (Optional[str]): Serial port for the real arm; `None` auto-detects the
  single STM32 CDC device.
- `sim_model_path` (Optional[str]): Path to the URDF model.
- `render` (bool): Whether to open the PyBullet GUI (default `True`).
- `mirror_first` (bool): If `True` (default), `start()` waits for the first mirrored
  frame before returning, so the simulation begins in the real arm's pose. If `False`,
  `start()` returns immediately and mirroring catches up in the background.
- `mirror_rate_hz` (float): Mirroring poll rate (default 50.0).

**Key methods:**

- `start()` — Start the simulation and the mirroring thread. Returns `self`.
- `movej(q, speed=1.0)` / `movej_sync(...)` / `move_p(...)` — Send to both arms.
  Returns `(real_result, sim_result)`, both `RobotState`.
- `move_l(...)` / `move_c(...)` / `move_path(...)` — Send to both arms.
  Returns `(CartPlan, CartPlan)`.
- `get_real_state(refresh=True, timeout=0.5)` — Real arm state, as a `Msg`.
- `get_sim_state()` — Simulation state, as a `Msg`.
- `get_tcp()` — Real arm TCP, as a `Msg`.
- `get_sim_tcp()` — Simulation TCP, as a `Msg`.
- `enable(attempts=12)` — Enable the real arm (and the simulation).
- `disable()` — Disable both.
- `emergency_stop()` — Emergency stop both.
- `reset()` — Clear the stop on both.
- `close()` — Close both. Idempotent.
- `real` / `sim` — The underlying arm objects.
- `mirroring` / `mirror_error` — Mirror thread state.

Both arms are always attempted: if the real arm raises, the simulation command still
runs, and the real arm's exception is the one you see. The real arm is never enabled
implicitly — `enable()` is a line you can point at in review.

## Three Modes

### Mode 1: Standalone Simulation

No real arm required. Pure simulation with PyBullet physics.

```python
from litearm_pybullet import PyBulletArm
from litearm_pybullet._compat import mat_to_rpy

with PyBulletArm(render=True) as arm:
    # Move to a comfortable pose
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

    # Read state
    print(f"Joint positions: {arm.get_state().value.q}")

    # Cartesian motion
    x, y, z, r, p, yw = arm.get_tcp().value
    arm.move_l([x, y, z - 0.1, r, p, yw], speed=0.2)

    # FK/IK — ik takes the 6-vector, fk returns the (pos, R) pair
    pos, R = arm.fk([0.0]*7)
    q_sol = arm.ik(list(pos) + mat_to_rpy(R))
```

### Mode 2: Mirror Mode

Simulation tracks the real arm's state in real time. Requires a connected
`litearm-python` (`pip install -e ../litearm-python`).

```python
import litearm as pa
from litearm_pybullet import PyBulletArm

# Connect to the real arm
real = pa.Arm(port=None).connect()      # None = auto-detect

# Create the simulation and start mirroring
sim = PyBulletArm(render=True).connect()
sim.mirror_from(real)

# The simulation now mirrors the real arm's motion.
# Move the real arm (by hand, teleop, or another program) and watch it follow.
if sim.mirror_error is not None:
    print("mirror failed:", sim.mirror_error)

# When done
sim.stop_mirroring()
sim.close()
real.close()
```

Alternatively, use the `MirrorMode` class:

```python
import litearm as pa
from litearm_pybullet import PyBulletArm, MirrorMode

real = pa.Arm(port=None).connect()
sim = PyBulletArm(render=True).connect()

mirror = MirrorMode(real, sim, rate_hz=50.0)
mirror.start()

# ... sim follows real arm ...
print(mirror.running, mirror.error)

mirror.stop()
sim.close()
real.close()
```

### Mode 3: Dual Control

Send commands to real and simulated arms simultaneously. The simulation mirrors the real
arm between commands.

```python
from litearm_pybullet import DualArm

dual = DualArm(real_port=None, render=True)
dual.start()
dual.enable()

# One command, both arms move
real_state, sim_state = dual.movej(
    [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2
)

# Cartesian motion
x, y, z, r, p, yw = dual.get_sim_tcp().value
dual.move_l([x, y, z - 0.1, r, p, yw], speed=0.1)

# Emergency stop both
dual.emergency_stop()

dual.close()
```

## Pose Format

`get_tcp()` and every motion method speak the SDK's 6-element form:

```python
pose = [x, y, z, roll, pitch, yaw]   # metres and radians
```

The three accepted spellings, normalized by the same helper the SDK uses
(`as_pose`):

```python
[x, y, z, roll, pitch, yaw]              # firmware form — what get_tcp() returns
([x, y, z], [[r00, r01, r02],            # position + 3x3 row-major rotation
             [r10, r11, r12],
             [r20, r21, r22]])
[[r00, r01, r02, x],                     # 4x4 homogeneous matrix
 [r10, r11, r12, y],
 [r20, r21, r22, z],
 [0,   0,   0,   1]]
```

A flat 9-element rotation is **not** accepted — it carries no position. Anything
malformed raises `InvalidCommandError` with the actual shape in the message.

`fk()` returns the `(position, R)` pair, and the `plan_*` helpers consume that same
pair — they are the simulation's own API, not the SDK's.

```python
pos, R = arm.fk([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0])
path = arm.plan_movel([0.0]*7, (pos, R))
```

In the neighbourhood of gimbal lock, roll and yaw are not unique. Both the firmware and
this package force `yaw = 0` there, but the two lock bands are different widths, so the
individual rpy components can differ while describing the same rotation. Compare
rotations, not components, if you are checking a pose yourself.

## Safety Notes

### enable() Is Explicit

Nothing in `PyBulletArm` or `DualArm` powers the physical arm implicitly. `DualArm.start()`
brings up the simulation and the mirror thread only; the line that makes the real arm
move is your `dual.enable()`, visible in review.

### disable() Drops the Arm

Calling `disable()` sets all motor torques to zero. Under gravity, the arm will collapse.
Use `enable()` to re-engage motors and hold the current pose.

### emergency_stop() Is Emergency Stop

`emergency_stop()` immediately cuts all motor torques — an emergency stop, not a graceful
deceleration. Use `reset()` to resume normal operation after investigating the cause.
It stays reachable while zero-g is held, as do `disable()` and `zero_g_stop()`.

### Motion is Blocking

All motion methods (`movej`, `movej_sync`, `move_p`, `move_l`, `move_c`, `move_path`,
`replay_*`) block the calling thread until the motion completes or fails. Completion
means the joints stayed inside `q_tol`/`dq_tol` for `arrive_frames` consecutive frames;
if that never happens you get `MotionTimeoutError` rather than a return value.

### Simulation is Not Real-Time Safe

The simulation loop runs at a fixed 500 Hz timestep, but the real-time synchronization is
approximate. Do not use simulation timing for safety-critical real-time guarantees.

### render=False for Headless

Use `render=False` for headless operation (e.g., in CI/CD pipelines, servers, or when
running many simulations in parallel). The PyBullet `DIRECT` connection mode is used,
which does not open a GUI window.

## FAQ

### Q: How do I run without a GUI?

Use `render=False`:

```python
arm = PyBulletArm(render=False).connect()
arm.movej([0.0]*7, speed=0.3)
state = arm.get_state().value
arm.close()
```

### Q: How do I swap between simulation and real arm?

Replace the import and constructor:

```python
# Real arm
import litearm
arm = litearm.Arm(port="/dev/ttyACM0").connect()

# Simulation
from litearm_pybullet import PyBulletArm
arm = PyBulletArm(render=True).connect()
```

All subsequent API calls are identical, down to the `Msg` envelope and the exception
classes.

### Q: Why does the arm jitter or oscillate?

The default PD gains are tuned for the real arm's motor dynamics. If you observe
oscillation, try lowering the gains:

```python
arm.set_gains(kp=[150]*7, kd=[8]*7)
```

### Q: Why does IK fail for some poses?

The IK solver uses PyBullet's built-in IK plus Levenberg-Marquardt refinement. If the
target pose is unreachable (outside the workspace, or in a singular configuration) it
raises `IKError`. Try a better `q_seed` or a different target pose. Note that
`IKError` derives from `litearm`'s error hierarchy when the SDK is installed, so
one `except` clause covers simulation and hardware.

### Q: Why did `movej` raise `MotionTimeoutError`?

The arm never got within `q_tol` of the target while slower than `dq_tol`, for
`arrive_frames` frames, within `move_timeout` seconds. Common causes: the target is
outside the joint limits, the arm is stalled against something, or `speed` is so low
that the move does not finish in time. There is no `settle_s` to lengthen — tune
`move_timeout` or the tolerances in the constructor.

### Q: How do I record and replay a trajectory?

```python
# Record (the position loop is down while sampling)
traj = arm.record_trajectory(duration_s=5.0, name="demo")
traj.save("trajectories/demo.json")

# Replay
arm.play_trajectory("trajectories/demo.json", speed=1.0)
```

Trajectories are a simulation-only concept — `litearm-python` has no trajectory object.

### Q: Can I use numpy arrays for poses?

Methods accept numpy arrays but always return plain Python lists. Convert if needed:

```python
import numpy as np
pos = np.array([0.5, 0.0, 0.4])
arm.move_l([pos[0], pos[1], pos[2], 0.0, 0.0, 0.0], speed=0.2)
```

### Q: What does the mirror mode actually do?

Mirror mode reads the real arm's joint state (`get_state(refresh=True).value.q`) at a
configurable rate (default 50 Hz) and writes it straight into the simulation's joints.
The simulation shows the real arm's pose. It is a poll loop, not a subscription: every
frame is a serial round trip, which is why the rate is capped and why each read asks for
a fresh frame.

### Q: How is DualArm different from MirrorMode?

- **MirrorMode**: Simulation passively follows the real arm. You move the real arm (by hand, teleop, or another program) and the simulation tracks it.
- **DualArm**: You send commands that execute on both arms simultaneously. Between commands, the simulation mirrors the real arm to stay synchronized.

### Q: The simulation appears frozen or stuck. What should I do?

Make sure you called `connect()` (or used the context manager). Check that
`emergency_stop()` or `disable()` has not been called. If using `movej`, verify the
simulation loop is running by checking `get_state().value.mode_name`. If you are
mirroring, look at `mirror_error` — a dead serial link shows up there, not as an
exception.

## Dependencies

### Core Dependencies

- `pybullet >= 3.2.5`
- `numpy >= 1.21`

### Optional Dependencies (mirror/dual mode)

- `litearm-python` — not on PyPI; install with `pip install -e ../litearm-python`

## Known Limitations

- The controller is joint-space PD with gravity/Coriolis feed-forward; it does not include full computed-torque or friction feed-forward
- The simulation has no serial link, so `connect(port=...)` ignores its argument
- Hand/gripper/teach-pendant are simulated proxies returning canned responses
- System management/logs/teleop are API-compatible stub implementations
- Collision detection is not configured by default; `get_collision_config()` returns an empty dict
- Trajectory recording drops the position loop so the arm can be moved by hand; this differs from the real arm's recording behavior
- The simulation is not real-time safe; timing is approximate

## Migrating from 0.1 (the litearm-python era)

The distribution name `litearm-python` covers two SDKs that share only the import
line. 0.1.0 was a Zenoh client that talked to a separate `litearm-server` process,
addressed with `endpoint=`/`arm_id=`. 2.1 is a thin client that speaks the firmware's
protocol over USB CDC, and its constructor takes `port=`. The backend here uses 2.1.

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
| `arm.movec(via, goal, speed)` | `arm.move_c(pose_start, via, goal, speed)` |
| `arm.movep(poses, speed)` | `arm.move_path(poses, speed)` |
| `arm.request_stop()` / `clear_stop()` | `arm.emergency_stop()` / `arm.reset()` |
| `arm.zero_gravity()` | `arm.zero_g()` |

The seven old names still exist as pure forwarding aliases, each emitting a
`DeprecationWarning`, so 0.1 code runs — but it should be updated.

**`settle_s` is gone** and motion methods return `RobotState`/`CartPlan` rather than
`bool`; failures raise (`MotionTimeoutError`, `IKError`, `InvalidCommandError`, …).
Arrival is decided by `|q − target| < q_tol (0.03)` and `|dq| < dq_tol (0.10)` over
`arrive_frames (3)` consecutive frames.

## License

Proprietary
