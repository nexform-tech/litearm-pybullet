# litearm-pybullet Developer Guide

> Based on litearm-pybullet v0.1.0

## Project Structure

```
litearm-pybullet/
├── pyproject.toml
├── README.md
├── .gitignore
├── src/
│   └── litearm_pybullet/
│       ├── __init__.py         # Exports PyBulletArm, DualArm, MirrorMode
│       ├── arm.py              # Core: PyBulletArm class (API-compatible with litearm.Arm)
│       ├── controller.py       # PID joint controller + trajectory generator
│       ├── kinematics.py       # Forward/inverse kinematics (FK/IK) + path planning
│       ├── mirror.py           # DualArm (dual control) + MirrorMode (mirroring)
│       ├── _litearm/           # Vendored SDK subset (trajectory types)
│       │   ├── __init__.py
│       │   └── types.py        # JointTrajectory / TrajectoryFrame
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
│   └── test_pybullet_arm.py
└── docs/
    └── DEVELOPER_GUIDE.md
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
    │ PyBullet    │    │ PyBullet + Zenoh   │
    │ Physics     │    │ → litearm-server   │
    │ PID Ctrl    │    │ → Real Arm         │
    └─────────────┘    └────────────────────┘
```

### Module Responsibilities

| Module | Responsibility |
|--------|---------------|
| `arm.py` | Main class `PyBulletArm`, API-compatible with `litearm.Arm`, background thread running physics simulation |
| `controller.py` | `JointPIDController` position controller + `TrajectoryGenerator` trapezoidal velocity trajectory generation |
| `kinematics.py` | `Kinematics` class: FK (forward kinematics), IK (damped least-squares with Levenberg-Marquardt refinement) + path planning (straight line / circular arc / multi-waypoint) |
| `mirror.py` | `DualArm` (simultaneous control of real + sim), `MirrorMode` (sim tracks real arm) |
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
│ State Cache        │  ← _state_cache (updated each step)
│ get_state() reads  │
└───────────────────┘
```

## Core Design Principles

### 1. API Compatibility

`PyBulletArm` implements the complete `litearm.Arm` API interface. Users only need to replace `litearm.Arm(endpoint=...)` with `PyBulletArm(render=True)` to run the same control code in simulation.

### 2. Three Operating Modes

| Mode | Class | Creation | Requires Real Arm |
|------|-------|----------|-------------------|
| Standalone | `PyBulletArm` | `PyBulletArm(render=True)` | No |
| Mirror | `PyBulletArm` + `mirror_from()` | `sim.mirror_from(real)` | Yes |
| Dual Control | `DualArm` | `DualArm(real_endpoint=...)` | Yes |

### 3. Thread Safety

- The simulation loop runs in a background thread, with data access protected by `self._lock`
- Kinematics computations use the live PyBullet client (joint state resets are scoped to the FK/IK call)
- `get_state()` returns a snapshot of the state cache, does not block the simulation loop

### 4. Zero-Dependency Standalone Mode

Standalone simulation mode requires only `pybullet>=3.2.5` and `numpy>=1.21`. Mirror/dual modes require the additional `litearm-pybullet[mirror]` install, and their dependency on `litearm-python` is optional and lazily loaded.

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

## Requirements and Installation

### Requirements

- `pybullet >= 3.2.5`
- `numpy >= 1.21`
- `Python >= 3.10`

### Optional Dependencies (mirror/dual mode)

- `litearm-python` (installed via `pip install "litearm-pybullet[mirror]"`)

### Installation

```bash
# Standalone simulation (no real-arm communication needed)
pip install litearm-pybullet

# Mirror/dual mode (requires real-arm communication)
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
    # Joint-space motion
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

    # Cartesian straight-line motion
    pos, R = arm.get_tcp_pose()
    arm.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.1)

    # Read state
    state = arm.get_state()
    print(state["q"])
```

## Connection Management

### start()

Starts the background simulation thread. The simulation loop runs at 500 Hz (1/dt where dt=0.002). If `start()` is not called explicitly, motion methods will auto-start the simulation on first use.

```python
arm = PyBulletArm(render=True)
arm.start()
```

### close()

Stops the simulation and disconnects the PyBullet client. Stops mirroring if active.

```python
arm.close()
```

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

Forward kinematics: compute TCP pose from joint angles.

**Parameters:**
- `q` (List[float]): Joint angles [rad], length 7.

**Returns:** `(position, rotation_matrix)` where position is `[px, py, pz]` and rotation_matrix is a 3x3 row-major matrix.

```python
pos, R = arm.fk([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0])
```

#### ik(pos_d, R_d, q_seed=None)

Inverse kinematics: compute joint angles from desired TCP pose. Uses PyBullet's built-in IK as an initial seed, then refines with Levenberg-Marquardt.

**Parameters:**
- `pos_d` (List[float]): Desired position `[x, y, z]`.
- `R_d` (List[List[float]]): Desired 3x3 rotation matrix.
- `q_seed` (Optional[List[float]]): Initial guess for joint angles (default: zeros).

**Returns:** `(q, success)` where `q` is the joint solution and `success` is a boolean.

```python
q_sol, ok = arm.ik([0.5, 0.0, 0.4], [[1,0,0],[0,1,0],[0,0,1]])
```

#### plan_movel(q_start, pose_goal)

Plan a straight-line Cartesian path from a start configuration to a goal pose.

**Parameters:**
- `q_start` (List[float]): Start joint angles.
- `pose_goal` (Tuple): `(position, rotation_matrix)` target pose.

**Returns:** `List[List[float]]` -- list of joint waypoints.

```python
path = arm.plan_movel(current_q, (target_pos, target_R))
```

#### plan_movec(q_start, pose_via, pose_goal)

Plan a circular-arc Cartesian path through a via-point.

**Parameters:**
- `q_start` (List[float]): Start joint angles.
- `pose_via` (Tuple): `(position, rotation_matrix)` via-point pose.
- `pose_goal` (Tuple): `(position, rotation_matrix)` goal pose.

**Returns:** `List[List[float]]` -- list of joint waypoints.

```python
path = arm.plan_movec(q_start, pose_via, pose_goal)
```

#### plan_movep(q_start, poses_goal)

Plan a multi-waypoint Cartesian path.

**Parameters:**
- `q_start` (List[float]): Start joint angles.
- `poses_goal` (List[Tuple]): List of `(position, rotation_matrix)` waypoint poses.

**Returns:** `List[List[float]]` -- list of joint waypoints.

```python
path = arm.plan_movep(q_start, [pose1, pose2, pose3])
```

### 4.2 Motion Control

All motion methods are blocking -- they run until the motion completes or is stopped.

#### movej(q_target, speed=1.0, settle_s=1.0, max_cycles=None, allow_start_collision_recovery=False, **kwargs)

Move to a joint target. Joint-space point-to-point motion with trapezoidal velocity profile.

**Parameters:**
- `q_target` (List[float]): Target joint angles [rad].
- `speed` (float): Speed multiplier (0.0-1.0+, default 1.0).
- `settle_s` (float): Settling time after reaching target (seconds).
- `max_cycles` (Optional[int]): Maximum control cycles (default: unlimited).

**Returns:** `bool` -- `True` on success, `False` if stopped.

```python
arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)
```

#### recover_joint_limits(speed=0.05, settle_s=0.5, max_cycles=None, inset_rad=0.0, **kwargs)

Slowly return out-of-limit joints to safe boundaries.

**Parameters:**
- `speed` (float): Recovery speed.
- `inset_rad` (float): Inset from joint limits (rad).

**Returns:** `bool`

```python
arm.recover_joint_limits(speed=0.05)
```

#### movel(pose_goal, speed=1.0, settle_s=0.8, max_cycles=None, **kwargs)

Move in a straight Cartesian line. Plans a path then executes it via IK waypoints.

**Parameters:**
- `pose_goal` (Tuple): `(position, rotation_matrix)` target pose.
- `speed` (float): Speed multiplier.
- `settle_s` (float): Settling time after reaching target.

**Returns:** `bool`

```python
pos, R = arm.get_tcp_pose()
arm.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.2)
```

#### movec(pose_via, pose_goal, speed=1.0, settle_s=0.8, max_cycles=None, **kwargs)

Move in a circular arc through a via-point.

**Parameters:**
- `pose_via` (Tuple): `(position, rotation_matrix)` via-point pose.
- `pose_goal` (Tuple): `(position, rotation_matrix)` goal pose.
- `speed` (float): Speed multiplier.

**Returns:** `bool`

```python
arm.movec(pose_via, pose_goal, speed=0.2)
```

#### movep(poses_goal, speed=1.0, settle_s=0.8, max_cycles=None, **kwargs)

Move through multiple Cartesian waypoints.

**Parameters:**
- `poses_goal` (List[Tuple]): List of `(position, rotation_matrix)` waypoints.
- `speed` (float): Speed multiplier.

**Returns:** `bool`

```python
arm.movep([pose1, pose2, pose3], speed=0.2)
```

#### replay_joint_path(q_path, speed=1.0, settle_s=0.5, goto_start=True, goto_speed=0.3, max_cycles=None, **kwargs)

Replay a sequence of joint configurations.

**Parameters:**
- `q_path` (List[List[float]]): List of joint positions.
- `speed` (float): Playback speed multiplier.
- `goto_start` (bool): Whether to move to the first waypoint before replaying.
- `goto_speed` (float): Speed for the goto_start move.

**Returns:** `bool`

```python
arm.replay_joint_path(path, speed=0.5)
```

#### replay_trajectory(traj_q, speed=1.0, goto_start=True, goto_speed=0.3, max_cycles=None, check_singularity=True, **kwargs)

Replay a `JointTrajectory` or path. Accepts `JointTrajectory` objects, dicts with `frames`, or raw `List[List[float]]`.

**Parameters:**
- `traj_q` (Any): JointTrajectory, dict, or list of joint positions.
- `speed` (float): Playback speed multiplier.

**Returns:** `bool`

```python
traj = arm.record_trajectory(duration_s=3.0)
arm.replay_trajectory(traj, speed=1.0)
```

#### replay_timed_trajectory(traj_q, traj_t, speed=1.0, goto_start=True, goto_speed=0.3, simplify_tolerance_rad=0.01, max_cycles=None, **kwargs)

Replay a measured trajectory on its recorded time axis.

**Parameters:**
- `traj_q` (List[List[float]]): Joint positions over time.
- `traj_t` (List[float]): Timestamps corresponding to each frame.
- `speed` (float): Playback speed multiplier.

**Returns:** `bool`

#### play_trajectory(trajectory, speed=1.0, goto_start=True, goto_speed=0.3, verify_robot=True, simplify_tolerance_rad=0.01, max_cycles=None, **kwargs)

Load and replay a saved trajectory. Accepts a file path (string), a `JointTrajectory` object, or a dict.

**Parameters:**
- `trajectory` (Union[Any, str]): Trajectory file path, JointTrajectory object, or dict.

**Returns:** `bool`

```python
arm.play_trajectory("trajectories/my_traj.json", speed=1.0)
```

#### record_trajectory(output="trajectories", duration_s=None, sample_rate_hz=100.0, filter_alpha=0.15, name=None, **kwargs)

Record a trajectory. Disables motors during recording to allow free-hand manipulation.

**Parameters:**
- `output` (str): Output directory or file path (unused in simulation).
- `duration_s` (Optional[float]): Recording duration (default: 5.0).
- `sample_rate_hz` (float): Sampling rate (Hz).
- `name` (Optional[str]): Trajectory name.

**Returns:** `JointTrajectory` -- the recorded trajectory object.

```python
traj = arm.record_trajectory(duration_s=3.0, name="my_traj")
traj.save("trajectories/my_traj.json")
```

#### hold(kp_scale=3.0, max_cycles=None, **kwargs)

Hold current position with increased stiffness.

**Returns:** `bool`

```python
arm.hold()
```

#### zero_gravity(max_cycles=None, duration_s=None, measured_overspeed_factor=None, vel_max=None, **kwargs)

Enable zero-gravity (free-drag) mode. Disables motors so the arm can be moved by hand.

**Parameters:**
- `duration_s` (Optional[float]): If set, auto-re-enable after this duration.

**Returns:** `bool`

```python
arm.zero_gravity(duration_s=10.0)
```

#### joint_impedance(q_des, K, B, tau_max=None, engage_sec=0.3, max_cycles=None, **kwargs)

Joint-space impedance control (simplified -- sets target and holds).

**Parameters:**
- `q_des` (List[float]): Desired joint positions.
- `K` (Any): Stiffness matrix/gains.
- `B` (Any): Damping matrix/gains.

**Returns:** `bool`

#### cartesian_impedance(q_des, K_cart, B_cart, v_des=None, tau_max=None, engage_sec=0.3, max_cycles=None, sigma_min_thresh=None, max_ori_err=None, measured_overspeed_factor=None, vel_max=None, **kwargs)

Cartesian-space impedance control (simplified -- sets target and holds).

**Parameters:**
- `q_des` (List[float]): Desired joint positions.
- `K_cart` (Any): Cartesian stiffness matrix.
- `B_cart` (Any): Cartesian damping matrix.

**Returns:** `bool`

#### joint_follow(K=None, B=None, speed_limit=None, accel_limit=None, engage_sec=0.3, max_cycles=None, duration_s=None, **kwargs)

Follow an external target provider (stub).

**Returns:** `bool`

### 4.3 State Reading

#### get_state(refresh=False)

Get the latest robot state. Returns a state dict matching `litearm.Arm.get_state()` format.

**Returns:** `dict` with keys: `q`, `dq`, `tau`, `fault`, `errs`, `temps`, `state`, `feedback`, `watchdog`, `robot_serial`, `config_checksum_sha256`.

```python
state = arm.get_state()
print(state["q"])          # Joint positions [rad]
print(state["dq"])         # Joint velocities [rad/s]
print(state["tau"])        # Joint torques [Nm]
print(state["state"])      # "ready", "stopping", or "disconnected"
print(state["robot_serial"])  # "PYBULLET-SIM-001"
```

#### get_tcp_pose()

Get current TCP pose.

**Returns:** `(position, rotation_matrix)` -- position is `[px, py, pz]`, rotation_matrix is 3x3 row-major.

```python
pos, R = arm.get_tcp_pose()
print(f"TCP: {pos}")
```

### 4.4 Emergency Stop / Enable

#### request_stop()

Emergency stop the simulation. Sets motors to zero torque.

```python
arm.request_stop()
```

#### clear_stop()

Clear the stop condition and resume normal operation.

```python
arm.clear_stop()
```

#### enable()

Enable motors and hold current pose.

```python
arm.enable()
```

#### disable()

Disable motors. **Warning:** the arm will drop under gravity.

```python
arm.disable()
```

#### clear_faults()

Clear motor faults (no-op in simulation).

**Returns:** `List[Tuple[int, int]]` -- empty list.

```python
arm.clear_faults()
```

### 4.5 Parameters

#### set_gains(kp=None, kd=None)

Set PD controller gains.

**Parameters:**
- `kp` (Optional[Any]): Position gain (scalar or list of 7).
- `kd` (Optional[Any]): Velocity gain (scalar or list of 7).

**Returns:** `dict` with `kp` and `kd` lists.

```python
arm.set_gains(kp=[300]*7, kd=[10]*7)
```

#### get_gains()

Get current PD gains.

**Returns:** `dict` with `kp` and `kd` lists.

```python
gains = arm.get_gains()
print(gains["kp"])
```

#### set_payload(mass, com=(0.0, 0.0, 0.0))

Set payload parameters (stored for API compatibility, does not affect simulation).

**Returns:** `dict` with `mass` and `com`.

```python
arm.set_payload(0.5, com=(0.0, 0.0, 0.05))
```

#### get_payload()

Get current payload settings.

**Returns:** `dict` with `mass` and `com`.

#### set_installation(base_rpy=None, gravity=None)

Set installation parameters (stored for API compatibility).

**Returns:** `dict`

#### get_installation()

Get installation parameters.

**Returns:** `dict` with `base_rpy` and `gravity`.

#### get_joint_limits()

Get joint position limits.

**Returns:** `dict` with keys `joint0` through `joint6`, each with `min` and `max`.

```python
limits = arm.get_joint_limits()
print(limits["joint0"]["min"], limits["joint0"]["max"])
```

#### set_joint_limits(limits)

Set joint position limits (stored for API compatibility).

**Returns:** `dict`

#### get_zero_offsets()

Get zero offsets.

**Returns:** `dict` with `offsets` (list of 7 zeros).

#### set_zero_offsets(offsets)

Set zero offsets (stored for API compatibility).

**Returns:** `dict`

#### get_end_effector()

Get end-effector configuration.

**Returns:** `dict` with `type` (always `"none"`).

#### set_end_effector(config)

Set end-effector configuration (stored for API compatibility).

**Returns:** `dict`

#### get_cartesian_limits()

Get Cartesian workspace limits.

**Returns:** `dict` with `x`, `y`, `z` ranges.

#### set_cartesian_limits(limits)

Set Cartesian workspace limits (stored for API compatibility).

**Returns:** `dict`

#### get_collision_config()

Get collision detection configuration.

**Returns:** `dict` (empty).

#### set_collision_config(config)

Set collision detection configuration (stored for API compatibility).

**Returns:** `dict`

### 4.6 Peripheral Devices

Simulated device proxies that return canned success responses.

#### device(device_id)

Get a simulated device proxy by ID.

**Parameters:**
- `device_id` (str): Device identifier (e.g. `"hand_0"`, `"gripper_0"`).

**Returns:** `_SimDevice` -- a proxy with methods: `open()`, `close()`, `set_gesture()`, `list_gestures()`, `set_force()`, `get_state()`, `get_status()`, `get_info()`, `connect()`, `disconnect()`, `clear_faults()`, `finger_move()`, `set_speed()`, `set_torque()`, `set_width()`, `get_width()`, `get_joints()`, `get_buttons()`.

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

Methods for API compatibility with the real arm's system management.

#### get_system_stats()

Get system statistics (simulated -- returns zeros).

**Returns:** `dict` with `cpu`, `memory`, `board_temp`, `uptime`.

#### get_logs(page=1, size=50, search="")

Get system logs (simulated -- returns empty).

**Returns:** `dict` with `logs`, `total`, `page`, `size`.

#### restart_service()

Restart the service (simulated -- returns ok).

**Returns:** `dict` with `status`.

### 4.8 Trajectory Management

#### start_recording()

Start recording (simulated stub).

**Returns:** `dict` with `status: "recording"`.

#### stop_recording()

Stop recording (simulated stub).

**Returns:** `dict` with `status: "stopped"`.

#### discard_recording()

Discard current recording (simulated stub).

**Returns:** `dict` with `status: "discarded"`.

#### get_recording_state()

Get recording state.

**Returns:** `dict` with `recording: False`.

#### list_trajectories()

List saved trajectories (simulated -- returns empty list).

**Returns:** `dict` with `trajectories: []`.

#### save_trajectory(id, name, points, duration=None)

Save a trajectory (simulated stub).

**Returns:** `dict` with `id`, `name`, `saved`.

#### delete_trajectory(id)

Delete a trajectory (simulated stub).

**Returns:** `dict` with `id`, `deleted`.

#### get_playback_state()

Get playback state.

**Returns:** `dict` with `playing: False`.

### 4.9 Device Management

#### list_device_types()

List available simulated device types.

**Returns:** `List[dict]` -- simulated hand and gripper.

```python
types = arm.list_device_types()
# [{"category": "hand", "subtype": "sim_hand", "label": "Simulated Hand"},
#  {"category": "gripper", "subtype": "sim_gripper", "label": "Simulated Gripper"}]
```

#### connect_device(category, subtype, device_id="end_0", can_iface="", config=None)

Connect a simulated device (stub -- always succeeds).

**Returns:** `dict` with `device_id`, `connected`.

#### disconnect_device(device_id="end_0")

Disconnect a simulated device (stub -- always succeeds).

**Returns:** `dict` with `device_id`, `disconnected`.

#### get_active_device(device_id="end_0")

Get active device status.

**Returns:** `dict` with `device_id`, `active`.

### 4.10 Teleop

Stub methods for API compatibility.

#### enter_teleop(mode, **params)

Enter teleop mode (simulated stub).

**Returns:** `dict` with `mode`, `active: True`.

#### exit_teleop()

Exit teleop mode (simulated stub).

**Returns:** `dict` with `active: False`.

#### get_teleop_status()

Get teleop status.

**Returns:** `dict` with `active: False`, `mode: "none"`.

### 4.11 Mirror Mode

#### mirror_from(real_arm, rate_hz=50.0)

Start mirroring the state of a real arm into this simulation. The simulation's background loop reads the real arm's joint state and sets it as the controller target.

**Parameters:**
- `real_arm` (Any): A `litearm.Arm` instance connected to a real robot.
- `rate_hz` (float): Mirroring update rate (Hz).

```python
import litearm
real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
sim = PyBulletArm(render=True)
sim.start()
sim.mirror_from(real)  # sim follows real arm
```

#### stop_mirroring()

Stop mirroring the real arm.

```python
sim.stop_mirroring()
```

### 4.12 DualArm

Control both a real arm and a simulated arm simultaneously. Sends the same motion commands to both arms.

```python
from litearm_pybullet import DualArm

dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
dual.start()

# Both arms execute the same motion
dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

state = dual.get_sim_state()
tcp = dual.get_sim_tcp_pose()

dual.close()
```

**Constructor parameters:**
- `real_endpoint` (str): Zenoh endpoint for the real arm.
- `real_arm_id` (str): Arm identifier (default `"armA"`).
- `sim_model_path` (Optional[str]): Path to URDF model.
- `render` (bool): Whether to open the PyBullet GUI (default `True`).
- `mirror_first` (bool): If `True`, start mirroring real arm state to simulation before sending any commands (default `True`).

**Key methods:**
- `start()` -- Start simulation and optional mirroring.
- `movej(q_target, speed, ...)` -- Send movej to both arms. Returns `(real_result, sim_result)`.
- `movel(pose_goal, speed, ...)` -- Send movel to both arms.
- `movec(pose_via, pose_goal, speed, ...)` -- Send movec to both arms.
- `movep(poses_goal, speed, ...)` -- Send movep to both arms.
- `get_real_state()` -- Get real arm state.
- `get_sim_state()` -- Get simulation state.
- `get_tcp_pose()` -- Get real arm TCP pose.
- `get_sim_tcp_pose()` -- Get simulation TCP pose.
- `request_stop()` -- Emergency stop both arms.
- `clear_stop()` -- Clear stop on both arms.
- `close()` -- Close both arms.

## Three Modes

### Mode 1: Standalone Simulation

No real arm required. Pure simulation with PyBullet physics.

```python
from litearm_pybullet import PyBulletArm

with PyBulletArm(render=True) as arm:
    # Move to a comfortable pose
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.3)

    # Read state
    state = arm.get_state()
    print(f"Joint positions: {state['q']}")

    # Cartesian motion
    pos, R = arm.get_tcp_pose()
    target_pos = [pos[0], pos[1], pos[2] - 0.1]
    arm.movel([target_pos, R], speed=0.2)

    # FK/IK
    pos, R = arm.fk([0.0]*7)
    q_sol, ok = arm.ik([0.5, 0.0, 0.4], R)
    print(f"IK success: {ok}, q={q_sol}")
```

### Mode 2: Mirror Mode

Simulation tracks the real arm's state in real-time. Requires `pip install "litearm-pybullet[mirror]"`.

```python
from litearm_pybullet import litearm, PyBulletArm

# Connect to real arm
real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

# Create simulation in mirror mode
sim = PyBulletArm(render=True)
sim.start()
sim.mirror_from(real)  # sim follows real arm

# The simulation now mirrors the real arm's motion
# Move the real arm (via teach pendant, teleop, or another program)
# and watch the simulation follow in real-time

# When done
sim.stop_mirroring()
sim.close()
real.close()
```

Alternatively, use the `MirrorMode` class:

```python
from litearm_pybullet import PyBulletArm, MirrorMode
import litearm

real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
sim = PyBulletArm(render=True)
sim.start()

mirror = MirrorMode(real, sim, rate_hz=50.0)
mirror.start()

# ... sim follows real arm ...

mirror.stop()
sim.close()
real.close()
```

### Mode 3: Dual Control

Send commands to real and simulated arms simultaneously. The simulation mirrors the real arm between commands.

```python
from litearm_pybullet import DualArm

dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
dual.start()

# One command, both arms move
real_ok, sim_ok = dual.movej(
    [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2
)

# Cartesian motion
pos, R = dual.get_tcp_pose()
dual.movel([[pos[0], pos[1], pos[2] - 0.1], R], speed=0.1)

# Emergency stop both
dual.request_stop()

dual.close()
```

## Pose Format

All poses use plain Python lists (no numpy arrays). The format is:

```python
pose = [position, rotation]
position = [px, py, pz]           # 3 floats in meters
rotation = [                      # 3x3 row-major rotation matrix
    [r00, r01, r02],
    [r10, r11, r12],
    [r20, r21, r22],
]
```

Example:

```python
# Identity rotation (TCP pointing straight down in base frame)
pose = [
    [0.5, 0.0, 0.4],           # position
    [[1, 0, 0], [0, 1, 0], [0, 0, 1]],  # rotation
]

# Use with movel
arm.movel(pose, speed=0.2)
```

## Safety Notes

### disable() Drops the Arm

Calling `disable()` sets all motor torques to zero. Under gravity, the arm will collapse. Use `enable()` to re-engage motors and hold the current pose.

### request_stop() is Emergency Stop

`request_stop()` immediately cuts all motor torques -- an emergency stop, not a graceful deceleration. Use `clear_stop()` to resume normal operation after investigating the cause.

### Motion is Blocking

All motion methods (`movej`, `movel`, `movec`, `movep`, `replay_*`) block the calling thread until the motion completes or is stopped. For non-blocking control, use `joint_follow()` or the mirror mode.

### Simulation is Not Real-Time Safe

The simulation loop runs at a fixed 500 Hz timestep, but the real-time synchronization is approximate. Do not use simulation timing for safety-critical real-time guarantees.

### render=False for Headless

Use `render=False` for headless operation (e.g., in CI/CD pipelines, servers, or when running many simulations in parallel). The PyBullet `DIRECT` connection mode is used, which does not open a GUI window.

## FAQ

### Q: How do I run without a GUI?

Use `render=False`:

```python
arm = PyBulletArm(render=False)
arm.start()
arm.movej([0.0]*7, speed=0.3)
state = arm.get_state()
arm.close()
```

### Q: How do I swap between simulation and real arm?

Replace the import and constructor:

```python
# Real arm
import litearm
arm = litearm.Arm(endpoint="tcp/192.168.31.139:7447")

# Simulation
from litearm_pybullet import PyBulletArm
arm = PyBulletArm(render=True)
```

All subsequent API calls are identical.

### Q: Why does the arm jitter or oscillate?

The default PD gains are tuned for the real arm's motor dynamics. If you observe oscillation, try lowering the gains:

```python
arm.set_gains(kp=[150]*7, kd=[8]*7)
```

### Q: Why does IK fail for some poses?

The IK solver uses PyBullet's built-in IK plus Levenberg-Marquardt refinement. If the target pose is unreachable (e.g., outside the workspace or in a singular configuration), IK will return `success=False`. Try providing a better `q_seed` or adjusting the target pose.

### Q: How do I record and replay a trajectory?

```python
# Record (motors are disabled during recording)
traj = arm.record_trajectory(duration_s=5.0, name="demo")
traj.save("trajectories/demo.json")

# Replay
arm.play_trajectory("trajectories/demo.json", speed=1.0)
```

### Q: Can I use numpy arrays for poses?

Methods accept numpy arrays but always return plain Python lists. Convert if needed:

```python
import numpy as np
pos = np.array([0.5, 0.0, 0.4])
R = np.eye(3)
arm.movel([pos.tolist(), R.tolist()], speed=0.2)
```

### Q: What does the mirror mode actually do?

Mirror mode reads the real arm's joint state (`q`) at a configurable rate (default 50 Hz) and uses it as the target for the simulation's PID controller. The simulation tries to match the real arm's pose, providing a visual representation of the real arm's motion.

### Q: How is DualArm different from MirrorMode?

- **MirrorMode**: Simulation passively follows the real arm. You move the real arm (by hand, teleop, or another program) and the simulation tracks it.
- **DualArm**: You send commands that execute on both arms simultaneously. Between commands, the simulation mirrors the real arm to stay synchronized.

### Q: The simulation appears frozen or stuck. What should I do?

Make sure you called `start()` or used the context manager. Check that `request_stop()` or `disable()` has not been called. If using `movej`, verify the simulation loop is running by checking `get_state()["state"]`.

## Dependencies

### Core Dependencies

- `pybullet >= 3.2.5`
- `numpy >= 1.21`

### Optional Dependencies (mirror/dual mode)

- `litearm-python`

## Known Limitations

- The controller is joint-space PD with gravity/Coriolis feed-forward; it does not include full computed-torque or friction feed-forward
- Hand/gripper/teach-pendant are simulated proxies returning canned responses
- System management/logs/teleop are API-compatible stub implementations
- Collision detection is not configured by default; `get_collision_config()` returns an empty dict
- Trajectory recording disables motors (zero-gravity mode) to allow free-hand manipulation; this differs from the real arm's recording behavior
- The simulation is not real-time safe; timing is approximate

## License

Proprietary