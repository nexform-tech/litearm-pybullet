"""PyBulletArm — PyBullet simulation of the LiteArm 7-DOF robot.

API-compatible with ``litearm_core.Arm``. You can swap between real and
simulated arms without changing your control code:

    # Real arm
    arm = litearm_core.Arm(port=None).connect()   # first USB CDC device

    # Simulation
    arm = PyBulletArm()

    # Same API for both:
    arm.movej([0.0] * 7, speed=0.5)      # -> RobotState
    state = arm.get_state().value        # -> RobotState
    tcp = arm.get_tcp().value            # -> (x, y, z, roll, pitch, yaw)
    arm.close()

Reads are wrapped in :class:`~litearm_pybullet.Msg` exactly as in litearm-core
2.x: the getters that return "the latest frame" hand back ``Msg(value, hz,
timestamp)``, so callers write ``.value`` on both sides.

Simulation-only extras (no litearm-core counterpart) are marked as such in
their docstrings: ``fk`` for an arbitrary configuration, ``plan_*``,
``record_trajectory``/``play_trajectory`` and the ``device``/``hand`` proxies.
"""
from __future__ import annotations

import math
import os
import tempfile
import threading
import time
import warnings
from collections import deque
from pathlib import Path
from typing import Any, Callable, Deque, Dict, List, Optional, Sequence, Tuple, Union

import numpy as np
import pybullet as p

from ._compat import (
    CART_START_POS_TOL,
    CART_START_RPY_TOL,
    FLAG_ENABLED_BIT,
    MODE_NAMES,
    CartesianPlanError,
    CartPlan,
    IKError,
    InvalidCommandError,
    JointState,
    MotionTimeoutError,
    MotorFaultError,
    Msg,
    RobotState,
    as_pose,
    mat_to_rpy,
    rpy_to_mat,
)
from .controller import DEFAULT_KD, DEFAULT_KP, N_JOINTS, JointPIDController, TrajectoryGenerator
# `_rotation_error` is the solver's own rotation metric; using it here too keeps
# "did we arrive" measured the same way the optimizer measured it.
from .kinematics import Kinematics, _rotation_error
from .trajectory import JointTrajectory, TrajectoryFrame

# Path to the default URDF model
_ASSETS_DIR = Path(__file__).parent / "assets"
_DEFAULT_URDF = _ASSETS_DIR / "litearm.urdf"
_MESH_DIR = _ASSETS_DIR / "meshes"

# Joint limits
Q_MIN = np.array([-2.82, -1.78, -2.83, -3.1325, -2.853, -1.568, -1.59])
Q_MAX = np.array([2.839, 1.817, 2.87, 0.647, 2.8587, 1.619, 1.623])
TAU_MAX = np.array([78.0, 78.0, 21.0, 21.0, 10.0, 10.0, 10.0])

# Inertia patch: URDF distal links have near-zero inertia; pad for stable PD
_MIN_INERTIA = 0.01
_MIN_MASS = 0.5

# Firmware mode ids, by the names litearm_core's MODE_NAMES uses.
_MODE_INIT = 0
_MODE_MOVE_J = 1
_MODE_MOVE_P = 2
_MODE_MOVE_JS = 3
_MODE_MOVE_MIT = 4
_MODE_MIT_ALL = 5
_MODE_EMERGENCY = 6
_MODE_ZERO_G = 7

# flags bit10 = cartesian motion in progress.  litearm-core publishes no named
# constant for it: the only way to read it is RobotState.cart_busy, and it is
# deliberately kept out of FLAG_NAMES (those are safety flags).
_FLAG_CART_BUSY_BIT = 10

# Simulated motor/coil temperatures.  The simulation has no thermal model, so
# every joint reports ambient — a constant that cannot be mistaken for a
# measurement.
_AMBIENT_C = 25.0

# What "this pose is reachable" means for `ik()`.  The DLS/LM solver reports
# success whenever it produced a finite in-limit configuration, so without a
# residual check it answers a target 10 m away with a confident-looking q and
# `except IKError` never fires — the sim would silently accept what the
# firmware refuses.  Measured on this model: a reachable pose lands at ~7 mm /
# 7 mrad, a pose 0.5 m out of reach at ~97 mm / 151 mrad.  The thresholds sit
# in that gap, with a few times the margin on either side.
_IK_POS_TOL = 0.02   # m
_IK_ROT_TOL = 0.05   # rad


class _FrameStats:
    """Arrival statistics for one kind of frame, feeding ``Msg.hz``/``Msg.timestamp``.

    Mirrors the bookkeeping litearm-core's ``Msg`` documents: ``hz`` is the
    average rate over a short recent window (not a lifetime average, which
    would take forever to react to a rate change), ``timestamp`` the local
    ``time.monotonic()`` of the most recent frame, ``0.0`` if none yet.
    """

    def __init__(self, window: int = 32) -> None:
        self._ts: Deque[float] = deque(maxlen=window)

    def note(self) -> None:
        self._ts.append(time.monotonic())

    def hz(self) -> float:
        if len(self._ts) < 2:
            return 0.0
        span = self._ts[-1] - self._ts[0]
        return (len(self._ts) - 1) / span if span > 0 else 0.0

    def timestamp(self) -> float:
        return self._ts[-1] if self._ts else 0.0


#: Playback cadence for geometrically spaced cartesian paths, in seconds per
#: waypoint at ``speed=1.0``. A cartesian plan carries no timing (the firmware's
#: planner produces it), so unlike a joint trajectory — whose waypoints are
#: already dt-spaced with `speed` folded in — this path needs a rate of its own.
#: 20 ms/waypoint is the cadence the previous movel/movec/movep implementation
#: used, so cartesian tracking behaves as before.
_CART_WAYPOINT_INTERVAL = 0.02

#: Trajectory generation cannot divide by a zero speed, and `speed=0.0` is legal
#: input (0..1 inclusive) meaning "don't move". Clamping the *generation* rate
#: keeps that from becoming a ZeroDivisionError: the arm simply never arrives,
#: and the caller gets MotionTimeoutError, which is the honest outcome.
_SPEED_FLOOR = 1e-3

#: `home()` takes no speed — the firmware hard-codes 0.10 (a low safety speed,
#: since homing is done by whoever is standing next to the arm, usually right
#: after a fault). The simulation moves at the same rate so a script that times
#: the two gets comparable numbers.
_HOME_SPEED = 0.10

#: Default zero-gravity keepalive period. litearm-core calls this
#: ``ZG_KEEPALIVE_S`` and re-sends `ZERO_G` every 0.04 s to stay ahead of the
#: firmware's 0.10 s watchdog. The simulation has no watchdog to feed, so the
#: period is validated against the same window and otherwise unused — see
#: :meth:`PyBulletArm.zero_g_start`.
_ZG_KEEPALIVE_S = 0.04
#: The keepalive period window, bounded by the firmware watchdog timeout.
_ZG_PERIOD_MIN = 0.005
_ZG_PERIOD_MAX = 0.10

#: How often the real arm is polled in mirror mode, and how long one such read
#: may block. litearm-core has no background reader thread, so every mirrored
#: frame costs a serial round trip; the physics loop runs at 500 Hz and asking
#: the link 500 times a second is what `rate_hz` exists to prevent. The read has
#: its own short timeout so a silent link bounds the mirror thread instead of
#: letting it sit in a 0.5 s read.
_MIRROR_RATE_HZ = 50.0
_MIRROR_READ_TIMEOUT = 0.05

#: Refusal text for action commands sent while zero gravity is held — copied
#: verbatim from litearm_core's ``ZERO_G_GUARD_MESSAGE``. Callers match on the
#: wording (the SDK's own cartesian guard compares against the same constant),
#: so a paraphrase here would read as a different refusal.
_ZERO_G_GUARD_MESSAGE = (
    "零重力保活正在进行, 拒绝其它下行命令 (会改写模式/看门狗, 与保活互相打架); "
    "先 arm.zero_g_stop() 退出")


def _orient_angle(rpy_a: Sequence[float], rpy_b: Sequence[float]) -> float:
    """Geodesic angle between two orientations, in radians."""
    Ra = np.asarray(rpy_to_mat(list(rpy_a)), dtype=float)
    Rb = np.asarray(rpy_to_mat(list(rpy_b)), dtype=float)
    c = float(np.trace(Ra.T @ Rb))
    return math.acos(max(-1.0, min(1.0, (c - 1.0) / 2.0)))


def _pose6_near(tcp: Sequence[float], goal: Sequence[float],
                pos_tol: float, rpy_tol: float) -> bool:
    """Is ``tcp`` at ``goal``, both ``[x, y, z, roll, pitch, yaw]``?

    Position is compared component-wise. Orientation is component-wise *or* by
    geodesic angle: ``mat_to_rpy`` forces ``yaw = 0`` at gimbal lock, so the same
    rotation can report two very different rpy triples, and a component-wise-only
    test would never accept the pose it is standing in.
    """
    if max(abs(tcp[i] - goal[i]) for i in range(3)) >= pos_tol:
        return False
    if max(abs(tcp[i] - goal[i]) for i in range(3, 6)) < rpy_tol:
        return True
    return _orient_angle(tcp[3:6], goal[3:6]) < rpy_tol


def _is_single_pose(x: Any) -> bool:
    """Is this one pose, or a sequence of poses?

    ``[pose1, pose2]`` and ``(pos[3], R[3x3])`` are both "length 2, two
    sequences", so length alone would misjudge a legitimate position/rotation
    pair as a sequence — and ``move_p`` must reject sequences by name.
    """
    if not isinstance(x, (list, tuple)) or isinstance(x, (str, bytes)):
        return False
    n = len(x)
    if n == 4 and all(isinstance(r, (list, tuple)) and len(r) == 4 for r in x):
        return True
    if n == 6 and not any(isinstance(v, (list, tuple)) for v in x):
        return True
    if n == 2 and all(isinstance(v, (list, tuple)) for v in x):
        p, R = list(x[0]), list(x[1])
        return (len(p) == 3 and len(R) == 3
                and all(isinstance(r, (list, tuple)) and len(r) == 3 for r in R))
    return False


def _as_pose6(pose: Any, label: str) -> List[float]:
    """Normalize a pose argument to ``[x, y, z, roll, pitch, yaw]``.

    Shape errors come from :func:`as_pose` with the offending shape in the
    message; the label says which argument of which entry point. Malformed
    *values* (``float("x")``) and wrong container types are folded into
    ``InvalidCommandError`` too — a caller branching on ``LiteArmError`` should
    not miss the "I passed a string" case just because it failed during
    conversion rather than shape checking.
    """
    try:
        pos, R = as_pose(pose)
    except InvalidCommandError as exc:
        raise InvalidCommandError(f"{label}: {exc}") from None
    except (ValueError, TypeError) as exc:
        raise InvalidCommandError(f"{label}: {exc}") from None
    return [float(v) for v in pos] + list(mat_to_rpy(R))


def _warn_deprecated(old: str, new: str) -> None:
    warnings.warn(f"{old}() is deprecated, use {new}() instead",
                  DeprecationWarning, stacklevel=3)


def _resolve_model_path(model_path: Optional[str] = None) -> str:
    """Resolve the model path, trying multiple locations."""
    if model_path is not None and os.path.exists(model_path):
        return model_path

    candidates = [
        _DEFAULT_URDF,
        Path("src/litearm_pybullet/assets/litearm.urdf"),
        Path("assets/litearm.urdf"),
    ]

    for p in candidates:
        if p.exists():
            return str(p)

    raise FileNotFoundError(
        f"URDF model not found. Tried: {[str(c) for c in candidates]}. "
        "Provide model_path= explicitly."
    )


def _resolve_urdf(model_path: str) -> str:
    """Replace package:// URIs with absolute mesh paths, write to temp file."""
    content = Path(model_path).read_text(encoding="utf-8")
    if "package://" in content:
        content = content.replace(
            "package://litearm_description/meshes/jxbURDF260327/",
            str(_MESH_DIR) + "/",
        )
    fd, path = tempfile.mkstemp(suffix=".urdf", prefix="litearm_")
    with os.fdopen(fd, "w", encoding="utf-8") as f:
        f.write(content)
    return path


def _load_arm(urdf_path: str, client_id: int):
    """Load arm model, return (body_id, joint_ids, ee_link_index).

    - Applies minimum mass/inertia patch for stable high-gain PD.
    - Overrides joint motors to VELOCITY_CONTROL (zero force) for torque control.
    """
    body_id = p.loadURDF(urdf_path, useFixedBase=True, physicsClientId=client_id)
    joint_ids = []
    ee_link_index = -1
    for jid in range(p.getNumJoints(body_id, physicsClientId=client_id)):
        info = p.getJointInfo(body_id, jid, physicsClientId=client_id)
        link_name = info[12].decode() if isinstance(info[12], bytes) else str(info[12])
        if info[2] == p.JOINT_REVOLUTE:
            joint_ids.append(jid)
            dyn = p.getDynamicsInfo(body_id, jid, physicsClientId=client_id)
            mass = max(dyn[0], _MIN_MASS)
            diag = np.maximum(np.asarray(dyn[2], float), _MIN_INERTIA)
            p.changeDynamics(body_id, jid, mass=mass,
                             localInertiaDiagonal=list(diag),
                             physicsClientId=client_id)
        if link_name == "ee_frame_link":
            ee_link_index = jid
    if len(joint_ids) != N_JOINTS or ee_link_index < 0:
        raise RuntimeError(f"URDF 解析异常: joints={joint_ids}, ee={ee_link_index}")
    p.setJointMotorControlArray(body_id, joint_ids, p.VELOCITY_CONTROL,
                                forces=[0.0] * len(joint_ids),
                                physicsClientId=client_id)
    return body_id, joint_ids, ee_link_index


def _read_joint_state(body_id, joint_ids, client_id):
    """Read (q[7], dq[7]) from PyBullet."""
    q = np.zeros(N_JOINTS)
    dq = np.zeros(N_JOINTS)
    for i, jid in enumerate(joint_ids):
        st = p.getJointState(body_id, jid, physicsClientId=client_id)
        q[i] = st[0]
        dq[i] = st[1]
    return q, dq


class PyBulletArm:
    """PyBullet simulation of LiteArm 7-DOF robot arm.

    API-compatible with ``litearm_core.Arm``: same method names, same argument
    names and units, same return types, same exceptions. All motion methods are
    blocking — they step the simulation until the motion arrives — matching the
    real arm's behaviour.

    Usage::

        # Standalone simulation
        with PyBulletArm(render=True) as arm:
            state = arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)
            print(state.q)

        # Mirror real arm
        import litearm_core
        real = litearm_core.Arm(port=None).connect()
        sim = PyBulletArm(render=True)
        sim.start()
        sim.mirror_from(real)  # sim follows real arm state
    """

    def __init__(
        self,
        model_path: Optional[str] = None,
        render: bool = False,
        dt: float = 0.002,
        kp: Union[float, List[float]] = DEFAULT_KP,
        kd: Union[float, List[float]] = DEFAULT_KD,
        max_velocity: float = 3.0,
        n_joints: int = N_JOINTS,
        key_callback: Optional[Callable[[int], None]] = None,
        q_tol: float = 0.03,
        dq_tol: float = 0.10,
        arrive_frames: int = 3,
        move_timeout: float = 15.0,
    ) -> None:
        """Initialize the PyBullet simulation.

        Args:
            model_path: Path to URDF model file. Default: built-in litearm.urdf.
            render: Whether to open the PyBullet GUI window.
            dt: Physics timestep (seconds).
            kp: Position gain for joint controller.
            kd: Velocity gain for joint controller.
            max_velocity: Maximum joint velocity (rad/s).
            n_joints: Number of joints (default 7).
            key_callback: Optional callback for keyboard events.
            q_tol: Arrival tolerance on joint position (rad), same name and
                meaning as litearm_core's ``Arm(q_tol=...)``.
            dq_tol: Arrival tolerance on joint velocity (rad/s).
            arrive_frames: Consecutive frames that must satisfy both tolerances
                before a motion counts as arrived.
            move_timeout: Seconds after which an unfinished motion raises
                ``MotionTimeoutError``.
        """
        model_path = _resolve_model_path(model_path)
        model_path = _resolve_urdf(model_path)

        self._n_joints = n_joints
        self._dt = dt
        self._render = render
        self._key_callback = key_callback

        # PyBullet client
        self._cid = p.connect(p.GUI if render else p.DIRECT)
        p.setGravity(0, 0, -9.81, physicsClientId=self._cid)
        p.setTimeStep(dt, physicsClientId=self._cid)
        p.setRealTimeSimulation(0, physicsClientId=self._cid)

        self._body, self._joints, self._ee = _load_arm(model_path, self._cid)

        # Initialize to zero pose
        q0 = np.zeros(N_JOINTS)
        for jid, qi in zip(self._joints, q0):
            p.resetJointState(self._body, jid, targetValue=float(qi),
                              physicsClientId=self._cid)

        # Controllers
        self._controller = JointPIDController(kp=kp, kd=kd, n_joints=n_joints)
        self._traj_gen = TrajectoryGenerator(max_velocity=max_velocity, dt=dt)
        self._kinematics = Kinematics(
            self._body, self._ee, self._joints, self._cid, n_joints=n_joints
        )

        # State
        self._lock = threading.Lock()
        self._sim_running = False
        self._closed = False
        self._sim_thread: Optional[threading.Thread] = None
        self._stopped = False
        self._enabled = True

        # Motion state, reported through RobotState the way the firmware
        # reports its own: one mode variable plus the cartesian-busy bit.
        self._mode = _MODE_INIT
        self._cart_busy = False
        self._seq = 0
        self._last_tau = np.zeros(n_joints)

        # Global speed governor: `set_speed(percent)` scales every motion, the
        # way the firmware's `gov_ratio = percent/100` does. 1.0 = full speed,
        # restored by `reset()`. It is a *separate* knob from the per-call
        # `speed` fraction, and the two multiply.
        self._gov_ratio = 1.0

        # MIT servo feed-forward, written by send_mit/send_mit_all. The
        # simulation has no per-joint impedance loop, so these are applied the
        # way the firmware's MIT frame is: as a torque added to the controller's.
        self._mit_tau = np.zeros(n_joints)

        # Zero-gravity (free-drag) mode: the position loop is dropped and only
        # gravity is compensated, which is what the firmware's `0x06` does.
        self._zero_g = False
        self._zero_g_error: Optional[BaseException] = None

        # The port the caller said the real arm would be on. The simulation has
        # no serial link, so this is recorded for introspection only — a script
        # written against the real arm keeps its `connect(port=...)` call.
        self._port: Optional[str] = None

        # One cartesian motion at a time: concurrent cartesian calls queue here,
        # the way litearm-core serializes them on `_cart_serial`.  `_motion_gen`
        # is the supersede counter — starting any motion bumps it, so a motion
        # in flight stops pushing its waypoints, which is what the firmware does
        # when a joint move invalidates a cartesian plan.
        self._motion_serial = threading.Lock()
        self._motion_gen = 0

        # Arrival criteria (litearm-core names and defaults)
        self._q_tol = float(q_tol)
        self._dq_tol = float(dq_tol)
        self._arrive_frames = int(arrive_frames)
        self._move_timeout = float(move_timeout)

        # Mirror mode: a real arm's state drives this simulation's target.
        self._mirror_arm: Optional[Any] = None
        self._mirror_thread: Optional[threading.Thread] = None
        self._mirroring = False
        self._mirror_rate_hz = _MIRROR_RATE_HZ
        self._mirror_error: Optional[BaseException] = None

        # Simulated devices
        self._devices: Optional[Any] = None

        self._controller.set_target(np.zeros(n_joints))
        self._state_cache: Optional[RobotState] = None
        self._frames_status = _FrameStats()
        self._frames_tcp = _FrameStats()

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def connect(self, port: Optional[str] = None) -> "PyBulletArm":
        """Start the simulation and return ``self`` (idempotent).

        ``port`` exists for call-site parity with ``litearm_core.Arm``: a script
        written for the real arm passes the serial device here, and running that
        same script against the simulation must not need editing. There is no
        serial link to open, so the value is recorded (``self.port``) and
        otherwise unused — the simulation is always "connected" once its loop is
        running. Calling it again is a no-op, as on the real arm.
        """
        if port is not None:
            self._port = port
        if self._sim_running:
            return self
        self._sim_running = True
        self._sim_thread = threading.Thread(
            target=self._sim_loop, daemon=True, name="pybullet_sim"
        )
        self._sim_thread.start()
        return self

    def start(self) -> None:
        """Legacy alias for :meth:`connect` — the simulation's historical name."""
        self.connect()

    def close(self) -> None:
        """Stop simulation and close viewer (idempotent).

        Idempotent in the strong sense: the second call does nothing at all.
        The physics client is gone after the first, so any work here — even
        something as innocent as re-anchoring a target — would fail on a dead
        client. ``disconnect()`` and ``__exit__`` both land here, and callers
        routinely reach for a second one by accident.
        """
        if self._closed:
            return
        self.zero_g_stop()
        self._sim_running = False
        self.stop_mirroring()

        if self._sim_thread and self._sim_thread.is_alive():
            self._sim_thread.join(timeout=2.0)
        try:
            p.disconnect(self._cid)
        except Exception:
            pass
        self._closed = True

    def disconnect(self) -> None:
        """Alias for :meth:`close`, the name litearm_core pairs with ``connect``.

        One operation, two names, one implementation: the real SDK delegates
        ``disconnect()`` to ``close()`` for exactly this reason.
        """
        self.close()

    @property
    def port(self) -> Optional[str]:
        """The port named to :meth:`connect`, or ``None``.

        ``None`` means "nobody named one" — the simulation needs no port, so
        there is nothing to search for and nothing to report as found.
        """
        return self._port

    def __enter__(self) -> "PyBulletArm":
        return self.connect()

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"PyBulletArm(n_joints={self._n_joints}, render={self._render})"

    # ── Simulation Loop ────────────────────────────────────────────────────────

    def _sim_loop(self) -> None:
        """Background simulation loop running at 1/dt Hz."""
        while self._sim_running:
            loop_start = time.time()

            with self._lock:
                # Compute control
                q_actual = np.zeros(self._n_joints)
                dq_actual = np.zeros(self._n_joints)
                for i, jid in enumerate(self._joints):
                    st = p.getJointState(self._body, jid, physicsClientId=self._cid)
                    q_actual[i] = st[0]
                    dq_actual[i] = st[1]

                if self._zero_g:
                    # Free drag: no position loop, gravity cancelled, plus any
                    # torque a caller asked for through move_js/send_mit.
                    ff_torque = np.asarray(p.calculateInverseDynamics(
                        self._body, list(q_actual), list(dq_actual),
                        [0.0] * self._n_joints, physicsClientId=self._cid,
                    ), dtype=float)[:self._n_joints]
                    tau = ff_torque + self._mit_tau
                elif self._enabled and not self._stopped:
                    # Gravity + Coriolis feed-forward (computed-torque style)
                    ff_torque = np.asarray(p.calculateInverseDynamics(
                        self._body, list(q_actual), list(dq_actual),
                        [0.0] * self._n_joints, physicsClientId=self._cid,
                    ), dtype=float)[:self._n_joints]
                    tau = self._controller.compute(
                        q_actual, dq_actual, self._dt,
                        ff_torque=ff_torque + self._mit_tau,
                    )
                else:
                    tau = np.zeros(self._n_joints)

                tau_clipped = np.clip(tau, -TAU_MAX, TAU_MAX)
                p.setJointMotorControlArray(
                    self._body, self._joints, p.TORQUE_CONTROL,
                    forces=list(tau_clipped), physicsClientId=self._cid,
                )
                self._last_tau = tau_clipped

                # Step physics
                p.stepSimulation(physicsClientId=self._cid)

                # Update state cache: this loop *is* the simulated status
                # stream, so every step produces one frame.
                self._state_cache = self._build_robot_state()
                self._frames_status.note()

            # Real-time sync
            elapsed = time.time() - loop_start
            if elapsed < self._dt:
                time.sleep(self._dt - elapsed)

    # ── State Reading ──────────────────────────────────────────────────────────

    def get_state(self, refresh: bool = False, timeout: float = 0.5) -> Msg:
        """Latest robot state, wrapped in a :class:`Msg`.

        ``refresh`` forces a freshly built frame; ``timeout`` is accepted for
        signature parity with ``litearm_core.Arm.get_state`` and is unused here
        — the simulation always has a state to read, so unlike the real arm
        this getter cannot come back empty (``Msg.value`` is never ``None``).

        Read ``.value`` for the :class:`RobotState`: ``.value.q``,
        ``.value.dq``, ``.value.tau``, ``.value.mode_name``, ``.value.enabled``,
        ``.value.faulted``.
        """
        with self._lock:
            if self._state_cache is None or refresh:
                self._state_cache = self._build_robot_state()
            value = self._state_cache
        return self._msg(value, self._frames_status)

    def get_status_now(self, timeout: float = 0.5) -> Msg:
        """State read that is guaranteed to be of this instant.

        On the real arm this actively requests a frame instead of consuming the
        passive stream, which is how you tell "the link is alive" from "the
        stream stopped". The simulation has no passive stream to fall behind:
        every read is a live PyBullet read, so both getters return a frame from
        *now* and ``value`` is never ``None``.
        """
        return self.get_state(refresh=True, timeout=timeout)

    def _build_robot_state(self) -> RobotState:
        """Snapshot the current PyBullet state as a litearm-core RobotState.

        Caller holds ``self._lock``.
        """
        q, dq = _read_joint_state(self._body, self._joints, self._cid)
        self._seq = (self._seq + 1) & 0xFFFF
        mode = self._mode_now()
        joints = [
            JointState(
                q=float(q[i]),
                dq=float(dq[i]),
                tau=float(self._last_tau[i]),
                # No thermal model in the simulation: report ambient rather
                # than a number that looks measured.
                t_mos=_AMBIENT_C,
                t_coil=_AMBIENT_C,
                err=0,
            )
            for i in range(self._n_joints)
        ]
        flags = 0
        if self._enabled:
            flags |= 1 << FLAG_ENABLED_BIT
        if self._cart_busy:
            flags |= 1 << _FLAG_CART_BUSY_BIT
        return RobotState(
            mode=mode,
            mode_name=MODE_NAMES.get(mode, str(mode)),
            flags=flags,
            # The simulation raises no safety flags. A stop is reported through
            # mode=EMERGENCY (hence RobotState.faulted), not by inventing a
            # FAULT bit the firmware would not have set.
            flag_names=[],
            seq=self._seq,
            joints=joints,
            joint_fault=0,
        )

    def _mode_now(self) -> int:
        """Current firmware-style mode id.

        EMERGENCY dominates: the stop latch overrides whatever mode the arm was
        in, the same precedence the firmware gives it. Zero gravity comes next,
        for the same reason: a joint move cannot be in progress while the
        position loop is dropped, and reporting MOVE_J then would be a mode the
        arm is not in.
        """
        if self._stopped:
            return _MODE_EMERGENCY
        if self._zero_g:
            return _MODE_ZERO_G
        return self._mode

    def _msg(self, value: Any, stats: _FrameStats) -> Msg:
        return Msg(value=value, hz=stats.hz(), timestamp=stats.timestamp())

    def get_tcp(self, timeout: float = 0.6) -> Msg:
        """TCP pose as ``Msg`` with ``value = (x, y, z, roll, pitch, yaw)``.

        Same six-number shape and ZYX-extrinsic-free convention as
        ``litearm_core.Arm.get_tcp``. ``timeout`` is accepted for parity and
        unused — the simulation always answers.
        """
        with self._lock:
            q, _ = _read_joint_state(self._body, self._joints, self._cid)
            pos, R = self._kinematics.fk(q.tolist())
        self._frames_tcp.note()
        return self._msg(tuple(list(pos) + mat_to_rpy(R)), self._frames_tcp)

    def get_tcp_pose(self) -> Tuple[List[float], List[List[float]]]:
        """Deprecated: use ``get_tcp()``, which is what litearm_core calls it.

        Kept because the 0.1 API returned ``(position, rotation_matrix)`` and
        callers still want a matrix. The rotation is rebuilt from the same
        rpy the SDK reports, so both spellings agree by construction.
        """
        rpy = self.get_tcp().value
        return list(rpy[:3]), rpy_to_mat(rpy[3:])

    @property
    def n(self) -> int:
        """Number of joints (litearm_core's name for it)."""
        return self._n_joints

    @property
    def firmware(self) -> str:
        """Simulated firmware banner, in the SDK's ``<name>-<n>J`` shape.

        Plainly not a real version string: nothing here can be flashed, and
        code that branches on firmware versions is not running against it.
        """
        return f"PyBulletSim-{self._n_joints}J"

    # ── Pure Computation (no simulation step needed) ───────────────────────────

    def fk(self, q: List[float]) -> Tuple[List[float], List[List[float]]]:
        """Forward kinematics: joint angles → (position, rotation_matrix).

        Simulation-only: litearm_core has no FK for an arbitrary configuration
        (the firmware only reports the TCP of the pose it is actually in).
        """
        return self._kinematics.fk(q)

    def ik(
        self,
        pose: Sequence[float],
        q_seed: Optional[Sequence[float]] = None,
        timeout: float = 3.0,
    ) -> List[float]:
        """Inverse kinematics: ``pose`` → ``q[7]``; failure raises ``IKError``.

        Accepts what ``litearm_core.Arm.ik`` accepts — ``[x, y, z, roll, pitch,
        yaw]`` — plus, because the simulation can be asked about a pose it is
        not standing in, the ``(position[3], R[3x3])`` and 4x4 forms
        :func:`as_pose` normalizes.

        ``q_seed`` defaults to the current joint configuration (the real arm
        seeds from its measured state too). ``timeout`` is unused here: the
        solver is local and cannot time out, only fail.
        """
        pos_d, R_d = as_pose(pose)
        if q_seed is None:
            with self._lock:
                q_seed = _read_joint_state(
                    self._body, self._joints, self._cid)[0].tolist()
        elif len(q_seed) != self._n_joints:
            raise InvalidCommandError(
                f"q_seed 需 {self._n_joints} 个, 收到 {len(q_seed)}")
        q_sol, ok = self._kinematics.ik(pos_d, R_d, list(q_seed))
        if not ok:
            raise IKError(f"IK 无解: pose={list(pos_d)}")

        # The solver's `ok` only means "a finite, in-limit configuration came
        # out"; check the residual so IKError keeps the firmware's meaning.
        pos_sol, R_sol = self._kinematics.fk(q_sol)
        pos_err = float(np.linalg.norm(np.asarray(pos_sol) - np.asarray(pos_d)))
        rot_err = float(np.linalg.norm(
            _rotation_error(np.asarray(R_sol, float), np.asarray(R_d, float))))
        if pos_err > _IK_POS_TOL or rot_err > _IK_ROT_TOL:
            raise IKError(
                f"IK 未收敛: 残差 {pos_err * 1000:.1f} mm / {rot_err:.4f} rad "
                f"(容差 {_IK_POS_TOL * 1000:.0f} mm / {_IK_ROT_TOL:.2f} rad)")
        return [float(v) for v in q_sol]

    def plan_movel(
        self, q_start: List[float], pose_goal: Any
    ) -> List[List[float]]:
        """Plan a straight-line Cartesian path."""
        return self._kinematics.plan_movel(q_start, pose_goal)

    def plan_movec(
        self, q_start: List[float], pose_via: Any, pose_goal: Any
    ) -> List[List[float]]:
        """Plan a circular-arc Cartesian path."""
        return self._kinematics.plan_movec(q_start, pose_via, pose_goal)

    def plan_movep(
        self, q_start: List[float], poses_goal: List[Any]
    ) -> List[List[float]]:
        """Plan a multi-waypoint Cartesian path."""
        return self._kinematics.plan_movep(q_start, poses_goal)

    # ── Joint Motion ───────────────────────────────────────────────────────────

    def movej(self, q: Sequence[float], speed: float = 1.0) -> RobotState:
        """Joint move to ``q``; each axis runs its own profile and stops when it
        gets there, so the intermediate TCP path is not predictable.

        ``speed`` is a 0..1 fraction of the trajectory rate (not a percentage —
        see ``set_speed`` for the global percent governor). Blocks until the
        joints have been within ``q_tol``/``dq_tol`` for ``arrive_frames``
        consecutive frames, then returns the state it arrived in; raises
        ``MotionTimeoutError`` if that never happens and ``MotorFaultError`` if
        the arm faults or stops first.
        """
        return self._move_joint(q, speed, label="movej", sync=False)

    def movej_sync(self, q: Sequence[float], speed: float = 1.0) -> RobotState:
        """Joint move to ``q`` with every axis arriving on the same step.

        The difference from :meth:`movej` is the trajectory *shape*: one scalar
        ``s: 0 -> 1`` drives all axes along the joint-space line, so the TCP
        stays on a predictable path a caller can validate. Paying for that is
        being held to the slowest axis — this is slower than ``movej``, which is
        why it is a separate mode rather than a replacement.
        """
        return self._move_joint(q, speed, label="movej_sync", sync=True)

    def move_p(self, pose: Any, speed: float = 1.0,
               pos_tol: float = 0.006, rpy_tol: float = 0.03) -> RobotState:
        """Point-to-point move to a single pose, ending when the *TCP* is there.

        Joint-space interpolation, not a straight line: the intermediate TCP
        path is whatever the joint interpolation produces. For a straight line,
        an arc or a waypoint list use :meth:`move_l` / :meth:`move_c` /
        :meth:`move_path`.

        Accepts one pose — ``[x, y, z, roll, pitch, yaw]``, ``(pos[3], R[3x3])``
        or a 4x4 matrix. A sequence of poses raises ``InvalidCommandError``
        naming :meth:`move_path`, because the two are not interchangeable.

        ``pos_tol``/``rpy_tol`` are the arrival criterion, same defaults and
        meaning as on the real arm. One simulation-only difference: the sim's
        local solver leaves a residual of a few millimetres, so a tolerance
        tighter than it can manage is reported as ``IKError`` *before* moving
        instead of after ``move_timeout`` — "this solver cannot stand there" is
        a different fact from "the arm did not get there".
        """
        self._reject_in_zero_g()
        self._check_speed(speed, "move_p")
        if not _is_single_pose(pose):
            raise InvalidCommandError(
                "move_p 只收单个位姿；多个位姿请用 move_path（注意语义不同："
                "move_p 是关节空间点到点，move_path 是笛卡尔多路点）")
        goal = _as_pose6(pose, "move_p")
        q_goal = self.ik(goal)
        self._require_reachable_within(q_goal, goal, pos_tol, rpy_tol)

        self._ensure_running()
        with self._lock:
            q0 = _read_joint_state(self._body, self._joints, self._cid)[0]
        path = self._traj_gen.linear_trajectory(
            q0, np.asarray(q_goal, dtype=float), speed=self._governed(speed))

        self._mode = _MODE_MOVE_P
        try:
            self._play(path, self._dt)
            return self._arrive_pose(goal, pos_tol, rpy_tol)
        finally:
            self._mode = _MODE_INIT

    def home(self, *, timeout: Optional[float] = None) -> RobotState:
        """Move to the URDF zero configuration.

        ``timeout`` overrides ``move_timeout`` for this call and is keyword-only,
        as in litearm_core — the older ``home(speed)`` spelling would silently
        feed a speed value to a timeout. There is likewise no ``speed``: the
        firmware hard-codes a low safety speed of 0.10, and the simulation uses
        the same one so the two take comparable time.
        """
        return self._move_joint([0.0] * self._n_joints, _HOME_SPEED,
                                label="home", timeout=timeout)

    # ── Cartesian Motion ───────────────────────────────────────────────────────

    def move_l(self, pose: Any, speed: float = 1.0, wait: bool = True) -> CartPlan:
        """Straight cartesian line to ``pose`` (attitude slerped along the way).

        ``pose`` is the goal; the start is the measured TCP, not an argument.

        ``wait=True`` plays the path and waits for the arm to stop on it, after
        which ``settled``/``q_final``/``settle_err_rad`` are filled in.
        ``wait=False`` returns as soon as the path is being played, leaving
        those four fields at their "did not wait" defaults — that is *not* the
        same as "did not arrive"; read ``get_state()`` if you need to know where
        the arm is.

        ``ok=True`` only means the path was planned and started. A motion
        superseded by another (a ``movej`` from anywhere, for instance) comes
        back ``ok=True, settled=False`` rather than raising.
        """
        goal = _as_pose6(pose, "move_l")
        return self._cartesian_start("move_l", [goal], speed, wait)

    def move_c(self, pose_start: Any, pose_via: Any, pose_goal: Any,
               speed: float = 1.0, wait: bool = True) -> CartPlan:
        """Circular arc through ``pose_via`` and ``pose_goal``.

        The arc's start is always the *measured* TCP, so ``pose_start`` must
        match it (``CART_START_POS_TOL`` / ``CART_START_RPY_TOL``) or the call
        raises ``InvalidCommandError`` — accepting a different start would
        command an arc the caller never described. ``pose_via`` contributes its
        position only; the attitude slerps from start to goal.
        """
        start = _as_pose6(pose_start, "move_c pose_start")
        via = _as_pose6(pose_via, "move_c pose_via")
        goal = _as_pose6(pose_goal, "move_c pose_goal")

        tcp = self.get_tcp().value
        if not _pose6_near(tcp, start, CART_START_POS_TOL, CART_START_RPY_TOL):
            raise InvalidCommandError(
                f"move_c: pose_start 与当前 TCP 不一致 —— 起点恒为实测 TCP, "
                f"给的 start={['%.4f' % v for v in start]}, "
                f"实际 tcp={['%.4f' % v for v in tcp]}")
        return self._cartesian_start("move_c", [via, goal], speed, wait)

    def move_path(self, poses: Sequence[Any], speed: float = 1.0,
                  wait: bool = True) -> CartPlan:
        """Visit every pose in ``poses`` in order, with sharp corners.

        The path is a polyline through the waypoints — the planner has no
        blending field in the protocol, so it does not round corners. The
        arrival check compares against the *last* waypoint: intermediate
        waypoints are passed, not stopped at.
        """
        pts = [_as_pose6(p, "move_path") for p in poses]
        if not pts:
            raise InvalidCommandError("move_path: 路径为空")
        if len(pts) > 32:
            raise InvalidCommandError(
                f"move_path: 路点数 {len(pts)} 超固件上限 32 (CART_MAX_GOAL)")
        return self._cartesian_start("move_path", pts, speed, wait)

    # ── Motion internals ───────────────────────────────────────────────────────

    def _move_joint(self, q: Sequence[float], speed: float, label: str,
                    sync: bool = False,
                    timeout: Optional[float] = None) -> RobotState:
        """Generate a joint trajectory, play it, and wait for arrival."""
        self._reject_in_zero_g()
        sp = self._check_speed(speed, label)
        q_target = [float(v) for v in q]
        if len(q_target) != self._n_joints:
            raise InvalidCommandError(
                f"{label} 需要 {self._n_joints} 个关节角 (N={self._n_joints})")

        self._ensure_running()
        with self._lock:
            q0 = _read_joint_state(self._body, self._joints, self._cid)[0]
        gen_speed = self._governed(sp)
        q_target_arr = np.asarray(q_target, dtype=float)
        if sync:
            # Same overall duration as the per-axis profile, but one scalar
            # drives every axis, so they land together.
            timing = self._traj_gen.linear_trajectory(q0, q_target_arr, speed=gen_speed)
            path = self._traj_gen.minimum_jerk_trajectory(
                q0, q_target_arr, len(timing) * self._dt)
        else:
            path = self._traj_gen.linear_trajectory(q0, q_target_arr, speed=gen_speed)

        self._mode = _MODE_MOVE_J
        try:
            self._play(path, self._dt)
            return self._arrive(q_target, timeout=timeout)
        finally:
            self._mode = _MODE_INIT

    def _cartesian_start(self, label: str, poses: List[List[float]],
                         speed: float, wait: bool) -> CartPlan:
        """Plan, then play (and optionally wait on) a cartesian path."""
        self._reject_in_zero_g()
        sp = self._check_speed(speed, label)
        self._ensure_running()

        with self._lock:
            q0 = _read_joint_state(self._body, self._joints, self._cid)[0].tolist()

        # The SDK speaks 6-vectors; the sim's planners speak (pos, R). Convert
        # on the way in so both ends keep their own convention.
        pairs = [([p[0], p[1], p[2]], rpy_to_mat(p[3:])) for p in poses]

        t_plan = time.perf_counter()
        if label == "move_l":
            path = self._plan_checked(self._kinematics.plan_movel, q0, poses[0],
                                      pairs[0], label=label)
        elif label == "move_c":
            path = self._plan_checked(self._kinematics.plan_movec, q0, poses[1],
                                      pairs[0], pairs[1], label=label)
        else:
            path = self._plan_checked(self._kinematics.plan_movep, q0, poses[-1],
                                      pairs, label=label)
        plan_us = int((time.perf_counter() - t_plan) * 1e6)
        if not path:
            raise CartesianPlanError(f"{label}: 规划为空")

        interval = self._rate_interval(_CART_WAYPOINT_INTERVAL, self._governed(sp))
        plan = CartPlan(ok=True, err=0, n_wp=len(path), plan_us=plan_us)

        # Serialize cartesian motions (concurrent calls queue, as the SDK
        # documents) and take the supersede token.
        self._motion_serial.acquire()
        gen = self._new_generation()
        self._cart_busy = True
        if wait:
            try:
                played = self._play(path, interval, gen=gen)
                plan.started_busy = True
                if played:
                    st = self._arrive(np.asarray(path[-1], dtype=float))
                    plan.settled = True
                    plan.q_final = list(st.q)
                    plan.settle_err_rad = float(np.max(
                        np.abs(np.asarray(st.q) - np.asarray(path[-1], dtype=float))))
                else:
                    # Superseded mid-path: the trajectory was invalidated, so
                    # report where the arm actually ended up.  Not an error —
                    # this is the SDK's documented ok=True / settled=False case.
                    plan.q_final = list(self.get_state().value.q)
            finally:
                self._cart_busy = False
                self._motion_serial.release()
        else:
            self._play_async(path, interval, gen)
        return plan

    def _plan_checked(self, planner: Callable[..., List[List[float]]],
                      q_start: List[float], goal6: Sequence[float],
                      *args: Any, label: str) -> List[List[float]]:
        """Plan a cartesian path, rejecting one that cannot be executed.

        The sim's planners return whatever the IK produced and say nothing about
        whether it reaches the goal, so an unreachable target would otherwise
        come back as a plausible-looking path. ``args`` are the poses in the
        sim planners' ``(pos, R)`` form; ``goal6`` is the final waypoint as a
        6-vector, used for the arrival check.
        """
        try:
            path = planner(q_start, *args)
        except (ValueError, TypeError) as exc:
            raise InvalidCommandError(f"{label}: 规划失败: {exc}") from None
        if not path:
            raise CartesianPlanError(f"{label}: 规划为空 (目标不可达?)")
        # Each waypoint must be a valid configuration for the arm.
        arr = np.asarray(path, dtype=float)
        if arr.ndim != 2 or arr.shape[1] != len(q_start) or not np.all(np.isfinite(arr)):
            raise CartesianPlanError(f"{label}: 规划结果非法 (形状/数值)")
        # The last waypoint must actually be at the goal: verify the IK landed
        # somewhere real instead of trusting the planner's silence.
        pos, R = self._fk_list(arr[-1].tolist())
        got = list(pos) + list(mat_to_rpy(R))
        if not _pose6_near(got, goal6, _IK_POS_TOL, _IK_ROT_TOL):
            raise CartesianPlanError(
                f"{label}: 末端到不了目标 (残差 "
                f"{max(abs(got[i] - goal6[i]) for i in range(3)) * 1000:.1f} mm)")
        return [row.tolist() for row in arr]

    def _fk_list(self, q: List[float]) -> Tuple[List[float], List[List[float]]]:
        with self._lock:
            return self._kinematics.fk(q)

    @staticmethod
    def _rate_interval(base_interval: float, speed: float) -> float:
        """Per-waypoint interval for a geometrically spaced path at ``speed``."""
        return base_interval / max(speed, _SPEED_FLOOR)

    def _governed(self, speed: float) -> float:
        """Apply the global governor (:meth:`set_speed`) to a per-call speed.

        The two knobs multiply, on the real arm as well: the per-call ``speed``
        scales one trajectory and ``gov_ratio`` scales everything the firmware
        runs. Floored so ``set_speed(0)`` yields a motion that never arrives
        (``MotionTimeoutError``) instead of a division by zero.
        """
        return max(speed * self._gov_ratio, _SPEED_FLOOR)

    @staticmethod
    def _check_speed(speed: Any, label: str) -> float:
        """``speed`` is a 0..1 fraction. Out of range raises, never clamps."""
        try:
            sp = float(speed)
        except (TypeError, ValueError):
            raise InvalidCommandError(
                f"{label}: speed 需 0..1 (给的是 {speed!r})") from None
        if not 0.0 <= sp <= 1.0:
            raise InvalidCommandError(f"{label}: speed 需 0..1 (给的是 {speed})")
        return sp

    def _new_generation(self) -> int:
        """Start a new motion, superseding whatever was in flight."""
        self._motion_gen += 1
        return self._motion_gen

    def _play(self, q_path: Sequence[Sequence[float]], interval: float,
              gen: Optional[int] = None) -> bool:
        """Push a joint path to the controller, one waypoint per ``interval``.

        Returns ``False`` if another motion superseded this one (the firmware
        invalidates a cartesian plan the moment a joint command arrives);
        ``True`` if the path was played out. Latched stops are left for the
        caller's arrival check to report.
        """
        self._ensure_running()
        if gen is None:
            gen = self._new_generation()
        t0 = time.monotonic()
        for i, q_des in enumerate(q_path):
            delay = (t0 + i * interval) - time.monotonic()
            if delay > 0:
                self._control_sleep_with_abort(delay)
            if self._stopped:
                return True
            if self._motion_gen != gen:
                return False
            with self._lock:
                self._controller.set_target(np.asarray(q_des, dtype=float))
        return True

    def _play_async(self, q_path: Sequence[Sequence[float]], interval: float,
                    gen: int) -> None:
        """Play a path in the background (``wait=False``).

        The background thread owns the cartesian slot until it finishes, so a
        second cartesian call queues behind it exactly as it would on the arm.
        """
        def _run() -> None:
            try:
                self._play(q_path, interval, gen=gen)
            finally:
                self._cart_busy = False
                self._motion_serial.release()

        threading.Thread(target=_run, daemon=True, name="pybullet_cart").start()

    def _arrive(self, target: Sequence[float],
                timeout: Optional[float] = None) -> RobotState:
        """Wait until the joints sit on ``target`` for ``arrive_frames`` frames."""
        budget = self._move_timeout if timeout is None else float(timeout)
        deadline = time.monotonic() + budget
        tgt = [float(v) for v in target]
        n_ok = 0
        while True:
            st = self.get_state(refresh=True).value
            if st.faulted:
                raise MotorFaultError(f"未到位即故障: mode={st.mode_name}")
            if len(st.q) == len(tgt):
                near = all(abs(st.q[i] - tgt[i]) < self._q_tol
                           for i in range(len(tgt)))
                slow = all(abs(d) < self._dq_tol for d in st.dq)
                n_ok = n_ok + 1 if (near and slow) else 0
                if n_ok >= self._arrive_frames:
                    return st
            else:
                n_ok = 0
            if time.monotonic() > deadline:
                raise MotionTimeoutError(f"未到位, 超时 {budget}s")
            self._control_sleep_with_abort(self._dt)

    def _require_reachable_within(self, q_goal: Sequence[float],
                                  goal: Sequence[float], pos_tol: float,
                                  rpy_tol: float) -> None:
        """Fail early when the solver cannot stand within ``pos_tol`` of ``goal``.

        The firmware plans ``move_p`` with its own IK, so a target it cannot
        reach ends in ``MotionTimeoutError`` there. The sim knows the answer
        before it moves; waiting out ``move_timeout`` first would report a
        solver limit as a motion failure.
        """
        pos, R = self._fk_list(list(q_goal))
        got = list(pos) + list(mat_to_rpy(R))
        pos_err = float(np.linalg.norm(np.asarray(got[:3]) - np.asarray(goal[:3])))
        if pos_err > pos_tol or _orient_angle(got[3:], goal[3:]) > rpy_tol:
            raise IKError(
                f"move_p: 该位姿的可达精度 {pos_err * 1000:.1f} mm 达不到要求的 "
                f"{pos_tol * 1000:.1f} mm (仿真 IK 残差); 放宽 pos_tol/rpy_tol "
                f"或换一个位姿")

    def _arrive_pose(self, goal: Sequence[float], pos_tol: float,
                     rpy_tol: float) -> RobotState:
        """Wait until the measured TCP is at ``goal`` (``move_p``'s criterion).

        The arm is judged by where its TCP is, not by how close its joints came
        to the IK solution — that is what ``move_p`` means on the real arm.
        """
        deadline = time.monotonic() + self._move_timeout
        while True:
            st = self.get_state(refresh=True).value
            if st.faulted:
                raise MotorFaultError(f"未到位即故障: mode={st.mode_name}")
            tcp = self.get_tcp().value
            if tcp is not None and _pose6_near(tcp, goal, pos_tol, rpy_tol):
                return st
            if time.monotonic() > deadline:
                raise MotionTimeoutError(
                    f"move_p 未到目标位姿, 超时 {self._move_timeout}s")
            self._control_sleep_with_abort(0.02)

    # ── Deprecated motion spellings ────────────────────────────────────────────

    def movel(self, pose_goal: Any, speed: float = 1.0, **kwargs: Any) -> CartPlan:
        """Deprecated: use :meth:`move_l`."""
        _warn_deprecated("movel", "move_l")
        return self.move_l(pose_goal, speed=speed, **kwargs)

    def movec(self, pose_via: Any, pose_goal: Any, speed: float = 1.0,
              **kwargs: Any) -> CartPlan:
        """Deprecated: use :meth:`move_c` (the start is filled in for you)."""
        _warn_deprecated("movec", "move_c")
        return self.move_c(self.get_tcp().value, pose_via, pose_goal,
                           speed=speed, **kwargs)

    def movep(self, poses_goal: Sequence[Any], speed: float = 1.0,
              **kwargs: Any) -> CartPlan:
        """Deprecated: use :meth:`move_path`."""
        _warn_deprecated("movep", "move_path")
        return self.move_path(poses_goal, speed=speed, **kwargs)

    # ── Continuous servo / pass-through ────────────────────────────────────────

    def move_js(self, q: Sequence[float], dq: Optional[Sequence[float]] = None,
                tau_ff: Optional[Sequence[float]] = None) -> None:
        """One frame of joint servo: drive to ``q`` at ``dq`` with feed-forward.

        A single frame, exactly as on the real arm — there is no trajectory and
        no arrival check, and no waiting. A servo loop re-sends this at >=10 Hz;
        on the real arm the firmware's 0.1 s watchdog drops the frame and
        fail-softs back to holding if the caller goes quiet. The simulation has
        no watchdog, so the last commanded point is held indefinitely, which is
        the same place the arm ends up either way.
        """
        self._reject_in_zero_g()
        self._ensure_running()
        if len(q) != self._n_joints:
            raise InvalidCommandError("move_js q 需 N 个")
        dq_list = [0.0] * self._n_joints if dq is None else list(dq)
        if len(dq_list) != self._n_joints:
            raise InvalidCommandError("move_js dq 需 N 个")
        tau_list = [0.0] * self._n_joints if tau_ff is None else list(tau_ff)
        if len(tau_list) != self._n_joints:
            raise InvalidCommandError("move_js tau_ff 需 N 个")

        self._mit_tau = np.asarray(tau_list, dtype=float)
        self._mode = _MODE_MOVE_JS
        with self._lock:
            self._controller.set_target(np.asarray(q, dtype=float),
                                        np.asarray(dq_list, dtype=float))

    def send_mit(self, idx: int, q: float, dq: float,
                 kp: float, kd: float, tau: float) -> None:
        """One MIT impedance frame for a single joint.

        ``kp``/``kd`` are the stiffness and damping the firmware uses for that
        joint and ``tau`` the feed-forward torque. The simulation has no
        per-joint impedance loop, so stiffness and damping go to the joint's
        controller gains and ``tau`` is added to the torque the loop applies —
        the same three numbers, in the same places.
        """
        self._reject_in_zero_g()
        self._ensure_running()
        if not 0 <= int(idx) < self._n_joints:
            raise InvalidCommandError("idx 越界")
        i = int(idx)
        self._mode = _MODE_MOVE_MIT
        with self._lock:
            self._controller.kp[i] = float(kp)
            self._controller.kd[i] = float(kd)
            target = self._controller.desired_q
            target[i] = float(q)
            self._mit_tau[i] = float(tau)
            self._controller.set_target(target)

    def send_mit_all(self, q: Sequence[float], dq: Sequence[float],
                     kp: Sequence[float], kd: Sequence[float],
                     tau: Sequence[float]) -> None:
        """One MIT impedance frame for every joint (see :meth:`send_mit`)."""
        self._reject_in_zero_g()
        self._ensure_running()
        for arr, label in ((q, "q"), (dq, "dq"), (kp, "kp"), (kd, "kd"), (tau, "tau")):
            if len(arr) != self._n_joints:
                raise InvalidCommandError(f"send_mit_all {label} 需 N 个")
        self._mode = _MODE_MIT_ALL
        with self._lock:
            self._controller.kp = np.asarray(kp, dtype=float).copy()
            self._controller.kd = np.asarray(kd, dtype=float).copy()
            self._mit_tau = np.asarray(tau, dtype=float)
            self._controller.set_target(np.asarray(q, dtype=float))

    # ── Zero gravity (free drag) ───────────────────────────────────────────────

    def zero_g(self, period: float = _ZG_KEEPALIVE_S) -> "_ZeroGSession":
        """Enter zero-gravity drag mode and return a context-manager handle.

        Both spellings work, as on the real arm::

            with arm.zero_g():      # exits the mode on block exit
                ...
            arm.zero_g(); ...; arm.zero_g_stop()
        """
        self.zero_g_start(period=period)
        return _ZeroGSession(self)

    def zero_g_start(self, period: float = _ZG_KEEPALIVE_S) -> None:
        """Enter zero gravity (idempotent).

        The position loop is dropped and only gravity is compensated, which is
        what the firmware's ``0x06`` does. ``period`` is validated against the
        same window as litearm-core and otherwise unused: the real SDK re-sends
        the command every ``period`` seconds to stay ahead of the firmware's
        0.10 s watchdog, and the simulation has no watchdog to stay ahead of.

        Cartesian motion in flight is refused rather than coasted through —
        entering mid-trajectory drops the position loop on the real arm too, and
        the arm slides to a stop on friction instead of being taken over.
        """
        if not _ZG_PERIOD_MIN <= period < _ZG_PERIOD_MAX:
            raise InvalidCommandError(
                f"保活周期需 ∈[{_ZG_PERIOD_MIN:.3f}, {_ZG_PERIOD_MAX:.2f}) —— "
                f"固件看门狗超时是 0.10s (给的是 {period})")
        if self._zero_g:
            return
        self._reject_if_cart_in_flight()
        self._zero_g = True
        self._zero_g_error = None
        self._mode = _MODE_ZERO_G

    def zero_g_stop(self, raise_on_lost: bool = False) -> None:
        """Leave zero gravity (idempotent).

        ``raise_on_lost`` reports a keepalive that died on its own; the
        simulation has no keepalive, so it has nothing to raise and is accepted
        for signature parity.
        """
        self._zero_g = False
        if self._mode == _MODE_ZERO_G:
            self._mode = _MODE_INIT
        with self._lock:
            q_current, _ = _read_joint_state(self._body, self._joints, self._cid)
            self._controller.set_target(q_current)

    @property
    def zero_g_active(self) -> bool:
        """Whether zero gravity is currently held."""
        return self._zero_g

    @property
    def zero_g_error(self) -> Optional[BaseException]:
        """Why zero gravity was lost, or ``None``."""
        return self._zero_g_error

    def zero_gravity(self, **kwargs: Any) -> "_ZeroGSession":
        """Deprecated: use :meth:`zero_g`."""
        _warn_deprecated("zero_gravity", "zero_g")
        return self.zero_g()

    def _reject_in_zero_g(self) -> None:
        """Refuse an action command while zero gravity is held.

        litearm_core puts this on the single write path every action command
        goes through, because writing any of them rewrites the mode and kicks
        the watchdog the keepalive is fighting over. The simulation has no
        keepalive thread, but it keeps the refusal: scripts that lean on it
        (and on the message) behave the same in both places, and it stops a
        motion from being commanded into a dropped position loop where it could
        only ever time out.

        Drain-power directions — ``emergency_stop()``, ``disable()``,
        ``zero_g_stop()`` — deliberately do not call this: cutting power must
        stay reachable from every state.

        Checked before the individual argument validations rather than after
        them, which is a step earlier than litearm_core's write-path guard. Both
        orders raise ``InvalidCommandError``; only the message differs on a call
        that is wrong twice over, and here the state of the arm is the more
        fundamental thing to be told about.
        """
        if self._zero_g:
            raise InvalidCommandError(_ZERO_G_GUARD_MESSAGE)

    def _reject_if_cart_in_flight(self) -> None:
        """Refuse to enter zero gravity while a cartesian plan is playing.

        Two signals, as on the real arm: the cartesian slot being held, and the
        live ``cart_busy`` bit. Either alone can be mid-flip, so both are
        checked — the lock is taken without blocking, because "somebody is
        using the link" and "a plan is running" answer the same question here.
        The message is litearm_core's ``CART_IN_FLIGHT_GUARD_MESSAGE`` word for
        word: refusing without naming a better stop action leaves the operator
        stuck, which is exactly why that text spells out ``movej()`` and
        ``emergency_stop()``.
        """
        got = self._motion_serial.acquire(blocking=False)
        if got:
            self._motion_serial.release()
        busy = self._cart_busy or self.get_status_now().value.cart_busy
        if busy or not got:
            raise InvalidCommandError(
                "笛卡尔运动在途, 拒绝进入零重力 (中途进场会丢掉位置环、只剩重力前馈, "
                "臂会靠摩擦滑停): 先 arm.movej() 受控接管收口, 或 arm.emergency_stop() "
                "急停, 或等它结束再进入")

    def recover_joint_limits(
        self,
        speed: float = 0.05,
        inset_rad: float = 0.0,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: walk out-of-limit joints back to safe boundaries."""
        self._ensure_running()
        with self._lock:
            q_current, _ = _read_joint_state(self._body, self._joints, self._cid)

        q_safe = q_current.copy()
        for j in range(self._n_joints):
            lo = Q_MIN[j] + inset_rad
            hi = Q_MAX[j] - inset_rad
            if lo < hi:
                q_safe[j] = np.clip(q_safe[j], lo, hi)

        if np.allclose(q_current, q_safe):
            return self.get_state().value
        return self._move_joint(q_safe.tolist(), speed, label="recover_joint_limits")

    def replay_joint_path(
        self,
        q_path: List[List[float]],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: replay a sequence of joint configurations.

        Played at ``dt/speed`` per waypoint and then waited on, so the returned
        state is where the arm ended up (raising rather than returning False if
        it never got there).
        """
        self._ensure_running()
        if not q_path:
            return self.get_state().value

        if goto_start:
            self.movej(q_path[0], speed=goto_speed)

        interval = self._rate_interval(self._dt, self._governed(
            self._check_speed(speed, "replay_joint_path")))
        self._play(q_path, interval)
        return self._arrive(q_path[-1])

    def replay_trajectory(
        self,
        traj_q: Any,
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        max_cycles: Optional[int] = None,
        check_singularity: bool = True,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: replay a JointTrajectory, dict, or joint path."""
        self._ensure_running()

        if hasattr(traj_q, 'q'):
            q_path = traj_q.q
        elif hasattr(traj_q, 'to_dict'):
            d = traj_q.to_dict()
            q_path = [f["q"] for f in d.get("frames", [])]
        elif isinstance(traj_q, dict):
            frames = traj_q.get("frames", [])
            q_path = [f["q"] for f in frames]
        else:
            q_path = traj_q

        return self.replay_joint_path(
            q_path, speed=speed,
            goto_start=goto_start, goto_speed=goto_speed,
        )

    def replay_timed_trajectory(
        self,
        traj_q: List[List[float]],
        traj_t: List[float],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        simplify_tolerance_rad: float = 0.01,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: replay a measured trajectory on its recorded clock."""
        self._ensure_running()
        sp = self._check_speed(speed, "replay_timed_trajectory")
        if not traj_q:
            return self.get_state().value

        if goto_start:
            self.movej(traj_q[0], speed=goto_speed)

        if len(traj_t) < 2:
            return self.replay_joint_path(traj_q, speed=sp)

        gen = self._new_generation()
        t_prev = traj_t[0]
        for q_des, t in zip(traj_q, traj_t):
            gap = (t - t_prev) / self._governed(sp)
            t_prev = t
            self._control_sleep_with_abort(max(gap, self._dt))
            if self._stopped or self._motion_gen != gen:
                break
            with self._lock:
                self._controller.set_target(np.asarray(q_des, dtype=float))

        return self._arrive(traj_q[-1])

    def play_trajectory(
        self,
        trajectory: Union[Any, str],
        speed: float = 1.0,
        goto_start: bool = True,
        goto_speed: float = 0.3,
        verify_robot: bool = True,
        simplify_tolerance_rad: float = 0.01,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: load and replay a saved trajectory."""
        if isinstance(trajectory, str):
            traj = JointTrajectory.load(trajectory)
        elif isinstance(trajectory, JointTrajectory):
            traj = trajectory
        else:
            traj = JointTrajectory.from_dict(trajectory)

        return self.replay_trajectory(traj, speed=speed, goto_start=goto_start,
                                      goto_speed=goto_speed)

    def record_trajectory(
        self,
        output: str = "trajectories",
        duration_s: Optional[float] = None,
        sample_rate_hz: float = 100.0,
        filter_alpha: float = 0.15,
        name: Optional[str] = None,
        **kwargs: Any,
    ) -> Any:
        """Record a trajectory (simulated)."""
        self._ensure_running()

        if duration_s is None:
            duration_s = 5.0

        dt_sample = 1.0 / sample_rate_hz
        n_samples = int(duration_s / dt_sample)
        frames = []

        old_enabled = self._enabled
        self._enabled = False

        t_start = time.time()
        for i in range(n_samples):
            target_t = t_start + (i + 1) * dt_sample
            sleep_t = target_t - time.time()
            if sleep_t > 0:
                time.sleep(sleep_t)

            with self._lock:
                q, dq = _read_joint_state(self._body, self._joints, self._cid)

            frames.append(TrajectoryFrame(
                t=i * dt_sample, q=q.tolist(), dq=dq.tolist(),
            ))

        self._enabled = old_enabled

        return JointTrajectory(
            frames=frames, name=name or "sim_recording",
            sample_rate_hz=sample_rate_hz, filter_alpha=filter_alpha,
        )

    def hold(
        self,
        kp_scale: float = 3.0,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: hold the current position (no motion commanded)."""
        self._ensure_running()
        with self._lock:
            q_current, _ = _read_joint_state(self._body, self._joints, self._cid)
            self._controller.set_target(q_current)
        return self.get_state().value

    def joint_impedance(
        self,
        q_des: List[float],
        K: Any,
        B: Any,
        tau_max: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: joint-space impedance (a stiff hold at ``q_des``)."""
        return self.joint_follow(q_des=q_des, K=K, B=B)

    def cartesian_impedance(
        self,
        q_des: List[float],
        K_cart: Any,
        B_cart: Any,
        v_des: Optional[Any] = None,
        tau_max: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        sigma_min_thresh: Optional[float] = None,
        max_ori_err: Optional[float] = None,
        measured_overspeed_factor: Optional[float] = None,
        vel_max: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: cartesian impedance (reduced to a joint hold)."""
        return self.joint_follow(q_des=q_des, K=K_cart, B=B_cart)

    def joint_follow(
        self,
        K: Optional[Any] = None,
        B: Optional[Any] = None,
        speed_limit: Optional[Any] = None,
        accel_limit: Optional[Any] = None,
        engage_sec: float = 0.3,
        max_cycles: Optional[int] = None,
        duration_s: Optional[float] = None,
        q_des: Optional[List[float]] = None,
        **kwargs: Any,
    ) -> RobotState:
        """Simulation-only: follow a driver.

        There is no external driver in the simulation, so this only establishes
        the target ``q_des`` (the current pose when not given) and returns the
        state. Update the target with ``set_joint_positions`` to drive it.
        """
        self._ensure_running()
        with self._lock:
            q_now, _ = _read_joint_state(self._body, self._joints, self._cid)
            q_target = q_now if q_des is None else np.asarray(
                q_des, dtype=float)[:self._n_joints]
            self._controller.set_target(q_target)
        return self.get_state().value

    # ── Emergency Stop ─────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        """Cut motor authority now: the arm stops wherever it is and holds nothing.

        A latched stop: nothing moves again until :meth:`reset`. Drain power is
        always allowed — this stays reachable while the arm is in zero-gravity
        or in the middle of a cartesian plan, and it cancels that plan rather
        than waiting for it.
        """
        self._cancel_motion()
        self._stopped = True
        self._mode = _MODE_EMERGENCY
        self._zero_g = False
        with self._lock:
            p.setJointMotorControlArray(
                self._body, self._joints, p.VELOCITY_CONTROL,
                forces=[0.0] * self._n_joints, physicsClientId=self._cid,
            )

    def reset(self) -> None:
        """Clear the emergency latch and any fault, returning to a ready idle.

        Also restores the global speed governor to 100%: the firmware's
        ``gov_ratio`` does not survive a reset either, so a script that set 5%
        and then hit the stop button does not come back up creeping. Jets of
        commanded motion do not survive it either — the arm holds the pose it is
        in, as after a real reset.
        """
        self._reject_in_zero_g()
        self._cancel_motion()
        self._stopped = False
        self._mode = _MODE_INIT
        self._zero_g = False
        self._gov_ratio = 1.0
        self._mit_tau = np.zeros(self._n_joints)
        self._enabled = True
        with self._lock:
            q_current, _ = _read_joint_state(self._body, self._joints, self._cid)
            self._controller.set_target(q_current)

    def request_stop(self) -> None:
        """Deprecated: use :meth:`emergency_stop`."""
        _warn_deprecated("request_stop", "emergency_stop")
        self.emergency_stop()

    def clear_stop(self) -> None:
        """Deprecated: use :meth:`reset`."""
        _warn_deprecated("clear_stop", "reset")
        self.reset()

    def _cancel_motion(self) -> None:
        """Supersede whatever is in flight, the way a stop or reset does.

        Bumping the generation is what actually stops a path being pushed: the
        playing loop sees the mismatch and yields the cartesian slot. Applying
        it on the way into *every* stop/reset path, rather than each of them
        clearing flags by hand, is what keeps "stopped" from meaning different
        things depending on which method was called.
        """
        self._new_generation()
        self._cart_busy = False

    # ── Enable / Disable ───────────────────────────────────────────────────────

    def enable(self, attempts: int = 12) -> None:
        """Enable motors and hold the current pose.

        ``attempts`` is accepted for signature parity with litearm_core, where it
        retries the firmware's "retryable" enable errors (code 0x03 — "feedback
        not ready", the normal first-enable path on real hardware). The
        simulation cannot fail to enable, so it never retries and never sleeps:
        the parameter is described rather than silently ignored.
        """
        self._reject_in_zero_g()
        self._enabled = True
        self._stopped = False
        self._mode = _MODE_INIT
        with self._lock:
            q_current, _ = _read_joint_state(self._body, self._joints, self._cid)
            self._controller.set_target(q_current)

    def disable(self) -> None:
        """Cut motor authority so the arm drops under gravity.

        A drain-power direction, so it stays reachable in every other mode: it
        ends zero-gravity and cancels an in-flight cartesian plan rather than
        refusing while one is running.
        """
        self._cancel_motion()
        self._enabled = False
        self._zero_g = False
        self._mode = _MODE_INIT
        with self._lock:
            p.setJointMotorControlArray(
                self._body, self._joints, p.VELOCITY_CONTROL,
                forces=[0.0] * self._n_joints, physicsClientId=self._cid,
            )

    # ── Parameter Tuning ───────────────────────────────────────────────────────

    def set_gains(self, kp: Optional[Any] = None, kd: Optional[Any] = None) -> Dict:
        """Set PD controller gains."""
        if kp is not None:
            self._controller.kp = np.broadcast_to(
                np.atleast_1d(np.asarray(kp, dtype=float)), self._n_joints
            ).copy()
        if kd is not None:
            self._controller.kd = np.broadcast_to(
                np.atleast_1d(np.asarray(kd, dtype=float)), self._n_joints
            ).copy()
        return {"kp": self._controller.kp.tolist(), "kd": self._controller.kd.tolist()}

    def get_gains(self) -> Dict:
        """Get current PD gains."""
        return {"kp": self._controller.kp.tolist(), "kd": self._controller.kd.tolist()}

    def clear_faults(self) -> None:
        """Clear latched motor faults.

        Returns ``None``, like ``litearm_core.Arm.clear_faults`` — the previous
        signature returned a list of ``(index, code)`` pairs, which invited
        ``if arm.clear_faults():`` and read as failure when nothing was wrong.
        The simulation raises no faults, so there is nothing to report; the
        state is still made readable again, since a caller who reached for this
        usually wants the arm usable afterwards.
        """
        self._reject_in_zero_g()
        self._stopped = False
        self._zero_g = False
        if self._mode == _MODE_EMERGENCY:
            self._mode = _MODE_INIT

    def park(self) -> None:
        """Declare the arm parked: hold the pose at full stiffness."""
        self.set_motion_mode(0)

    def set_motion_mode(self, mode: int) -> None:
        """Declare a motion mode. This firmware only understands ``0`` (park).

        Any other value raises ``InvalidCommandError`` rather than sending a
        command whose only effect would be to clear the park declaration — the
        firmware answers ACK while its mode does not change, so accepting it
        would report success for something that did not happen. The simulation
        keeps the same line: there is no mode to switch to, so claiming one
        would be a lie with an ACK on it.
        """
        self._reject_in_zero_g()
        m = int(mode)
        if not 0 <= m <= 255:
            raise InvalidCommandError(f"mode 需 0..255 (给的是 {mode})")
        if m != 0:
            raise InvalidCommandError(
                f"set_motion_mode({m}): 本固件只识别 0 (park 声明) —— 其它值固件回 ACK "
                f"但模式不变; 要声明 park 请用 arm.park()")

    def set_speed(self, percent: int) -> None:
        """Set the **global** speed governor to an integer percentage 0..100.

        Not the same knob as ``speed`` on a motion call: that is a 0..1
        fraction for one trajectory, this scales every motion until
        :meth:`reset`. ``set_speed(1)`` means *1%*, so calling it with a 0..1
        value thinking it is a multiplier gives an arm that creeps.

        Only an ``int`` is accepted; ``bool`` is rejected despite being an
        ``int`` subclass, because ``set_speed(True)`` reads as "on" and would
        silently mean 1%.
        """
        self._reject_in_zero_g()
        if isinstance(percent, bool) or not isinstance(percent, int):
            raise InvalidCommandError(
                f"percent 需整数百分比 0..100 (给的是 {percent!r}); "
                f"若手里是 0..1 的倍率请乘 100")
        if not 0 <= percent <= 100:
            raise InvalidCommandError("percent 需 0..100")
        self._gov_ratio = percent / 100.0

    def set_payload(
        self, mass: float, com: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    ) -> None:
        """Accepted for call-site parity with litearm-core, and nothing more.

        On the real arm this writes the tool mass and its centre of mass into
        the firmware's gravity feed-forward. The simulation takes gravity from
        the URDF through PyBullet's inverse dynamics, so there is no such
        parameter to set and this changes nothing — the same call, the same
        return, and no effect, rather than a return value that implies one.
        """
        return None

    def get_payload(self) -> Dict:
        return {"mass": 0.0, "com": [0.0, 0.0, 0.0]}

    def set_installation(
        self,
        base_rpy: Optional[List[float]] = None,
        gravity: Optional[List[float]] = None,
    ) -> Dict:
        return {"base_rpy": base_rpy, "gravity": gravity}

    def get_installation(self) -> Dict:
        return {"base_rpy": [0, 0, 0], "gravity": [0, 0, -9.81]}

    # ── System / Settings / Trajectory / Device / Teleop ───────────────────────

    def get_system_stats(self) -> Dict[str, Any]:
        return {"cpu": 0.0, "memory": 0.0, "board_temp": 25.0, "uptime": 0.0}

    def get_logs(self, page: int = 1, size: int = 50, search: str = "") -> Dict[str, Any]:
        return {"logs": [], "total": 0, "page": page, "size": size}

    def restart_service(self) -> Dict[str, Any]:
        return {"status": "ok"}

    def get_joint_limits(self) -> Dict[str, Any]:
        limits = {}
        for j in range(self._n_joints):
            limits[f"joint{j}"] = {
                "min": float(Q_MIN[j]),
                "max": float(Q_MAX[j]),
            }
        return limits

    def set_joint_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        return limits

    def get_zero_offsets(self) -> Dict[str, Any]:
        return {"offsets": [0.0] * self._n_joints}

    def set_zero_offsets(self, offsets: Dict[str, Any]) -> Dict[str, Any]:
        return offsets

    def get_end_effector(self) -> Dict[str, Any]:
        return {"type": "none"}

    def set_end_effector(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return config

    def get_cartesian_limits(self) -> Dict[str, Any]:
        return {"x": [-1, 1], "y": [-1, 1], "z": [0, 1.5]}

    def set_cartesian_limits(self, limits: Dict[str, Any]) -> Dict[str, Any]:
        return limits

    def get_collision_config(self) -> Dict[str, Any]:
        return {}

    def set_collision_config(self, config: Dict[str, Any]) -> Dict[str, Any]:
        return config

    def start_recording(self) -> Dict[str, Any]:
        return {"status": "recording"}

    def stop_recording(self) -> Dict[str, Any]:
        return {"status": "stopped"}

    def discard_recording(self) -> Dict[str, Any]:
        return {"status": "discarded"}

    def get_recording_state(self) -> Dict[str, Any]:
        return {"recording": False}

    def get_playback_state(self) -> Dict[str, Any]:
        return {"playing": False}

    def list_trajectories(self) -> Dict[str, Any]:
        return {"trajectories": []}

    def save_trajectory(
        self, id: str, name: str, points: List[List[float]],
        duration: Optional[float] = None,
    ) -> Dict[str, Any]:
        return {"id": id, "name": name, "saved": True}

    def delete_trajectory(self, id: str) -> Dict[str, Any]:
        return {"id": id, "deleted": True}

    def list_device_types(self) -> List[Dict[str, Any]]:
        return [
            {"category": "hand", "subtype": "sim_hand", "label": "Simulated Hand"},
            {"category": "gripper", "subtype": "sim_gripper", "label": "Simulated Gripper"},
        ]

    def connect_device(
        self, category: str, subtype: str, device_id: str = "end_0",
        can_iface: str = "", config: Optional[Dict[str, Any]] = None,
    ) -> Dict[str, Any]:
        return {"device_id": device_id, "connected": True}

    def disconnect_device(self, device_id: str = "end_0") -> Dict[str, Any]:
        return {"device_id": device_id, "disconnected": True}

    def get_active_device(self, device_id: str = "end_0") -> Dict[str, Any]:
        return {"device_id": device_id, "active": False}

    def enter_teleop(self, mode: str, **params: Any) -> Dict[str, Any]:
        return {"mode": mode, "active": True}

    def exit_teleop(self) -> Dict[str, Any]:
        return {"active": False}

    def get_teleop_status(self) -> Dict[str, Any]:
        return {"active": False, "mode": "none"}

    # ── Device (for API compatibility) ─────────────────────────────────────────

    def device(self, device_id: str) -> Any:
        """Return a simulated device proxy."""
        if self._devices is None:
            self._devices = _SimDeviceManager()
        return self._devices.get(device_id)

    @property
    def devices(self) -> Any:
        """Device manager (simulated). Supports ``arm.devices["hand_0"]`` syntax."""
        if self._devices is None:
            self._devices = _SimDeviceManager()
        return self._devices

    @property
    def hand(self) -> Any:
        """Backward-compatible hand property."""
        return self.device("hand_0")

    # ── Mirror Mode ────────────────────────────────────────────────────────────

    def mirror_from(self, real_arm: Any, rate_hz: float = _MIRROR_RATE_HZ) -> None:
        """Start mirroring the state of a real arm into this simulation.

        A daemon thread polls ``real_arm`` and puts each frame's joint vector
        into the simulation, so the arm on screen follows the physical one.

        Args:
            real_arm: A ``litearm_core.Arm`` (anything answering
                ``get_state(refresh=True)`` with a ``Msg`` whose ``value.q`` is
                the joint vector).
            rate_hz: How often to poll. Every frame is a serial round trip — the
                link has no background reader to consume — so this is a real
                load on the wire, not a display preference.

        The read is ``refresh=True`` on purpose: ``refresh=False`` would hand
        back the last frame *this caller* read, and a mirror that reports the
        same pose forever is exactly the failure you cannot see.

        Do not send commands to ``real_arm`` yourself while mirroring: the two
        readers steal each other's frames (litearm-core counts that as
        ``_foreign``). Call :meth:`stop_mirroring` first.

        Usage::

            import litearm_core as pa
            real = pa.Arm(port="/dev/ttyACM0").connect()
            sim = PyBulletArm(render=True)
            sim.connect()
            sim.mirror_from(real)
            # Now sim follows real arm's motion
        """
        self.stop_mirroring()
        self._mirror_arm = real_arm
        self._mirror_rate_hz = float(rate_hz)
        self._mirror_error = None
        self._mirroring = True
        self._ensure_running()
        self._mirror_thread = threading.Thread(
            target=self._mirror_loop, daemon=True, name="pybullet_mirror"
        )
        self._mirror_thread.start()

    def stop_mirroring(self) -> None:
        """Stop mirroring the real arm (idempotent)."""
        self._mirroring = False
        thread, self._mirror_thread = self._mirror_thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)
        self._mirror_arm = None

    def _mirror_loop(self) -> None:
        """Poll the real arm at ``rate_hz`` and drive the simulation's target."""
        period = 1.0 / max(self._mirror_rate_hz, 1e-3)
        while self._mirroring:
            started = time.monotonic()
            try:
                msg = self._mirror_arm.get_state(
                    refresh=True, timeout=_MIRROR_READ_TIMEOUT
                )
                q_real = getattr(msg.value, "q", None)
                if q_real is None:
                    raise MotionTimeoutError(
                        "镜像: 真臂没有回帧 (Msg.value 为 None)"
                    )
                if len(q_real) < self._n_joints:
                    raise InvalidCommandError(
                        f"镜像: 真臂只有 {len(q_real)} 个关节, 仿真有 {self._n_joints} 个"
                    )
                # Teleport rather than chase the target through the PD loop: the
                # point of a mirror is to show where the real arm *is*, not how
                # well the simulation would have tracked it.
                self.set_joint_positions(list(q_real))
                self._mirror_error = None
            except Exception as exc:  # noqa: BLE001 - a dropped frame must not
                # end the mirror: the link is allowed to be quiet for a while.
                # It is *recorded* so "mirroring silently does nothing" stays
                # diagnosable — read `mirror_error`.
                self._mirror_error = exc
            time.sleep(max(0.0, period - (time.monotonic() - started)))

    @property
    def mirroring(self) -> bool:
        """True while the mirror thread is polling a real arm."""
        return self._mirroring

    @property
    def mirror_error(self) -> Optional[BaseException]:
        """Why the last mirror frame failed, or ``None`` if it worked.

        The mirror loop cannot raise into the caller — it runs on its own
        thread — so this is how a failed mirror is told apart from an idle one.
        """
        return self._mirror_error

    # ── Internal Helpers ───────────────────────────────────────────────────────

    def _ensure_running(self) -> None:
        """Start the simulation thread if not already running."""
        if not self._sim_running:
            self.start()
            time.sleep(0.05)

    def _control_sleep_with_abort(self, duration: float) -> None:
        """Sleep for a duration, but abort early if stopped."""
        if duration <= 0:
            return
        check_interval = min(duration, 0.01)
        end = time.time() + duration
        while time.time() < end:
            if self._stopped:
                return
            time.sleep(min(check_interval, end - time.time()))

    def set_joint_positions(self, q: List[float]) -> None:
        """Directly set joint positions (for mirror mode)."""
        q_arr = np.asarray(q, dtype=float)[:self._n_joints]
        with self._lock:
            for jid, qi in zip(self._joints, q_arr):
                p.resetJointState(self._body, jid, targetValue=float(qi),
                                  physicsClientId=self._cid)
            self._controller.set_target(q_arr)


class _ZeroGSession:
    """The handle ``PyBulletArm.zero_g()`` returns, usable as a context manager.

    Same shape as litearm-core's ``_ZeroGSession``: entering the block yields
    the arm, leaving it stops zero gravity. Leaving it may also report why the
    session was lost, but never in a way that hides an exception from the block
    body — an error raised inside the ``with`` is the one worth seeing.
    """

    def __init__(self, arm: "PyBulletArm") -> None:
        self._arm = arm

    def __enter__(self) -> "PyBulletArm":
        return self._arm

    def __exit__(self, exc_type: Any, exc: Any, tb: Any) -> bool:
        self._arm.zero_g_stop(raise_on_lost=exc_type is None)
        return False


class _SimDevice:
    """Simulated device proxy (returns canned responses)."""

    def __init__(self, device_id: str) -> None:
        self._device_id = device_id

    @property
    def device_id(self) -> str:
        return self._device_id

    def open(self) -> bool: return True
    def close(self) -> bool: return True
    def set_gesture(self, gesture: str) -> bool: return True
    def list_gestures(self) -> list: return ["open", "close", "pinch"]
    def set_force(self, force: float) -> bool: return True
    def get_state(self) -> dict: return {"connected": True}
    def get_status(self) -> dict: return {"ok": True}
    def get_info(self) -> dict: return {"type": "sim"}
    def connect(self) -> bool: return True
    def disconnect(self) -> bool: return True
    def clear_faults(self) -> bool: return True
    def finger_move(self, pose: list) -> bool: return True
    def set_speed(self, speed: list) -> bool: return True
    def set_torque(self, torque: list) -> bool: return True
    def set_width(self, width: float) -> bool: return True
    def get_width(self) -> float: return 0.5
    def get_joints(self) -> list: return [0.0] * 7
    def get_buttons(self) -> dict: return {}

    def __repr__(self) -> str:
        return f"_SimDevice({self._device_id!r})"


class _SimDeviceManager:
    """Simulated device manager (API-compatible with litearm.DeviceManager).

    Supports ``arm.devices["hand_0"]`` and ``arm.devices.get("hand_0")``.
    """

    def __init__(self) -> None:
        self._devices: dict = {}

    def get(self, device_id: str) -> "_SimDevice":
        if device_id not in self._devices:
            self._devices[device_id] = _SimDevice(device_id)
        return self._devices[device_id]

    def __getitem__(self, device_id: str) -> "_SimDevice":
        return self.get(device_id)

    def __contains__(self, device_id: str) -> bool:
        return device_id in self._devices

    def __repr__(self) -> str:
        return f"_SimDeviceManager({list(self._devices.keys())})"