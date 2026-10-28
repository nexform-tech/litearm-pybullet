"""DualArm — drive a real arm and its simulation at the same time.

``DualArm`` sends every command to both arms, so a script can be rehearsed on
the simulation and then run against hardware with the same motion. ``MirrorMode``
is the opposite direction: the simulation follows the real arm, for watching the
real arm's state.

Both need ``litearm-python`` (the real-arm SDK). The simulation half needs only
pybullet + numpy.

Usage::

    from litearm_pybullet import DualArm, MirrorMode

    # Mode 1: dual control — the same command goes to both arms
    dual = DualArm(real_port="/dev/ttyACM0", render=True)
    dual.enable()                      # explicit: this moves a real arm
    dual.movej([0.0]*7, speed=0.2)
    dual.close()

    # Mode 2: mirror — the simulation follows the real arm
    import litearm as pa
    real = pa.Arm(port="/dev/ttyACM0").connect()
    sim = PyBulletArm(render=True)
    sim.connect()
    with MirrorMode(real, sim):
        time.sleep(5.0)
"""
from __future__ import annotations

import threading
import time
from typing import Any, List, Optional, Sequence, Tuple

from . import _compat
from .arm import PyBulletArm

#: How often the real arm is polled. Every poll is a serial round trip — the SDK
#: has no background reader thread — so this is a real load on the link.
_DEFAULT_MIRROR_HZ = 50.0

#: How long one mirror read may block. Short, because a silent link should cost
#: the mirror a few frames, not a stalled thread.
_MIRROR_READ_TIMEOUT = 0.05


def _require_sdk() -> None:
    """Raise the install hint when ``litearm`` is not importable."""
    if not _compat.HAS_LITEARM:
        raise ImportError(
            "DualArm/MirrorMode 需要 litearm-python (真机后端)。它不在 PyPI 上, "
            "要从源码装: pip install -e ../litearm-python"
        ) from _compat.SDK_IMPORT_ERROR


def _frame_q(real_arm: Any, n_joints: int, timeout: float) -> List[float]:
    """Read one joint vector from a real arm, or raise.

    ``refresh=True`` is not optional: ``refresh=False`` hands back the last
    frame *this caller* read, so a mirror built on it would report the same pose
    forever and look like a hung arm rather than a stale read.
    """
    msg = real_arm.get_state(refresh=True, timeout=timeout)
    q = getattr(msg.value, "q", None)
    if q is None:
        raise _compat.MotionTimeoutError("镜像: 真臂没有回帧 (Msg.value 为 None)")
    if len(q) < n_joints:
        raise _compat.InvalidCommandError(
            f"镜像: 真臂只有 {len(q)} 个关节, 仿真有 {n_joints} 个"
        )
    return list(q)


class _MirrorPoller:
    """Polls a real arm at a fixed rate and writes each frame into a simulation.

    Shared by :class:`DualArm` and :class:`MirrorMode`, which differ only in
    whose thread the *commands* run on. One implementation, so the two cannot
    drift on the things that matter here: ``refresh=True``, the rate limit, and
    what happens to a failed frame.
    """

    def __init__(self, real_arm: Any, sim_arm: PyBulletArm,
                 rate_hz: float = _DEFAULT_MIRROR_HZ) -> None:
        self._real = real_arm
        self._sim = sim_arm
        self._period = 1.0 / max(float(rate_hz), 1e-3)
        self._on = threading.Event()
        self._stop = threading.Event()
        self._first = threading.Event()
        self._lock = threading.Lock()
        self._error: Optional[BaseException] = None
        self._thread: Optional[threading.Thread] = None

    # ── Start/stop ─────────────────────────────────────────────────────────────

    def start(self, timeout: float = 2.0) -> bool:
        """Start polling; return whether a first frame arrived within ``timeout``.

        Waiting for that first frame is what makes "mirror first, then command"
        mean anything: otherwise the simulation is still wherever it was when the
        first command lands. ``timeout=0`` polls without waiting (and reports
        False for the frame it did not wait for) — that is not a failure, so it
        does not set :attr:`error`.
        """
        if self._thread is not None:
            return True
        self._stop.clear()
        self._on.set()
        self._error = None
        self._first.clear()
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="dual_mirror"
        )
        self._thread.start()
        if timeout <= 0:
            return False
        got = self._first.wait(timeout)
        if not got:
            self._error = self._error or _compat.MotionTimeoutError(
                f"镜像: {timeout:.1f}s 内没有拿到第一帧"
            )
        return got

    def stop(self, timeout: float = 2.0) -> None:
        """Stop polling and wait for the thread to leave (idempotent)."""
        self._on.clear()
        self._stop.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=timeout)

    def pause(self) -> None:
        """Stop mirroring, and guarantee no frame lands after this returns.

        Commands and mirroring must not overlap: a mirrored frame arriving
        mid-move would overwrite the target the command just set. Taking the
        lock is what makes the guarantee real — an already-running iteration
        finishes (or is waited out) before this returns.
        """
        with self._lock:
            self._on.clear()

    def resume(self) -> None:
        """Undo :meth:`pause`."""
        self._on.set()

    # ── Observation ────────────────────────────────────────────────────────────

    @property
    def active(self) -> bool:
        """True while frames are being applied."""
        return self._on.is_set() and self._thread is not None

    @property
    def error(self) -> Optional[BaseException]:
        """Why the last frame failed, or ``None`` if it worked.

        The loop runs on its own thread and cannot raise into the caller, so
        this is how a broken mirror is told apart from an idle one.
        """
        return self._error

    # ── The loop ───────────────────────────────────────────────────────────────

    def _loop(self) -> None:
        while not self._stop.is_set():
            started = time.monotonic()
            if self._on.is_set():
                with self._lock:
                    if self._on.is_set():
                        self._once()
            time.sleep(max(0.0, self._period - (time.monotonic() - started)))

    def _once(self) -> None:
        try:
            q = _frame_q(self._real, self._sim.n, _MIRROR_READ_TIMEOUT)
            # Teleport rather than chase the target through the PD loop: a mirror
            # shows where the real arm *is*, not how well the sim would track it.
            self._sim.set_joint_positions(q)
            self._error = None
        except Exception as exc:  # noqa: BLE001 - a dropped frame must not end
            # the mirror; the link is allowed to be quiet for a while. Recorded
            # rather than raised, so read `error`.
            self._error = exc
        else:
            self._first.set()


class DualArm:
    """Control both a real arm and a simulated arm simultaneously.

    The same motion command goes to both, so a motion can be tried on the
    simulation and then repeated on hardware. Commands run on the caller's
    thread for the real arm and on a background thread for the simulation,
    because the real arm's calls block on the serial link while the simulation's
    block until the arm arrives — running them in sequence would take the sum of
    the two.

    The real arm is **not** enabled automatically. ``enable()`` sends current
    through motors that are attached to something physical, and that should be a
    line the operator can see — see :meth:`enable`.

    Usage::

        dual = DualArm(real_port="/dev/ttyACM0", render=True)
        dual.enable()
        dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)
        dual.close()
    """

    def __init__(
        self,
        real_port: Optional[str] = None,
        sim_model_path: Optional[str] = None,
        render: bool = True,
        mirror_first: bool = True,
        mirror_rate_hz: float = _DEFAULT_MIRROR_HZ,
        **sim_kwargs: Any,
    ) -> None:
        """Initialize DualArm and connect both arms.

        Args:
            real_port: Serial port of the real arm. ``None`` lets litearm-python
                find the single CDC device, as ``litearm.Arm()`` does.
            sim_model_path: Path to URDF model (default: built-in).
            render: Whether to open the PyBullet GUI.
            mirror_first: If True, :meth:`start` waits for the real arm's first
                frame before returning, so the simulation is already at the real
                pose when the first command lands. If False it starts polling
                without waiting.
            mirror_rate_hz: How often to poll the real arm while mirroring.
            **sim_kwargs: Additional arguments passed to PyBulletArm.
        """
        _require_sdk()
        self._real = _compat.Arm(port=real_port).connect()
        try:
            self._sim = PyBulletArm(
                model_path=sim_model_path, render=render, **sim_kwargs
            )
            self._sim.connect()
        except BaseException:
            # Never leave a live serial link behind for a constructor that
            # failed: the port would stay open until the process exits.
            self._real.close()
            raise

        self._mirror = _MirrorPoller(self._real, self._sim, mirror_rate_hz)
        self._mirror_first = mirror_first
        self._closed = False

    def start(self) -> "DualArm":
        """Start mirroring the real arm into the simulation; returns ``self``.

        Use as ``DualArm(...).start()`` to get both arms running and aligned.
        ``mirror_first`` decides whether this waits for the arm's first frame, so
        that both arms begin from the same pose instead of whatever pose the
        simulation happened to be left in.
        """
        self._mirror.start(timeout=2.0 if self._mirror_first else 0.0)
        return self

    # ── Commands: both arms ────────────────────────────────────────────────────

    def _both(self, label: str, sim_call: Any, real_call: Any) -> Tuple[Any, Any]:
        """Run ``sim_call`` and ``real_call``, then return ``(real, sim)``.

        Both are always attempted before anything is raised — a failed command
        on one arm must not leave the other one having silently never been told.
        The real arm's exception wins when both fail: it names the failure the
        operator has to deal with.
        """
        self._mirror.pause()
        sim_box: List[Any] = []
        sim_out: List[Any] = []

        def _run_sim() -> None:
            try:
                sim_out.append(sim_call())
            except BaseException as exc:  # noqa: BLE001 - re-raised below
                sim_box.append(exc)

        thread = threading.Thread(
            target=_run_sim, daemon=True, name=f"dual_sim_{label}"
        )
        thread.start()
        real_exc: Optional[BaseException] = None
        real_out: Any = None
        try:
            real_out = real_call()
        except BaseException as exc:  # noqa: BLE001 - re-raised below
            real_exc = exc
        finally:
            thread.join()
            self._mirror.resume()
        if real_exc is not None:
            raise real_exc
        if sim_box:
            raise sim_box[0]
        return real_out, sim_out[0]

    def movej(self, q: Sequence[float], speed: float = 1.0) -> Tuple[Any, Any]:
        """``movej`` on both arms -> ``(real_state, sim_state)``."""
        return self._both(
            "movej",
            lambda: self._sim.movej(q, speed=speed),
            lambda: self._real.movej(q, speed=speed),
        )

    def movej_sync(self, q: Sequence[float], speed: float = 1.0) -> Tuple[Any, Any]:
        """``movej_sync`` on both arms -> ``(real_state, sim_state)``."""
        return self._both(
            "movej_sync",
            lambda: self._sim.movej_sync(q, speed=speed),
            lambda: self._real.movej_sync(q, speed=speed),
        )

    def move_p(self, pose: Any, speed: float = 1.0,
               pos_tol: float = 0.006, rpy_tol: float = 0.03) -> Tuple[Any, Any]:
        """``move_p`` on both arms -> ``(real_state, sim_state)``.

        Tolerances default to the SDK's, but the simulation's IK is local and
        leaves a few millimetres, so it may need the looser values a real arm
        does not.
        """
        return self._both(
            "move_p",
            lambda: self._sim.move_p(pose, speed=speed, pos_tol=pos_tol,
                                     rpy_tol=rpy_tol),
            lambda: self._real.move_p(pose, speed=speed, pos_tol=pos_tol,
                                      rpy_tol=rpy_tol),
        )

    def move_l(self, pose: Any, speed: float = 1.0,
               wait: bool = True) -> Tuple[Any, Any]:
        """``move_l`` on both arms -> ``(real_plan, sim_plan)``."""
        return self._both(
            "move_l",
            lambda: self._sim.move_l(pose, speed=speed, wait=wait),
            lambda: self._real.move_l(pose, speed=speed, wait=wait),
        )

    def move_c(self, pose_start: Any, pose_via: Any, pose_goal: Any,
               speed: float = 1.0, wait: bool = True) -> Tuple[Any, Any]:
        """``move_c`` on both arms -> ``(real_plan, sim_plan)``.

        ``pose_start`` must match each arm's *measured* TCP, and the two arms
        measure their own — so this is the one command where the arguments both
        arms share can still be rejected by one of them.
        """
        return self._both(
            "move_c",
            lambda: self._sim.move_c(pose_start, pose_via, pose_goal,
                                     speed=speed, wait=wait),
            lambda: self._real.move_c(pose_start, pose_via, pose_goal,
                                      speed=speed, wait=wait),
        )

    def move_path(self, poses: Sequence[Any], speed: float = 1.0,
                  wait: bool = True) -> Tuple[Any, Any]:
        """``move_path`` on both arms -> ``(real_plan, sim_plan)``."""
        return self._both(
            "move_path",
            lambda: self._sim.move_path(poses, speed=speed, wait=wait),
            lambda: self._real.move_path(poses, speed=speed, wait=wait),
        )

    # ── State ──────────────────────────────────────────────────────────────────

    def get_real_state(self, refresh: bool = True,
                       timeout: float = 0.5) -> Any:
        """Real arm state as a ``Msg[RobotState]`` — read ``.value``.

        ``refresh`` defaults to True here, unlike on the arm itself: this getter
        is for watching the real arm, and ``refresh=False`` would keep returning
        the last frame *this caller* read instead of the current pose.
        ``Msg.value`` is ``None`` when no frame arrived.
        """
        return self._real.get_state(refresh=refresh, timeout=timeout)

    def get_sim_state(self) -> Any:
        """Simulation state as a ``Msg[RobotState]`` — read ``.value``.

        Never ``None``: the simulation always has a state to hand back.
        """
        return self._sim.get_state()

    def get_tcp(self, timeout: float = 0.6) -> Any:
        """Real arm TCP pose as a ``Msg`` — ``.value`` is xyz+rpy or ``None``."""
        return self._real.get_tcp(timeout=timeout)

    def get_sim_tcp(self, timeout: float = 0.6) -> Any:
        """Simulation TCP pose as a ``Msg`` — ``.value`` is xyz+rpy."""
        return self._sim.get_tcp(timeout=timeout)

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def enable(self, attempts: int = 12) -> None:
        """Enable both arms, real one first.

        Deliberately not called by the constructor: enabling drives current
        through a physical arm, and a script that energises hardware as a side
        effect of building an object is a script nobody can review. Call it
        where it can be seen.

        If the real arm refuses, nothing on the simulation side has been touched
        yet; if the simulation then fails, the real arm stays enabled — disable
        it with :meth:`disable` or :meth:`close`.
        """
        self._real.enable(attempts=attempts)
        self._sim.enable(attempts=attempts)

    def disable(self) -> None:
        """Disable both arms, simulation first."""
        self._sim.disable()
        self._real.disable()

    def emergency_stop(self) -> None:
        """Cut both arms, real one first — this is the path that must not wait."""
        self._real.emergency_stop()
        self._sim.emergency_stop()

    def reset(self) -> None:
        """Clear the stop latch on both arms, real one first."""
        self._real.reset()
        self._sim.reset()

    @property
    def real(self) -> Any:
        """The ``litearm.Arm``, for anything this wrapper does not forward."""
        return self._real

    @property
    def sim(self) -> PyBulletArm:
        """The :class:`PyBulletArm`."""
        return self._sim

    @property
    def mirroring(self) -> bool:
        """True while the simulation is being driven by the real arm."""
        return self._mirror.active

    @property
    def mirror_error(self) -> Optional[BaseException]:
        """Why the last mirrored frame failed, or ``None`` if it worked."""
        return self._mirror.error

    def close(self) -> None:
        """Stop mirroring, close the simulation, close the real arm (idempotent)."""
        if self._closed:
            return
        self._closed = True
        self._mirror.stop()
        try:
            self._sim.close()
        finally:
            # Always, even if the viewer refuses to die: a serial port outliving
            # its owner is the failure that needs a reboot to clear.
            self._real.close()

    def __enter__(self) -> "DualArm":
        return self.start()

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"DualArm(real={self._real!r}, sim={self._sim!r})"


class MirrorMode:
    """Simulation tracks a real arm, at a chosen rate.

    ``PyBulletArm.mirror_from`` does the same thing; this variant owns the loop,
    so it can be started and stopped independently of the simulation, and works
    with an arm object it did not create.

    Usage::

        import litearm as pa
        from litearm_pybullet import PyBulletArm, MirrorMode

        real = pa.Arm(port="/dev/ttyACM0").connect()
        sim = PyBulletArm(render=True)
        sim.connect()

        with MirrorMode(real, sim, rate_hz=50.0):
            time.sleep(5.0)      # sim follows real arm

        sim.close()
        real.close()
    """

    def __init__(self, real_arm: Any, sim_arm: PyBulletArm,
                 rate_hz: float = _DEFAULT_MIRROR_HZ) -> None:
        """Args:
            real_arm: A connected ``litearm.Arm``.
            sim_arm: The :class:`PyBulletArm` to drive.
            rate_hz: How often to poll the real arm. Every poll is a serial
                round trip, so this is a load on the link, not a display
                preference.
        """
        _require_sdk()
        self._mirror = _MirrorPoller(real_arm, sim_arm, rate_hz)

    @property
    def real(self) -> Any:
        return self._mirror._real

    @property
    def sim(self) -> PyBulletArm:
        return self._mirror._sim

    def start(self, timeout: float = 2.0) -> bool:
        """Start following; returns whether the first frame arrived in time."""
        self.sim.connect()
        return self._mirror.start(timeout=timeout)

    def stop(self) -> None:
        """Stop following (idempotent)."""
        self._mirror.stop()

    @property
    def running(self) -> bool:
        """True while frames are being applied."""
        return self._mirror.active

    @property
    def error(self) -> Optional[BaseException]:
        """Why the last mirrored frame failed, or ``None`` if it worked."""
        return self._mirror.error

    def __enter__(self) -> "MirrorMode":
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
        self.stop()
