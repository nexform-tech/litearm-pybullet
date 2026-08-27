"""DualArm — control real and simulated arms simultaneously.

Provides DualArm for sending commands to both real and simulated arms,
and MirrorMode for having the simulation track the real arm's state.

Usage::

    from litearm_pybullet import DualArm, MirrorMode

    # Mode 1: Dual control — send commands to both
    dual = DualArm(real_endpoint="tcp/192.168.31.139:7447")
    dual.start()
    dual.movej([0.0]*7, speed=0.2)  # Both arms move!
    dual.close()

    # Mode 2: Mirror — simulation follows real arm
    import litearm
    real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
    sim = PyBulletArm(render=True)
    sim.start()
    sim.mirror_from(real)  # sim tracks real arm
"""
from __future__ import annotations

import threading
import time
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from .arm import PyBulletArm


class DualArm:
    """Control both a real arm and a simulated arm simultaneously.

    Sends the same motion commands to both arms. The simulation can be
    optionally rendered for visualization.

    Usage::

        dual = DualArm(real_endpoint="tcp/192.168.31.139:7447", render=True)
        dual.start()

        # Both arms execute the same motion
        dual.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.2)

        dual.close()
    """

    def __init__(
        self,
        real_endpoint: str = "tcp/192.168.31.139:7447",
        real_arm_id: str = "armA",
        sim_model_path: Optional[str] = None,
        render: bool = True,
        mirror_first: bool = True,
        **sim_kwargs: Any,
    ) -> None:
        """Initialize DualArm.

        Args:
            real_endpoint: Zenoh endpoint for the real arm.
            real_arm_id: Arm identifier.
            sim_model_path: Path to URDF model (default: built-in).
            render: Whether to open the PyBullet GUI.
            mirror_first: If True, start mirroring real arm state to simulation
                         before sending any commands.
            **sim_kwargs: Additional arguments passed to PyBulletArm.
        """
        try:
            from . import _litearm as litearm
        except ImportError:
            raise ImportError(
                "DualArm requires litearm-python dependencies. Install with: "
                "pip install litearm-pybullet[mirror]"
            )

        self._real = litearm.Arm(endpoint=real_endpoint, arm_id=real_arm_id)
        self._sim = PyBulletArm(model_path=sim_model_path, render=render, **sim_kwargs)
        self._sim.start()

        self._mirror_first = mirror_first
        self._mirroring = False
        self._mirror_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()

    def start(self) -> None:
        """Start the simulation and optional mirroring."""
        if self._mirror_first:
            self._start_mirroring()

    def _start_mirroring(self) -> None:
        """Start mirroring real arm state to simulation."""
        self._mirroring = True
        self._mirror_thread = threading.Thread(
            target=self._mirror_loop, daemon=True, name="dual_mirror"
        )
        self._mirror_thread.start()
        time.sleep(0.5)

    def _mirror_loop(self) -> None:
        """Continuously read real arm state and update simulation."""
        while self._mirroring:
            try:
                state = self._real.get_state()
                if state and state.get("q"):
                    q = state["q"]
                    with self._lock:
                        self._sim.set_joint_positions(q)
                        self._sim._controller.set_target(np.asarray(q))
            except Exception:
                pass
            time.sleep(0.02)  # ~50 Hz

    def _pause_mirroring(self) -> None:
        self._mirroring = False

    def _resume_mirroring(self) -> None:
        self._mirroring = True

    # ── Motion Control ─────────────────────────────────────────────────────────

    def movej(
        self, q_target: List[float], speed: float = 1.0,
        settle_s: float = 1.0, **kwargs: Any
    ) -> Tuple[bool, bool]:
        """Send movej to both arms."""
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movej(q_target, speed=speed,
                                             settle_s=settle_s, **kwargs)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.movej(q_target, speed=speed,
                                        settle_s=settle_s, **kwargs)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    def movel(
        self, pose_goal: Any, speed: float = 1.0,
        settle_s: float = 0.8, **kwargs: Any
    ) -> Tuple[bool, bool]:
        """Send movel to both arms."""
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movel(pose_goal, speed=speed,
                                             settle_s=settle_s, **kwargs)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.movel(pose_goal, speed=speed,
                                        settle_s=settle_s, **kwargs)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    def movec(
        self, pose_via: Any, pose_goal: Any, speed: float = 1.0,
        settle_s: float = 0.8, **kwargs: Any
    ) -> Tuple[bool, bool]:
        """Send movec to both arms."""
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movec(pose_via, pose_goal, speed=speed,
                                             settle_s=settle_s, **kwargs)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.movec(pose_via, pose_goal, speed=speed,
                                        settle_s=settle_s, **kwargs)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    def movep(
        self, poses_goal: List[Any], speed: float = 1.0,
        settle_s: float = 0.8, **kwargs: Any
    ) -> Tuple[bool, bool]:
        """Send movep to both arms."""
        self._pause_mirroring()

        sim_result = [True]

        def _sim_move():
            sim_result[0] = self._sim.movep(poses_goal, speed=speed,
                                             settle_s=settle_s, **kwargs)

        sim_thread = threading.Thread(target=_sim_move, daemon=True)
        sim_thread.start()

        real_result = self._real.movep(poses_goal, speed=speed,
                                        settle_s=settle_s, **kwargs)
        sim_thread.join()

        self._resume_mirroring()
        return real_result, sim_result[0]

    # ── State ──────────────────────────────────────────────────────────────────

    def get_real_state(self) -> Optional[dict]:
        """Get real arm state."""
        return self._real.get_state()

    def get_sim_state(self) -> Optional[dict]:
        """Get simulation state."""
        return self._sim.get_state()

    def get_tcp_pose(self) -> Tuple[List[float], List[List[float]]]:
        """Get real arm TCP pose."""
        return self._real.get_tcp_pose()

    def get_sim_tcp_pose(self) -> Tuple[List[float], List[List[float]]]:
        """Get simulation TCP pose."""
        return self._sim.get_tcp_pose()

    # ── Emergency Stop ─────────────────────────────────────────────────────────

    def request_stop(self) -> None:
        """Emergency stop both arms."""
        self._real.request_stop()
        self._sim.request_stop()

    def clear_stop(self) -> None:
        """Clear stop on both arms."""
        self._real.clear_stop()
        self._sim.clear_stop()

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    @property
    def real(self) -> Any:
        return self._real

    @property
    def sim(self) -> PyBulletArm:
        return self._sim

    def close(self) -> None:
        """Close both arms."""
        self._mirroring = False
        if self._mirror_thread and self._mirror_thread.is_alive():
            self._mirror_thread.join(timeout=2.0)
        try:
            self._sim.close()
        except Exception:
            pass
        try:
            self._real.close()
        except Exception:
            pass

    def __enter__(self) -> "DualArm":
        self.start()
        return self

    def __exit__(self, *args: Any) -> None:
        self.close()

    def __repr__(self) -> str:
        return f"DualArm(real={self._real!r}, sim={self._sim!r})"


class MirrorMode:
    """Explicit mirror mode: simulation tracks real arm.

    Usage::

        import litearm
        from litearm_pybullet import PyBulletArm, MirrorMode

        real = litearm.Arm(endpoint="tcp/192.168.31.139:7447")
        sim = PyBulletArm(render=True)
        sim.start()

        mirror = MirrorMode(real, sim)
        mirror.start()  # sim follows real arm

        mirror.stop()
        sim.close()
        real.close()
    """

    def __init__(
        self, real_arm: Any, sim_arm: PyBulletArm, rate_hz: float = 50.0
    ) -> None:
        self._real = real_arm
        self._sim = sim_arm
        self._rate_hz = rate_hz
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        """Start mirroring."""
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="mirror_mode"
        )
        self._thread.start()

    def stop(self) -> None:
        """Stop mirroring."""
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._sim.stop_mirroring()

    def _loop(self) -> None:
        """Main mirror loop."""
        dt = 1.0 / self._rate_hz
        while self._running:
            try:
                state = self._real.get_state()
                if state and state.get("q"):
                    self._sim.set_joint_positions(state["q"])
                    self._sim._controller.set_target(np.asarray(state["q"]))
            except Exception:
                pass
            time.sleep(dt)