"""Joint-level PID controller and trajectory generator for PyBullet simulation.

Provides position control and trajectory planning for the 7-DOF LiteArm.
Used by PyBulletArm to track desired joint trajectories in the background
physics loop.
"""
from __future__ import annotations

from typing import Optional

import numpy as np

N_JOINTS = 7

# ── Default PD gains ────────────────────────────────────────────────────────────
# Aligned to the real LiteArm MIT-mode follow gains (from litearm.yaml
# `control.follow`: kp/kd).  Wrist joints (5-7) use lower stiffness, matching
# the smaller DM4310 motors.  Nm/rad and Nm·s/rad.
DEFAULT_KP = (260.0, 260.0, 150.0, 150.0, 50.0, 50.0, 50.0)
DEFAULT_KD = (5.0, 5.0, 4.0, 5.0, 2.5, 2.5, 2.5)


class JointPIDController:
    """Joint-space PID position controller with feed-forward.

    Usage::

        ctrl = JointPIDController(kp=[500]*7, kd=[20]*7)
        tau = ctrl.compute(q_actual, dq_actual, dt)
        data.ctrl[:] = tau
    """

    def __init__(
        self,
        kp: np.ndarray | list | float = 500.0,
        kd: np.ndarray | list | float = 20.0,
        ki: np.ndarray | list | float = 0.0,
        max_torque: float = 200.0,
        n_joints: int = N_JOINTS,
    ) -> None:
        self.kp = np.broadcast_to(
            np.atleast_1d(np.asarray(kp, dtype=float)), n_joints
        ).copy()
        self.kd = np.broadcast_to(
            np.atleast_1d(np.asarray(kd, dtype=float)), n_joints
        ).copy()
        self.ki = np.broadcast_to(
            np.atleast_1d(np.asarray(ki, dtype=float)), n_joints
        ).copy()
        self.max_torque = max_torque
        self.n_joints = n_joints

        self._integral = np.zeros(n_joints)
        self._q_des = np.zeros(n_joints)
        self._dq_des = np.zeros(n_joints)

    def set_target(
        self, q_des: np.ndarray, dq_des: Optional[np.ndarray] = None
    ) -> None:
        """Set desired joint positions (and optionally velocities)."""
        self._q_des = np.asarray(q_des, dtype=float)[:self.n_joints]
        if dq_des is not None:
            self._dq_des = np.asarray(dq_des, dtype=float)[:self.n_joints]
        else:
            self._dq_des = np.zeros(self.n_joints)

    def reset(self) -> None:
        """Reset integral term and targets."""
        self._integral = np.zeros(self.n_joints)
        self._q_des = np.zeros(self.n_joints)
        self._dq_des = np.zeros(self.n_joints)

    def compute(
        self,
        q_actual: np.ndarray,
        dq_actual: np.ndarray,
        dt: float,
        ff_torque: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """Compute control torque.

        Args:
            q_actual: Current joint positions (rad).
            dq_actual: Current joint velocities (rad/s).
            dt: Timestep (s).
            ff_torque: Optional feed-forward torque (e.g. gravity compensation).

        Returns:
            tau: Control torque for each joint.
        """
        q_actual = np.asarray(q_actual, dtype=float)[:self.n_joints]
        dq_actual = np.asarray(dq_actual, dtype=float)[:self.n_joints]

        pos_err = self._q_des - q_actual
        vel_err = self._dq_des - dq_actual

        # Integral with anti-windup
        self._integral += pos_err * dt
        self._integral = np.clip(
            self._integral, -self.max_torque * 0.3, self.max_torque * 0.3
        )

        tau = self.kp * pos_err + self.kd * vel_err + self.ki * self._integral

        if ff_torque is not None:
            tau += np.asarray(ff_torque, dtype=float)[:self.n_joints]

        return np.clip(tau, -self.max_torque, self.max_torque)

    @property
    def q_des(self) -> np.ndarray:
        return self._q_des.copy()

    @property
    def dq_des(self) -> np.ndarray:
        return self._dq_des.copy()


class TrajectoryGenerator:
    """Generate joint-space trajectories with velocity/acceleration limits.

    Supports:
    - Linear interpolation with trapezoidal velocity profile
    - Minimum jerk interpolation
    """

    def __init__(
        self,
        max_velocity: float = 3.0,  # rad/s
        max_acceleration: float = 10.0,  # rad/s²
        dt: float = 0.002,
    ) -> None:
        self.max_vel = max_velocity
        self.max_acc = max_acceleration
        self.dt = dt

    def linear_trajectory(
        self,
        q_start: np.ndarray,
        q_end: np.ndarray,
        speed: float = 1.0,
    ) -> list[np.ndarray]:
        """Generate a trapezoidal velocity profile trajectory.

        Args:
            q_start: Start joint positions (rad).
            q_end: Target joint positions (rad).
            speed: Speed multiplier (0.0 - 1.0+).

        Returns:
            List of joint position waypoints, one per timestep.
        """
        q_start = np.asarray(q_start, dtype=float)
        q_end = np.asarray(q_end, dtype=float)

        delta = q_end - q_start
        max_joint_delta = np.max(np.abs(delta))

        if max_joint_delta < 1e-6:
            return [q_end.copy()]

        v_max = self.max_vel * speed
        a_max = self.max_acc * speed

        # Trapezoidal profile: accel, cruise, decel
        t_acc = v_max / a_max
        d_acc = 0.5 * a_max * t_acc**2

        total_dist = max_joint_delta

        if 2 * d_acc > total_dist:
            # Triangular profile (no cruise)
            t_acc = np.sqrt(total_dist / a_max)
            t_total = 2 * t_acc
            has_cruise = False
        else:
            d_cruise = total_dist - 2 * d_acc
            t_cruise = d_cruise / v_max
            t_total = 2 * t_acc + t_cruise
            has_cruise = True

        waypoints = []
        t = 0.0
        direction = (
            delta / max_joint_delta
            if max_joint_delta > 1e-10
            else np.ones_like(delta)
        )

        while t <= t_total:
            if t < t_acc:
                s = 0.5 * a_max * t**2
            elif has_cruise and t < t_acc + t_cruise:
                s = d_acc + v_max * (t - t_acc)
            else:
                t_dec = t_total - t
                s = total_dist - 0.5 * a_max * t_dec**2

            q = q_start + direction * s
            waypoints.append(q)
            t += self.dt

        if len(waypoints) > 0:
            waypoints[-1] = q_end.copy()
        else:
            waypoints.append(q_end.copy())

        return waypoints

    def minimum_jerk_trajectory(
        self,
        q_start: np.ndarray,
        q_end: np.ndarray,
        duration: float,
    ) -> list[np.ndarray]:
        """Generate a minimum jerk trajectory.

        q(t) = q_start + (q_end - q_start) * (10(t/T)³ - 15(t/T)⁴ + 6(t/T)⁵)
        """
        q_start = np.asarray(q_start, dtype=float)
        q_end = np.asarray(q_end, dtype=float)
        delta = q_end - q_start

        n_steps = max(2, int(duration / self.dt))
        waypoints = []

        for i in range(n_steps + 1):
            t_norm = i / n_steps
            s = 10 * t_norm**3 - 15 * t_norm**4 + 6 * t_norm**5
            q = q_start + delta * s
            waypoints.append(q)

        return waypoints