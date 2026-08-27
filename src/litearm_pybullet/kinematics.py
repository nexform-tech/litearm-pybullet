"""Forward and inverse kinematics using PyBullet.

Provides FK/IK/planning functions that match the litearm.Arm API:
- fk(q) → (position, rotation_matrix)
- ik(pos, R, q_seed) → (q, success)
- plan_movel / plan_movec / plan_movep
"""
from __future__ import annotations

import math
from typing import List, Optional, Tuple

import numpy as np
import pybullet as p

N_JOINTS = 7

# Joint limits (aligned with real LiteArm)
Q_MIN = np.array([-2.82, -1.78, -2.83, -3.1325, -2.853, -1.568, -1.59])
Q_MAX = np.array([2.839, 1.817, 2.87, 0.647, 2.8587, 1.619, 1.623])


def _quat_to_rotmat(quat) -> np.ndarray:
    """Convert [x, y, z, w] PyBullet quaternion to 3x3 rotation matrix."""
    x, y, z, w = quat
    return np.array([
        [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
        [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
        [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y],
    ])


def _rotmat_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to [x, y, z, w] PyBullet quaternion."""
    m00, m01, m02 = R[0, 0], R[0, 1], R[0, 2]
    m10, m11, m12 = R[1, 0], R[1, 1], R[1, 2]
    m20, m21, m22 = R[2, 0], R[2, 1], R[2, 2]

    trace = m00 + m11 + m22
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return np.array([x, y, z, w])


def _rotation_error(R_current: np.ndarray, R_desired: np.ndarray) -> np.ndarray:
    """Compute rotation error as angular velocity (3-vector).

    Uses the log map of SO(3): the angular velocity that would rotate
    R_current to R_desired in unit time.
    """
    R_err = R_desired @ R_current.T
    trace = np.clip(R_err[0, 0] + R_err[1, 1] + R_err[2, 2], -1.0, 3.0)
    theta = math.acos((trace - 1.0) / 2.0)

    if abs(theta) < 1e-10:
        return np.zeros(3)

    w_hat = (R_err - R_err.T) / (2.0 * math.sin(theta))
    omega = np.array([w_hat[2, 1], w_hat[0, 2], w_hat[1, 0]])
    return omega * theta


def _pose_to_list(
    pos: np.ndarray, rot: np.ndarray
) -> Tuple[List[float], List[List[float]]]:
    """Convert numpy arrays to list format matching litearm.Arm."""
    return pos.tolist(), rot.tolist()


class Kinematics:
    """Forward and inverse kinematics using PyBullet.

    Usage::

        kin = Kinematics(body_id, ee_link_index, joint_ids, client_id)
        pos, R = kin.fk(q)
        q_sol, ok = kin.ik(pos_d, R_d, q_seed)
    """

    def __init__(
        self,
        body_id: int,
        ee_link_index: int,
        joint_ids: List[int],
        client_id: int,
        n_joints: int = N_JOINTS,
    ) -> None:
        self._body = body_id
        self._ee = ee_link_index
        self._joints = list(joint_ids)
        self._cid = client_id
        self._n_joints = n_joints

    def fk(self, q: List[float]) -> Tuple[List[float], List[List[float]]]:
        """Forward kinematics: joint angles → (position, rotation_matrix)."""
        for jid, qi in zip(self._joints, q):
            p.resetJointState(self._body, jid, targetValue=float(qi),
                              physicsClientId=self._cid)
        ls = p.getLinkState(self._body, self._ee, computeForwardKinematics=True,
                            physicsClientId=self._cid)
        pos = np.asarray(ls[0], dtype=float)
        R = _quat_to_rotmat(ls[1])
        return _pose_to_list(pos, R)

    def ik(
        self,
        pos_d: List[float],
        R_d: List[List[float]],
        q_seed: Optional[List[float]] = None,
        max_iter: int = 200,
        tol_pos: float = 1e-4,
        tol_rot: float = 1e-4,
        damping: float = 0.1,
    ) -> Tuple[List[float], bool]:
        """Inverse kinematics using damped least squares.

        Uses PyBullet's built-in calculateInverseKinematics as initial seed,
        then refines with Levenberg-Marquardt to close the residual.

        Args:
            pos_d: Desired position [x, y, z].
            R_d: Desired 3x3 rotation matrix.
            q_seed: Initial guess (default: zeros).
            max_iter: Maximum iterations.
            tol_pos: Position tolerance (m).
            tol_rot: Rotation tolerance (rad).
            damping: Damping factor for Levenberg-Marquardt.

        Returns:
            (q_solution, success).
        """
        pos_d = np.asarray(pos_d, dtype=float)
        R_d = np.asarray(R_d, dtype=float)

        # PyBullet IK as initial seed
        orn = _rotmat_to_quat(R_d)
        try:
            sol = p.calculateInverseKinematics(
                self._body, self._ee, list(pos_d), list(orn),
                self._joints,
                maxNumIterations=200, residualThreshold=1e-5,
                physicsClientId=self._cid,
            )
        except Exception:
            return ([0.0] * self._n_joints, False)

        q = np.asarray(sol[:self._n_joints], dtype=float)
        if not np.all(np.isfinite(q)):
            return ([0.0] * self._n_joints, False)

        # LM refinement
        q = self._refine_lm(q, pos_d, R_d, max_iter, tol_pos, tol_rot, damping)
        q = np.clip(q, Q_MIN, Q_MAX)

        for i in range(self._n_joints):
            if not math.isfinite(q[i]) or q[i] < Q_MIN[i] - 1e-6 or q[i] > Q_MAX[i] + 1e-6:
                return ([0.0] * self._n_joints, False)

        return (q.tolist(), True)

    def _refine_lm(
        self, q_seed: np.ndarray, pos_d: np.ndarray, R_d: np.ndarray,
        max_iter: int, tol_pos: float, tol_rot: float, damping: float,
    ) -> np.ndarray:
        """Levenberg-Marquardt refinement of IK solution."""
        q = q_seed.copy()
        lam = damping
        h = 1e-6

        def _err(qq):
            for jid, qi in zip(self._joints, qq):
                p.resetJointState(self._body, jid, targetValue=float(qi),
                                  physicsClientId=self._cid)
            ls = p.getLinkState(self._body, self._ee, computeForwardKinematics=True,
                                physicsClientId=self._cid)
            p_now = np.asarray(ls[0], dtype=float)
            R_now = _quat_to_rotmat(ls[1])
            pos_e = pos_d - p_now
            rot_e = _rotation_error(R_now, R_d)
            return np.concatenate([pos_e, rot_e])

        e = _err(q)

        for _ in range(max_iter):
            if np.linalg.norm(e[:3]) < tol_pos and np.linalg.norm(e[3:]) < tol_rot:
                break

            # Numerical Jacobian
            J = np.zeros((6, self._n_joints))
            for j in range(self._n_joints):
                qp = q.copy()
                qp[j] += h
                for jid, qi in zip(self._joints, qp):
                    p.resetJointState(self._body, jid, targetValue=float(qi),
                                      physicsClientId=self._cid)
                ls = p.getLinkState(self._body, self._ee, computeForwardKinematics=True,
                                    physicsClientId=self._cid)
                p_p = np.asarray(ls[0], dtype=float)
                R_p = _quat_to_rotmat(ls[1])
                J[:3, j] = (p_p - (pos_d - e[:3])) / h
                J[3:, j] = (_rotation_error(R_p, R_d) - e[3:]) / h

            for _ in range(12):
                try:
                    dq = np.linalg.solve(J.T @ J + lam * np.eye(self._n_joints),
                                         J.T @ e)
                except np.linalg.LinAlgError:
                    break
                q_new = np.clip(q + dq, Q_MIN, Q_MAX)
                e_new = _err(q_new)
                if np.linalg.norm(e_new) < np.linalg.norm(e):
                    q = q_new
                    e = e_new
                    lam = max(lam / 4, 1e-7)
                    break
                lam *= 10
            else:
                break

        return q

    def plan_movel(
        self,
        q_start: List[float],
        pose_goal: Tuple[List[float], List[List[float]]],
        num_waypoints: int = 50,
    ) -> List[List[float]]:
        """Plan a straight-line Cartesian path from q_start to pose_goal."""
        q_start = np.asarray(q_start, dtype=float)[:self._n_joints]
        pos_goal, R_goal = pose_goal
        pos_goal = np.asarray(pos_goal, dtype=float)
        R_goal = np.asarray(R_goal, dtype=float)

        pos_start, R_start = self.fk(q_start.tolist())
        pos_start = np.asarray(pos_start)
        R_start = np.asarray(R_start)

        path = [q_start.tolist()]
        q_current = q_start.copy()

        for i in range(1, num_waypoints + 1):
            t = i / num_waypoints
            pos_i = pos_start + t * (pos_goal - pos_start)
            R_i = self._slerp(R_start, R_goal, t)

            q_i, ok = self.ik(pos_i, R_i, q_seed=q_current.tolist())
            if not ok:
                return path

            path.append(q_i)
            q_current = np.asarray(q_i)

        return path

    def plan_movec(
        self,
        q_start: List[float],
        pose_via: Tuple[List[float], List[List[float]]],
        pose_goal: Tuple[List[float], List[List[float]]],
        num_waypoints: int = 50,
    ) -> List[List[float]]:
        """Plan a circular-arc Cartesian path through a via-point."""
        q_start = np.asarray(q_start, dtype=float)[:self._n_joints]
        pos_via, R_via = pose_via
        pos_goal, R_goal = pose_goal
        pos_via = np.asarray(pos_via, dtype=float)
        pos_goal = np.asarray(pos_goal, dtype=float)
        R_via = np.asarray(R_via, dtype=float)
        R_goal = np.asarray(R_goal, dtype=float)

        pos_start, R_start = self.fk(q_start.tolist())
        pos_start = np.asarray(pos_start)
        R_start = np.asarray(R_start)

        center = self._circle_center(pos_start, pos_via, pos_goal)
        r = np.linalg.norm(pos_start - center)

        v_start = pos_start - center
        v_goal = pos_goal - center

        theta_start = math.atan2(v_start[1], v_start[0])
        theta_goal = math.atan2(v_goal[1], v_goal[0])
        total_theta = theta_goal - theta_start

        path = [q_start.tolist()]
        q_current = q_start.copy()

        for i in range(1, num_waypoints + 1):
            t = i / num_waypoints
            theta = theta_start + t * total_theta
            pos_i = center + r * np.array([math.cos(theta), math.sin(theta), 0])
            pos_i[2] = pos_start[2] + t * (pos_goal[2] - pos_start[2])
            R_i = self._slerp(R_start, R_goal, t)

            q_i, _ = self.ik(pos_i, R_i, q_seed=q_current.tolist())
            path.append(q_i)
            q_current = np.asarray(q_i)

        return path

    def plan_movep(
        self,
        q_start: List[float],
        poses_goal: List[Tuple[List[float], List[List[float]]]],
        waypoints_per_segment: int = 30,
    ) -> List[List[float]]:
        """Plan a multi-waypoint Cartesian path."""
        q_current = np.asarray(q_start, dtype=float)[:self._n_joints]
        full_path = [q_current.tolist()]

        pos_current, R_current = self.fk(q_current.tolist())
        pos_current = np.asarray(pos_current)
        R_current = np.asarray(R_current)

        for pose_goal in poses_goal:
            pos_goal, R_goal = pose_goal
            pos_goal = np.asarray(pos_goal, dtype=float)
            R_goal = np.asarray(R_goal, dtype=float)

            for i in range(1, waypoints_per_segment + 1):
                t = i / waypoints_per_segment
                pos_i = pos_current + t * (pos_goal - pos_current)
                R_i = self._slerp(R_current, R_goal, t)

                q_i, ok = self.ik(pos_i, R_i, q_seed=q_current.tolist())
                if not ok:
                    return full_path
                full_path.append(q_i)
                q_current = np.asarray(q_i)

            pos_current = pos_goal
            R_current = R_goal

        return full_path

    @staticmethod
    def _circle_center(
        p1: np.ndarray, p2: np.ndarray, p3: np.ndarray
    ) -> np.ndarray:
        """Compute the center of a circle passing through three 3D points."""
        v1 = p2 - p1
        v2 = p3 - p1
        n = np.cross(v1, v2)
        if np.linalg.norm(n) < 1e-10:
            return (p1 + p3) / 2.0

        n12 = np.cross(v1, n)
        n13 = np.cross(v2, n)
        mid12 = (p1 + p2) / 2.0
        mid13 = (p1 + p3) / 2.0

        A = np.column_stack([n12, -n13])
        b = mid13 - mid12
        try:
            ts, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
            center = mid12 + ts[0] * n12
        except np.linalg.LinAlgError:
            center = (p1 + p2 + p3) / 3.0

        return center

    @staticmethod
    def _slerp(R1: np.ndarray, R2: np.ndarray, t: float) -> np.ndarray:
        """Spherical linear interpolation between two rotation matrices."""
        q1 = _rotmat_to_quat(R1)
        q2 = _rotmat_to_quat(R2)

        dot = np.dot(q1, q2)
        if dot < 0:
            q2 = -q2
            dot = -dot

        if dot > 0.9995:
            q = q1 + t * (q2 - q1)
            q = q / np.linalg.norm(q)
        else:
            theta = math.acos(np.clip(float(dot), -1.0, 1.0))
            sin_theta = math.sin(theta)
            s1 = math.sin((1 - t) * theta) / sin_theta
            s2 = math.sin(t * theta) / sin_theta
            q = s1 * q1 + s2 * q2

        return _quat_to_rotmat(q)