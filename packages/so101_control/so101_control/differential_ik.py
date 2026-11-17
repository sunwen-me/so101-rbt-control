"""Local bounded differential IK prototype.

This module is deliberately separate from the production Placo path.  It uses
the same calibrated URDF FK, fixed local box, and position-first selection, but
computes a damped numerical Jacobian and performs a small line search.
"""
from __future__ import annotations

import time
import numpy as np


from .math3d import DifferentialIKError, _project_rotation, _rotvec


class DifferentialIK:
    """Bounded position-first differential IK around the supplied seed."""

    def __init__(self, kinematics, *, jacobian_eps_deg: float = 0.02,
                 damping: float = 1e-4, max_iterations: int = 16,
                 position_scale: float = 1000.0, orientation_weight: float = 0.01):
        self.kinematics = kinematics
        self.jacobian_eps_deg = float(jacobian_eps_deg)
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.position_scale = float(position_scale)
        self.orientation_weight = float(orientation_weight)
        if (self.jacobian_eps_deg <= 0 or self.damping <= 0 or
                not np.isfinite(self.position_scale) or self.position_scale <= 0 or
                not np.isfinite(self.orientation_weight) or self.orientation_weight < 0 or
                not 1 <= self.max_iterations <= 64):
            raise ValueError("invalid differential IK settings")
        self.lower_deg = np.asarray(kinematics.lower_deg, dtype=float)
        self.upper_deg = np.asarray(kinematics.upper_deg, dtype=float)
        self.last_ik = None

    @staticmethod
    def _metrics(pose, target):
        position = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
        rel = _project_rotation(pose[:3, :3]).T @ _project_rotation(target[:3, :3])
        orientation = float(np.linalg.norm(_rotvec(rel)))
        return position, orientation

    def _error(self, q, target):
        pose = np.asarray(self.kinematics.forward_kinematics(q), dtype=float)
        if pose.shape != (4, 4) or not np.isfinite(pose).all():
            raise DifferentialIKError("invalid FK", {"status": "invalid_fk"})
        # Scale metres to millimetres before the solve.  Without this, the
        # 1e-4 damping term dominates a Jacobian whose translational entries
        # are only ~1e-3 m/degree, producing almost-zero joint updates.
        e = np.concatenate((self.position_scale * (target[:3, 3] - pose[:3, 3]),
                            self.orientation_weight * _rotvec(pose[:3, :3].T @ target[:3, :3])))
        return pose, e, self._metrics(pose, target)

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose, *,
                           max_joint_step_deg: float = 20.0) -> np.ndarray:
        started = time.perf_counter()
        current = np.asarray(current_joint_pos, dtype=float)
        target = np.asarray(desired_ee_pose, dtype=float)
        n = len(self.kinematics.joint_names)
        if current.ndim != 1 or current.size < n or not np.isfinite(current).all():
            raise DifferentialIKError("invalid joint input", {"status": "invalid_input"})
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise DifferentialIKError("invalid target", {"status": "invalid_input"})
        if not np.isfinite(max_joint_step_deg) or max_joint_step_deg <= 0:
            raise DifferentialIKError("invalid local step", {"status": "invalid_input"})
        seed = current[:n].copy()
        lo = np.maximum(self.lower_deg, seed - max_joint_step_deg)
        hi = np.minimum(self.upper_deg, seed + max_joint_step_deg)
        if np.any(lo > hi):
            raise DifferentialIKError("empty local bounds", {"status": "empty_local_bounds"})
        _, seed_error_vec, seed_error = self._error(seed, target)
        q = seed.copy()
        _, best_error_vec, best_error = self._error(q, target)
        seed_feasible = bool(np.all(seed >= lo) and np.all(seed <= hi))
        best = q.copy() if seed_feasible else None
        if not seed_feasible:
            best_error = (float("inf"), float("inf"))
        accepted = 0
        rejected = 0
        candidate_errors = []
        for iteration in range(1, self.max_iterations + 1):
            if best_error[0] <= 1e-4:
                break
            _, e, _ = self._error(q, target)
            jac = np.empty((6, n), dtype=float)
            for j in range(n):
                qp = q.copy()
                qp[j] = min(hi[j], qp[j] + self.jacobian_eps_deg)
                if qp[j] == q[j]:
                    qm = q.copy()
                    qm[j] = max(lo[j], qm[j] - self.jacobian_eps_deg)
                    _, em, _ = self._error(qm, target)
                    jac[:, j] = (em - e) / (qm[j] - q[j]) if qm[j] != q[j] else 0.0
                else:
                    _, ep, _ = self._error(qp, target)
                    jac[:, j] = (ep - e) / (qp[j] - q[j])
            lhs = jac.T @ jac + self.damping * np.eye(n)
            # ``jac`` is the derivative of the residual e(q), so the Newton
            # step solves J*dq ~= -e (not +e).
            rhs = -jac.T @ e
            try:
                dq = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                dq = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            if not np.isfinite(dq).all() or np.max(np.abs(dq)) < 1e-8:
                break
            accepted_this_round = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = np.clip(q + alpha * dq, lo, hi)
                if np.max(np.abs(candidate - q)) < 1e-8:
                    continue
                _, candidate_vec, candidate_error = self._error(candidate, target)
                candidate_errors.append(float(candidate_error[0] * 1000.0))
                if (best is None or candidate_error[0] < best_error[0] - 1e-10 or
                        abs(candidate_error[0] - best_error[0]) <= 1e-10 and candidate_error[1] < best_error[1]):
                    q = candidate
                    best = candidate.copy()
                    best_error_vec = candidate_vec
                    best_error = candidate_error
                    accepted += 1
                    accepted_this_round = True
                    break
                rejected += 1
            if not accepted_this_round:
                break
        if best is None:
            best = np.clip(seed, lo, hi)
            _, best_error_vec, best_error = self._error(best, target)
        status = ("converged" if best_error[0] <= 1e-4 else
                  "partial_progress" if best_error[0] < seed_error[0] else
                  "no_position_progress")
        self.last_ik = {
            "status": status, "iterations": iteration if 'iteration' in locals() else 0,
            "selected_iteration": accepted, "seed_position_error_mm": seed_error[0] * 1000.0,
            "position_error_mm": best_error[0] * 1000.0,
            "orientation_error_deg": np.rad2deg(best_error[1]),
            "max_joint_delta_deg": float(np.max(np.abs(best - seed))),
            "accepted_steps": accepted, "rejected_worse_candidates": rejected,
            "candidate_position_error_mm": candidate_errors,
            "elapsed_ms": (time.perf_counter() - started) * 1000.0,
            "seed_within_urdf": seed_feasible,
        }
        result = current.copy()
        result[:n] = best
        return result


class PlacoDifferentialIK(DifferentialIK):
    """Differential IK using Placo's analytic frame Jacobian.

    The Jacobian is expressed in ``local_world_aligned`` coordinates and
    returned per radian.  The translational residual is scaled to millimetres;
    this keeps the damping term numerically meaningful on SO101 link lengths.
    """

    def __init__(self, kinematics, *, damping: float = 1e-3,
                 max_iterations: int = 16, orientation_weight: float = 0.01):
        super().__init__(kinematics, jacobian_eps_deg=0.02, damping=damping,
                         max_iterations=max_iterations, position_scale=1000.0,
                         orientation_weight=orientation_weight)

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose, *,
                           max_joint_step_deg: float = 20.0) -> np.ndarray:
        started = time.perf_counter()
        current = np.asarray(current_joint_pos, dtype=float)
        target = np.asarray(desired_ee_pose, dtype=float)
        names = list(self.kinematics.joint_names)
        n = len(names)
        if current.ndim != 1 or current.size < n or not np.isfinite(current).all():
            raise DifferentialIKError("invalid joint input", {"status": "invalid_input"})
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise DifferentialIKError("invalid target", {"status": "invalid_input"})
        seed = current[:n].copy()
        lo = np.maximum(self.lower_deg, seed - max_joint_step_deg)
        hi = np.minimum(self.upper_deg, seed + max_joint_step_deg)
        if np.any(lo > hi):
            raise DifferentialIKError("empty local bounds", {"status": "empty_local_bounds"})
        seed_pose = np.asarray(self.kinematics.forward_kinematics(seed), dtype=float)
        seed_error = self._metrics(seed_pose, target)
        seed_feasible = bool(np.all(seed >= lo) and np.all(seed <= hi))
        best = seed.copy() if seed_feasible else None
        best_error = seed_error if seed_feasible else (float('inf'), float('inf'))
        q = seed.copy()
        accepted = rejected = 0
        candidate_errors = []
        columns = [self.kinematics.robot.get_joint_v_offset(name) for name in names]
        for iteration in range(1, self.max_iterations + 1):
            pose = np.asarray(self.kinematics.forward_kinematics(q), dtype=float)
            pos_error = 1000.0 * (target[:3, 3] - pose[:3, 3])
            rot_error = self.orientation_weight * _rotvec(target[:3, :3] @ pose[:3, :3].T)
            error = np.concatenate((pos_error, rot_error))
            if np.linalg.norm(pos_error) <= 0.1:
                break
            full_jac = np.asarray(self.kinematics.robot.frame_jacobian(
                self.kinematics.target_frame_name, 'local_world_aligned'), dtype=float)
            jac = full_jac[:, columns].copy()
            jac[:3] *= 1000.0
            jac[3:] *= self.orientation_weight
            lhs = jac.T @ jac + self.damping * np.eye(n)
            rhs = jac.T @ error
            try:
                dq_rad = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                dq_rad = np.linalg.lstsq(lhs, rhs, rcond=None)[0]
            if not np.isfinite(dq_rad).all() or np.max(np.abs(dq_rad)) < 1e-10:
                break
            improved = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = np.clip(q + np.rad2deg(alpha * dq_rad), lo, hi)
                if np.max(np.abs(candidate - q)) < 1e-8:
                    continue
                candidate_pose = np.asarray(self.kinematics.forward_kinematics(candidate), dtype=float)
                candidate_error = self._metrics(candidate_pose, target)
                candidate_errors.append(float(candidate_error[0] * 1000.0))
                if (best is None or candidate_error[0] < best_error[0] - 1e-10 or
                        abs(candidate_error[0] - best_error[0]) <= 1e-10 and candidate_error[1] < best_error[1]):
                    q = candidate
                    best = candidate.copy()
                    best_error = candidate_error
                    accepted += 1
                    improved = True
                    break
                rejected += 1
            if not improved:
                break
        if best is None:
            best = np.clip(seed, lo, hi)
            best_pose = np.asarray(self.kinematics.forward_kinematics(best), dtype=float)
            best_error = self._metrics(best_pose, target)
        status = ('converged' if best_error[0] <= 1e-4 else
                  'partial_progress' if best_error[0] < seed_error[0] else
                  'no_position_progress')
        self.last_ik = {
            'status': status, 'iterations': iteration if 'iteration' in locals() else 0,
            'selected_iteration': accepted, 'seed_position_error_mm': seed_error[0] * 1000.0,
            'position_error_mm': best_error[0] * 1000.0,
            'orientation_error_deg': float(np.rad2deg(best_error[1])),
            'max_joint_delta_deg': float(np.max(np.abs(best - seed))),
            'accepted_steps': accepted, 'rejected_worse_candidates': rejected,
            'candidate_position_error_mm': candidate_errors,
            'seed_within_urdf': seed_feasible,
            'elapsed_ms': (time.perf_counter() - started) * 1000.0,
        }
        result = current.copy()
        result[:n] = best
        return result
