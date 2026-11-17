"""Pinocchio FK/Jacobian and local LM/box-QP inverse kinematics."""
from __future__ import annotations

import time
import numpy as np

from .math3d import DifferentialIKError, _project_rotation, _rotvec


class PinocchioBoxIK:
    """Weighted soft-pose IK; each arm owns independent model/data state."""

    POSITION_TOLERANCE_M = 1e-4
    ORIENTATION_TOLERANCE_RAD = np.deg2rad(0.1)

    def __init__(self, urdf_path: str, *, target_frame_name: str = "gripper",
                 joint_names: list[str] | None = None, damping: float = 1e-3,
                 max_iterations: int = 16, qp_iterations: int = 40,
                 orientation_weight: float = 0.01):
        import pinocchio as pin
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(urdf_path))
        self.data = self.model.createData()
        self.target_frame_name = target_frame_name
        self.frame_id = self.model.getFrameId(target_frame_name)
        self.joint_names = list(joint_names or [n for n in self.model.names if n != "universe"])
        self.joint_ids = [self.model.getJointId(n) for n in self.joint_names]
        self.q_indices = [self.model.idx_qs[j] for j in self.joint_ids]
        self.v_indices = [self.model.idx_vs[j] for j in self.joint_ids]
        self.lower_deg = np.rad2deg(np.asarray([self.model.lowerPositionLimit[i] for i in self.q_indices], float))
        self.upper_deg = np.rad2deg(np.asarray([self.model.upperPositionLimit[i] for i in self.q_indices], float))
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.qp_iterations = int(qp_iterations)
        self.orientation_weight = float(orientation_weight)
        self.last_ik = None
        if not np.isfinite(self.lower_deg).all() or not np.isfinite(self.upper_deg).all():
            raise ValueError("Pinocchio model has nonfinite joint bounds")

    def _q(self, joint_deg):
        q = pin_q = self.pin.neutral(self.model)
        for idx, value in zip(self.q_indices, np.deg2rad(np.asarray(joint_deg, float)[:len(self.q_indices)])):
            q[idx] = value
        return q

    def _fk_jac(self, joint_deg, need_jac=False):
        q = self._q(joint_deg)
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        pose = np.asarray(self.data.oMf[self.frame_id].homogeneous, dtype=float).copy()
        jac = None
        if need_jac:
            self.pin.computeJointJacobians(self.model, self.data, q)
            self.pin.updateFramePlacements(self.model, self.data)
            full = np.asarray(self.pin.getFrameJacobian(
                self.model, self.data, self.frame_id, self.pin.LOCAL_WORLD_ALIGNED), dtype=float)
            jac = full[:, self.v_indices].copy()
        return pose, jac

    @staticmethod
    def _metrics(pose, target):
        pos = float(np.linalg.norm(pose[:3, 3] - target[:3, 3]))
        rot = _rotvec(_project_rotation(target[:3, :3]) @ _project_rotation(pose[:3, :3]).T)
        return pos, float(np.linalg.norm(rot))

    @staticmethod
    def _objective(error, position_weight, orientation_weight):
        """Weighted SE(3) least-squares objective in metres and radians."""
        return float((position_weight * error[0]) ** 2 +
                     (orientation_weight * error[1]) ** 2)

    def _box_qp(self, hessian, gradient, lower, upper):
        """Projected-gradient solve of min .5*x'Hx - g'x over a box."""
        eig = float(np.linalg.eigvalsh(hessian).max())
        step = 1.0 / max(eig, 1e-9)
        x = np.clip(np.linalg.solve(hessian, gradient), lower, upper)
        for _ in range(self.qp_iterations):
            nxt = np.clip(x - step * (hessian @ x - gradient), lower, upper)
            if np.max(np.abs(nxt - x)) < 1e-10:
                break
            x = nxt
        return x

    def forward_kinematics(self, joint_pos_deg):
        return self._fk_jac(joint_pos_deg, False)[0]

    def inverse_kinematics(self, current_joint_pos, desired_ee_pose,
                           position_weight: float = 1.0,
                           orientation_weight: float | None = None, *,
                           max_joint_step_deg: float = 20.0):
        started = time.perf_counter()
        current = np.asarray(current_joint_pos, dtype=float)
        target = np.asarray(desired_ee_pose, dtype=float)
        orientation_weight = (self.orientation_weight if orientation_weight is None
                              else float(orientation_weight))
        n = len(self.joint_names)
        if (current.ndim != 1 or current.size < n or not np.isfinite(current).all() or
                not np.isfinite([position_weight, orientation_weight]).all() or
                position_weight <= 0 or orientation_weight < 0):
            raise DifferentialIKError("invalid joint input", {"status": "invalid_input"})
        if target.shape != (4, 4) or not np.isfinite(target).all():
            raise DifferentialIKError("invalid target", {"status": "invalid_input"})
        seed = current[:n].copy()
        lo = np.maximum(self.lower_deg, seed - max_joint_step_deg)
        hi = np.minimum(self.upper_deg, seed + max_joint_step_deg)
        if np.any(lo > hi):
            raise DifferentialIKError("empty local bounds", {"status": "empty_local_bounds"})
        seed_pose, _ = self._fk_jac(seed)
        seed_error = self._metrics(seed_pose, target)
        seed_objective = self._objective(seed_error, position_weight, orientation_weight)
        feasible = bool(np.all(seed >= lo) and np.all(seed <= hi))
        best = seed.copy() if feasible else None
        best_error = seed_error if feasible else (float("inf"), float("inf"))
        best_objective = seed_objective if feasible else float("inf")
        q = seed.copy()
        accepted = rejected = 0
        candidate_errors = []
        for iteration in range(1, self.max_iterations + 1):
            pose, full_jac = self._fk_jac(q, True)
            position_error = target[:3, 3] - pose[:3, 3]
            rotation_error = _rotvec(
                _project_rotation(target[:3, :3]) @ _project_rotation(pose[:3, :3]).T)
            err = np.concatenate((position_weight * position_error,
                                  orientation_weight * rotation_error))
            current_error = (float(np.linalg.norm(position_error)),
                             float(np.linalg.norm(rotation_error)))
            if (current_error[0] <= self.POSITION_TOLERANCE_M and
                    (orientation_weight == 0 or
                     current_error[1] <= self.ORIENTATION_TOLERANCE_RAD)):
                break
            jac = full_jac.copy()
            jac[:3] *= position_weight
            jac[3:] *= orientation_weight
            h = jac.T @ jac + self.damping * np.eye(n)
            g = jac.T @ err
            dq_rad = self._box_qp(h, g, np.deg2rad(lo - q), np.deg2rad(hi - q))
            if not np.isfinite(dq_rad).all() or np.max(np.abs(dq_rad)) < 1e-10:
                break
            improved = False
            for alpha in (1.0, 0.5, 0.25, 0.125, 0.0625):
                candidate = np.clip(q + np.rad2deg(alpha * dq_rad), lo, hi)
                if np.max(np.abs(candidate - q)) < 1e-8:
                    continue
                cand_pose, _ = self._fk_jac(candidate)
                cand_error = self._metrics(cand_pose, target)
                cand_objective = self._objective(cand_error, position_weight, orientation_weight)
                candidate_errors.append(float(cand_error[0] * 1000.0))
                if best is None or cand_objective < best_objective - 1e-16:
                    q = candidate
                    best = candidate.copy()
                    best_error = cand_error
                    best_objective = cand_objective
                    accepted += 1
                    improved = True
                    break
                rejected += 1
            if not improved:
                break
        if best is None:
            best = np.clip(seed, lo, hi)
            best_error = self._metrics(self.forward_kinematics(best), target)
            best_objective = self._objective(best_error, position_weight, orientation_weight)
        converged = (best_error[0] <= self.POSITION_TOLERANCE_M and
                     (orientation_weight == 0 or best_error[1] <= self.ORIENTATION_TOLERANCE_RAD))
        status = ("converged" if converged else
                  "partial_progress" if best_objective < seed_objective - 1e-16 else
                  "no_position_progress")
        self.last_ik = {
            "status": status, "iterations": iteration if 'iteration' in locals() else 0,
            "selected_iteration": accepted, "seed_position_error_mm": seed_error[0] * 1000.0,
            "position_error_mm": best_error[0] * 1000.0,
            "orientation_error_deg": float(np.rad2deg(best_error[1])),
            "seed_weighted_objective": seed_objective,
            "weighted_objective": best_objective,
            "max_joint_delta_deg": float(np.max(np.abs(best - seed))),
            "accepted_steps": accepted, "rejected_worse_candidates": rejected,
            "candidate_position_error_mm": candidate_errors,
            "seed_within_urdf": feasible,
            "elapsed_ms": (time.perf_counter() - started) * 1000.0,
        }
        result = current.copy()
        result[:n] = best
        return result
