# Adapted from Hugging Face LeRobot, Apache-2.0; see NOTICE.md.
from __future__ import annotations
import numpy as np
import time
import xml.etree.ElementTree as ET


class BoundedIKError(ValueError):
    """No command is available; report contains the failed solve diagnostics."""
    def __init__(self, message, report):
        super().__init__(message)
        self.report = report

class RobotKinematics:
    """Robot kinematics using placo library for forward and inverse kinematics."""

    def __init__(
        self,
        urdf_path: str,
        target_frame_name: str = "gripper_frame_link",
        joint_names: list[str] | None = None,
    ):
        """
        Initialize placo-based kinematics solver.

        Args:
            urdf_path (str): Path to the robot URDF file
            target_frame_name (str): Name of the end-effector frame in the URDF
            joint_names (list[str] | None): List of joint names to use for the kinematics solver
        """
        import placo

        self.robot = placo.RobotWrapper(urdf_path)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.mask_fbase(True)  # Fix the base

        self.target_frame_name = target_frame_name

        # Set joint names
        self.joint_names = list(self.robot.joint_names()) if joint_names is None else joint_names

        # Immutable bounds: local tick boxes must NEVER become the next tick's
        # global limits. These angles are in the calibrated policy/URDF frame.
        joints = {j.attrib["name"]: j for j in ET.parse(urdf_path).getroot().findall("joint")}
        limits = []
        for name in self.joint_names:
            limit = joints[name].find("limit")
            if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
                raise ValueError(f"{name}: bounded IK requires finite URDF joint limits")
            limits.append((float(limit.attrib["lower"]), float(limit.attrib["upper"])))
        bounds = np.rad2deg(np.asarray(limits, dtype=np.float64))
        if not np.isfinite(bounds).all() or np.any(bounds[:, 0] > bounds[:, 1]):
            raise ValueError("invalid URDF joint limits")
        self.lower_deg, self.upper_deg = bounds[:, 0].copy(), bounds[:, 1].copy()
        self.lower_deg.flags.writeable = self.upper_deg.flags.writeable = False
        self.last_ik = None

        # Initialize frame task for IK
        self.tip_frame = self.solver.add_frame_task(self.target_frame_name, np.eye(4))

    def forward_kinematics(self, joint_pos_deg: np.ndarray) -> np.ndarray:
        """
        Compute forward kinematics for given joint configuration given the target frame name in the constructor.

        Args:
            joint_pos_deg: Joint positions in degrees (numpy array)

        Returns:
            4x4 transformation matrix of the end-effector pose
        """

        # Convert degrees to radians
        joint_pos_rad = np.deg2rad(joint_pos_deg[: len(self.joint_names)])

        # Update joint positions in placo robot
        for i, joint_name in enumerate(self.joint_names):
            self.robot.set_joint(joint_name, joint_pos_rad[i])

        # Update kinematics
        self.robot.update_kinematics()

        # Get the transformation matrix
        return self.robot.get_T_world_frame(self.target_frame_name)

    def inverse_kinematics(
        self,
        current_joint_pos: np.ndarray,
        desired_ee_pose: np.ndarray,
        position_weight: float = 1.0,
        orientation_weight: float = 0.01,
        *,
        max_joint_step_deg: float = 20.0,
        max_iterations: int = 16,
    ) -> np.ndarray:
        """
        Bounded local Placo solve; returns a feasible partial step, not a promise
        of reaching an arbitrary 6D target with a 5DOF arm. Position residual is
        primary; rotation breaks position ties. Preserve task weights 1/0.01.
        The box stays anchored to the ORIGINAL tick seed for all iterations.
        last_ik distinguishes convergence, partial progress and holding still.
        Uses the supplied seed directly, including out-of-URDF starting states.
        Bounds constrain the returned command, not eligibility of the input.
        """
        started = time.perf_counter()
        report = dict(status="validating", iterations=0, selected_iteration=0)
        self.last_ik = report

        def fail(status, message):
            report.update(status=status, elapsed_ms=(time.perf_counter()-started)*1000)
            raise BoundedIKError(message, dict(report))

        current = np.asarray(current_joint_pos, dtype=np.float64)
        target = np.asarray(desired_ee_pose, dtype=np.float64)
        n = len(self.joint_names)
        if current.ndim != 1 or current.size < n or not np.isfinite(current).all():
            fail("invalid_input", "IK requires finite joint angles")
        if target.shape != (4, 4) or not np.isfinite(target).all():
            fail("invalid_input", "IK requires a finite 4x4 target")
        if (type(max_iterations) is not int or not 1 <= max_iterations <= 64 or
                not np.isfinite([max_joint_step_deg, position_weight, orientation_weight]).all() or
                max_joint_step_deg <= 0 or position_weight <= 0 or orientation_weight < 0):
            fail("invalid_input", "invalid IK iteration/step/weight settings")
        seed = current[:n].copy()
        initial = seed.copy()
        lo = np.maximum(self.lower_deg, seed-max_joint_step_deg)
        hi = np.minimum(self.upper_deg, seed+max_joint_step_deg)
        if np.any(lo > hi):
            fail("empty_local_bounds", "URDF and tick bounds have empty intersection")
        report.update(local_lower_deg=lo.tolist(), local_upper_deg=hi.tolist())

        def metrics(q):
            pose = np.asarray(self.forward_kinematics(q)).copy()
            if pose.shape != (4, 4) or not np.isfinite(pose).all():
                fail("invalid_fk", "nonfinite or malformed FK result")
            rotation = pose[:3, :3].T @ target[:3, :3]
            # SVD removes float32 target orthogonality noise for diagnostics.
            u, _, vh = np.linalg.svd(rotation)
            rotation = u @ np.diag([1., 1., np.linalg.det(u @ vh)]) @ vh
            sine = np.linalg.norm([rotation[2, 1]-rotation[1, 2],
                                  rotation[0, 2]-rotation[2, 0],
                                  rotation[1, 0]-rotation[0, 1]])/2
            angle = float(np.arctan2(sine, (np.trace(rotation)-1)/2))
            return float(np.linalg.norm(pose[:3, 3]-target[:3, 3])), angle

        def converged(error):
            return error[0] <= 1e-4 and (orientation_weight == 0 or error[1] <= np.deg2rad(.1))

        seed_error = metrics(initial)
        report.update(seed_position_error_mm=seed_error[0]*1000,
                      seed_orientation_error_deg=float(np.rad2deg(seed_error[1])))
        # An outside seed can initialize Placo, but is not itself a feasible
        # OUTPUT candidate. Do not reject it or secretly clip the model state.
        seed_feasible = bool(np.all(seed >= lo) and np.all(seed <= hi))
        best = initial.copy() if seed_feasible else None
        best_error = seed_error if seed_feasible else (float("inf"), float("inf"))
        report["seed_within_urdf"] = seed_feasible
        def is_better(error, reference):
            """Compare the actual FK error, with position as the primary objective."""
            position_eps = 1e-10
            if error[0] < reference[0] - position_eps:
                return True
            if abs(error[0] - reference[0]) <= position_eps:
                return error[1] < reference[1]
            return False

        report["candidate_position_error_mm"] = []
        report["rejected_worse_candidates"] = 0
        try:
            for name, lower, upper in zip(self.joint_names, np.deg2rad(lo), np.deg2rad(hi)):
                self.robot.set_joint_limits(name, float(lower), float(upper))
            self.solver.enable_joint_limits(True)
            self.solver.enable_velocity_limits(False)
            # Refresh after setting the seed, before solve: no stale Jacobians.
            self.forward_kinematics(initial)
            self.tip_frame.T_world_frame = target
            self.tip_frame.configure(self.target_frame_name, "soft", position_weight, orientation_weight)
            if not converged(best_error):
                for iteration in range(1, max_iterations+1):
                    self.solver.solve(True)
                    self.robot.update_kinematics()
                    report["iterations"] = iteration
                    q = np.rad2deg([self.robot.get_joint(name) for name in self.joint_names])
                    if not np.isfinite(q).all() or np.any(q < lo-1e-6) or np.any(q > hi+1e-6):
                        fail("invalid_candidate", "IK candidate violates its fixed local bounds")
                    q = np.clip(q, lo, hi)  # numerical endpoint noise only
                    error = metrics(q)
                    report["candidate_position_error_mm"].append(float(error[0] * 1000.0))
                    if best is None or is_better(error, best_error):
                        best, best_error = q.copy(), error
                        report["selected_iteration"] = iteration
                    else:
                        report["rejected_worse_candidates"] += 1
                    # Keep the next Placo iteration anchored at the best FK-verified
                    # candidate instead of continuing from a worse transient iterate.
                    self.forward_kinematics(best)
                    if converged(best_error):
                        break
            # Final selection is evaluated in FK, NOT clipped into a new pose.
            final_error = metrics(best)
            status = ("converged" if converged(final_error) else
                      "partial_progress" if final_error[0] < seed_error[0] else
                      "no_position_progress" if np.array_equal(best, seed) else "feasible_step")
            report.update(status=status, position_error_mm=final_error[0]*1000,
                          orientation_error_deg=float(np.rad2deg(final_error[1])),
                          max_joint_delta_deg=float(np.max(np.abs(best-seed))),
                          elapsed_ms=(time.perf_counter()-started)*1000)
            result = current.copy()
            result[:n] = best
            return result
        except BoundedIKError:
            self.forward_kinematics(initial)
            raise
        except Exception as exc:
            self.forward_kinematics(initial)
            fail("solver_error", f"bounded IK failed: {exc}")
        finally:
            # Restore global bounds even on failure; no per-tick shrink leakage.
            for name, lower, upper in zip(self.joint_names, np.deg2rad(self.lower_deg), np.deg2rad(self.upper_deg)):
                self.robot.set_joint_limits(name, float(lower), float(upper))
