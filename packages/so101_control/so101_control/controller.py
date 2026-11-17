"""Small orchestration interface for traditional joint and Cartesian tasks."""
from .execution import execute_trajectory
from .trajectory import plan_joint_move, trajectory_from_waypoints
from .types import validate_arms

class DualArmController:
    def __init__(self, client, *, kinematics=None):
        self.client, self.kinematics = client, kinematics

    def read(self):
        return validate_arms(self.client.snapshot()["arms"])

    def plan_joints(self, target, duration_s, *, start=None, **options):
        return plan_joint_move(self.read() if start is None else start, target, duration_s, **options)

    def plan_pose_waypoints(self, targets, *, start=None, waypoint_period_s=0.1,
                            multiplier=3, max_joint_step_deg=20.0, orientation_weight=None):
        if self.kinematics is None:
            raise RuntimeError("kinematics must be supplied for Cartesian targets")
        origin = self.read() if start is None else validate_arms(start)
        current = origin
        endpoints, diagnostics = [], []
        for poses in targets:
            current, report = self.kinematics.solve(current, poses,
                max_joint_step_deg=max_joint_step_deg, orientation_weight=orientation_weight)
            endpoints.append(current)
            diagnostics.append(report)
        trajectory = trajectory_from_waypoints(origin, endpoints,
            waypoint_period_s=waypoint_period_s, multiplier=multiplier)
        trajectory.diagnostics = tuple(diagnostics)
        return trajectory

    def execute(self, trajectory, **options):
        return execute_trajectory(self.client, trajectory, **options)
