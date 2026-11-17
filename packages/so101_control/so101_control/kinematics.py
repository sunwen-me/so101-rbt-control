"""Native joint interface around one independent Pinocchio solver per arm."""
import json
from pathlib import Path
import numpy as np
from .mapping import JointMapping
from .pinocchio_ik import PinocchioBoxIK
from .types import SIDES, validate_arms

class ArmKinematics:
    def __init__(self, urdf, *, mapping=None,
                 joint_names=("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"),
                 target_frame="gripper", **solver_options):
        self.mapping = mapping or JointMapping()
        self.solver = PinocchioBoxIK(str(urdf), joint_names=list(joint_names),
                                   target_frame_name=target_frame, **solver_options)

    def forward(self, native_joints):
        values = np.asarray(native_joints, float)
        if values.shape not in ((5,), (6,)):
            raise ValueError("five native degree joints with optional gripper required")
        return self.solver.forward_kinematics(self.mapping.to_urdf(values[:5]))

    def inverse(self, native_seed, target_pose, *, max_joint_step_deg=20.0, orientation_weight=None):
        values = np.asarray(native_seed, float)
        if values.shape not in ((5,), (6,)) or not np.isfinite(values).all():
            raise ValueError("five native degree joints with optional gripper required")
        q = self.solver.inverse_kinematics(self.mapping.to_urdf(values[:5]), target_pose,
                max_joint_step_deg=max_joint_step_deg, orientation_weight=orientation_weight)
        result = values.copy()
        result[:5] = self.mapping.to_native(q)
        return result

    @property
    def last_ik(self):
        return self.solver.last_ik

class DualKinematics:
    def __init__(self, left, right):
        if left is right or left.solver is right.solver:
            raise ValueError("separate mutable solvers required for the two arms")
        self.arms = dict(left=left, right=right)

    def forward(self, native_arms):
        native = validate_arms(native_arms)
        return {s: self.arms[s].forward(native[s]) for s in SIDES}

    def solve(self, native_seed, targets, *, max_joint_step_deg=20.0, orientation_weight=None):
        native = validate_arms(native_seed)
        if not set(targets).issubset(SIDES) or not targets:
            raise ValueError("target poses must specify left and/or right")
        result = {s: native[s].copy() for s in SIDES}
        diagnostics = {}
        for side, pose in targets.items():
            result[side] = self.arms[side].inverse(native[side], pose,
                max_joint_step_deg=max_joint_step_deg, orientation_weight=orientation_weight)
            diagnostics[side] = dict(self.arms[side].last_ik)
        return result, diagnostics

    @classmethod
    def from_config(cls, path):
        path = Path(path).resolve()
        data = json.loads(path.read_text())
        arms = {}
        for side in SIDES:
            c = data["arms"][side]
            urdf = Path(c["urdf"])
            if not urdf.is_absolute():
                urdf = path.parent / urdf
            mapping = JointMapping(**{k: tuple(v) for k, v in c.get("joint_mapping", {}).items()})
            arms[side] = ArmKinematics(urdf, mapping=mapping,
                joint_names=c.get("urdf_joint_names", ["1", "2", "3", "4", "5"]),
                target_frame=c.get("target_frame", "gripper"))
        return cls(arms["left"], arms["right"])
