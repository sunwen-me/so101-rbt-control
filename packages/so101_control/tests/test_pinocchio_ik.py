"""Pinocchio IK regressions for soft orientation targets."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

from so101_control.pinocchio_ik import PinocchioBoxIK


ROOT = Path(__file__).resolve().parents[1]


@unittest.skipUnless(importlib.util.find_spec("pinocchio"), "Pinocchio not installed")
class PinocchioIKTests(unittest.TestCase):
    def test_exact_position_seed_still_optimizes_soft_orientation(self):
        seed = np.array([-9.5824175824, 6.989010989, 32.615384615,
                         32.527472527, -11.736263736])
        angle = np.deg2rad(5.0)
        local_z_rotation = np.array([
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ])

        urdf = ROOT / "config" / "urdf" / "so101_new_calib.urdf"
        for side in ("left", "right"):
            with self.subTest(side=side):
                kin = PinocchioBoxIK(str(urdf), target_frame_name="gripper",
                    joint_names=["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"])
                target = kin.forward_kinematics(seed)
                target[:3, :3] = target[:3, :3] @ local_z_rotation

                result = kin.inverse_kinematics(seed, target,
                    orientation_weight=0.01, max_joint_step_deg=20.0)
                final_pose = kin.forward_kinematics(result)
                seed_error = kin._metrics(kin.forward_kinematics(seed), target)
                final_error = kin._metrics(final_pose, target)

                self.assertLess(final_error[0] * 1000.0, 0.1)
                self.assertLess(final_error[1], np.deg2rad(4.0))
                self.assertLess(final_error[1], seed_error[1] - np.deg2rad(1.0))
                self.assertLessEqual(np.max(np.abs(result[:5] - seed)), 20.0 + 1e-8)
                self.assertLess(kin.last_ik["weighted_objective"],
                                kin.last_ik["seed_weighted_objective"])


if __name__ == "__main__":
    unittest.main()
