"""Create a nearby FK target and recover it with local IK; no device access."""
from pathlib import Path
import json

import numpy as np

from so101_control.kinematics import DualKinematics


def main():
    root = Path(__file__).resolve().parents[1]
    kin = DualKinematics.from_config(root / "packages/so101_control/config/board.json")
    seed = np.array([0.0, -25, 30, 15, -30, 40])
    desired = seed + np.array([2, -3, 2, 2, 1, 0])
    for side, arm in kin.arms.items():
        target = arm.forward(desired)
        result = arm.inverse(seed, target, max_joint_step_deg=20)
        actual = arm.forward(result)
        print(json.dumps({
            "arm": side,
            "seed_deg_gripper": seed.tolist(),
            "result_deg_gripper": result.tolist(),
            "target_position_m": target[:3, 3].tolist(),
            "actual_position_m": actual[:3, 3].tolist(),
            "ik": arm.last_ik,
        }, indent=2))


if __name__ == "__main__":
    main()
