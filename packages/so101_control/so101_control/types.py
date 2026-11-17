"""Native arm values: five degree joints and a calibrated 0..100 gripper scale."""
from collections.abc import Mapping
import numpy as np

SIDES = ("left", "right")
JOINT_NAMES = ("shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll")

def validate_arms(arms: Mapping) -> dict[str, np.ndarray]:
    if set(arms) != set(SIDES):
        raise ValueError("expected left/right native arm values")
    result = {}
    for side in SIDES:
        values = np.asarray(arms[side], dtype=np.float64)
        if values.shape != (6,) or not np.isfinite(values).all():
            raise ValueError(f"{side}: five finite degree joints and one gripper value required")
        result[side] = values.copy()
    return result

