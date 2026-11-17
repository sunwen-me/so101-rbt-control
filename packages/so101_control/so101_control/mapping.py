"""Explicit native joint/URDF coordinate mapping; no policy normalization."""
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class JointMapping:
    signs: tuple = (1, 1, 1, 1, 1)
    offsets_deg: tuple = (0, 0, 0, 0, 0)

    def __post_init__(self):
        if len(self.signs) != 5 or any(x not in (-1, 1) for x in self.signs):
            raise ValueError("five joint signs (+1 or -1) required")
        if len(self.offsets_deg) != 5 or not np.isfinite(self.offsets_deg).all():
            raise ValueError("five finite joint offsets required")

    def _joints(self, values):
        result = np.asarray(values, dtype=float)
        if result.shape != (5,) or not np.isfinite(result).all():
            raise ValueError("five finite joint angles required")
        return result

    def to_urdf(self, native_deg):
        return self._joints(native_deg) * np.asarray(self.signs) + np.asarray(self.offsets_deg)

    def to_native(self, urdf_deg):
        return (self._joints(urdf_deg) - np.asarray(self.offsets_deg)) * np.asarray(self.signs)

def transform_pose(parent_from_base, base_from_tool):
    """Compose supplied 4x4 poses; translation units are metres."""
    a, b = np.asarray(parent_from_base, float), np.asarray(base_from_tool, float)
    if a.shape != (4, 4) or b.shape != (4, 4) or not np.isfinite(a).all() or not np.isfinite(b).all():
        raise ValueError("finite 4x4 transforms required")
    return a @ b

