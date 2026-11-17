"""Rotation helpers retained from the existing control solver."""
import numpy as np

class DifferentialIKError(ValueError):
    def __init__(self, message, report):
        super().__init__(message)
        self.report = report

def _project_rotation(rotation: np.ndarray) -> np.ndarray:
    u, _, vh = np.linalg.svd(rotation)
    return u @ np.diag([1.0, 1.0, np.linalg.det(u @ vh)]) @ vh

def _rotvec(rotation: np.ndarray) -> np.ndarray:
    r = _project_rotation(rotation)
    c = float(np.clip((np.trace(r) - 1.0) / 2.0, -1.0, 1.0))
    angle = float(np.arccos(c))
    if angle < 1e-8:
        return np.array([(r[2, 1] - r[1, 2]), (r[0, 2] - r[2, 0]),
                         (r[1, 0] - r[0, 1])], dtype=float) / 2.0
    sine = float(np.sin(angle))
    if abs(sine) > 1e-6:
        axis = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0],
                         r[1, 0] - r[0, 1]], dtype=float) / (2.0 * sine)
        return axis * angle
    # Stable enough for the rare pi rotation in a diagnostic solver.
    vals, vecs = np.linalg.eigh((r + np.eye(3)) / 2.0)
    axis = vecs[:, int(np.argmax(vals))]
    return axis * angle

IKError = DifferentialIKError

