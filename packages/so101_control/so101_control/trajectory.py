"""Native joint trajectories independent of observations and model chunks."""
from dataclasses import dataclass
import math
import numpy as np
from .types import SIDES, validate_arms

class JointInterpolator:
    """Upsample policy waypoints, with optional jerk-limited chunk boundaries.

    The historical path remains the default (``boundary_cycles=0``).  When a
    runner has to wait between chunks, ``boundary_cycles`` adds a short
    minimum-jerk ramp at the beginning and end of the prepared trajectory.  It
    keeps the first command from starting at full velocity and the last command
    from stopping abruptly, without changing the policy-frame timing in the
    middle of the chunk.
    """
    def __init__(self, origin, endpoints, multiplier, *, boundary_cycles=0,
                 smooth_gripper=False):
        self.origin = validate_arms(origin)
        self.endpoints = [validate_arms(p) for p in endpoints]
        if type(multiplier) is not int or multiplier < 1 or not self.endpoints:
            raise ValueError("positive integer multiplier and nonempty endpoints required")
        if type(boundary_cycles) is not int or boundary_cycles < 0:
            raise ValueError("boundary_cycles must be a nonnegative integer")
        if type(smooth_gripper) is not bool:
            raise ValueError("smooth_gripper must be bool")
        self.multiplier = multiplier
        self.boundary_cycles = boundary_cycles
        self.smooth_gripper = smooth_gripper
        self._cached_samples = None

    @staticmethod
    def _quintic_from_rest(t, final_normalized_velocity):
        """0->1 quintic, rest at t=0, matching velocity at t=1.

        The normalized final velocity is estimated from the unchanged linear
        trajectory immediately after the startup ramp.  Reversing this profile
        gives the shutdown ramp.
        """
        d = np.asarray(final_normalized_velocity, dtype=np.float64)
        t = np.asarray(t, dtype=np.float64)
        return ((10.0 - 4.0 * d) * t**3 +
                (7.0 * d - 15.0) * t**4 +
                (6.0 - 3.0 * d) * t**5)

    def _base_samples(self):
        samples = []
        for index in range(len(self)):
            frame, substep = divmod(index, self.multiplier)
            alpha = (substep + 1) / self.multiplier
            previous = self.origin if frame == 0 else self.endpoints[frame - 1]
            target = {}
            for side in SIDES:
                goal = self.endpoints[frame][side].copy()
                goal[:5] = previous[side][:5] + alpha * (goal[:5] - previous[side][:5])
                target[side] = goal
            samples.append(target)
        return samples

    @staticmethod
    def _normalised_velocity(delta, velocity, duration):
        result = np.zeros_like(delta, dtype=np.float64)
        mask = np.abs(delta) > 1e-9
        result[mask] = velocity[mask] * duration / delta[mask]
        # A bad/very sharp policy corner must not turn the boundary ramp into
        # an overshoot.  Normal policy motion remains unchanged.
        return np.clip(result, -2.0, 2.0)

    def _smooth_boundaries(self, samples):
        cycles = min(self.boundary_cycles, len(samples) // 2)
        if cycles <= 0:
            return samples
        duration = float(cycles)
        dimensions = slice(None) if self.smooth_gripper else slice(0, 5)

        # Startup: start at rest at the measured pose, then match the original
        # linear trajectory's velocity at the end of the ramp.
        head_end = samples[cycles - 1]
        head_next = samples[cycles]
        for side in SIDES:
            start = self.origin[side].copy()
            end = head_end[side].copy()
            velocity = head_next[side] - head_end[side]
            d = self._normalised_velocity(end[dimensions] - start[dimensions],
                                          velocity[dimensions], duration)
            for index in range(cycles):
                t = (index + 1) / duration
                w = self._quintic_from_rest(t, d)
                value = samples[index][side].copy()
                value[dimensions] = start[dimensions] + w * (end[dimensions] - start[dimensions])
                samples[index][side] = value

        # Shutdown: match the original trajectory's incoming velocity, then
        # arrive at the final target with zero velocity and acceleration.
        tail_start = len(samples) - cycles
        tail_origin = self.origin if tail_start == 0 else samples[tail_start - 1]
        tail_end = samples[-1]
        tail_prev = tail_origin if tail_start < 2 else samples[tail_start - 2]
        for side in SIDES:
            start = tail_origin[side].copy()
            end = tail_end[side].copy()
            velocity = start - tail_prev[side]
            d = self._normalised_velocity(end[dimensions] - start[dimensions],
                                          velocity[dimensions], duration)
            for offset in range(cycles):
                t = (offset + 1) / duration
                # Reverse a rest->moving quintic to obtain moving->rest.
                w = 1.0 - self._quintic_from_rest(1.0 - t, d)
                value = samples[tail_start + offset][side].copy()
                value[dimensions] = start[dimensions] + w * (end[dimensions] - start[dimensions])
                samples[tail_start + offset][side] = value
        return samples

    def __len__(self):
        return len(self.endpoints) * self.multiplier

    def sample(self, index):
        if not 0 <= index < len(self):
            raise IndexError(index)
        frame, substep = divmod(index, self.multiplier)
        target = self._samples[index]
        return frame, substep, {side: target[side].tolist() for side in SIDES}

    @property
    def _samples(self):
        # The trajectory is small (at most 30 policy frames); computing this
        # once keeps sample() deterministic and avoids per-tick allocation.
        if self._cached_samples is None:
            self._cached_samples = self._smooth_boundaries(self._base_samples())
        return self._cached_samples

@dataclass
class JointTrajectory:
    origin: dict
    samples: tuple
    period_s: float
    diagnostics: tuple = ()

    def __post_init__(self):
        self.origin = validate_arms(self.origin)
        self.samples = tuple(validate_arms(p) for p in self.samples)
        if not self.samples or not math.isfinite(self.period_s) or self.period_s <= 0:
            raise ValueError("nonempty trajectory and positive finite period required")

    def __len__(self):
        return len(self.samples)

    @property
    def duration_s(self):
        return len(self) * self.period_s

    def to_dict(self):
        return dict(period_s=self.period_s, duration_s=self.duration_s,
                    origin={s: self.origin[s].tolist() for s in SIDES},
                    samples=[{s: p[s].tolist() for s in SIDES} for p in self.samples],
                    diagnostics=list(self.diagnostics))

def plan_joint_move(origin, target, duration_s, *, hz=30.0, profile="linear", gripper="hold"):
    """Time-parametrized move; gripper is held until the final sample by default."""
    origin, target = validate_arms(origin), validate_arms(target)
    if not np.isfinite([duration_s, hz]).all() or duration_s <= 0 or hz <= 0:
        raise ValueError("positive finite duration and frequency required")
    if profile not in ("linear", "smoothstep") or gripper not in ("hold", "linear", "step"):
        raise ValueError("unknown interpolation profile or gripper mode")
    count = max(1, math.ceil(duration_s * hz))
    samples = []
    for index in range(count):
        alpha = (index + 1) / count
        weight = alpha if profile == "linear" else alpha * alpha * (3 - 2 * alpha)
        p = {}
        for side in SIDES:
            value = origin[side] + weight * (target[side] - origin[side])
            if gripper == "hold":
                value[5] = target[side][5] if index == count - 1 else origin[side][5]
            elif gripper == "step":
                value[5] = target[side][5]
            else:
                value[5] = origin[side][5] + alpha * (target[side][5] - origin[side][5])
            if index == count - 1:
                value = target[side].copy()
            p[side] = value
        samples.append(p)
    return JointTrajectory(origin, tuple(samples), duration_s / count)

def trajectory_from_waypoints(origin, endpoints, *, waypoint_period_s=0.1, multiplier=3,
                              boundary_cycles=0, smooth_gripper=False):
    sequence = JointInterpolator(origin, endpoints, multiplier,
                                 boundary_cycles=boundary_cycles,
                                 smooth_gripper=smooth_gripper)
    return JointTrajectory(origin, tuple(sequence.sample(i)[2] for i in range(len(sequence))),
                           waypoint_period_s / multiplier)
