from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

MOTOR_NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
ARM_MOTOR_NAMES = MOTOR_NAMES[:5]
RESOLUTION = 4095


@dataclass(frozen=True)
class MotorCalibration:
    motor_id: int
    drive_mode: int
    homing_offset: int
    range_min: int
    range_max: int

    def __post_init__(self) -> None:
        if any(type(v) is not int for v in (self.motor_id, self.drive_mode, self.homing_offset, self.range_min, self.range_max)):
            raise ValueError("calibration values must be integers")
        if not 0 <= self.range_min < self.range_max <= RESOLUTION:
            raise ValueError("calibration range must lie within 0..4095")
        if self.range_min >= self.range_max:
            raise ValueError("calibration range must have positive width")
        if not 0 <= self.drive_mode <= 1:
            raise ValueError("drive_mode must be 0 or 1")


@dataclass(frozen=True)
class ArmCalibration:
    motors: Mapping[str, MotorCalibration]
    apply_drive_mode: bool = True

    def __post_init__(self) -> None:
        if tuple(self.motors) != MOTOR_NAMES:
            raise ValueError(f"motor order must be {MOTOR_NAMES}")
        ids = [self.motors[name].motor_id for name in MOTOR_NAMES]
        if ids != [1, 2, 3, 4, 5, 6]:
            raise ValueError("SO-101 motor IDs must be 1 through 6 in canonical order")

    def normalize_raw(self, raw: Mapping[str, int]) -> tuple[float, ...]:
        """Match LeRobot FeetechMotorsBus normalization for STS3215."""
        if set(raw) != set(MOTOR_NAMES):
            raise ValueError("raw state must contain exactly the six SO-101 motors")
        out: list[float] = []
        for name in MOTOR_NAMES:
            if type(raw[name]) is not int:
                raise ValueError("raw positions must be integers")
            value = raw[name]
            cal = self.motors[name]
            drive_flip = self.apply_drive_mode and bool(cal.drive_mode)
            if name == "gripper":
                bounded = min(cal.range_max, max(cal.range_min, value))
                normalized = (bounded - cal.range_min) / (cal.range_max - cal.range_min) * 100.0
                out.append(100.0 - normalized if drive_flip else normalized)
            else:
                midpoint = (cal.range_min + cal.range_max) / 2.0
                # LeRobot intentionally does not clip degree-mode values.
                out.append((value - midpoint) * 360.0 / RESOLUTION)
        return tuple(out)

    def unnormalize(self, joints: tuple[float, ...] | list[float]) -> dict[str, int]:
        """Match LeRobot conversion, including truncation and gripper clipping."""
        if len(joints) != 6:
            raise ValueError("SO-101 target must contain five degree joints and one gripper percent")
        if not all(math.isfinite(float(v)) for v in joints):
            raise ValueError("joint target contains NaN or infinity")
        out: dict[str, int] = {}
        for name, value in zip(MOTOR_NAMES, joints, strict=True):
            cal = self.motors[name]
            drive_flip = self.apply_drive_mode and bool(cal.drive_mode)
            if name == "gripper":
                adjusted = 100.0 - float(value) if drive_flip else float(value)
                bounded = min(100.0, max(0.0, adjusted))
                out[name] = int((bounded / 100.0) * (cal.range_max - cal.range_min) + cal.range_min)
            else:
                midpoint = (cal.range_min + cal.range_max) / 2.0
                out[name] = int((float(value) * RESOLUTION / 360.0) + midpoint)
        return out


def load_arm_calibration(path: str | Path, *, apply_drive_mode: bool = True) -> ArmCalibration:
    data = json.loads(Path(path).read_text())
    if not isinstance(data, dict) or tuple(data) != MOTOR_NAMES:
        raise ValueError("calibration JSON has unexpected motor names or order")
    motors = {
        name: MotorCalibration(
            motor_id=data[name]["id"],
            drive_mode=data[name]["drive_mode"],
            homing_offset=data[name]["homing_offset"],
            range_min=data[name]["range_min"],
            range_max=data[name]["range_max"],
        )
        for name in MOTOR_NAMES
    }
    return ArmCalibration(motors=motors, apply_drive_mode=apply_drive_mode)


def encode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    max_magnitude = (1 << sign_bit_index) - 1
    magnitude = abs(int(value))
    if magnitude > max_magnitude:
        raise ValueError(f"magnitude {magnitude} exceeds sign-magnitude capacity {max_magnitude}")
    return ((1 if value < 0 else 0) << sign_bit_index) | magnitude


def decode_sign_magnitude(value: int, sign_bit_index: int) -> int:
    sign = (int(value) >> sign_bit_index) & 1
    magnitude = int(value) & ((1 << sign_bit_index) - 1)
    return -magnitude if sign else magnitude


def model_state(raw_state: tuple[float, ...], offsets: Mapping[str, Mapping[str, float]], arm: str) -> tuple[float, ...]:
    """Apply XR1 policy-frame offsets once, outside the hardware calibration."""
    if arm not in ("left", "right") or len(raw_state) != 6:
        raise ValueError("invalid arm name or state dimension")
    values = list(raw_state)
    values[4] += float(offsets[arm]["wrist_roll_deg"])
    values[5] += float(offsets[arm]["gripper"])
    return tuple(values)


def hardware_target(policy_state: tuple[float, ...], offsets: Mapping[str, Mapping[str, float]], arm: str) -> tuple[float, ...]:
    """Invert policy-frame offsets before converting back to servo ticks."""
    if arm not in ("left", "right") or len(policy_state) != 6:
        raise ValueError("invalid arm name or target dimension")
    values = list(policy_state)
    values[4] -= float(offsets[arm]["wrist_roll_deg"])
    values[5] -= float(offsets[arm]["gripper"])
    return tuple(values)
