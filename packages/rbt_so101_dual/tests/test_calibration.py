from __future__ import annotations

import unittest
from pathlib import Path

from so101_rbt.calibration import (
    MOTOR_NAMES,
    decode_sign_magnitude,
    encode_sign_magnitude,
    load_arm_calibration,
    model_state,
    hardware_target,
)

FIXTURES = Path(__file__).resolve().parent / "fixtures"


class CalibrationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.left = load_arm_calibration(FIXTURES / "dual_follower_left.json")
        self.right = load_arm_calibration(FIXTURES / "dual_follower_right.json")

    def test_normalization_matches_frozen_lerobot_formula_exhaustively(self) -> None:
        for calibration in (self.left, self.right):
            for motor in MOTOR_NAMES:
                cal = calibration.motors[motor]
                for raw in range(4096):
                    result = calibration.normalize_raw({name: raw for name in MOTOR_NAMES})
                    actual = result[MOTOR_NAMES.index(motor)]
                    if motor == "gripper":
                        bounded = min(cal.range_max, max(cal.range_min, raw))
                        expected = (bounded - cal.range_min) / (cal.range_max - cal.range_min) * 100
                    else:
                        expected = (raw - (cal.range_min + cal.range_max) / 2) * 360 / 4095
                    self.assertAlmostEqual(actual, expected, places=12)

    def test_unnormalization_truncates_and_clips_like_lerobot(self) -> None:
        for calibration in (self.left, self.right):
            for value in (-10.0, -1.0, -0.1, 0.0, 0.1, 45.0, 99.9, 100.0, 120.0):
                out = calibration.unnormalize((value, value, value, value, value, value))
                for motor in MOTOR_NAMES[:5]:
                    cal = calibration.motors[motor]
                    mid = (cal.range_min + cal.range_max) / 2
                    self.assertEqual(out[motor], int(value * 4095 / 360 + mid))
                grip = calibration.motors["gripper"]
                bounded = min(100.0, max(0.0, value))
                self.assertEqual(out["gripper"], int(bounded / 100 * (grip.range_max - grip.range_min) + grip.range_min))

    def test_zero_offsets_are_separate_and_invertible(self) -> None:
        import json

        offsets = json.loads((FIXTURES / "zero_offsets.json").read_text())
        sample = (1.0, 2.0, 3.0, 4.0, -5.0, 6.0)
        for arm in ("left", "right"):
            mapped = model_state(sample, offsets, arm)
            self.assertEqual(hardware_target(mapped, offsets, arm), sample)

    def test_sign_magnitude_roundtrip_and_overflow(self) -> None:
        for value in (-32767, -4095, -1, 0, 1, 4095, 32767):
            self.assertEqual(decode_sign_magnitude(encode_sign_magnitude(value, 15), 15), value)
        with self.assertRaises(ValueError):
            encode_sign_magnitude(32768, 15)


if __name__ == "__main__":
    unittest.main()
