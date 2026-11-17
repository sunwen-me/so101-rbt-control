"""Run the two actual upstream methods via AST, without importing LeRobot/Torch."""
import ast
import hashlib
import random
import unittest
from enum import Enum
from pathlib import Path
from types import SimpleNamespace

from so101_rbt.calibration import MOTOR_NAMES, ArmCalibration, MotorCalibration, load_arm_calibration, decode_sign_magnitude
from so101_rbt.backend import FeetechSTS3215Backend

ROOT = Path(__file__).resolve().parents[1]
FIXTURES = ROOT / "tests/fixtures"
EXPECTED_HASH = "6ce6109e32590ac2830f6995d13bb478b1a984695cc9ca59c0763ff091666393"


class MotorNormMode(Enum):
    DEGREES = 1
    RANGE_0_100 = 2
    RANGE_M100_100 = 3


def original_methods(cal):
    source = ROOT / "upstream/motors_bus.py"
    assert hashlib.sha256(source.read_bytes()).hexdigest() == EXPECTED_HASH
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "SerialMotorsBus")
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in ("_normalize", "_unnormalize")]
    module = ast.Module(body=methods, type_ignores=[])
    env = {"MotorNormMode": MotorNormMode}
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), env)
    id_name = lambda i: MOTOR_NAMES[i - 1]
    fake = SimpleNamespace(calibration={n: SimpleNamespace(range_min=c.range_min, range_max=c.range_max, drive_mode=c.drive_mode)
                                        for n, c in cal.motors.items()},
        apply_drive_mode=cal.apply_drive_mode, _id_to_name=id_name, _id_to_model=lambda i: "sts3215",
        model_resolution_table={"sts3215": 4096},
        motors={n: SimpleNamespace(norm_mode=MotorNormMode.RANGE_0_100 if n == "gripper" else MotorNormMode.DEGREES)
                for n in MOTOR_NAMES})
    return fake, env["_normalize"], env["_unnormalize"]


class UpstreamEquivalenceTests(unittest.TestCase):
    def test_prepared_degree_goals_outside_calibration_match_actual_upstream(self):
        for side in ("left", "right"):
            cal = load_arm_calibration(FIXTURES / f"dual_follower_{side}.json")
            fake, _, unnormalize = original_methods(cal)
            backend = object.__new__(FeetechSTS3215Backend)
            backend.calibration = cal
            backend.allow_goal_writes = True
            for index, name in enumerate(MOTOR_NAMES[:5]):
                motor = cal.motors[name]
                midpoint = (motor.range_min + motor.range_max) / 2
                for raw_target in (motor.range_min - 17, motor.range_max + 17):
                    target = [0, 0, 0, 0, 0, 50]
                    target[index] = (raw_target - midpoint) * 360 / 4095
                    expected = unnormalize(fake, dict(zip(range(1, 7), target)))
                    with self.subTest(side=side, motor=name, raw_target=raw_target):
                        prepared = backend.prepare_target(tuple(target))
                        self.assertEqual([decode_sign_magnitude(low | high << 8, 15) for _,low,high in prepared],
                                         [expected[i] for i in range(1,7)])

    def test_exhaustive_raw_and_random_target_match_actual_upstream_methods(self):
        rng = random.Random(42)
        count = 0
        for side in ("left", "right"):
            base = load_arm_calibration(FIXTURES / f"dual_follower_{side}.json")
            for drive_mode, apply_drive_mode in ((0, True), (1, True), (1, False)):
                cal = ArmCalibration({n: MotorCalibration(c.motor_id, drive_mode, c.homing_offset, c.range_min, c.range_max)
                                      for n, c in base.motors.items()}, apply_drive_mode)
                fake, normalize, unnormalize = original_methods(cal)
                for value in range(4096):
                    old = normalize(fake, dict.fromkeys(range(1, 7), value))
                    new = cal.normalize_raw(dict.fromkeys(MOTOR_NAMES, value))
                    self.assertEqual(tuple(old[i] for i in range(1, 7)), new)
                    count += 6
                for _ in range(2000):
                    values = tuple(rng.uniform(-300, 300) for _ in MOTOR_NAMES)
                    old = unnormalize(fake, dict(zip(range(1, 7), values)))
                    new = cal.unnormalize(values)
                    self.assertEqual([old[i] for i in range(1, 7)], list(new.values()))
                    count += 6
        print(f"actual upstream AST equivalence: {count} scalar comparisons, exact equality")


if __name__ == "__main__":
    unittest.main()
