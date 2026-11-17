import ast
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import unittest
import numpy as np

from so101_control import (JointMapping, JointInterpolator, plan_joint_move,
    trajectory_from_waypoints, execute_trajectory, DualArmController, transform_pose)
from so101_control.types import validate_arms

ROOT = Path(__file__).resolve().parents[1]
def arms(value=0.0, grip=7.0):
    return {s: [value] * 5 + [grip] for s in ("left", "right")}

class Clock:
    def __init__(self): self.now = 0.0
    def monotonic(self): return self.now
    def sleep(self, duration):
        assert duration >= 0
        self.now += duration

class Client:
    def __init__(self, clock, delay=0.0, send=True):
        self.send = send
        self.clock, self.delay = clock, delay
        self.targets, self.times = [], []
    def send_target(self, target):
        self.times.append(self.clock.now)
        self.targets.append(validate_arms(target))
        self.clock.now += self.delay
        return {"command_sequence": len(self.targets), "status": "published_unconfirmed"}
    def snapshot(self): return {"arms": arms()}

class ControlTests(unittest.TestCase):
    def test_import_does_not_load_optional_dependencies(self):
        code = ("import sys, so101_control; forbidden=['xr1_runner','torch','transformers','rbt','pinocchio','placo','onnxruntime']; "
                "assert not set(forbidden).intersection(sys.modules), set(forbidden).intersection(sys.modules)")
        subprocess.run([sys.executable, "-c", code], cwd=ROOT, check=True)

    def test_source_has_no_vla_or_camera_imports(self):
        forbidden = {"xr1_runner", "torch", "transformers", "onnxruntime", "PIL", "cv2"}
        for path in (ROOT / "so101_control").glob("*.py"):
            for node in ast.walk(ast.parse(path.read_text())):
                names = [n.name for n in node.names] if isinstance(node, ast.Import) else [node.module or ""] if isinstance(node, ast.ImportFrom) else []
                self.assertFalse({n.split(".")[0] for n in names} & forbidden, str(path))
                if "placo" in names:
                    self.assertEqual(path.name, "placo_ik.py")

    def test_identity_mapping(self):
        q = np.arange(5.0)
        np.testing.assert_array_equal(JointMapping().to_urdf(q), q)

    def test_mapping_roundtrip_and_official_zero_mapping(self):
        mapping = JointMapping(signs=(1, -1, 1, -1, 1), offsets_deg=(0, 2, 0, 0, 90))
        q = np.array([10, 20, -30, 40, -100.0])
        np.testing.assert_array_equal(mapping.to_native(mapping.to_urdf(q)), q)
        config = json.loads((ROOT / "config/board.json").read_text())
        expected_joints = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
        for side in ("left", "right"):
            self.assertEqual(config["arms"][side]["urdf_joint_names"], expected_joints)
            self.assertEqual(config["arms"][side]["target_frame"], "gripper")
            self.assertEqual(config["arms"][side]["joint_mapping"],
                             {"signs": [1] * 5, "offsets_deg": [0] * 5})
            self.assertEqual(config["arms"][side]["urdf"], "urdf/so101_new_calib.urdf")
        self.assertTrue((ROOT / "config/urdf/so101_new_calib.urdf").is_file())

    def test_transform_composition(self):
        a, b = np.eye(4), np.eye(4)
        a[0, 3], b[1, 3] = 1.0, 2.0
        np.testing.assert_array_equal(transform_pose(a, b)[:3, 3], [1, 2, 0])

    def test_interpolator_matches_runner_convention(self):
        origin = {"left": [0] * 5 + [7], "right": [90] * 5 + [8]}
        endpoints = [{"left": [3] * 5 + [12], "right": [93] * 5 + [13]},
                     {"left": [6] * 5 + [17], "right": [96] * 5 + [18]}]
        seq = JointInterpolator(origin, endpoints, 3)
        self.assertEqual(len(seq), 6)
        self.assertEqual(seq.sample(0)[2]["left"], [1] * 5 + [12])
        self.assertEqual(seq.sample(2)[2], endpoints[0])
        self.assertEqual(seq.sample(5)[2], endpoints[-1])

    def test_interpolator_boundary_ramp_is_opt_in(self):
        origin, goal = arms(0), arms(12)
        endpoints = [arms(3), arms(6), arms(9), goal]
        plain = JointInterpolator(origin, endpoints, 3)
        ramped = JointInterpolator(origin, endpoints, 3, boundary_cycles=3)
        self.assertEqual(plain.sample(0)[2]['left'], [1] * 5 + [7])
        self.assertLess(ramped.sample(0)[2]['left'][0], plain.sample(0)[2]['left'][0])
        self.assertEqual(ramped.sample(11)[2], plain.sample(11)[2])
        self.assertEqual(ramped.sample(len(ramped) - 1)[2]['left'][:5], goal['left'][:5])

    def test_thirty_hz_plan_duration_and_endpoints(self):
        seq = plan_joint_move(arms(), arms(15, 20), 2.0, hz=30)
        self.assertEqual(len(seq), 60)
        self.assertEqual(seq.duration_s, 2.0)
        np.testing.assert_array_equal(seq.samples[-1]["left"], arms(15, 20)["left"])
        self.assertEqual(seq.samples[0]["left"][5], 7)

    def test_gripper_modes(self):
        for mode, expected in (("hold", 7), ("step", 27), ("linear", 17)):
            seq = plan_joint_move(arms(), arms(0, 27), 1, hz=2, gripper=mode)
            self.assertEqual(seq.samples[0]["left"][5], expected)

    def test_smoothstep_explicit(self):
        linear = plan_joint_move(arms(), arms(8), 1, hz=4)
        smooth = plan_joint_move(arms(), arms(8), 1, hz=4, profile="smoothstep")
        self.assertLess(smooth.samples[0]["left"][0], linear.samples[0]["left"][0])
        np.testing.assert_array_equal(smooth.samples[-1]["left"], linear.samples[-1]["left"])

    def test_waypoint_duration(self):
        seq = trajectory_from_waypoints(arms(), [arms(3), arms(6)], waypoint_period_s=.1, multiplier=3)
        self.assertEqual(len(seq), 6)
        self.assertAlmostEqual(seq.duration_s, .2)

    def test_invalid_inputs(self):
        with self.assertRaises(ValueError): validate_arms({"left": [0] * 6})
        with self.assertRaises(ValueError): plan_joint_move(arms(), arms(), 0)
        with self.assertRaises(ValueError): JointMapping(signs=(1, 1))
        with self.assertRaises(ValueError): JointInterpolator(arms(), [arms()], True)

    def test_execution_absolute_deadlines(self):
        clock = Clock(); client = Client(clock, delay=.0002)
        seq = plan_joint_move(arms(), arms(10), 1, hz=30)
        result = execute_trajectory(client, seq, clock=clock)
        self.assertEqual(result["published_count"], 30)
        self.assertEqual(result["skipped_samples"], 0)
        self.assertTrue(result["completed"])
        self.assertAlmostEqual(client.times[-1], 1.0)

    def test_slow_sender_skips_not_bursts(self):
        clock = Clock(); client = Client(clock, delay=.08)
        seq = plan_joint_move(arms(), arms(10), 1, hz=30)
        result = execute_trajectory(client, seq, clock=clock)
        self.assertGreater(result["skipped_samples"], 0)
        self.assertTrue(all(b - a >= .079 for a, b in zip(client.times, client.times[1:])))
        np.testing.assert_array_equal(client.targets[-1]["left"], seq.samples[-1]["left"])

    def test_stop_and_readonly_no_publication(self):
        clock = Clock(); client = Client(clock)
        seq = plan_joint_move(arms(), arms(10), 1)
        result = execute_trajectory(client, seq, clock=clock, stop_requested=lambda: True)
        self.assertEqual(result["published_count"], 0)
        with self.assertRaises(PermissionError): execute_trajectory(Client(clock, send=False), seq, clock=clock)

    def test_controller_joint_path_without_kinematics(self):
        ctl = DualArmController(Client(Clock()))
        seq = ctl.plan_joints(arms(5), 1)
        self.assertEqual(len(seq), 30)
        with self.assertRaises(RuntimeError): ctl.plan_pose_waypoints([{"left": np.eye(4)}])

@unittest.skipUnless(importlib.util.find_spec("pinocchio"), "Pinocchio not installed")
class KinematicsTests(unittest.TestCase):
    def setUp(self):
        from so101_control.kinematics import DualKinematics
        self.kin = DualKinematics.from_config(ROOT / "config/board.json")

    def test_independent_solvers_and_mapping(self):
        self.assertIsNot(self.kin.arms["left"].solver.data, self.kin.arms["right"].solver.data)
        for side in ("left", "right"):
            arm = self.kin.arms[side]
            pose = arm.forward(np.zeros(6))
            ref = arm.solver.forward_kinematics(arm.mapping.to_urdf(np.zeros(5)))
            np.testing.assert_array_equal(pose, ref)

    def test_both_arms_use_official_model_limits_and_identity_mapping(self):
        left, right = self.kin.arms["left"], self.kin.arms["right"]
        np.testing.assert_array_equal(left.solver.lower_deg, right.solver.lower_deg)
        np.testing.assert_array_equal(left.solver.upper_deg, right.solver.upper_deg)
        for arm in (left, right):
            np.testing.assert_array_equal(arm.mapping.signs, np.ones(5))
            np.testing.assert_array_equal(arm.mapping.offsets_deg, np.zeros(5))
            self.assertEqual(arm.solver.joint_names,
                             ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"])

    def test_fk_ik_and_gripper_passthrough(self):
        for side in ("left", "right"):
            arm = self.kin.arms[side]
            seed = np.array([0., -25, 30, 15, -30, 41])
            desired = seed.copy(); desired[:5] += [2, -3, 2, 2, 1]
            pose = arm.forward(desired)
            result = arm.inverse(seed, pose)
            self.assertEqual(result[5], seed[5])
            self.assertLessEqual(np.max(np.abs(result[:5]-seed[:5])), 20 + 1e-8)
            self.assertLessEqual(arm.last_ik["position_error_mm"], arm.last_ik["seed_position_error_mm"] + 1e-8)

    def test_unselected_arm_is_held(self):
        seed = arms()
        result, diagnostic = self.kin.solve(seed, {"left": self.kin.arms["left"].forward(seed["left"])})
        np.testing.assert_array_equal(result["right"], seed["right"])
        self.assertEqual(set(diagnostic), {"left"})

    def test_cartesian_planning_keeps_grippers_and_other_arm(self):
        origin = arms()
        target = self.kin.arms["left"].forward(origin["left"])
        target[2, 3] += .002
        ctl = DualArmController(Client(Clock()), kinematics=self.kin)
        seq = ctl.plan_pose_waypoints([{"left": target}], start=origin)
        self.assertEqual(len(seq), 3)
        for point in seq.samples:
            np.testing.assert_array_equal(point["right"], origin["right"])
            self.assertEqual(point["left"][5], origin["left"][5])
        self.assertIn("position_error_mm", seq.diagnostics[0]["left"])

if __name__ == "__main__": unittest.main()
