import unittest
import inspect
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from so101_rbt.backend import FeetechSTS3215Backend
from so101_rbt.calibration import load_arm_calibration


class FakeReader:
    available = True
    comm = 0
    cleared = False
    def addParam(self, i): return True
    calls = 0
    def txRxPacket(self): self.calls += 1; return self.comm
    def isAvailable(self, *args): return self.available
    def getData(self, *args): return 2048
    def clearParam(self): self.cleared = True


class FakeWriter:
    comm = 0
    sent = 0
    def __init__(self): self.params = []
    def addParam(self, i, params): self.params.append((i, params)); return True
    def txPacket(self): self.sent += 1; return self.comm
    def clearParam(self): pass


class BackendTests(unittest.TestCase):
    def setUp(self):
        self.reader = FakeReader()
        self.writer = FakeWriter()
        self.backend = object.__new__(FeetechSTS3215Backend)
        self.backend.port = "fake"
        self.backend.port_handler = object()
        self.backend.packet_handler = object()
        self.backend.allow_goal_writes = True
        self.backend.num_read_retries = 2
        self.backend.calibration = load_arm_calibration(Path(__file__).resolve().parent / "fixtures/dual_follower_left.json")
        self.backend._scs = SimpleNamespace(COMM_SUCCESS=0,
            GroupSyncRead=lambda *args: self.reader, GroupSyncWrite=lambda *args: self.writer)

    def test_read_missing_id_rejected_and_resources_cleared(self):
        self.reader.available = False
        with self.assertRaises(ConnectionError): self.backend.read_state()
        self.assertTrue(self.reader.cleared)
        self.assertEqual(self.reader.calls,3)

    def test_communication_failure_rejected(self):
        self.reader.comm = -1
        with self.assertRaises(ConnectionError): self.backend.read_state()
        self.assertEqual(self.reader.calls,3)

    def test_tick_conversion_and_little_endian_goal(self):
        prepared = self.backend.prepare_target((0, 0, 0, 0, 0, 50))
        raw = self.backend.calibration.unnormalize((0, 0, 0, 0, 0, 50))
        self.backend.send_prepared(prepared)
        self.assertEqual(self.writer.sent, 1)
        self.assertEqual([p[1][0] | p[1][1] << 8 for p in self.writer.params], list(raw.values()))

    def test_readonly_never_transmits(self):
        self.backend.allow_goal_writes = False
        with self.assertRaises(PermissionError): self.backend.prepare_target((0,)*6)
        with self.assertRaises(PermissionError): self.backend.send_prepared(())
        self.assertEqual(self.writer.sent, 0)

    def test_invalid_targets_prevent_send(self):
        for target in ((5000, 0, 0, 0, 0, 0), (float("nan"),)*6):
            with self.assertRaises(ValueError): self.backend.prepare_target(target)
        self.assertEqual(self.writer.sent, 0)

    def test_degree_goal_beyond_calibration_is_not_a_new_software_stop(self):
        cal = self.backend.calibration.motors["elbow_flex"]
        midpoint = (cal.range_min + cal.range_max) / 2.0
        for raw_target in (cal.range_min - 17, cal.range_max + 17):
            target = (0, 0, (raw_target - midpoint) * 360.0 / 4095, 0, 0, 50)
            expected = self.backend.calibration.unnormalize(target)
            self.assertFalse(cal.range_min <= expected["elbow_flex"] <= cal.range_max)
            self.writer.params.clear()
            sent_before = self.writer.sent
            self.backend.send_prepared(self.backend.prepare_target(target))
            self.assertEqual(self.writer.sent, sent_before + 1)
            self.assertEqual([p[1][0] | p[1][1] << 8 for p in self.writer.params], list(expected.values()))

    def test_malformed_packet_bytes_still_do_not_transmit(self):
        prepared = list(self.backend.prepare_target((0, 0, 0, 0, 0, 50)))
        prepared[2] = (3, 256, 0)
        with self.assertRaises(ValueError): self.backend.send_prepared(tuple(prepared))
        self.assertEqual(self.writer.sent, 0)

    def test_failed_write_not_retried(self):
        self.writer.comm = -1
        with self.assertRaises(ConnectionError):
            self.backend.send_prepared(self.backend.prepare_target((0,)*6))
        self.assertEqual(self.writer.sent, 1)

    def test_defaults_match_lerobot_read_retries_and_sdk_write(self):
        value = inspect.signature(FeetechSTS3215Backend).parameters["write_timeout_s"].default
        self.assertIsNone(value)
        self.assertEqual(inspect.signature(FeetechSTS3215Backend).parameters["num_read_retries"].default,2)

    def test_invalid_write_deadlines_rejected_before_open(self):
        for value in (True, False, 0, -1, float("nan"), float("inf")):
            with self.subTest(value=value), self.assertRaises(ValueError):
                FeetechSTS3215Backend("/dev/so101_left", self.backend.calibration, write_timeout_s=value)

    def test_two_comm_failures_then_success(self):
        self.reader.txRxPacket=Mock(side_effect=[-1,-1,0])
        self.assertEqual(len(self.backend.read_raw()),6)
        self.assertEqual(self.reader.txRxPacket.call_count,3)
        self.assertTrue(self.reader.cleared)

    def test_missing_reply_then_success_retries_complete_read(self):
        self.reader.isAvailable=Mock(side_effect=[False]*6+[True]*6)
        self.assertEqual(len(self.backend.read_raw()),6)
        self.assertEqual(self.reader.calls,2)

    def test_no_retries_on_success_and_zero_can_disable_retries(self):
        self.backend.read_raw();self.assertEqual(self.reader.calls,1)
        self.backend.num_read_retries=0;self.reader.calls=0;self.reader.comm=-1
        with self.assertRaises(ConnectionError):self.backend.read_raw()
        self.assertEqual(self.reader.calls,1)

    def test_signed_feedback_not_rejected_by_new_single_turn_gate(self):
        self.reader.getData=lambda *args:0x8001
        self.assertEqual(set(self.backend.read_raw().values()),{-1})

    def test_connect_checks_models_not_extra_register_equality(self):
        self.backend.configuration={}
        self.backend._read_register=Mock(return_value=777)
        self.backend._verify_inventory_and_calibration()
        self.assertEqual(self.backend._read_register.call_count,6)
        self.assertEqual({c.args[1] for c in self.backend._read_register.call_args_list},{'Model_Number'})

    def test_gripper_clips_in_conversion_like_lerobot(self):
        for grip,endpoint in ((-1,0),(101,100)):
            self.assertEqual(self.backend.prepare_target((0,0,0,0,0,grip)),
                             self.backend.prepare_target((0,0,0,0,0,endpoint)))

    def test_read_transport_exception_not_retried_and_reader_cleared(self):
        self.reader.txRxPacket = Mock(side_effect=TimeoutError("transport deadline"))
        with self.assertRaises(TimeoutError):
            self.backend.read_state()
        self.reader.txRxPacket.assert_called_once()
        self.assertTrue(self.reader.cleared)

    def test_goal_transport_exception_not_retried(self):
        self.writer.txPacket = Mock(side_effect=TimeoutError("uncertain transmission"))
        with self.assertRaises(TimeoutError):
            self.backend.send_prepared(self.backend.prepare_target((0,)*6))
        self.writer.txPacket.assert_called_once()


if __name__ == "__main__":
    unittest.main()
