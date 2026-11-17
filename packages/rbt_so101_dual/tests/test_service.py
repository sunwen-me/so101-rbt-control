import unittest
from unittest.mock import patch

import pytest

pytest.importorskip("rbt", reason="RBT Python SDK 0.8.0 is installed separately")

from rbt.time import HostClock, SensorTiming, Timestamp
from so101_rbt.backend import MockBackend
from so101_rbt.service import DriverService, make_command


class TrackingBackend(MockBackend):
    fail_prepare = False
    fail_send = False
    sent = 0

    def prepare_target(self, target):
        if self.fail_prepare:
            raise ValueError("invalid target")
        return super().prepare_target(target)

    def send_prepared(self, prepared):
        if self.fail_send:
            raise ConnectionError("fake bus failure")
        self.sent += 1
        super().send_prepared(prepared)


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.left = TrackingBackend("left")
        self.right = TrackingBackend("right")
        self.driver = DriverService(self.left, self.right, "controller")
        self.clock = HostClock()

    def send(self, *, sequence=1, mutate=None):
        state = self.driver.read_snapshot()
        message = make_command(controller_id="controller", driver_session=self.driver.session,
            sequence=sequence, reference_state_sequence=state["sequence"], left=[1]*6, right=[2]*6)
        stamp = self.clock.monotonic_now()
        message["issued_monotonic_ns"] = stamp.nanoseconds
        timing = SensorTiming.from_legacy(stamp)
        if mutate:
            message, timing = mutate(message, timing)
        self.driver._handle_command(message, timing, self.clock.monotonic_identity)

    def test_mock_target_and_ack(self):
        self.send()
        self.assertEqual(self.driver.last_command_status, "accepted_mock")
        self.assertEqual(self.left.state, (1.0,)*6)
        self.assertEqual(self.right.state, (2.0,)*6)

    def test_both_targets_prevalidated_before_first_send(self):
        self.right.fail_prepare = True
        self.send()
        self.assertEqual(self.left.sent, 0)
        self.assertEqual(self.right.sent, 0)
        self.assertIsNone(self.driver.fault)

    def test_partial_bus_failure_latches_and_blocks_later_motion(self):
        self.right.fail_send = True
        self.send()
        self.assertEqual(self.left.sent, 1)
        self.assertIn("partial_left_sent_right_failed", self.driver.fault)
        self.right.fail_send = False
        self.send(sequence=2)
        self.assertEqual(self.left.sent, 1)
        self.assertEqual(self.right.sent, 0)

    def test_left_failure_does_not_send_right(self):
        self.left.fail_send = True
        self.send()
        self.assertIsNotNone(self.driver.fault)
        self.assertEqual(self.right.sent, 0)

    def test_readonly_never_sends(self):
        self.driver.mode = "readonly"
        self.send()
        self.assertEqual(self.left.sent, 0)
        self.assertIn("read-only", self.driver.last_command_status)

    def test_clock_or_stamp_not_an_extra_gate(self):
        def stamp_mismatch(message, timing):
            message["issued_monotonic_ns"] -= 1
            return message, timing
        self.send(mutate=stamp_mismatch)
        self.assertEqual(self.left.sent, 1)
        def wrong_clock(message, timing):
            from rbt.time import ClockIdentity
            identity = timing.received.clock
            other = ClockIdentity(identity.kind, identity.source_id, identity.generation + 1)
            return message, SensorTiming.from_legacy(Timestamp(other, timing.received.nanoseconds))
        self.send(mutate=wrong_clock)
        self.assertEqual(self.left.sent, 2)

    def test_reference_state_not_an_extra_gate(self):
        def wrong_ref(message, timing):
            message["reference_state_sequence"] = 999999
            return message, timing
        self.send(mutate=wrong_ref)
        self.assertEqual(self.left.sent, 1)

    def test_sequence_is_diagnostic_not_restart_gate(self):
        self.send()
        self.send()
        self.assertEqual(self.left.sent, 2)


if __name__ == "__main__":
    unittest.main()
