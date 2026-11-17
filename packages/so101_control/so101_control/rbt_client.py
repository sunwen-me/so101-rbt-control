from __future__ import annotations

import time

import numpy as np
import rbt
from rbt.time import HostClock
from so101_rbt.protocol import UNITS, decode_message
from so101_rbt.service import make_command
from so101_rbt.transport import RbtChannels

from .types import validate_arms


class ArmClient:
    """No serial access: current stamped state in, native joint targets out."""
    def __init__(self, controller_id, *, prefix="rbt/so101/dual", send=False, mock_only=False):
        self.controller_id, self.send, self.mock_only = controller_id, send, mock_only
        self.channels = RbtChannels(publish_commands=True, channel_prefix=prefix)
        self.subscriber = None
        self.prefix = prefix
        self.clock = HostClock()
        self.session = None
        self.sequence = 0
        self.latest = None

    def connect(self, timeout=5.0):
        from rbt import StreamProfile
        from rbt.time_stream import StampedBytesSubscriber, stamped_bytes_descriptor
        deadline = time.monotonic() + timeout
        while self.subscriber is None:
            try:
                self.subscriber = StampedBytesSubscriber.open(stamped_bytes_descriptor(
                    f"{self.prefix}/state/v1", profile=StreamProfile.SENSOR))
            except rbt.RbtError:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(0.05)
        return self.snapshot(timeout=max(0.01, deadline - time.monotonic()))

    def _decode_state(self, sample):
        view = sample.payload_view()
        try:
            state = decode_message(view)
        finally:
            view.release()
        if state.get("fault") or state.get("kind") != "state":
            raise RuntimeError(f"driver fault: {state}")
        if state.get("units") != UNITS or state.get("controller_id") != self.controller_id:
            raise ValueError("driver units/controller mismatch")
        if type(state.get("sequence")) is not int or state["sequence"] != sample.sequence:
            raise ValueError("state sequence differs from native header")
        validate_arms(state["arms"])
        self.session = state["driver_session"]
        if self.mock_only and state["mode"] != "mock":
            raise RuntimeError("mock client refuses any non-mock driver")
        if self.send and state["mode"] not in (("mock",) if self.mock_only else ("hardware",)):
            raise RuntimeError("driver write mode mismatch")
        return state

    def _drain_states(self):
        # Latest-value subscription; timestamps/session are diagnostic metadata.
        # max_borrowed=1: release every sample before obtaining the next loan.
        for _ in range(64):
            sample = self.subscriber.try_receive()
            if sample is None:
                break
            try:
                state = self._decode_state(sample)
                self.latest = state
            finally:
                del sample

    def snapshot(self, timeout=1.0, after_sequence=None):
        """Return latest available state; no age gate or per-command ACK barrier.

        after_sequence is only for callers explicitly requesting a newer sample.
        The startup timeout applies only when no matching sample exists.
        """
        if self.subscriber is None:
            raise RuntimeError("connect first")
        if not np.isfinite(timeout) or timeout < 0:
            raise ValueError("snapshot timeout must be nonnegative and finite")
        deadline = time.monotonic() + timeout
        while True:
            self._drain_states()
            state = self.latest
            if state is not None and (after_sequence is None or state["sequence"] > after_sequence):
                return state
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("no dual-arm state received")
            time.sleep(min(0.001, remaining))

    def send_target(self, targets):
        if not self.send:
            raise PermissionError("client is read-only")
        targets = {s: values.tolist() for s, values in validate_arms(targets).items()}
        self.sequence += 1
        message = make_command(controller_id=self.controller_id, driver_session=self.session,
            sequence=self.sequence, reference_state_sequence=self.latest["sequence"] if self.latest else 0,
            left=targets["left"], right=targets["right"])
        self.channels.publish_command(message, self.sequence)
        # Publication is NOT confirmation that either serial bus sent the goal.
        return {"command_sequence": self.sequence, "status": "published_unconfirmed"}

    def close(self):
        if self.subscriber is not None:
            self.subscriber.close()
        self.channels.close()

