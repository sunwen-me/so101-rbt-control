from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass
from typing import Any

from rbt.time import HostClock

from .backend import ArmBackend
from .protocol import PROTOCOL, UNITS, ProtocolError, validate_command
from .transport import RbtChannels


@dataclass
class DriverService:
    left: ArmBackend
    right: ArmBackend
    controller_id: str
    mode: str = "mock"
    poll_hz: float = 30.0
    channel_prefix: str = "rbt/so101/dual"

    def __post_init__(self) -> None:
        if not self.controller_id or not math.isfinite(self.poll_hz) or not 0 < self.poll_hz <= 200:
            raise ValueError("controller_id and finite poll_hz in (0, 200] required")
        if self.mode not in ("mock", "readonly", "hardware"):
            raise ValueError("unknown driver mode")
        self.session = uuid.uuid4().hex
        self.clock = HostClock()
        self.state_sequence = 0
        self.command_sequence = 0
        self.last_command_status = "no_command"
        self.fault: str | None = None

    def read_snapshot(self) -> dict[str, Any]:
        arms = {}
        intervals = {}
        for name, backend in (("left", self.left), ("right", self.right)):
            start = self.clock.monotonic_now().nanoseconds
            values = backend.read_state()
            end = self.clock.monotonic_now().nanoseconds
            if len(values) != 6 or not all(math.isfinite(v) for v in values):
                raise ValueError(f"{name} backend returned an invalid state")
            arms[name] = list(values)
            intervals[name] = {"start_ns": start, "end_ns": end}
        self.state_sequence += 1
        return {
            "protocol": PROTOCOL, "kind": "state", "units": UNITS,
            "controller_id": self.controller_id, "driver_session": self.session,
            "sequence": self.state_sequence, "monotonic_ns": self.clock.monotonic_now().nanoseconds,
            "mode": self.mode, "arms": arms, "read_intervals": intervals,
            "capture_time_known": False,
        }

    def run(self, *, duration: float | None = None) -> None:
        if duration is not None and (not math.isfinite(duration) or duration <= 0):
            raise ValueError("duration must be positive and finite")
        channels = None
        started = time.monotonic()
        period = 1.0 / self.poll_hz
        try:
            channels = RbtChannels(subscribe_commands=True, publish_states=True,
                                   channel_prefix=self.channel_prefix)
            while duration is None or time.monotonic() - started < duration:
                cycle_started = time.monotonic()
                # Never send a target unless BOTH current serial reads succeeded.
                try:
                    state = self.read_snapshot()
                except Exception as exc:
                    self.fault = f"state_read_failed:{type(exc).__name__}:{exc}"
                    channels.publish_state({
                        "protocol": PROTOCOL, "kind": "fault", "driver_session": self.session,
                        "fault": self.fault, "mode": self.mode,
                    }, self.state_sequence + 1)
                    raise
                try:
                    received = channels.receive_command()
                    if received is not None:
                        message, timing = received
                        self._handle_command(message, timing, channels.clock.monotonic_identity)
                except ProtocolError as exc:
                    self.last_command_status = f"rejected:{exc}"
                state.update(last_command_sequence=self.command_sequence,
                             last_command_status=self.last_command_status, fault=self.fault)
                channels.publish_state(state, self.state_sequence)
                remaining = period - (time.monotonic() - cycle_started)
                if remaining > 0:
                    time.sleep(remaining)
        finally:
            try:
                if channels is not None:
                    channels.close()
            finally:
                try:
                    self.left.close()
                finally:
                    self.right.close()

    def _handle_command(self, message: dict[str, Any], timing: Any, expected_clock: Any) -> None:
        try:
            if self.fault is not None:
                raise ProtocolError(f"latched fault; restart and diagnose before motion: {self.fault}")
            now = self.clock.monotonic_now().nanoseconds
            left, right = validate_command(message, controller_id=self.controller_id,
                driver_session=self.session, sequence_after=self.command_sequence, now_ns=now)
            if self.mode == "readonly":
                raise ProtocolError("driver is read-only")
            # Fully prepare BOTH targets before any bus write.
            prepared_left = self.left.prepare_target(left)
            prepared_right = self.right.prepare_target(right)
            self.command_sequence = message["sequence"]
        except (ProtocolError, ValueError, PermissionError) as exc:
            self.last_command_status = f"rejected:{type(exc).__name__}:{exc}"
            return
        try:
            self.left.send_prepared(prepared_left)
        except Exception as exc:
            self.fault = f"left_send_failed_or_uncertain:{type(exc).__name__}:{exc}"
            self.last_command_status = self.fault
            return
        try:
            self.right.send_prepared(prepared_right)
        except Exception as exc:
            self.fault = f"partial_left_sent_right_failed_or_uncertain:{type(exc).__name__}:{exc}"
            self.last_command_status = self.fault
            return
        self.last_command_status = "accepted_mock" if self.mode == "mock" else "sent_unconfirmed"


def make_command(*, controller_id: str, driver_session: str, sequence: int,
                 reference_state_sequence: int, left: tuple[float, ...] | list[float],
                 right: tuple[float, ...] | list[float], valid_for_ms: int | None = None) -> dict[str, Any]:
    if type(sequence) is not int or sequence < 1:
        raise ValueError("sequence must be a positive integer")
    return {
        "protocol": PROTOCOL, "kind": "command", "units": UNITS,
        "controller_id": controller_id, "driver_session": driver_session,
        "sequence": sequence, "reference_state_sequence": reference_state_sequence,
        "targets": {"left": list(left), "right": list(right)},
    }
