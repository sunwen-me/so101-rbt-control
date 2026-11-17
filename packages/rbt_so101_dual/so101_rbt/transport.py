from __future__ import annotations

from typing import Any

from rbt import StreamProfile
from rbt.time import HostClock, SensorTiming
from rbt.time_stream import (
    StampedBytesPublisher,
    StampedBytesSubscriber,
    stamped_bytes_descriptor,
)

from .protocol import CHANNEL_COMMAND, CHANNEL_STATE, decode_message, encode_message


class RbtChannels:
    """RBT/iceoryx2 stamped-byte channels; not a Composer generic message port."""

    def __init__(
        self,
        *,
        subscribe_commands: bool = False,
        publish_states: bool = False,
        publish_commands: bool = False,
        subscribe_states: bool = False,
        channel_prefix: str = "rbt/so101/dual",
    ) -> None:
        if not channel_prefix or channel_prefix.endswith("/"):
            raise ValueError("channel_prefix must be nonempty without a trailing slash")
        self.clock = HostClock()
        self.command_subscriber = None
        self.command_publisher = None
        self.state_publisher = None
        self.state_subscriber = None
        try:
            if subscribe_commands:
                descriptor = stamped_bytes_descriptor(f"{channel_prefix}/command/v1", profile=StreamProfile.CONTROL)
                self.command_subscriber = StampedBytesSubscriber.open(descriptor)
            if publish_commands:
                descriptor = stamped_bytes_descriptor(f"{channel_prefix}/command/v1", profile=StreamProfile.CONTROL)
                self.command_publisher = StampedBytesPublisher.open(descriptor, maximum_payload_bytes=4096)
            if publish_states:
                descriptor = stamped_bytes_descriptor(f"{channel_prefix}/state/v1", profile=StreamProfile.SENSOR)
                self.state_publisher = StampedBytesPublisher.open(descriptor, maximum_payload_bytes=4096)
            if subscribe_states:
                descriptor = stamped_bytes_descriptor(f"{channel_prefix}/state/v1", profile=StreamProfile.SENSOR)
                self.state_subscriber = StampedBytesSubscriber.open(descriptor)
        except BaseException:
            self.close()
            raise

    def receive_command(self, timeout: float = 0.0) -> tuple[dict[str, Any], Any] | None:
        if self.command_subscriber is None:
            return None
        sample = self.command_subscriber.receive(timeout=timeout) if timeout else self.command_subscriber.try_receive()
        if sample is None:
            return None
        try:
            view = sample.payload_view()
            try:
                message = decode_message(view)
            finally:
                view.release()
            from .protocol import ProtocolError
            if type(message.get("sequence")) is not int or message["sequence"] != sample.sequence:
                raise ProtocolError("RBT sequence differs from JSON command sequence")
            return message, sample.timing
        finally:
            # The application payload is copied/decoded above; release the borrowed sample now.
            del sample

    def publish_state(self, message: dict[str, Any], sequence: int) -> None:
        if self.state_publisher is None:
            return
        self._publish(self.state_publisher, message, sequence, self.clock.monotonic_now())

    def publish_command(self, message: dict[str, Any], sequence: int) -> int:
        if self.command_publisher is None:
            raise RuntimeError("command publisher is not open")
        stamp = self.clock.monotonic_now()
        message["issued_monotonic_ns"] = stamp.nanoseconds
        self._publish(self.command_publisher, message, sequence, stamp)
        return stamp.nanoseconds

    @staticmethod
    def _publish(publisher: Any, message: dict[str, Any], sequence: int, stamp: Any) -> None:
        payload = encode_message(message)
        loan = publisher.loan(SensorTiming.from_legacy(stamp), len(payload), sequence=sequence)
        view = loan.payload_view()
        view[:] = payload
        view.release()
        loan.send()

    def receive_state(self, timeout: float = 0.0) -> dict[str, Any] | None:
        if self.state_subscriber is None:
            return None
        sample = self.state_subscriber.receive(timeout=timeout) if timeout else self.state_subscriber.try_receive()
        if sample is None:
            return None
        try:
            view = sample.payload_view()
            try:
                return decode_message(view)
            finally:
                view.release()
        finally:
            del sample

    def close(self) -> None:
        if self.command_subscriber is not None:
            self.command_subscriber.close()
        if self.command_publisher is not None:
            self.command_publisher.close()
        if self.state_publisher is not None:
            self.state_publisher.close()
        if self.state_subscriber is not None:
            self.state_subscriber.close()
