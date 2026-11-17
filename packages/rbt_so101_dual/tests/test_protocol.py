from __future__ import annotations

import unittest

from so101_rbt.protocol import PROTOCOL, UNITS, ProtocolError, decode_message, validate_command


def command(**overrides):
    value = {
        "protocol": PROTOCOL,
        "units": UNITS,
        "kind": "command",
        "controller_id": "so101-dual-1",
        "driver_session": "session-1",
        "sequence": 1,
        "issued_monotonic_ns": 1_000_000_000,
        "valid_for_ns": 100_000_000,
        "targets": {"left": [0.0] * 6, "right": [0.0] * 6},
    }
    value.update(overrides)
    return value


class ProtocolTests(unittest.TestCase):
    def test_accepts_valid_command(self) -> None:
        targets = validate_command(
            command(), controller_id="so101-dual-1", driver_session="session-1", sequence_after=0,
            now_ns=1_050_000_000,
        )
        self.assertEqual(targets, ((0.0,) * 6, (0.0,) * 6))

    def test_controller_routing_is_retained(self) -> None:
        with self.assertRaises(ProtocolError):
            validate_command(command(controller_id="other"),controller_id="so101-dual-1",
                driver_session="session-1",sequence_after=0,now_ns=1_010_000_000)

    def test_session_order_and_expiry_are_not_extra_gates(self) -> None:
        cases = (
            (command(driver_session="old"), {"controller_id": "so101-dual-1", "driver_session": "session-1", "sequence_after": 0, "now_ns": 1_010_000_000}),
            (command(sequence=1), {"controller_id": "so101-dual-1", "driver_session": "session-1", "sequence_after": 1, "now_ns": 1_010_000_000}),
            (command(), {"controller_id": "so101-dual-1", "driver_session": "session-1", "sequence_after": 0, "now_ns": 1_200_000_000}),
            (command(), {"controller_id": "so101-dual-1", "driver_session": "session-1", "sequence_after": 0, "now_ns": 999_999_999}),
        )
        for message, kwargs in cases:
            with self.subTest(message=message, kwargs=kwargs):
                self.assertEqual(validate_command(message, **kwargs),((0.,)*6,(0.,)*6))

    def test_rejects_nonfinite_wrong_dimensions_and_bool(self) -> None:
        for left in ([float("nan")] * 6, [0.0] * 5, [True, 0, 0, 0, 0, 0]):
            msg = command(targets={"left": left, "right": [0.0] * 6})
            with self.subTest(left=left), self.assertRaises(ProtocolError):
                validate_command(msg, controller_id="so101-dual-1", driver_session="session-1", sequence_after=0, now_ns=1_010_000_000)

    def test_decoder_rejects_duplicate_keys_and_nonstandard_constants(self) -> None:
        for payload in (
            b'{"protocol":"rbt.so101.dual.v1","kind":"state","kind":"command"}',
            b'{"protocol":"rbt.so101.dual.v1","value":NaN}',
            b'{"protocol":"rbt.so101.dual.v1","value":Infinity}',
        ):
            with self.subTest(payload=payload), self.assertRaises(ProtocolError):
                decode_message(payload)


if __name__ == "__main__":
    unittest.main()
