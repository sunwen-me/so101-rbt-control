from __future__ import annotations

import json
import math
from typing import Any

PROTOCOL = "rbt.so101.dual.v1"
CHANNEL_STATE = "rbt/so101/dual/state/v1"
CHANNEL_COMMAND = "rbt/so101/dual/command/v1"
MAX_COMMAND_BYTES = 4096
UNITS = "joint_deg_gripper_percent"


class ProtocolError(ValueError):
    pass


def _unique_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result:
            raise ProtocolError("duplicate JSON key")
        result[key] = value
    return result


def _reject_constant(value: str) -> None:
    raise ProtocolError(f"invalid JSON constant: {value}")


def encode_message(message: dict[str, Any]) -> bytes:
    return json.dumps(message, allow_nan=False, separators=(",", ":"), sort_keys=True).encode("utf-8")


def decode_message(payload: bytes | bytearray | memoryview) -> dict[str, Any]:
    if len(payload) > MAX_COMMAND_BYTES:
        raise ProtocolError("message too large")
    try:
        value = json.loads(bytes(payload), object_pairs_hook=_unique_object,
                           parse_constant=_reject_constant)
    except (UnicodeDecodeError, ValueError, RecursionError) as exc:
        raise ProtocolError("invalid JSON payload") from exc
    if not isinstance(value, dict) or value.get("protocol") != PROTOCOL:
        raise ProtocolError("protocol/version mismatch")
    return value


def validate_command(
    message: dict[str, Any],
    *,
    controller_id: str,
    driver_session: str,
    sequence_after: int,
    now_ns: int,
    max_age_ns: int = 500_000_000,
) -> tuple[tuple[float, ...], tuple[float, ...]]:
    if message.get("protocol") != PROTOCOL or message.get("units") != UNITS:
        raise ProtocolError("command protocol or units mismatch")
    if message.get("kind") != "command":
        raise ProtocolError("expected a command")
    if message.get("controller_id") != controller_id:
        raise ProtocolError("controller identity mismatch")
    sequence = message.get("sequence")
    # Session, timestamps and sequence are diagnostics, not new motion gates.
    if type(sequence) is not int or sequence < 1:
        raise ProtocolError("command sequence must be a positive integer")
    targets = message.get("targets")
    if not isinstance(targets, dict) or set(targets) != {"left", "right"}:
        raise ProtocolError("targets must contain left and right arms")
    result = []
    for arm in ("left", "right"):
        values = targets[arm]
        if not isinstance(values, list) or len(values) != 6:
            raise ProtocolError(f"{arm} target must have six values")
        try:
            finite = all(type(v) in (int, float) and math.isfinite(v) for v in values)
        except OverflowError:
            finite = False
        if not finite:
            raise ProtocolError(f"{arm} target contains non-finite or non-numeric values")
        result.append(tuple(float(v) for v in values))
    return result[0], result[1]
