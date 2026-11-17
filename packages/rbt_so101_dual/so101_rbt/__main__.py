from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path

from .backend import FeetechSTS3215Backend, MockBackend
from .calibration import load_arm_calibration
from .protocol import PROTOCOL
from .service import DriverService


def main() -> None:
    parser = argparse.ArgumentParser(description="SO-101 dual-arm RBT driver (safe mock default)")
    parser.add_argument("--backend", choices=("mock", "hardware"), default="mock")
    parser.add_argument("--left-port", default="/dev/so101_left")
    parser.add_argument("--right-port", default="/dev/so101_right")
    parser.add_argument("--controller-id", default="so101-dual-1")
    parser.add_argument("--left-calibration", type=Path, required=True)
    parser.add_argument("--right-calibration", type=Path, required=True)
    parser.add_argument("--duration", type=float)
    parser.add_argument("--poll-hz", type=float, default=30.0)
    parser.add_argument("--serial-write-timeout-ms", type=float,
                        default=None, help="optional serial TX deadline; SDK default is unlimited")
    parser.add_argument("--num-read-retries", type=int, default=2,
                        help="additional read attempts, matching LeRobot default (2)")
    parser.add_argument("--channel-prefix", default="rbt/so101/dual")
    parser.add_argument("--enable-goal-writes", action="store_true")
    parser.add_argument("--ack-controller-id", default="")
    args = parser.parse_args()

    if args.duration is not None and (not math.isfinite(args.duration) or args.duration <= 0):
        parser.error("duration must be positive and finite")
    if not math.isfinite(args.poll_hz) or not 0 < args.poll_hz <= 200:
        parser.error("poll-hz must be finite in (0, 200]")
    if args.serial_write_timeout_ms is not None and (
            not math.isfinite(args.serial_write_timeout_ms) or args.serial_write_timeout_ms <= 0):
        parser.error("serial-write-timeout-ms must be positive and finite")
    if args.num_read_retries < 0:
        parser.error("num-read-retries must be nonnegative")
    write_timeout = None if args.serial_write_timeout_ms is None else args.serial_write_timeout_ms / 1000
    mode = "mock"
    left_cal = load_arm_calibration(args.left_calibration)
    right_cal = load_arm_calibration(args.right_calibration)

    if args.backend == "hardware":
        mode = "hardware" if args.enable_goal_writes else "readonly"
        if os.path.samefile(args.left_port, args.right_port):
            raise SystemExit("left/right ports must be distinct")
        left = FeetechSTS3215Backend(args.left_port, left_cal, allow_goal_writes=args.enable_goal_writes,
                                    write_timeout_s=write_timeout, num_read_retries=args.num_read_retries)
        try:
            right = FeetechSTS3215Backend(args.right_port, right_cal, allow_goal_writes=args.enable_goal_writes,
                                         write_timeout_s=write_timeout, num_read_retries=args.num_read_retries)
        except BaseException:
            left.close()
            raise
    else:
        left = MockBackend("left")
        right = MockBackend("right")

    print(json.dumps({"event": "driver_start", "protocol": PROTOCOL, "mode": mode,
                      "controller_id": args.controller_id, "writes_enabled": mode == "hardware",
                      "num_read_retries": args.num_read_retries,
                      "serial_write_timeout_ms": args.serial_write_timeout_ms if args.backend == "hardware" else None},
                     separators=(",", ":")), flush=True)
    try:
        DriverService(left, right, args.controller_id, mode=mode, poll_hz=args.poll_hz,
                      channel_prefix=args.channel_prefix).run(duration=args.duration)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
