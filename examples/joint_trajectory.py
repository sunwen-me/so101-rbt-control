"""Generate and inspect a 30 Hz joint trajectory without publishing it."""
import json

from so101_control import plan_joint_move


def main():
    start = {side: [0, 0, 0, 0, 0, 20] for side in ("left", "right")}
    target = {"left": [5, -5, 5, 0, 0, 30], "right": start["right"]}
    trajectory = plan_joint_move(start, target, duration_s=2,
                                 hz=30, profile="smoothstep", gripper="linear")
    print(json.dumps({
        "samples": len(trajectory),
        "period_s": trajectory.period_s,
        "duration_s": trajectory.duration_s,
        "first": {side: values.tolist() for side, values in trajectory.samples[0].items()},
        "last": {side: values.tolist() for side, values in trajectory.samples[-1].items()},
    }, indent=2))


if __name__ == "__main__":
    main()
