"""Preview joint trajectories or perform offline FK; never publish commands."""
import argparse
import json
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    plan = commands.add_parser("plan", help="write a native joint trajectory preview")
    plan.add_argument("--start", type=Path, required=True)
    plan.add_argument("--target", type=Path, required=True)
    plan.add_argument("--duration", type=float, required=True)
    plan.add_argument("--hz", type=float, default=30.0)
    plan.add_argument("--profile", choices=("linear", "smoothstep"), default="linear")
    plan.add_argument("--gripper", choices=("hold", "linear", "step"), default="hold")
    plan.add_argument("--output", type=Path, required=True)
    fk = commands.add_parser("fk", help="calculate poses in each arm URDF base frame")
    fk.add_argument("--config", type=Path, required=True)
    fk.add_argument("--state", type=Path, required=True)
    args = parser.parse_args()
    if args.command == "plan":
        from .trajectory import plan_joint_move
        trajectory = plan_joint_move(json.loads(args.start.read_text()),
            json.loads(args.target.read_text()), args.duration, hz=args.hz,
            profile=args.profile, gripper=args.gripper)
        args.output.write_text(json.dumps(trajectory.to_dict(), indent=2) + "\n")
        print(json.dumps(dict(samples=len(trajectory), duration_s=trajectory.duration_s, commands_sent=0)))
    else:
        from .kinematics import DualKinematics
        poses = DualKinematics.from_config(args.config).forward(json.loads(args.state.read_text()))
        print(json.dumps({s: t.tolist() for s, t in poses.items()}, indent=2))

if __name__ == "__main__":
    main()

