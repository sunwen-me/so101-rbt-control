"""Run a separate mock RBT driver and verify trajectory feedback."""
import json
from pathlib import Path
import subprocess
import sys
import time
import uuid

import numpy as np

from so101_control import DualArmController
from so101_control.rbt_client import ArmClient


def main():
    root = Path(__file__).resolve().parents[1]
    fixtures = root / "packages/rbt_so101_dual/tests/fixtures"
    prefix = "rbt/so101/example/" + uuid.uuid4().hex
    client = ArmClient("mock-example", prefix=prefix, send=True, mock_only=True)
    driver = None
    try:
        driver = subprocess.Popen([
            sys.executable, "-m", "so101_rbt", "--backend", "mock",
            "--controller-id", "mock-example", "--channel-prefix", prefix,
            "--duration", "5",
            "--left-calibration", str(fixtures / "dual_follower_left.json"),
            "--right-calibration", str(fixtures / "dual_follower_right.json"),
        ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        client.connect(timeout=4)
        controller = DualArmController(client)
        origin = controller.read()
        target = {side: q + np.array([2, -1, 3, -2, 1, 5]) for side, q in origin.items()}
        trajectory = controller.plan_joints(target, 0.3, start=origin,
                                            hz=30, gripper="linear")
        result = controller.execute(trajectory)
        deadline = time.monotonic() + 2
        while True:
            state = client.snapshot()
            if all(np.allclose(state["arms"][side], target[side]) for side in target):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("mock feedback did not reach final target")
            time.sleep(0.01)
        assert result["completed"]
        assert state["mode"] == "mock"
        assert state["last_command_status"] == "accepted_mock"
        assert state["last_command_sequence"] == result["last_publication"]["command_sequence"]
        print(json.dumps({"status": "PASS", "backend": "mock",
                          "execution": result, "feedback": state["arms"]}, indent=2))
    finally:
        client.close()
        if driver is not None:
            try:
                output, _ = driver.communicate(timeout=7)
            except subprocess.TimeoutExpired:
                driver.terminate()
                output, _ = driver.communicate(timeout=3)
            if driver.returncode != 0:
                raise RuntimeError(output)


if __name__ == "__main__":
    main()
