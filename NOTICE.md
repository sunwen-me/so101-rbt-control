# Attribution

This repository packages the SO101 control library and RBT dual-arm driver
extracted from an existing robot application. The runtime control algorithms
are preserved during packaging.

- The driver's calibration conversion and Feetech bus conventions follow
  Hugging Face LeRobot (Apache-2.0). A frozen `motors_bus.py` reference and its
  original license are retained under `packages/rbt_so101_dual/upstream/` for
  exact normalization/unnormalization tests.
- The optional Placo kinematics implementation is adapted from Hugging Face
  LeRobot (Apache-2.0), with local bounded iteration and FK candidate evaluation.
- `so101_new_calib.urdf` is from TheRobotStudio/SO-ARM100,
  `Simulation/SO101/so101_new_calib.urdf` (Apache-2.0).
  Its upstream license is retained as `packages/so101_control/LICENSE.SO-ARM100`.
- The existing Apache-2.0 license and Xiaomi copyright notice are preserved.

Pinocchio, Placo, RBT and the Feetech SDK are dependencies with their own
licenses. The RBT SDK is installed separately.

Reference hashes and source locations are recorded in `docs/sources.md`.
