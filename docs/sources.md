# Sources and provenance

The two runtime Python packages were copied from the existing SO101 control
and RBT driver implementation at version 0.1.3. Packaging does not change
their solver, mapping, trajectory or serial implementation. Build metadata,
documentation, examples and test fixture paths are prepared for this repository.
The kinematics extra pins urdfdom 4.0.1 and tinyxml2 10.0.0 to match the
Pinocchio 3.8 wheel ABI; a clean install otherwise selected urdfdom 6 and
failed to load `liburdfdom_sensor.so.4.0`.

## Official robot model

- Repository: https://github.com/TheRobotStudio/SO-ARM100
- Path: `Simulation/SO101/so101_new_calib.urdf`
- Git blob SHA-1: `9552a231d8b23bed68ec15779eba620c5d875ec4`
- Local file SHA-256: `3a65d2d35e68a8d2f0c2cc176d19b884506543c93ba72980145b80abe276022c`
- License: Apache-2.0, retained as `packages/so101_control/LICENSE.SO-ARM100`.

The configured end-effector is the `gripper` joint frame. Mesh files referenced
by the URDF are obtained separately from the upstream model.

## LeRobot conversion reference

- Repository: https://github.com/huggingface/lerobot
- Source module: `lerobot/motors/motors_bus.py`
- Frozen file: `packages/rbt_so101_dual/upstream/motors_bus.py`
- SHA-256: `6ce6109e32590ac2830f6995d13bb478b1a984695cc9ca59c0763ff091666393`
- License: Apache-2.0, retained beside the reference file.

The exact upstream commit of this frozen file is not established here; its
content is identified by the hash. Tests extract the original conversion
methods via AST, verify that hash, and compare conversion results exactly.

## Test data

All calibration files in `packages/rbt_so101_dual/tests/fixtures` are synthetic.
They are newly written examples for formula, packet and mock tests.
The repository contains no machine-specific calibration.
