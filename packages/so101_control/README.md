# so101-control

SO101 双臂控制库：关节映射、Pinocchio FK/Jacobian、局部 Box-QP IK、轨迹插值和 RBT 客户端。

安装基础库：`python -m pip install -e .`；安装 FK/IK 依赖：`python -m pip install -e '.[kinematics]'`。

完整示例与算法说明见仓库根目录 README 和 `docs/`。
关节接口使用度，夹爪使用 0–100，末端位姿使用米和 4×4 齐次变换矩阵。
默认模型为 SO-ARM100 的 `so101_new_calib.urdf`，末端帧为 `gripper`。

配置与 URDF 随 wheel 安装到 `share/so101-control/config`；源码安装时可直接使用 `config/board.json`。
