# SO101 RBT Control

一套 SO101 双臂机械臂控制代码，分为两个 Python 包：

| 包 | 功能 |
| --- | --- |
| `so101-control` | FK/IK、关节坐标映射、轨迹规划与插值、RBT 控制客户端 |
| `rbt-so101-dual` | 双串口舵机读写、标定换算、RBT 状态发布与目标接收 |

默认 IK 使用 **Pinocchio FK/Jacobian + 阻尼最小二乘 + Box-QP**，单次求解的局部范围为种子关节角 ±20°，同时受 URDF 关节范围约束。左右臂各用一套独立求解器。

## 安装

Python 3.10+；测试环境为 Python 3.12。

```bash
git clone https://github.com/sunwen-me/so101-rbt-control.git
cd so101-rbt-control
python -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e './packages/so101_control[kinematics]'
```

只做关节轨迹规划时，安装 `./packages/so101_control` 即可。

使用 RBT 通信前，先安装适合本机平台的 **RBT Python SDK 0.8.0**，再安装驱动：

```bash
python -m pip install -e './packages/rbt_so101_dual[hardware]'
```

RBT SDK 需从自己的 SDK 发行包或源码构建安装。

## 先跑离线示例

```bash
python examples/offline_fk_ik.py
python examples/joint_trajectory.py
```

第一个示例从已知关节角做 FK，稍微改变目标后求 IK，打印关节角和实际 FK 残差；第二个生成 30 Hz 关节轨迹并打印首末采样。

```python
from so101_control import plan_joint_move

start = {"left": [0, 0, 0, 0, 0, 20], "right": [0, 0, 0, 0, 0, 20]}
goal = {"left": [5, -5, 5, 0, 0, 30], "right": [0, 0, 0, 0, 0, 20]}
trajectory = plan_joint_move(start, goal, duration_s=2.0,
                             hz=30, profile="smoothstep", gripper="linear")
print(len(trajectory), trajectory.duration_s)  # 60, 2.0
```

## RBT 双臂 mock

安装两个包及 RBT SDK 后：

```bash
python examples/mock_dual_arm.py
```

示例在独立 topic 前缀下启动 mock 驱动，读取双臂状态、执行一小段轨迹，并比较回传目标。它使用仓库内的虚构标定文件。

真实机械臂的串口、标定与只读启动方式见 [硬件接入](docs/hardware.md)。

## 数据约定

- 每臂六个值依次为 `shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper`。
- 前五项单位为度；夹爪为 0–100，开合方向由该臂标定决定。
- 末端位姿为 4×4 齐次矩阵，位置单位为米，分别表达在各自 URDF 基坐标系中。
- 默认模型为 SO-ARM100 的 `so101_new_calib.urdf`，关节映射为零偏置、正方向不变。
- 默认末端帧 `gripper` 位于夹爪关节处。需要指尖夹持点时，应另行定义 TCP 变换。

## 读代码的顺序

1. [架构和数据流](docs/architecture.md)：看应用、控制库、驱动如何配合。
2. `mapping.py`、`calibration.py`：看关节角、编码器和 URDF 如何换算。
3. [IK 算法](docs/ik.md) 和 `pinocchio_ik.py`：看雅可比、局部边界和候选解选择。
4. `trajectory.py`、`execution.py`：看轨迹怎样变成定时目标。
5. `rbt_client.py`、`service.py`、`backend.py`：看目标怎样到达舵机。

源码分别位于 `packages/so101_control/so101_control/` 和 `packages/rbt_so101_dual/so101_rbt/`。

## 测试

```bash
python -m pip install pytest
python -m pip install --no-deps -e ./packages/rbt_so101_dual
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
```

测试覆盖轨迹时序、坐标映射、FK/IK、协议、串口重试和 LeRobot 换算公式的一致性。未安装 RBT SDK 时，RBT service 测试会明确跳过；安装后可另跑上面的跨进程 mock 示例。

已执行的验证见 [测试记录](docs/testing.md)。

URDF 的来源与哈希见 [sources](docs/sources.md)，许可证为 [Apache-2.0](LICENSE)。
