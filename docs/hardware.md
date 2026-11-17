# 接入真实双臂

## 设备与依赖

- 两个 SO101 follower 舵机组，每臂 STS3215，电机 ID 按关节顺序为 1–6。
- 两条独立串口，波特率 1,000,000。
- 对应平台的 RBT Python SDK 0.8.0，以及驱动的 `hardware` 依赖。
- 本机左右臂各自的 LeRobot 风格标定 JSON。

串口建议使用稳定的 `/dev/serial/by-id/...` 或自定义 udev 别名，避免重插后设备编号变化。

## 标定与 URDF 映射

标定 JSON 的键必须按以下顺序排列：

```text
shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper
```

每项包含整数 `id`、`drive_mode`、`homing_offset`、`range_min` 和 `range_max`。仓库 `tests/fixtures` 仅展示格式并服务于测试；真实机器使用自己测得的文件，放在被 git 忽略的 `local/` 下即可。

前五个关节的换算为：

```text
degree = (raw − (range_min + range_max)/2) × 360/4095
raw_target = int(degree × 4095/360 + (range_min + range_max)/2)
```

夹爪按标定范围换算为 0–100，转换中处理 `drive_mode`，并裁剪到该范围。角度模式不裁剪到标定采样范围；标定采样范围与 URDF 运动学限位是两件事。

`homing_offset` 被读入标定对象，但运行时不会再次加到上述公式，也不会写入电机。电机里的 homing 配置需要与用于读取的标定匹配。

默认 `config/board.json` 使用官方模型与零偏置映射。它不会自动推断每台实机的机械零位。可复制该文件，为左右臂分别设置 `signs`、`offsets_deg`、URDF 和末端帧。

URDF 来自 SO-ARM100，此处只打包用于运动学的 XML，未包含它引用的 STL 网格。FK/IK 不需要这些网格；渲染或碰撞模型需要从上游另取资源。

## 先启动只读驱动

以下命令在仓库根目录运行，替换串口路径：

```bash
python -m so101_rbt --backend hardware \
  --left-port /dev/so101_left \
  --right-port /dev/so101_right \
  --left-calibration local/left.json \
  --right-calibration local/right.json \
  --poll-hz 30
```

另一个进程读取：

```python
from so101_control.rbt_client import ArmClient

client = ArmClient("so101-dual-1")
try:
    print(client.connect()["arms"])
finally:
    client.close()
```

驱动开启目标写入需显式增加 `--enable-goal-writes`，客户端需 `send=True`。本驱动不自动标定、回零或使能扭矩；这些由设备启动流程负责。确认当前姿态和目标一致、活动区域无障碍后再执行应用轨迹。

串口完整状态读取失败后默认额外重试两次；目标写入失败不自动重试，以免重复发送结果不明确的动作。
