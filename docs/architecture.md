# 架构与数据流

```text
应用：关节目标 / 末端位姿序列
        │
        ▼
so101-control
  JointMapping → FK / Pinocchio Box-QP IK → 关节路点
  关节插值 → 定时轨迹执行 → ArmClient
        │  RBT 目标消息
        ▼
rbt-so101-dual
  DriverService → 标定反换算 → 左右 STS3215 串口
        ▲
        │  编码器读数 → 标定换算 → RBT 状态
        └───────────────────────────── ArmClient
```

## 控制库

`DualArmController` 把状态读取、轨迹规划和执行组合起来。直接给关节目标时不需要 IK；给末端路点时，按顺序求解，每个路点以上一解作为种子，再插值执行。

`ArmKinematics` 管理单臂映射和求解器；`DualKinematics` 管理两套独立的模型与可变计算数据。左右基座之间的外参由应用提供，库不会自动把它们放进一个世界坐标系。

`JointTrajectory` 保存双臂目标和采样间隔。`execution.py` 按绝对截止时间发送；调用过慢时跳过已经过期的中间采样，避免补发一串积压目标。

## RBT 接口

默认 channel 前缀为 `rbt/so101/dual`。具体状态、目标通道及 stamped bytes 描述见 `transport.py`。payload 使用 JSON 编码，包含 controller、sequence、单位和双臂向量。

客户端读取最新可用状态，不为每个目标等待 ACK。目标发布成功表示消息已发布，不能直接当作舵机已到位。

驱动先准备两臂的目标包，再依次发送左右串口。两条总线不是硬件原子事务；若只发出一臂便出现故障，service 会记录并锁存该部分发送故障。

## 时间尺度

路点频率和舵机目标发布频率分别设置。例如：

```python
trajectory_from_waypoints(start, points, waypoint_period_s=0.1, multiplier=3)
```

表示路点间隔 100 ms，分成三个采样，即 30 Hz 发布。30 个路点对应 3 秒轨迹。若路点本来来自 30 FPS 数据，应显式设置 `waypoint_period_s=1/30`；插值倍数只提高发布密度，不改变总时长。

这里的 Hz 是目标发布频率，不是舵机固件内部控制环频率。
