# rbt-so101-dual

通过 RBT 发布 SO101 双臂状态、接收双臂目标，再通过两条串口总线下发 STS3215 关节目标。

需要匹配的 **RBT Python SDK 0.8.0**。先按 SDK 的说明安装对应平台构建，再从本目录安装：

```bash
python -m pip install -e '.[hardware]'
python -m so101_rbt --help
```

命令行默认 mock；硬件模式默认只读，使用 `--enable-goal-writes` 开启目标写入。
驱动读取已有标定，不执行舵机标定或自动使能扭矩。
接线、标定格式和只读启动示例见仓库的 `docs/hardware.md`。

`tests/fixtures` 中的 JSON 为虚构测试数据。
`upstream/motors_bus.py` 仅用于对照 LeRobot 的归一化公式，不在运行时导入。
