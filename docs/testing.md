# 验证记录

本次整理验证的是仓库代码、安装流程和 mock 通信。

| 环境 / 检查 | 结果 |
| --- | --- |
| Python 3.12，已有 RBT SDK 0.8.0，使用本仓库源码 | 59 项测试通过 |
| 新建 Python 3.12 venv，从 wheel 安装，未安装 RBT SDK | 51 项通过，RBT service 测试模块跳过 |
| 跨进程 RBT mock：读取 → 规划 → 发布 → 状态回传 | 通过，9 个目标发布，末端目标状态一致 |
| 离线 FK/IK 和 30 Hz 轨迹示例 | 通过 |
| 两个包构建 wheel | 通过，包含许可证及运动学模型配置 |
| 与整理前运行时源码比较 | 20 个 Python 文件逐字节一致 |

测试使用虚构标定，串口测试使用 fake reader/writer。本次没有连接实机或发送硬件动作。

## 重跑

按根目录 README 安装后：

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q
python examples/offline_fk_ik.py
python examples/joint_trajectory.py
```

安装 RBT SDK 后再运行：

```bash
python examples/mock_dual_arm.py
```

GitHub Actions 自动运行不需要 RBT SDK 的测试和两个离线示例。
