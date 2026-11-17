# Pinocchio Box-QP IK

入口：`so101_control.pinocchio_ik.PinocchioBoxIK`。

## 每次求解做什么

1. 从种子关节角计算当前末端 FK 和 `LOCAL_WORLD_ALIGNED` 雅可比。
2. 计算目标位置误差和旋转向量误差，以权重缩放误差和雅可比的对应行。
3. 加阻尼，构建一个带关节增量边界的小型二次问题。
4. 用投影梯度法近似求 Box-QP；用回溯步长逐个计算候选姿态的 FK。
5. 保留加权残差目标值更小的候选解，继续迭代，返回最佳可用解和诊断。

这是一套局部数值求解器。SO101 只有五个运动自由度，任意六维目标位姿未必可达。

## 目标与边界

令缩放后的误差为 `e = [wp * position_error, wr * rotation_vector_error]`，相应雅可比为 `Jw`：

```text
H = Jwᵀ Jw + λI
g = Jwᵀ e
minimize  ½ Δqᵀ H Δq − gᵀ Δq
subject to  lower − q ≤ Δq ≤ upper − q
```

关节增量内部用弧度计算。整次求解的局部范围固定为 `URDF 范围 ∩ [seed − 20°, seed + 20°]`，不会随迭代累积扩大。

默认 `wp=1`、`wr=0.01`、阻尼 `λ=0.001`；最多 16 次外层迭代、40 次 QP 投影梯度迭代。`wr` 是行缩放系数，因此旋转项在平方目标中的系数为 `0.0001`。

候选解按 `||wp * position_error||² + ||wr * rotation_error||²` 比较，**不是单独按位置误差排序**。位置与朝向可以有权衡。

## 如何看结果

`solver.last_ik` 包含：

| 字段 | 含义 |
| --- | --- |
| `position_error_mm` | 返回关节角重新做 FK 后的位置残差 |
| `orientation_error_deg` | 朝向残差 |
| `weighted_objective` | 返回解的加权残差目标值 |
| `seed_weighted_objective` | 种子状态的目标值 |
| `max_joint_delta_deg` | 相对种子的最大关节变化 |
| `status` | `converged`、`partial_progress` 或 `no_position_progress` |

`partial_progress` 表示目标值变好，但没有达到收敛阈值。`no_position_progress` 是保留的历史状态名，实际判据仍然是加权总目标。种子可行且没有找到更优候选时返回种子；种子在范围外时按代码中的可行边界处理，局部边界为空则报错。

默认收敛阈值为位置 0.1 mm、朝向 0.1°。实际定位精度还取决于关节标定、连杆模型、TCP 与机构误差。

## 轨迹与边界过渡

`plan_joint_move` 提供线性或 cubic smoothstep 时间曲线；`JointInterpolator` 提供路点插值及可选的五次多项式启停过渡。它们生成离散位置目标，不是带全局速度、加速度和 jerk 上限的时间最优规划器。
