# Tiltquad 修改摘要

## 1. 构型划分

- `CA_AIRFRAME=8`：通用单轴倾转多旋翼，使用 `CA_ROTORx_TILT`。
- `CA_AIRFRAME=16`：双轴倾转四旋翼 Tiltquad，每个旋翼同时配置 Roll-like 和 Pitch-like 两个倾转舵机。
- 双轴舵机顺序为 `[roll0, pitch0, roll1, pitch1, ...]`，旋翼通过 `CA_ROTORx_TR` 和 `CA_ROTORx_TP` 关联对应舵机对。

## 2. 单轴 MCTilt 兼容

- 在公共旋翼几何中恢复 `CA_ROTORx_TILT` 单轴索引。
- 倾转映射分为 `SingleAxis` 和 `DualAxis`，避免 MCTilt、Tiltrotor VTOL 被 Tiltquad 的偶/奇舵机映射影响。
- 新增 `14003_single_axis_tiltquad`，配置为 4 电机、4 个 Pitch-like 单轴舵机，并允许舵机参与 `Yaw and Pitch` 控制。

## 3. Tiltquad 六自由度分配

- 根据每个旋翼的双轴舵机角度实时计算三维推力方向。
- 计算完整机体系六维输出：`Mx/My/Mz/Fx/Fy/Fz`。
- 考虑旋翼位置力矩、螺旋桨反扭矩、电机推力和倾转角之间的非线性耦合。
- 在 `updateSetpoint()` 中使用数值雅可比和高斯-牛顿迭代，联合求解 4 个电机和 8 个倾转舵机。
- 迭代步长仅作为非线性求解的信赖域；实际舵机速度统一由 PX4 标准 `CA_SVx_SLEW` 限制。
- 输出层限速、限幅后的真实指令会反馈给非线性模型，作为下一周期求解初值，避免模型状态与实际指令不同步。
- 使用非线性模型的实际输出回填六轴未分配控制量。

### 单轴 MCTilt 矢量控制

- 当 `CA_AIRFRAME=8` 且配置为 4 电机、4 个倾转舵机时，启用单轴非线性联合分配。
- 联合求解 4 个电机和 4 个俯仰方向倾转舵机，AUX6 独立控制机体 Pitch。
- 可实现固定位置改变 Pitch，以及固定 Pitch 改变前后位置。
- 单轴俯仰构型不能直接产生 `Fy`，因此侧向位置仍需通过机体 Roll 倾斜控制，不属于满秩六自由度构型。
- 舵机速度统一由 `CA_SV0_SLEW` 至 `CA_SV3_SLEW` 限制。

## 4. 位置与姿态解耦

当 `CA_AIRFRAME=16` 时，位置控制器向控制分配器传递完整机体系三维推力：

- 普通 Roll/Pitch 摇杆仍用于位置模式下的水平移动。
- AUX5/AUX6 独立设置机体 Roll/Pitch 姿态偏置。
- 位置环生成的 NED 合力会转换到带姿态偏置的期望机体系。
- 因此可实现位置尽量保持不变时改变姿态，或姿态保持不变时改变位置。

相关参数：

- `MC_AUX_ATT_EN`：启用 AUX 姿态目标。
- `MC_AUX_ROLL_MAX`：AUX5 对应的最大机体滚转角。
- `MC_AUX_PITCH_MAX`：AUX6 对应的最大机体俯仰角。

## 5. QGC 与机架配置

- `Multirotor with Tilt` 页面使用单轴 `CA_ROTORx_TILT`。
- `Tiltquad` 页面使用 `CA_ROTORx_TR/TP`，并显示倾转轴 `CA_SV_TLx_AX`。
- `14002_tiltquad` 中：
  - Roll-like 舵机 `TL0/TL2/TL4/TL6` 设置 `TD=90`，正倾转指向机体右侧 `+Y`。
  - Pitch-like 舵机 `TL1/TL3/TL5/TL7` 设置 `TD=0`，正倾转指向机体前方 `+X`。

## 6. 已完成验证

- `git diff --check` 通过。
- `libmodules__control_allocator.a` 编译通过。
- `libmodules__mc_pos_control.a` 编译通过。

## 7. 实机测试注意事项

- 当前六自由度分配属于实验性实现，首次测试必须拆除螺旋桨检查电机和舵机方向。
- `CA_SV_TLx_TD` 必须与实际机械正倾转方向一致；必要时还需检查输出反向和 `MINA/MAXA`。
- 建议先将 `MC_AUX_ROLL_MAX`、`MC_AUX_PITCH_MAX` 设置为约 `5°`，确认位置补偿正确后逐步增大。
- 可实现姿态范围受舵机倾转范围、电机推力余量及构型几何约束。
- 重点观察 `control_allocator_status.unallocated_thrust`、`unallocated_torque`、舵机饱和和电机饱和情况。
- 机架文件使用 `param set-default`；已有参数不会自动被新默认值覆盖，必要时需在 QGC 中重置或手动更新。
