# Tiltquad / MCTilt 修改摘要

本文概括两个倾转构型的代码修改流程、起飞前设置及主要自定义参数，具体数值以当前源码和实际机械配置为准。

## 1. 代码修改工作流程

1. **区分构型与舵机映射**：Tiltquad 使用双轴映射，每个电机对应 Roll、Pitch 两个舵机；MCTilt 恢复单轴映射，每个电机对应一个 Pitch 舵机，并分别配置机架文件和 QGC 参数页面。
2. **实现电机与舵机联合分配**：Tiltquad 分配三轴力和三轴力矩；MCTilt 分配前向力、升力及姿态力矩，侧向移动仍依靠机体滚转。两者都加入阻尼、限幅、异常保护和限速后指令反馈，并减少迭代计算量。
3. **打通位置与姿态控制**：位置摇杆负责移动，AUX 通道独立给定机体姿态，位置环通过倾转推力补偿。随后修正 Tiltquad 双轴变姿时的航向耦合，以及 MCTilt 大俯仰时的滚转计算。
4. **结合日志逐项优化**：增加电机均衡、舵机分工和姿态输入滤波；将 MCTilt 的机械回中改为合力方向参考，减少倾斜悬停时的分配冲突，并增加滚转优先、偏航积分释放选项。
5. **扩展 MCTilt Offboard 测试**：加入“位置轨迹＋独立俯仰”输入、限角、限速及超时回正；机载包统一为 `mctilt_offboard`，支持八字轨迹和原地变姿两种测试。
6. **按同一流程验证**：代码修改 → 定向测试与日志回放 → 固件编译 → 拆桨检查 → 小倾角实测 → 根据日志继续调整。

主要代码位于 `src/modules/control_allocator/`、`mc_pos_control/`、`mc_att_control/`、`mc_rate_control/`，Offboard 接口涉及 `mavlink/`、消息和日志模块。

## 2. 两个构型的起飞前设置

先选对机架，再核对下表。机架脚本使用 `param set-default`，**刷写固件不会自动覆盖已保存的参数**。

| 项目 | Tiltquad（双轴） | MCTilt（单轴） |
|---|---|---|
| 机架 / `SYS_AUTOSTART` | [14002_tiltquad](ROMFS/px4fmu_common/init.d/airframes/14002_tiltquad) / `14002` | [14003_single_axis_tiltquad](ROMFS/px4fmu_common/init.d/airframes/14003_single_axis_tiltquad) / `14003` |
| 构型 `CA_AIRFRAME` | `16` | `8` |
| 电机 / 舵机数量 | `CA_ROTOR_COUNT=4`，`CA_SV_TL_COUNT=8` | `CA_ROTOR_COUNT=4`，`CA_SV_TL_COUNT=4` |
| 电机与舵机关联 | 旋翼 0～3 的 `CA_ROTORx_TR/TP` 分别为 `1、2、3、4`，对应四组 Roll/Pitch 舵机对 | 旋翼 0～3 的 `CA_ROTORx_TILT` 分别为 `1、2、3、4` |
| 舵机顺序与控制类型 | TL0/2/4/6 为 Roll：`CT=6`（Yaw and Roll）；TL1/3/5/7 为 Pitch：`CT=2` | TL0～3 均为 Pitch：`CT=3`（Yaw and Pitch） |
| 倾转轴与正方向 | Roll 舵机：`AX=1、TD=90`；Pitch 舵机：`AX=0、TD=0` | 全部 `AX=0、TD=0` |
| 舵机行程 | TL0～7 的 `MINA=-20、MAXA=20` | TL0～3 的 `MINA=-20、MAXA=20` |
| 舵机限速 | `CA_SV0_SLEW`～`CA_SV7_SLEW=1.5 s` | `CA_SV0_SLEW`～`CA_SV3_SLEW=1.5 s` |
| AUX 变姿起步设置 | `MC_AUX_ATT_EN=1`，`MC_AUX_ROLL_MAX=5°`，`MC_AUX_PITCH_MAX=5°` | `MC_AUX_ATT_EN=1`，`MC_AUX_ROLL_MAX=0°`，`MC_AUX_PITCH_MAX=5°` |
| AUX 平滑滤波 | `MC_AUX_ATT_TAU=0.30 s` | `MC_AUX_ATT_TAU=0.30 s` |

表中 `CT/AX/TD/MINA/MAXA` 是 `CA_SV_TLx_` 参数后缀；`TD=0` 表示正倾转朝机体前方，`TD=90` 表示朝右。行程和限速是机架默认值，应与实际机构一致。`CA_SVx_SLEW` 表示走完整个行程至少需要的秒数，**数值越大越慢**。

起飞前重点确认：

- **遥控器**：Tiltquad 将 `RC_MAP_AUX5/AUX6` 分别映射到滚转、俯仰旋钮；MCTilt 使用 `RC_MAP_AUX6`。填写实际接收机通道号，AUX5/AUX6 不等于固定的第 5/6 通道。起飞时旋钮回中。
- **执行器**：拆桨检查电机顺序、旋向、舵机正反向、零位及行程；`CA_ROTORx_PX/PY/PZ/KM` 按实际几何和旋向核对，不能直接照搬机架示例尺寸。
- **首次基线**：保留已验证的 PID；Tiltquad 的 `CA_TQ_MBAL=0`，MCTilt 的 `CA_MCT_MBAL=0、CA_MCT_RPRI=0、MC_MCT_YAW_AW=0`，`CA_MCT_SREF=0.20`。普通 AUX 测试保持 `MC_MCT_OBP_EN=0`。
- **测试顺序**：先在 Position 模式稳定悬停，再小角度变姿，观察位置、姿态、电机最低指令及分配残差。机体姿态限角与舵机机械行程是不同参数；自稳模式下变姿不会自动保持位置。

## 3. 主要自定义参数

### 共用姿态输入

| 参数 | 软件默认值 | 作用 |
|---|---|---|
| `MC_AUX_ATT_EN` | `0` | AUX 姿态输入开关，设 `1` 启用 |
| `MC_AUX_ROLL_MAX` | `0°` | AUX5 最大机体滚转角；MCTilt 位置模式不使用 AUX5 |
| `MC_AUX_PITCH_MAX` | `0°` | AUX6 最大机体俯仰角 |
| `MC_AUX_ATT_TAU` | `0.30 s` | 姿态输入平滑时间，越大越平缓，`0` 关闭滤波 |

### 分配与稳定性优化

| 参数 | 适用构型 | 软件默认值 | 作用 |
|---|---|---|---|
| `CA_TQ_MBAL` | Tiltquad | `0` | 用舵机分担部分偏航任务，减小较大的电机指令差；需四个内层舵机 `CT=6` |
| `CA_TQ_PIT_DW` | Tiltquad | `0.20` | 减少外层 Pitch 舵机的差动，让内层优先承担偏航；不削弱共同俯仰，设 `1` 恢复等权 |
| `CA_MCT_SREF` | MCTilt | `0.20 1/s` | 让舵机缓慢趋向期望合力方向，减少不必要角差；设 `0` 关闭，不恢复旧机械回中 |
| `CA_MCT_MBAL` | MCTilt | `0` | 电机均衡增益，单位 `1/s`，减小持续差速和低端余量不足 |
| `CA_MCT_RPRI` | MCTilt | `0` | 大倾转且电机余量不足、滚转欠分配时，适当降低偏航权重以优先保持滚转 |
| `MC_MCT_YAW_AW` | MCTilt | `0` | 滚转、偏航同时欠分配且电机接近低端时，缓慢释放相关偏航积分，单位 `1/s` |

MCTilt 上述分配优化针对四电机、四个独立 `Yaw and Pitch` 舵机。均衡是软目标，不保证电机等速，也不设置电机硬下限。

历史专项试验设置：Tiltquad 均衡建议从 `CA_TQ_MBAL=0.03` 验证；MCTilt 2026-10-23 的复测组合为 `CA_MCT_SREF=0.20、CA_MCT_MBAL=0.30`。这些是待实机验证的试验值，首次基线仍按上一节设置，逐项开启并比较日志。

## 4. MCTilt Offboard 设置与验证范围

仅适用于四电机、四舵机 MCTilt。落地、未解锁时设置以下参数；普通 Position 模式仍使用 AUX6。

| 参数 | 软件默认值 | Offboard 测试设置 / 作用 |
|---|---|---|
| `MC_MCT_OBP_EN` | `0` | 设 `1` 启用外部独立俯仰；位置目标需持续单独发送 |
| `MC_MCT_OBP_MAX` | `20°` | 外部机体俯仰限角，独立于 AUX6 限角及舵机行程 |
| `MC_MCT_OBP_RATE` | `5°/s` | 俯仰参考最大变化速度，也用于超时回正和接回 AUX6 |
| `MC_MCT_OBP_TMO` | `0.5 s` | 俯仰流超时后告警并平滑回正；位置流失联仍按原 Offboard 策略处理 |

机载包为 `mctilt_offboard`：`pitch_hold.launch` 用于原地变姿，`figure8.launch` 用于八字轨迹，两者只运行一个。脚本默认俯仰为 `10°`，应按已验证范围调整。流程为 **Position 稳定悬停 → 启动脚本 → 遥控器切 Offboard → 执行测试 → 回正并由飞手退出**；脚本不自动解锁、切模式或起降。

MCTilt 的 Roll 仍负责侧向位置控制，不能作为完全独立的六自由度构型。详细配置与操作见 [MCTILT_OFFBOARD.md](mctilt_offboard/MCTILT_OFFBOARD.md)。

历史记录已有 CUAV X7 Pro / 7 Nano 的阶段性固件编译、C++ / Python 定向测试和日志回放结果；完整 SITL 曾受 Micro-CDR 下载问题阻断。离线结果不等于实机验证，大倾角稳定性和新 Offboard 链路仍需实测确认。本次仅整理文档。
