# MCTilt：位置＋独立俯仰同步规划

仅用于本次修改后的四电机/四舵机 `CA_AIRFRAME=8` 固件。没有修改 Tiltquad、PID 或分配器。
根目录及 ROS 包名为 **mctilt_offboard**，不是 `Tools/` 旧副本。
入口为 `scripts/offboard_figure8.py` 和 `scripts/offboard_pitch_hold.py`；公共实现由 `setup.py` 安装在 `src/mctilt_offboard/`，兼容 catkin devel/install。

## 接口约定

- `mavros/setpoint_raw/local`：位置、可选速度/加速度前馈、偏航。
- `mavros/setpoint_raw/attitude`：mask=71（忽略全部角速度和推力）的四元数，仅提取机体俯仰。飞控不把它当作纯姿态 Offboard，不覆盖位置环推力，也不更新 Offboard 心跳。
- MCTilt 的 Roll 仍由侧向位置控制计算，Yaw 由位置轨迹决定；姿态流的 Roll/Yaw 不参与控制。这不是六轴完全独立控制。
- 两流使用相同规划/发送时间，但不是原子同步消息。俯仰以飞控本地接收时间独立检查超时。
- `pitch_deg` 使用 NED/FRD，正值抬头；脚本生成 ROS ENU/FLU 四元数，MAVROS 转换后得到正确符号。不是舵机角度。
- 只允许一个目标发布节点。不能把双流脚本用于原版/旧 PX4 或 Tiltquad。

## 飞控设置

落地、未解锁时刷入 `build/cuav_7-nano_default/cuav_7-nano_default.px4`。保留已有几何、舵机校准和已验证 PID，不重置机架参数。首次用 10°验证：

```sh
param set MC_MCT_OBP_EN 1
param set MC_MCT_OBP_MAX 20
param set MC_MCT_OBP_RATE 5
param set MC_MCT_OBP_TMO 0.5
```

| 参数 | 含义 |
|---|---|
| MC_MCT_OBP_EN | 默认 0；启用后 MCTilt 的姿态 MAVLink 入口专用于独立俯仰，不支持同时纯姿态 Offboard |
| MC_MCT_OBP_MAX | 外部机体俯仰限幅，deg，独立于 AUX6 限幅和舵机行程 |
| MC_MCT_OBP_RATE | 外部参考及回正/接回 AUX6 的最大变化率，deg/s，不是 CA_SVx_SLEW |
| MC_MCT_OBP_TMO | 独立俯仰流超时，s；丢失时告警并限速回正，不突然归零 |

普通 Position 保留原 AUX6；Offboard 外部参考替代 AUX6，不相加。切回 Position 时限速接回 AUX6，飞手应注意旋钮位置。
沿用 `MC_AUX_ATT_TAU` 低通时间常数，无需改变 `MC_ROLLRATE_D=0.00035` 等已验证参数。
两条流全部中断时，位置型 Offboard 仍执行原 PX4 失联策略，必须预先验证。
仅俯仰流失效时位置轨迹仍继续，飞控限速回正，飞手应接管。

## 机载脚本

脚本保持原 Python3 入口；使用机载端已经能导入 `rospy/mavros_msgs/tf` 的 Python 环境。
Ubuntu18.04/Melodic 默认 Python2 环境不能假定能运行此 Python3 包，应先核对现有运行环境；本次未重建 ROS。
不需要自定义 MAVROS 消息或 MAVLink dialect。

编辑 `config/figure8.yaml`：

```yaml
control_pitch: true
pitch_deg: 10.0
pitch_amplitude_deg: 0.0   # 0=固定俯仰；非零=八字过程中叠加平滑正弦俯仰
pitch_period: 18.0         # 俯仰正弦周期，和位置 period 相同则同周期
pitch_ramp_time: 8.0       # 进入倾角和结束回正的时间
settle_time: 10.0          # >= pitch_ramp_time，另留稳定等待时间
```

位置长宽、周期、速度/加速度前馈开关保持原含义。
启动前脚本通过 `mavros/param/get` 检查机型、功能开关、限角和限速，不自动修改飞控参数；检查失败则不发送姿态流。
`abs(pitch_deg)+abs(pitch_amplitude_deg)` 不得超过飞控限角。
五次过渡峰值速度为 `1.875*角度变化/过渡时间`；例如 60°和 5 deg/s 至少需要 22.5 s，可选 `pitch_ramp_time=24, settle_time=27`。
这只是规划可行性检查，**不是允许直接测试大倾角轨迹**。

MAVROS `setpoint_raw/thrust_scaling` 必须在启动 MAVROS 前配置为 1.0（官方 px4_config 通常已有）。
MAVROS 源 system ID 应与飞控一致，component ID 与飞控不同（通常 240）；飞控拒绝其他系统来源。
不要运行时改变功能参数或启动其他位置/姿态发布器。

```sh
roslaunch mctilt_offboard figure8.launch
```

新增原地变姿入口（二者只运行一个）：

```sh
roslaunch mctilt_offboard pitch_hold.launch
```

原地脚本固定启动时捕获的位置和航向，不调用八字位置生成器。
配置 `config/pitch_hold.yaml` 在共享 `figure8.yaml` 后加载：默认用 8 s 进入 `pitch_deg=10°`，`settle_time=10 s` 包含进入及等待阶段，随后额外保持 `pitch_hold_time=20 s`，最后平滑回正。
原地脚本要求 `control_pitch=true`、`pitch_amplitude_deg=0`。负 `pitch_deg` 用于反向俯仰；位置误差保护默认 0.4 m，超差中止后在当前反馈位置保持并回正。
ROS 节点统一为 `mctilt_offboard`，参考和阶段话题为 `mctilt_offboard/reference`、`mctilt_offboard/phase`；MAVROS 目标话题及飞控接口不变。
目录改名后重新复制整个包到 catkin 工作空间并构建/source；不要仅复制入口脚本，也不要保留运行中的旧节点。

流程：飞手 Position 稳定悬停 → 启动脚本预发送 → 遥控器切 Offboard → 原地缓慢进入倾角 → 等待稳定 → 八字位置＋俯仰同步规划 → 保持中心并回正 → 等飞手退出 Offboard。
原地测试将其中的八字阶段替换为 `PITCH_HOLD` 固定位置保持阶段，其余流程相同。
脚本拒绝在已处于 Offboard 时直接启动，不解锁、不自动切模式、不自动起降。
持续位置或俯仰超差会中止轨迹、保持当前位置并回正；定位/连接失效则停止两条流，让 PX4 失联策略接管。

恢复原位置轨迹+AUX6 行为须落地后同时设置 `MC_MCT_OBP_EN=0` 和 YAML `control_pitch:false`。

## 验证与日志

先模拟/拆桨验证，不能只看 ROS 话题有数据就上桨：

```sh
# ROS
rostopic hz /mavros/setpoint_raw/local
rostopic hz /mavros/setpoint_raw/attitude
rostopic echo /mctilt_offboard/reference
# PX4 控制台
listener mctilt_offboard_pitch
listener mctilt_pitch_status
listener offboard_control_mode
```

确认俯仰正负号、位置型 Offboard 未被切换、应用参考限速、单独断俯仰流会平滑回正、整流中断失联策略正确，再小倾角实飞。
`mctilt_offboard_pitch`、`mctilt_pitch_status` 已加入可选 ULog 记录。
`mctilt_offboard/reference` 是 ENU/FLU 规划参考，Roll=0 只是占位；最终滚转设定看飞控 `vehicle_attitude_setpoint`，不能用脚本占位值评估滚转误差。

```sh
python3 -m unittest discover -s mctilt_offboard/tests -v
CCACHE_DIR=/tmp/px4-mctilt-ccache make px4_sitl_test unit-MctiltPitchReference -j4
build/px4_sitl_test/unit-MctiltPitchReference
```

离线测试不代表 MAVROS 实际通信及闭环飞行已验证。完整 SITL 全量测试当前被 Micro-CDR 下载的网络/代理错误阻断；定向单元测试不依赖该下载。

接口核对依据：[官方 MAVROS1 setpoint_raw 插件](https://github.com/mavlink/mavros/blob/master/mavros/src/plugins/setpoint_raw.cpp)、[官方 px4_config](https://github.com/mavlink/mavros/blob/master/mavros/launch/px4_config.yaml)。本次俯仰专用路由是 PX4 本地扩展，不能据此认为官方固件原生支持位置和姿态双流叠加。
