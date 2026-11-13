# MCTilt Offboard：八字轨迹与原地变姿

**2026-10-26 更新：本包默认启用 MCTilt 独立俯仰规划。必须先阅读 [MCTilt 专用配置与验证流程](MCTILT_OFFBOARD.md)，该说明优先于下方旧的位置跟踪说明。旧固件/Tiltquad 不可使用双流模式。**

根目录与 ROS 包名现统一为 `mctilt_offboard`，不再使用旧 `figure8_mavros` 包名。
两个测试只能二选一，同一飞行器不要同时运行；也不要在飞行中通过启动另一个节点替换正在运行的测试。

```bash
# 八字轨迹：位置运动＋俯仰参考
roslaunch mctilt_offboard figure8.launch
# 原地变姿：固定启动时的位置和航向，仅改变俯仰参考
roslaunch mctilt_offboard pitch_hold.launch
```

原地测试设置见 `config/pitch_hold.yaml`：`pitch_deg=10`、`pitch_ramp_time=8`、`settle_time=10`、`pitch_hold_time=20`。
即切入 Offboard 后用 8 秒平滑进入 10°、再等 2 秒稳定、保持 20 秒，最后用 8 秒平滑回正并继续保持原位置，等待飞手退出。
`pitch_deg` 可设为负值测试另一方向。正常流程不改变捕获的位置；超差中止时，为避免继续追赶旧目标，沿用安全逻辑在当前反馈位置保持并回正。
原地配置在 `figure8.yaml` 共享参数之后加载覆盖，八字长宽/周期不参与原地测试的位置输出。

这是可直接放入 catkin 工作空间 `src/` 的独立包，面向 Ubuntu 20.04 + ROS Noetic + PX4/MAVROS。它使用机载电脑上的 MAVROS，从当前悬停位置生成水平 Gerono 八字参考轨迹，并以 50 Hz 向 `/mavros/setpoint_raw/local` 发布位置及可选速度前馈。**需要飞手先在 Position 模式起飞并稳定悬停，再由飞手用遥控器切入 OFFBOARD；程序不解锁、不起飞、不切模式、不降落。**

## 目录

```text
mctilt_offboard/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   ├── figure8.yaml                 # 八字轨迹及共享参数
│   └── pitch_hold.yaml              # 原地变姿覆盖参数
├── launch/
│   ├── figure8.launch               # 只启动轨迹节点
│   ├── pitch_hold.launch            # 只启动原地变姿节点
│   └── mavros_px4.launch            # 可选：单独启动 MAVROS
├── scripts/
│   ├── offboard_figure8.py           # 八字入口
│   └── offboard_pitch_hold.py        # 原地变姿入口
├── setup.py                        # catkin 共享 Python 模块安装
└── src/mctilt_offboard/
    ├── figure8_controller.py        # 公共安全状态机＋八字规划
    └── pitch_hold.py               # 固定位置变姿实现
```

数据路径：飞控 → MAVROS `/mavros/state`、`/mavros/local_position/pose` → 本节点 → MAVROS `/mavros/setpoint_raw/local` → 飞控。监控话题为 `/mctilt_offboard/reference`（`PoseStamped`）和 `/mctilt_offboard/phase`（`String`）。设置 `vehicle_ns:=uav1` 后，以上话题统一变为 `/uav1/...`。

## 与 502.zip 的关系

压缩包是旧的 catkin 工作空间 `502/`：`src/offboard_sample` 含 C++/Python 状态机、`cfg/mission_cfg.yaml` 和多个 launch；还含 `planner`、`swarm`、`uav_simulator`，以及 `build/`、`devel/` 和 Git/IDE 缓存。ZIP 条目校验通过，但这只能说明压缩数据完整，不能证明旧代码可编译或适合当前飞控。本包参考它的 ROS 1 目录结构和 MAVROS 话题接口，不依赖旧包、旧的 `quadrotor_msgs`、键盘节点或规划器，也不复用已有编译产物。

旧的 `offb_node`、`offboard_fsm.py` 等节点会向相同 MAVROS setpoint 话题发送控制指令。运行本包时，必须关闭这些节点以及其他位置、速度、姿态 setpoint 发布器；只保留一个控制源。MAVROS 可以是原系统中已经运行的实例，不需要重复启动。

## 安装

在机载电脑的 Ubuntu/ROS 环境中执行。先确认已有 ROS Noetic 和 MAVROS。按 [PX4 的 ROS 1 MAVROS 安装说明](https://docs.px4.io/main/en/ros/mavros_installation) 安装 MAVROS、`mavros_msgs` 和 GeographicLib 数据集。首次安装常用命令：

```bash
sudo apt update
sudo apt install ros-noetic-mavros ros-noetic-mavros-extras ros-noetic-mavros-msgs \
  ros-noetic-geometry-msgs ros-noetic-std-msgs ros-noetic-rospy ros-noetic-tf
```

MAVROS 的 GeographicLib 数据集安装脚本及其运行方法见上面的 PX4 官方说明。若机载电脑已经正常运行 MAVROS，无需重装。

建议先使用干净的工作空间，以免 `502/` 中其他包的缺失依赖影响构建。将下载后的 `mctilt_offboard` 目录复制到 `~/mctilt_ws/src/`：

```bash
source /opt/ros/noetic/setup.bash
mkdir -p ~/mctilt_ws/src
cp -a /path/to/mctilt_offboard ~/mctilt_ws/src/
chmod +x ~/mctilt_ws/src/mctilt_offboard/scripts/offboard_*.py
cd ~/mctilt_ws
catkin_make
source ~/mctilt_ws/devel/setup.bash
rospack find mctilt_offboard
```

`rosdep install` 只是自动补齐依赖的便利步骤，本包不要求先运行它。构建前可用下列命令检查依赖；若都能找到，直接 `catkin_make`：

```bash
source /opt/ros/noetic/setup.bash
for p in catkin rospy geometry_msgs mavros_msgs std_msgs tf mavros; do
  rospack find "$p" || echo "缺少: $p"
done
```

若缺少依赖，再解决 apt 软件源和网络问题后安装。Ubuntu 20.04/ROS Noetic 的 ROS rosdep 项目文档使用 `python3-rosdep`；部分 Ubuntu 软件源也提供 `python3-rosdep2`。若 apt 报 `Could not resolve 'mirrors.tuna.tsinghua.edu.cn'` 或 `Could not resolve 'packages.ros.org'`，是域名解析或网络连接问题，切换包名或使用 `--fix-missing` 不会解决。先检查 `ip route`、`getent hosts packages.ros.org`、`getent hosts mirrors.tuna.tsinghua.edu.cn` 和机载电脑网络设置。恢复网络后再运行 `sudo apt update` 与安装命令。确实需要 rosdep 时，首次使用还要执行 `sudo rosdep init`、`rosdep update`；步骤见 [rosdep 官方说明](https://github.com/ros-infrastructure/rosdep/blob/master/doc/overview.rst)。

如果旧 `502` 工作空间仍可正常构建，也可以仅把本包复制到 `~/502/src/` 再构建。不要复制压缩包里的 `build/` 或 `devel/`；这些路径绑定了原机器环境。

## 连接飞控

若原系统已启动 MAVROS，只需验证：

```bash
rostopic echo -n 1 /mavros/state
rostopic hz /mavros/local_position/pose
```

`connected: True` 且本地位姿连续、坐标稳定后才能继续。若没有运行 MAVROS，可在另一终端启动本包提供的连接 launch，并按实际设备修改串口和波特率：

```bash
source /opt/ros/noetic/setup.bash
source ~/mctilt_ws/devel/setup.bash
roslaunch mctilt_offboard mavros_px4.launch fcu_url:=/dev/ttyACM0:57600
```

SITL 示例：`roslaunch mctilt_offboard mavros_px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"`。这些是连接形式示例，端口、设备名、MAVLink 实例和波特率必须按当前飞控配置核对。若已有 `/mavros` 节点，直接使用它，不启动第二个连接进程。命名空间系统可同时对连接 launch 和轨迹 launch 传入相同的 `vehicle_ns:=uav1`。

## 飞行前设置

1. 在地面通过 QGroundControl/PX4 配置并核实遥控接管、失联动作、OFFBOARD 失联超时（如 `COM_OF_LOSS_T`、对应固件版本的 OFFBOARD 失联动作参数）以及可用的本地定位。请以**实际 PX4 固件版本**的参数说明为准。
2. 先在仿真和低风险场地试验。确保八字覆盖范围内有足够空间：默认长轴总长 `2 × amplitude_x = 2.0 m`，短轴总宽 `amplitude_y = 0.6 m`，再加定位误差和制动距离。
3. 确认 `/mavros/local_position/pose` 提供有效的局部 ENU 位姿；本包不提供定位。当前默认双流模式只用于已启用 `MC_MCT_OBP_EN` 的专用 MCTilt 固件，会发送位置和独立俯仰参考，不发送 AUX 指令。详见专用配置说明；不要连接旧固件/Tiltquad 运行双流测试。
4. 修改 `config/figure8.yaml`。首次测试建议保留小幅度、低速参数；`period` 必须大于 `2 × ramp_time`，`prestream_time` 至少 1 秒。`align_with_initial_yaw: true` 表示长轴沿起始机头方向；改为 `false` 则沿 ENU 的 x 轴。

## 使用步骤

1. 启动 `roscore`（如果 MAVROS 的 launch 尚未启动它）、MAVROS、地面站和定位系统；检查上面的两个话题。
2. 飞手在 **Position** 模式手动解锁、起飞，稳定悬停在预定八字中心与高度。PX4 官方示例建议从 Position 模式进入 OFFBOARD，以便退出后悬停。
3. 在机载电脑新终端启动轨迹节点：

   ```bash
   source /opt/ros/noetic/setup.bash
   source ~/mctilt_ws/devel/setup.bash
   roslaunch mctilt_offboard figure8.launch
   ```

4. 观察 `/mctilt_offboard/phase` 进入 `WAIT_OFFBOARD`，此时保持目标点已连续发送。飞手再用遥控器切入 **OFFBOARD**。程序先 `SETTLE` 悬停，然后执行 `TRACKING`，结束后持续 `FINAL_HOLD`。
5. 飞手切回 Position 模式并手动降落；节点检测离开 OFFBOARD 后退出。不要在仍处于 OFFBOARD 时直接关闭终端，除非已准备由 PX4 的失联保护接管。

默认 `vehicle_ns:=/` 使用根命名空间。若机载电脑上还是旧版 launch 文件并提示 `<group> tag has an empty 'ns' attribute`，可立即运行 `roslaunch mctilt_offboard figure8.launch vehicle_ns:=/`；永久修复是把 `launch/figure8.launch` 中的 `<arg name="vehicle_ns" default=""/>` 改成 `<arg name="vehicle_ns" default="/"/>`。本包当前版本已修复。

监控命令：

```bash
rostopic echo /mctilt_offboard/phase
rostopic hz /mavros/setpoint_raw/local
rostopic echo -n 1 /mctilt_offboard/reference
rostopic echo -n 1 /mavros/state
```

参数可在启动前修改 YAML。默认轨迹时间为 `cycles × period = 54 s`，前后 `ramp_time` 使用平滑包络。`use_velocity_ff` 控制速度前馈；若当前 PX4 控制器不支持位置和速度组合 setpoint，可在地面验证后设为 `false`。`max_tracking_error` 连续超过 `error_timeout`、位姿或状态超时、解除解锁或退出 OFFBOARD 时，节点停止轨迹并进入/退出保持阶段。`final_hold_time` 只控制提醒开始时间，不会停止 setpoint 流。程序不会自动改飞行模式或自动降落。

`pose_timeout` 与 `state_timeout` 分开设置：默认分别为 0.5 秒和 3 秒。`/mavros/state` 常跟随较慢的心跳更新，不能用 0.5 秒的位姿阈值判断。若节点在 `WAIT_OFFBOARD` 退出，先在 Position 模式执行 `rostopic hz /mavros/state`、`rostopic hz /mavros/local_position/pose` 和 `rostopic echo -n 1 /mavros/state`，核对日志中的具体超时项。位姿数据仅“有输出”不足以说明其更新足够快；不要为绕过低频或不稳定定位而盲目增大 `pose_timeout`。

### TELEM1 位姿仅约 0.44 Hz 的处理

若 `rostopic hz /mavros/local_position/pose` 显示 `average rate: 0.441`、约每 2.27 秒一条，那么 0.5 秒位姿超时必然反复触发。对已分析的 `log_13_2024-8-20-10-24-52.ulg`，PX4 日志里的本地位置记录连续，但连接 Khadas 的 MAVLink 实例为 `MAV_0_CONFIG=101`（TELEM1）、`MAV_0_MODE=0`（Normal）、`MAV_0_RATE=1200` B/s，流速缩放系数约 0.4407。因此先修正 TELEM1 的 MAVLink 输出配置，不要只增大 `pose_timeout`。

在地面、飞行器未解锁时，于 QGroundControl 参数页面记录旧值，再将 `MAV_0_MODE` 设为 `Onboard`（数值 2），`MAV_0_RATE` 设为 `0`（由 PX4 根据串口波特率取约一半的理论带宽）。保持 `MAV_0_CONFIG=TELEM1`；确认 `SER_TEL1_BAUD` 与 Khadas 上 MAVROS 串口 `fcu_url` 的波特率一致。上述 ULog 中 `SER_TEL1_BAUD=921600`，但仍须按当前机上的参数核对。保存后重启飞控并重启 MAVROS。

只启动 MAVROS，在地面连续测量 15–20 秒：

```bash
rostopic hz -w 100 /mavros/local_position/pose
rostopic echo -n 1 /mavros/state
```

位姿应稳定达到至少 10 Hz，且 `rostopic hz` 显示的最大间隔低于 0.5 秒，然后才启动 `roslaunch mctilt_offboard figure8.launch` 并考虑切入 OFFBOARD。若仍约 0.44 Hz，在 QGroundControl 读回 `MAV_0_MODE`、`MAV_0_RATE`、`SER_TEL1_BAUD`，确认修改已生效且 MAVROS 实际连接 TELEM1；再检查连接线、串口丢包和是否有其他程序占用串口。`rostopic hz` 只反映接收速率，不能单独证明定位数值和时间戳正确。

参见 [PX4 v1.16 MAVLink 外设文档](https://docs.px4.io/v1.16/en/peripherals/mavlink_peripherals)：`Onboard` 是机载电脑的消息流模式，`MAV_X_RATE` 限制该实例所有流量之和，值 `0` 使用约一半理论带宽。

## 官方依据

- [PX4 ROS 1/MAVROS 安装](https://docs.px4.io/main/en/ros/mavros_installation)：ROS 1 机载电脑与 PX4 使用 MAVROS 桥接。
- [PX4 MAVROS Python Offboard 示例](https://docs.px4.io/v1.16/en/ros/mavros_offboard_python)：先连续发送 setpoint，再切入 OFFBOARD；发布频率必须高于 2 Hz；真实飞行需保留手动接管能力。
- [PX4 Offboard 模式](https://docs.px4.io/main/en/flight_modes/offboard)：说明连续 setpoint、OFFBOARD 丢失保护和相关参数。
- [catkin Python 脚本安装](https://docs.ros.org/en/kinetic/api/catkin/html/howto/format1/installing_python.html)：`scripts/` 与 `catkin_install_python` 的标准布局。
