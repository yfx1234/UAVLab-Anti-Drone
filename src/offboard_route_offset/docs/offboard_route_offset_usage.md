# offboard_route_offset 3.0 使用说明

## 1. 实际项目与运行环境

工作空间为 `/home/danyu/offboard_guojun_ws`，包名为 `offboard_route_offset`，可执行文件为
`offboard_route_offset_node`，ROS 节点名为 `/offboard_route_offset`。本机验证环境如下：

| 组件 | 实际版本或目标 |
|---|---|
| PX4 | v1.15.4，`/home/danyu/PX4-Autopilot` |
| ROS 2 | Jazzy |
| Gazebo Sim | Harmonic，`gz sim` 8.14.0 |
| PX4 SITL | `gz_x500`，airframe 4001，model `x500` |
| world | `Tools/simulation/gz/worlds/default.sdf`，名称 `default` |
| XRCE Agent | 2.4.0，UDP 8888 |

本项目使用 Gazebo Sim，不使用 Gazebo Classic。禁止使用 `gazebo-classic` 或
`make px4_sitl gazebo-classic`。

主要文件：

```text
src/offboard_route_offset/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/route_offset.yaml
├── launch/offboard_route_offset.launch.py
├── scripts/
│   ├── tune_route_offset.sh
│   ├── collect_offset_metrics.py
│   ├── analyze_offset_metrics.py
│   └── plot_velocity_path_comparison.py
├── docs/
│   ├── offboard_route_offset_usage.md
│   └── velocity_path_test_report.md
└── src/offboard_route_offset_node.cpp
```

没有自定义头文件或 ROS 消息。调试话题使用 `std_msgs/msg/Float64MultiArray`，飞控接口直接使用
`px4_msgs`。

## 2. 安全边界

节点没有 `VehicleCommand` publisher，因此不会主动执行解锁、起飞、模式切换、Land、RTL 或
上锁。飞手必须通过遥控器、QGC 或 SITL PX4 shell 完成这些动作。

节点只在允许状态发布：

```text
WAIT_FOR_TAKEOFF_REFERENCE    不发布轨迹 setpoint
IDLE                          不发布轨迹 setpoint
PRESTREAM                     位置控制，保持当前位置
APPROACH_START                位置控制，平滑接近固定起点
RUNNING_TRAJECTORY            按有效后端执行
TERMINAL_CAPTURE              速度控制，低速收敛端点
MISSION_COMPLETE_HOLD         位置控制，保持精确终点
PAUSED_AFTER_OFFBOARD_EXIT    停止轨迹 setpoint
LANDING_OR_DISARMED           停止心跳和轨迹 setpoint
FAULT                         停止发布
```

切出 Offboard 后立即清除主动 setpoint。Land、RTL、Descend、接地或上锁时保持静默。重新进入
Offboard 时重新构造接近段、速度规划器、投影滤波器和 yaw 限制器，不从旧进度突跳恢复。

节点必须在解锁前启动。它在未解锁且本地位置/落地状态可靠时缓存地面位置，并在解锁跃迁时记录
本次飞行起飞参考。若节点在已经起飞后启动且无法可靠恢复地面参考，任务会被拒绝。

## 3. PX4 话题

订阅：

```text
/fmu/out/vehicle_local_position   px4_msgs/msg/VehicleLocalPosition
/fmu/out/vehicle_status           px4_msgs/msg/VehicleStatus
/fmu/out/vehicle_land_detected    px4_msgs/msg/VehicleLandDetected
```

发布：

```text
/fmu/in/offboard_control_mode               px4_msgs/msg/OffboardControlMode
/fmu/in/trajectory_setpoint                  px4_msgs/msg/TrajectorySetpoint
/offboard_route_offset/trajectory_debug      std_msgs/msg/Float64MultiArray
```

NED 坐标为 `x=North`、`y=East`、`z=Down`。正任务高度会从起飞参考 `z` 中减去。

## 4. 控制后端与 setpoint

### position_feedforward

保留 2.1 的位置目标、速度前馈、加速度前馈和 yaw，用于回归对照。正式路径仍由 `s_ref` 推进，
并受动态预瞄和 `max_reference_lead_distance_m` 限制。该模式可能在实际飞机跟踪落后时触发
“参考点限速/暂停—飞机追上—重新推进”的循环。

### velocity_path

默认和推荐后端。在 `RUNNING_TRAJECTORY` 与 `TERMINAL_CAPTURE` 中发布：

```text
OffboardControlMode.position = false
OffboardControlMode.velocity = true
TrajectorySetpoint.position = [NaN, NaN, NaN]
TrajectorySetpoint.velocity = [vx, vy, vz]
```

`velocity_path_acceleration_feedforward_enabled=false` 时 acceleration 为全 NaN；开启后，正式运行段
使用切向 jerk 规划加速度与 `normal × curvature × speed²`。终端捕获的纠偏运动不冒充路径运动学
前馈，因此 acceleration 仍为 NaN。

飞机实际 XY 位置在上一周期进度附近的 `projection_search_window_m` 局部窗口投影到离散弧长路径。
投影进度限制单周期变化，闭合路径跨圈解缠，再按
`projection_filter_time_constant_s` 低通得到 `s_vehicle_filtered`。`s_plan` 只用于预览和调试，不作为
PX4 位置目标。

Frenet 跟踪器计算：

```text
cross_error = dot(path_position - vehicle_position, normal)
v_normal = clamp(cross_track_gain * cross_error, ±normal_limit)
normal_limit = min(max_cross_track_correction_speed_mps,
                   cross_track_speed_ratio * planner_speed)
v_tangent = sqrt(max(planner_speed² - v_normal², 0))
v_xy = tangent * v_tangent + normal * v_normal
```

横向误差从 `cross_track_slowdown_start_m` 到 `cross_track_stop_error_m` 通过 smootherstep 连续降速；
严重误差目标为零时也必须经过同一个 jerk 速度规划器。目标速度统一取巡航、曲率、剩余距离、横向
误差和终端限制的最小值。

高度外环使用 `vz_target = altitude_velocity_gain * (target_z - vehicle_z)`，再经过垂直速度、加速度和
jerk 限制。所有位置/速度/时间步均检查有限性；高度误差超过 `max_altitude_error_m` 进入 FAULT。

## 5. 轨迹实现

- `line`：实际投影进度与 jerk 停车模型提前制动，终点进入 `TERMINAL_CAPTURE`。
- `line_reciprocating/stop_reverse`：正向减速到零、终端捕获、再反向加速。固定 yaw 时保持原航向
  倒飞；切线 yaw 时停稳后按 yaw rate/acceleration 限制平滑转 180°。
- `line_reciprocating/racetrack`：两条直线用半圆连接。有效半径自动满足
  `r >= speed² / max_safe_lateral_acceleration_mps2`。
- `circle`：离散弧长统一路径、闭合跨圈投影和曲率限速。
- `sine`：离散弧长路径、局部投影、正负曲率预览。
- `square/rounded`：直线与四段圆弧组成切线连续闭合路径，不进入顶点 settle/yaw 状态。
- `square/stop_turn`：保留原位置控制精确顶点模式。即使配置后端为 `velocity_path`，该组合也会
  明确告警并使用保留的 position-feedforward 顶点状态机。
- `z_shape`：使用多段线和二次 Bezier 圆滑连接的统一速度路径，但仍标记为实验功能，未完成专项
  仿真验收前不建议实机使用。

## 6. yaw 模式

`yaw_mode=fixed` 为默认值。正式轨迹开始时固定为 `mission_start_yaw_deg`，直线返程保持机头不变，
圆形、正弦和圆角正方形也不随切线摆头。

`yaw_mode=tangent` 显式启用切线跟随。内部 yaw 解缠后沿最短方向追踪，角速度和角加速度分别受
`max_yaw_rate_dps`、`max_yaw_acceleration_dps2` 限制。启用 `yaw_rate_feedforward_enabled` 时，
`curvature × signed_speed` 进入 yaw rate 规划器，不重复叠加到最终 yawspeed。发布 yaw 前重新归一化
到 `[-pi, pi]`。

## 7. 参数表

下表列出 3.0 新增或改变执行语义的参数；非法枚举、NaN、无穷值和越界值会在启动时给出明确错误。

| 参数 | 默认值 | 单位/范围 | 说明 |
|---|---:|---|---|
| trajectory_control_backend | velocity_path | 枚举 | `velocity_path` / `position_feedforward` |
| trajectory_setpoint_rate_hz | 50 | Hz，(2, 200] | 心跳与 setpoint 定时器 |
| cross_track_gain | 0.5 | 1/s，[0, 10] | Frenet 法向比例增益 |
| max_cross_track_correction_speed_mps | 0.30 | m/s，(0, 10] | 法向速度绝对上限 |
| cross_track_speed_ratio | 0.40 | 比例，(0, 1] | 法向速度占规划速度上限 |
| cross_track_slowdown_start_m | 0.60 | m，>=0 | 连续降速起点 |
| cross_track_stop_error_m | 1.50 | m，>slowdown | 严重误差零速目标 |
| projection_filter_time_constant_s | 0.15 | s，[0, 10] | 投影进度低通常数 |
| projection_search_window_m | 2.0 | m，(0, 100] | 上一进度附近搜索窗口 |
| terminal_capture_enabled | true | bool | 启用终端低速状态 |
| terminal_capture_distance_m | 0.50 | m，(0, 100] | 进入终端状态距离 |
| terminal_gain | 0.5 | 1/s，(0, 10] | 终端位置误差速度增益 |
| terminal_max_speed_mps | 0.15 | m/s，(0, 10] | 终端最大规划速度 |
| altitude_velocity_gain | 0.8 | 1/s，(0, 10] | NED-z 外环增益 |
| max_vertical_speed_mps | 0.4 | m/s，(0, 10] | 垂直速度上限 |
| max_vertical_acceleration_mps2 | 0.5 | m/s²，(0, 20] | 垂直加速度上限 |
| max_vertical_jerk_mps3 | 1.0 | m/s³，(0, 100] | 垂直 jerk 上限 |
| max_altitude_error_m | 2.0 | m，>0 | 超限进入 FAULT |
| velocity_path_acceleration_feedforward_enabled | true | bool | 路径运动学加速度前馈 |
| square_corner_mode | rounded | 枚举 | `rounded` / `stop_turn` |
| square_corner_radius_m | 1.0 | m，>0 | 自动限制到边长的 49% |
| line_turnaround_mode | stop_reverse | 枚举 | `stop_reverse` / `racetrack` |
| line_turn_radius_m | 2.0 | m，>0 | 跑道形配置半径 |
| yaw_mode | fixed | 枚举 | `fixed` / `tangent` |
| yaw_rate_feedforward_enabled | true | bool | 曲率 yaw rate 前馈 |
| max_yaw_rate_dps | 45 | deg/s，(0, 360] | yaw rate 上限 |
| max_yaw_acceleration_dps2 | 90 | deg/s²，(0, 2000] | yaw acceleration 上限 |

通用几何和安全参数：

| 参数组 | 默认值 | 单位/约束 |
|---|---|---|
| mission_start_altitude_m / east_offset_m / yaw_deg | 5 / 3 / 0 | m / m / deg；有限值，高度大于 minimum |
| approach_horizontal_speed_mps / vertical_speed_mps | 1 / 0.5 | m/s，>0 |
| approach_arrival_tolerance_m / yaw_tolerance_deg / settle_time_s | 0.20 / 5 / 0.5 | m / deg / s |
| line_distance_m / line_speed_mps / reciprocation_count | 10 / 2 / 3 | m>0 / m/s>0 / integer>=0 |
| line_endpoint_hold_time_s | 0 | s，>=0 |
| line_length | 3 | m，>0 |
| circle_radius / turns / direction | 1.5 / 1 / cw | m>0 / >0 / cw或ccw |
| square_side | 3 | m，>0 |
| sine_length / amplitude / cycles | 4 / 1 / 1 | m>0 / m>=0 / >0 |
| z_width / z_height / z_repeats | 4 / 2 / 1 | m>0 / m>0 / integer>=1 |
| curve_samples_per_meter | 8 | 1/m，>0 |
| max_acceleration / deceleration / jerk | 1.5 / 1.8 / 4 | m/s² / m/s² / m/s³，>0 |
| lookahead_time / min / max | 0.25 / 0.10 / 0.35 | s / m / m，min<=max<legacy lead |
| max_reference_lead_distance_m | 0.50 | m，>0；只用于旧后端 |
| endpoint position/speed/settle | 0.12 / 0.20 / 0.50 | m / m/s / s |
| endpoint altitude/vspeed | 0.25 / 0.30 | m / m/s |
| max_safe_lateral_acceleration_mps2 | 1.5 | m/s²，>0 |
| local_position_timeout_s | 1.0 | s，>0 |
| prestream_after_exit_delay_s | 1.0 | s，>=0 |
| minimum_start_altitude_m | 0.5 | m，>=0 |
| takeoff_reference_candidate_timeout_s | 3.0 | s，>0 |

`publish_rate_hz` 是旧配置兼容参数，默认同步为 50 Hz；实际定时器只使用
`trajectory_setpoint_rate_hz`。

## 8. tune_route_offset.sh 调用链

源脚本位于：

```text
/home/danyu/offboard_guojun_ws/src/offboard_route_offset/scripts/tune_route_offset.sh
```

构建后安装到 `install/offboard_route_offset/lib/offboard_route_offset/`，主要调用链为：

```text
ros2 run offboard_route_offset tune_route_offset.sh
  -> 解析交互菜单或 CLI
  -> 本地合法性检查和数值格式化
  -> 检查 /offboard_route_offset 是否已存在
  -> mktemp 生成 /tmp/offboard_route_offset.XXXXXX.yaml
  -> ros2 launch offboard_route_offset offboard_route_offset.launch.py config_file:=临时YAML
  -> launch 启动 offboard_route_offset_node
```

脚本不搜索工作空间，也不自动执行 `source`；调用者必须先 source ROS 2 和本工作空间。参数是启动时
加载，不通过 ROS 参数服务动态修改。脚本不负责起飞、模式切换或飞行数据记录。退出 launch 后临时
YAML 自动删除。3.0 增加了重复节点保护、后端/yaw/50 Hz/速度路径/圆角/跑道形参数入口。

常用命令：

```bash
ros2 run offboard_route_offset tune_route_offset.sh \
  --trajectory-type line --control-backend velocity_path --yaw-mode fixed \
  --setpoint-rate 50 --altitude 3 --east-offset 0 --yaw 0 \
  --line-length 40 --line-speed 1 --accel-time 1 --decel-time 1

ros2 run offboard_route_offset tune_route_offset.sh \
  --trajectory-type square --control-backend velocity_path --yaw-mode fixed \
  --square-side 4 --square-corner-mode rounded --square-corner-radius 1 \
  --line-speed 1 --continuous

ros2 run offboard_route_offset tune_route_offset.sh \
  --trajectory-type line_reciprocating --line-distance 10 --repeat 3 \
  --line-turnaround-mode racetrack --line-turn-radius 2 --line-speed 1
```

## 9. 构建与 Gazebo Sim 启动

必要时只清理本包缓存：

```bash
cd /home/danyu/offboard_guojun_ws
rm -rf build/offboard_route_offset install/offboard_route_offset
source /opt/ros/jazzy/setup.bash
colcon build --packages-select offboard_route_offset --symlink-install \
  --event-handlers console_direct+
source install/setup.bash
```

终端 1，先启动 XRCE-DDS Agent：

```bash
/home/danyu/Micro-XRCE-DDS-Agent/build/MicroXRCEAgent udp4 -p 8888
```

终端 2，启动 PX4 v1.15.4 与 Gazebo Sim：

```bash
cd /home/danyu/PX4-Autopilot
HEADLESS=1 make px4_sitl gz_x500
```

本机冷启动时首次 model create 偶尔在 Gazebo server 尚未就绪前超时；确认 `gz sim` server 已运行后，
再次执行同一条 `HEADLESS=1 make px4_sitl gz_x500`，日志应显示
`gazebo already running world: default` 与 `model name: x500_0`。不要改用 Classic。

终端 3，检查 DDS：

```bash
cd /home/danyu/offboard_guojun_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 topic list | grep '^/fmu/'
ros2 topic echo --once --qos-reliability best_effort \
  /fmu/out/vehicle_local_position px4_msgs/msg/VehicleLocalPosition
```

终端 4，用脚本启动节点。然后由飞手起飞并切入 Offboard。SITL shell 等价人工操作示例：

```text
commander takeoff
commander mode offboard
commander mode auto:loiter    # 模拟切出 Offboard
commander land
```

这些命令仅用于 SITL 验收，不由 ROS 节点发送。

## 10. 调试数据、CSV 与图表

速度后端调试数组共有 36 项：

```text
0 timestamp, 1 trajectory_type_code, 2 state, 3 backend, 4 yaw_mode,
5 s_vehicle_raw, 6 s_vehicle_filtered, 7 s_plan, 8 remaining_distance,
9 projected_x, 10 projected_y, 11 cross_track_error, 12 along_track_error,
13 planner_target_speed, 14 planner_speed, 15 planner_acceleration,
16 tangent_speed_command, 17 normal_speed_command, 18 total_speed_command,
19 vertical_speed_command, 20 actual_horizontal_speed, 21 actual_tangent_speed,
22 actual_normal_speed, 23 actual_vertical_speed, 24 curvature,
25 curve_speed_limit, 26 terminal_speed_limit, 27 cross_track_speed_scale,
28 yaw_setpoint, 29 yaw_rate_setpoint, 30 actual_heading,
31 endpoint_position_ok, 32 endpoint_speed_ok, 33 endpoint_altitude_ok,
34 endpoint_vertical_speed_ok, 35 endpoint_settle_elapsed
```

CSV 记录：

```bash
ros2 run offboard_route_offset collect_offset_metrics.py \
  /tmp/line_velocity.csv 45 --wait-active 180
```

rosbag 记录：

```bash
ros2 bag record -o /tmp/offboard_velocity_path \
  /fmu/out/vehicle_local_position /fmu/out/vehicle_status \
  /fmu/out/vehicle_land_detected /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint /offboard_route_offset/trajectory_debug
```

对比图：

```bash
ros2 run offboard_route_offset plot_velocity_path_comparison.py \
  --before before.csv --after after.csv --curve sine.csv \
  --fixed-yaw reciprocal_fixed.csv --tangent-yaw reciprocal_tangent.csv \
  --output comparison.png
```

## 11. 停止与残留检查

先由飞手切出 Offboard 并 Land。确认上锁后对 `tune_route_offset.sh` 所在终端按 Ctrl-C，再在 PX4 shell
执行 `shutdown`，最后 Ctrl-C Gazebo 启动终端和 Agent。检查：

```bash
ros2 node list | grep offboard_route_offset || true
ps -eo pid,cmd | grep -E 'MicroXRCEAgent|px4|gz sim' | grep -v grep
```

不要在同一架飞机上同时启动 `offboard_route` 与 `offboard_route_offset`。

## 12. 实机低速流程

1. 拆桨，检查 ROS domain、PX4 v1.15.4 消息版本、NED z 符号和所有 NaN 字段。
2. 拆桨状态切换 Offboard/Position/Land/RTL，确认切出后 setpoint 立即静默，节点不发布 VehicleCommand。
3. 装桨前检查 `yaw_mode=fixed`、`velocity_path`、50 Hz、0.3--0.5 m/s、短直线、足够空域。
4. 首飞只做固定 yaw 长直线，不触发终点；观察命令速度、实际速度、EKF 和饱和。
5. 依次启用横向修正、终端捕获、圆形/正弦、往返、圆角正方形。
6. 切线 yaw、100 Hz、跑道形和实验 Z 形必须分别重新仿真后再实机。
7. 若命令已平滑但实际速度仍周期振荡，再检查 PX4 速度环、EKF、振动、推力/姿态饱和和机体参数，
   不先用降低 PX4 增益掩盖上层命令问题。
