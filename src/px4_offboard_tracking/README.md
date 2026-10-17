# PX4 ROS 2 Offboard 自主跟踪

> `feat/predictive-shot-referee-v1` 更新：末端事件已经改为 400 m/s 预测交会
> `VirtualShotEvent`，不再等待 0.1 s `CAPTURE_CONFIRM`。仿真命中由独立
> `virtual_interception_referee` 裁决，裁判结果不反馈本控制包。下文出现的
> `VirtualCaptureEvent` 和保持窗说明属于历史兼容链路；当前部署以仓库
> [根 README](../../README.md) 为准。

> 当前正式开发入口是 `pure_interceptor_sim.launch.py`（仿真）和
> `pure_interceptor_real.launch.py`（实机接口）。真值只由仿真相机侧读取；估计和控制只
> 订阅 `/perception/search/observation`、`/perception/depth/observation` 以及本机 PX4
> 标准状态。下文涉及 `dual_tracking.launch.py`、旧 MPC、旧走廊和 B0～B17 的内容仅作为
> 重构前基线保留，不是当前正式启动入口。

双相机、IMM-EKF、统一三维制导、任务状态机和打击门限的统一说明见
[`双相机跟踪打击统一方案`](../../docs/开发状态/双相机跟踪打击统一方案.md)。

## 当前预测视场屏障与分阶段距离基线（2026-07-29）

正式双机组合为目标机`x500/4001`、拦截机
`gz_my_quad_real/22001`。两机Gazebo外部生成高度为0.01 m；目标机任务起飞
高度为30 m，拦截机起飞高度为5 m。模型SDF内部贴地偏移不得在启动脚本中重复
补偿。

双目实际Body `-Z`/LOS误差按横、纵分量分别管理，并用图像角速度预测未来0.5 s：

- 0～5°：正常跟踪；
- 5～8°：连续增强居中约束；
- 8～10°：`FOV_RECOVERY`，统一制导保持运行但暂停正向逼近；
- 超过10°：双目不可见，仅在协方差和超时边界内进入`IMM_COAST`；
- 预测误差连续0.5 s低于6°后才退出恢复，避免状态反复跨界。

距离控制不追踪世界固定点。`SEARCH/DEPTH_HANDOVER/DEPTH_ACQUIRE`使用20～30 m，
`INTERCEPT`使用17～25 m，射击窗保持15～35 m，17 m仅为缓慢变化的软偏好。
每次进入阶段均以当前距离初始化参考；FOV恢复优先于接近。高度约束独立执行：
3 m为硬下界，5 m以下启用软爬升保护，不引入固定高度、固定正下方或固定垂直
距离。

当前状态链为：

```text
SEARCH → BEARING_TRACK → DEPTH_HANDOVER → DEPTH_ACQUIRE
       → INTERCEPT → EGRESS_REATTACK → 二次进入
```

`FOV_RECOVERY`和`IMM_COAST`是有界临时状态。控制器只接收双相机消息、
IMM-EKF估计与拦截机自身PX4状态；仿真真值仅供逻辑相机与独立裁判使用。
自动仿真默认在目标运动就绪后等待10 s；人工实机在遥控器切入Offboard时立即开始任务，
不再附加固定等待。连续跟踪2 s后用1 s平滑收紧终端瞄准，开火仍须满足连续跟踪保持、
2°硬安全门、15～35 m、两次间隔至少3 s，以及当前和短时预测攻击轴误差都在1°内并
稳定默认0.04 s。

2026-08-02的5 m/s直线回归在10 s最早开火时间门下，0.236 s建立方位、
1.831 s进入`INTERCEPT`、3.400 s完成持续速度匹配，整轮2/2命中并通过
完成后悬停验收。首次开火授权为15.264 s，因为10 s只是时间下界，中途视场恢复
不会绕过连续观测和1°开火门。单参数将`transverse_velocity_gain`提高到1.2的
对照轮丢失明显增多且无命中，因此各入口仍保持已验证的1.0。

## 三项新增验证链（2026-07-29）

正式控制器的`SEARCH`现已接入`StartupSearch`：安全起飞后或相机/IMM全部丢失时保持
高度、按最后估计目标速度有界衰减跟随，并持续偏航扫描。参数为
`search_pattern_enabled`、`search_yaw_rate_rad_s`、`search_pattern_turns`和
`search_repeat_until_acquired`；扫描状态和进度发布在
`/tracking/pure_interceptor/status`。

坐标驱动相机测试有三个并行入口：

```bash
# 单PX4，虚拟运动靶，全程自动Offboard
ros2 launch px4_offboard_tracking coordinate_camera_auto.launch.py \
  target_source:=virtual target_speed_mps:=5.0

# 单PX4或实物拦截机，人工起飞并用遥控器切入/切出Offboard
ros2 launch px4_offboard_tracking coordinate_camera_manual.launch.py \
  target_source:=external_px4

# 实物拦截机，切入Offboard后在当前机位正上方25 m生成虚拟敌机
ros2 launch px4_offboard_tracking coordinate_camera_manual_real.launch.py \
  hover_thrust:=0.42 \
  maximum_thrust:=0.70 \
  target_scenario:=straight target_speed_mps:=2.0
```

上述推力值对应 2.76 kg、T6143 机体（2200 电池配置）的实测值；更换电池或载荷
配置后必须重新标定 `hover_thrust`。实机入口默认使用 Fast DDS，
嵌套仿真入口仍默认使用 CycloneDDS。

`target_state_source`可内部生成运动靶，也可订阅另一台飞控的
`/target/fmu/out/vehicle_global_position`和`vehicle_local_position`。坐标只进入
`simulation_truth_bridge`和两个逻辑相机，正式估计器/控制器仍只收规范相机消息和本机
PX4状态；`pure_interceptor_real.launch.py`不包含该测试源。实物虚拟敌机入口需要有效
的本机`VehicleLocalPosition`位置和速度，不使用WGS84公共原点。若切入时本地NED为
`(x,y,z)`，目标严格从`(x,y,z-25)`开始，且会跟随PX4的本地位置重置量平移锚点；
同时必须显式填写实测悬停推力和本架次最大推力，缺少参数时拒绝启动。

终端瞄准把“提前居中”和“允许发射”分开：连续跟踪默认2 s后，以1 s斜坡从宽跟踪锥
平滑收紧；攻击延时只负责开火授权。`ShotTimingOptimizer`检查当前误差、0.04 s预测
误差、误差角速度和稳定保持；1°优选开火窗与2°硬门继续保留。使用10 s攻击延时的
5 m/s历史回归中，
搜索30 Hz/双目20 Hz动态验证中，双PX4两次误差为
0.464°/0.892°，单PX4虚拟坐标靶为0.644°/0.392°；当时旧裁判的理想射线结果均为
2/2，不能替代新版实际Body -Z物理轴验收。

完整命令、外部坐标话题、人工切出停发和统一数据记录见
[`无人机实物与仿真实验统一手册`](../../docs/交付/无人机实物与仿真实验统一手册.md)。

5 m/s直线、圆形、S形和8字形历史轮均已完成两次射击；其旧裁判2/2结果只保留为
理想射线历史记录，后续以新版实际Body -Z结果为准。
每场六张图片、原始数据和中文报告位于
`/home/key/codex_work/5mps预测视场屏障分阶段距离验收_20260728/`。

## 并行最小下方跟踪闭环

`simple_below_tracker` 是不改变正式拦截器的实物验证基线。它只订阅规范双目
`/perception/depth/observation` 和本机 PX4 状态，不启动搜索交接、IMM-EKF、预测射击或
裁判。控制目标仅为消除目标在本机 NED 水平面的相对偏差，垂直方向锁定启用时的本机高度，
因此不会主动爬升到目标高度。

两种模式使用同一套安全门和速度 Offboard 输出：

- `direct`：水平速度与当前双目相对水平位置成比例；
- `predictive`：由连续双目观测估计相对速度，再加本机PX4速度恢复目标公共速度，对目标水平
  投影前视默认 `0.2 s` 后使用同一控制律。

两者均限制水平速度、加速度和最大预测距离。节点已启用且保持在Offboard期间，观测过期、
目标不在上方、距离越界、本机状态过期或速度超限时，水平设定值按加速度限制平滑回零；
垂直通道继续保持锁存高度。控制被维护接口关闭或人工切出Offboard后不再发布控制设定值。
状态话题为：

```text
/tracking/simple_below/status
```

### 双目逻辑相机仿真

先按正式流程启动 DDS Agent 和两架 PX4/Gazebo，但不要同时启动其它 Pursuer 控制器。然后
分别运行两种基线：

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash

ros2 launch px4_offboard_tracking simple_below_tracker_sim.launch.py \
  tracking_mode:=direct

ros2 launch px4_offboard_tracking simple_below_tracker_sim.launch.py \
  tracking_mode:=predictive prediction_horizon_s:=0.2
```

仿真入口分别启动 `logical_search_camera_simulator` 和
`logical_depth_camera_simulator` 两个进程，独立发布双相机 V1 消息。真值只进入逻辑相机；
简单控制节点不订阅 `/evaluation/*`。这个深度专用基线没有搜索相机交接，因此默认目标高度为
45 m、Pursuer为5 m，使实际双机生成偏置处于未放宽的10°双目视场内。观察：

```bash
ros2 topic echo /tracking/simple_below/status
ros2 topic hz /perception/depth/observation
```

对比两轮的 `relative_position_ned.x/y`、`predicted_relative_position_ned.x/y` 和
`velocity_setpoint_ned.x/y`，即可判断0.2 s前视是否降低匀速目标下的稳态滞后。

逐终端运行、参数覆盖和数据采集命令见
[`无人机实物与仿真实验统一手册`](../../docs/交付/无人机实物与仿真实验统一手册.md)。
仿真launch默认保留自动起飞模式；也可通过文档中的四个`pursuer_*`参数切换为
QGroundControl人工起飞、人工切入/切出Offboard的实机流程演练。

### 实物飞机入口

实机入口默认 `auto_engage=false`、`control_enabled=true`、`takeoff_enabled=false`、
`auto_disable_on_offboard_exit=true`。它不会自动解锁、不会自动切换 Offboard，也不会
自动起飞。默认 PX4 前缀是单机 `/fmu`，速度和加速度限制比仿真更保守：
`1.5 m/s` 和 `0.8 m/s²`。

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
ros2 launch px4_offboard_tracking simple_below_tracker_real.launch.py \
  tracking_mode:=predictive prediction_horizon_s:=0.2
```

实物验证顺序：

1. 在地面启动节点，再由人工在受控场地起飞并稳定悬停；
2. 确认双目采集时间戳、坐标方向、深度和PX4本机状态均正常；
3. 节点从地面开始只预发送`OffboardControlMode`心跳；人工模式飞行期间只更新内部锁存，
   不发布任何轨迹、姿态、角速度或推力setpoint；
4. 由操作者切入 Offboard，节点收到新鲜`VehicleStatus`确认后立即冻结锁存并开始发布
   setpoint；只有已解锁、已处于Offboard、目标在上方且双目新鲜时才开始水平跟踪；
5. 正常退出时先由操作者切回Position等人工模式。节点检测到曾进入Offboard又切出后，
   会先关闭自身，再停止发布`OffboardControlMode`和`TrajectorySetpoint`，随后人工降落；
6. `false` 是停发控制流的维护/紧急接口。在仍处于Offboard时直接停发会触发PX4配置的
   Offboard丢失保护，不应替代正常的人工切模顺序。

状态中的`setpoint_streaming`只表示本节点当前是否正在发布实际控制setpoint；进入
Offboard前应为`false`，此时仍有独立的`OffboardControlMode`心跳流；
`offboard_session_seen`表示本次启动后是否确认进入过Offboard；
`stopped_after_offboard_exit`表示是否因人工切出而自动停发。切出检测以PX4
`VehicleStatus`反馈为准，因此在飞控状态反馈抵达前可能已经有一条在途样本；节点在识别
切出后的同一控制周期、下一次发布前停止，不会继续形成设定值流。

搜索相机和双目相机是两个独立话题、独立序号和独立采集时间戳，正式控制器分别订阅并按
各自新鲜度处理，不要求两路同帧或同频。最小下方跟踪只订阅双目话题。仿真同样使用两个
独立进程和定时器，`search_camera_rate_hz`、`depth_camera_rate_hz` 可分别配置；
两者默认均为30 Hz。异步回归可显式设置
`search_camera_rate_hz:=30.0 depth_camera_rate_hz:=20.0`。

实物敌机由另一台电脑独立执行其 PX4 Offboard 任务。本机实机 launch 只接入拦截机自身
`/fmu` 和两路相机消息，不会启动敌机任务、向敌机发送命令或等待敌机状态。

必须保留人工接管、PX4 Offboard丢失保护和独立安全观察员。首次实物试验应使用低速目标、
宽阔场地和更低的 `maximum_horizontal_speed_mps`；该最小节点不是完整避障或任务安全系统。
完整的移植、编译、相机异步检查、direct/predictive分飞和退出核查命令见
[`无人机实物与仿真实验统一手册`](../../docs/交付/无人机实物与仿真实验统一手册.md)。

本包用于在 ROS 2 Humble 中验证 PX4 Offboard 控制与双无人机追踪算法。
当前仿真后端采用 **PX4 SITL + Gazebo Harmonic（gzsim）**，不依赖 UE，因而更适合
日常开发、算法调试和自动化测试。后续如需接入 UE，只需替换状态或传感器输入，
不需要修改控制算法。

## 历史纯控制链与P7验收基线（2026-07-27）

正式双机采用实例0 `x500/4001` 目标机与实例1
`gz_my_quad_real/22001` 追踪机。两机外部生成高度均为0.02 m，模型SDF内部的
贴地偏移保持不变。

当前正式控制器不再使用主动MPC或固定高度/垂直走廊。执行链为：

```text
搜索相机纯方位发现
  → 姿态接管
  → 双目深度确认
  → IMM-EKF公共NED相对状态
  → 统一三维APN/径向闭合速度制导
  → 姿态+总推力
  → 打击
  → 连续制导脱离
  → 搜索或双目重入
  → 第二次打击
```

打击距离窗口为15～35 m，径向速度控制以17 m为调节目标，使最终射击保持在约
15～18 m并给15 m硬下界保留控制余量；该范围不是固定高度
或位置走廊。攻击轴始终为机体`-Z`，2°为不可放宽的最差对准门，终端制导先进入
1°目标区，真正开火要求稳定1°窗口。
当前自动仿真首次打击授权默认等待10 s；人工实机以切入Offboard为任务起点、固定延时为
0 s，但仍至少完成3 s连续跟踪保持、2 s终端瞄准准备和1 s平滑收紧，并通过全部射击门。
每次事件使用不同的新鲜双目帧，两次打击间隔至少3 s。

打击后的脱离仍在统一姿态/加速度链中执行，不切回世界固定位置。目标暂时离开
双目后，搜索阶段用最后一份相机/IMM估计的目标公共速度进行匹配，并为每次重新
进入SEARCH提供完整扫描窗口；该过程不读取目标真值。真值只存在于仿真相机节点
和离线评估侧。

V1闭环验收采用目标5 m/s逃离、追踪指令上限8 m/s和安全上限9 m/s。稳定跟踪
阶段在双目安全视场内保留调速自由度，攻击使能后才将攻击轴约束收紧到优选1°；
最终两次打击距离25.112/24.634 m、间隔4.709 s，真实机体-Z/LOS误差
1.440°/0.068°，追踪机峰值速度4.781 m/s，无安全中止。更高速档位按当前任务
要求暂停。

仿真配置入口：

```bash
ros2 launch px4_offboard_tracking p7_interceptor_sim.launch.py \
  profile:=v1 encounter:=escape
```

`config/high_speed_profiles.yaml`仍保留后续档位定义，但在明确恢复提速任务前不继续执行。

仿真中两机同时完成起飞，但追踪机保持在`TAKEOFF_SAFE`，不会开始搜索。目标机
先等待5 s并开始3 s速度坡道；确认目标已经沿轨迹运动0.2 s后，仿真编排层只
发布一次“任务允许”布尔信号，追踪机才进入`SEARCH`。目标机不再等待追踪机
进入`INTERCEPT`。搜索、双目交接及首次攻击前10 s仿真等待会与目标轨迹并行进行，
所以进入攻击判定时目标早已完成速度坡道并稳定运行。该信号只允许
任务开始，不携带目标位置、速度、距离或真值，也不参与估计、制导和打击判定；
实机入口默认不要求该仿真信号，闭环仍只使用搜索相机、双目相机和追踪机自身
PX4状态。

该时序已用V0目标2 m/s逃离场景闭环验证：追踪机先完成安全起飞，目标随后启动
轨迹，之后才进入搜索；两次打击距离24.716/27.213 m、间隔4.493 s，峰值速度
4.887 m/s，无安全中止。结果位于
`P7距离窗口验收_20260727/20260727_120444_v0_escape/`。V1目标5 m/s在同一新
时序下仍会发生双目交接丢失或过近安全中止，已保留为下一步控制优化项，未通过
放宽安全限制处理。

P7验收结束后，可直接从状态CSV和两架PX4 ULog生成五张中文图片，不需要重新飞行：

```bash
cd ~/UAV_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run px4_offboard_tracking plot_p7_acceptance \
  ~/codex_work/项目验收与回退归档/验收结果/P7距离窗口验收_20260727/20260727_113237_v1_escape
```

图片直接写入该轮结果根目录：

- `01_三维轨迹与攻击轴.png`
- `02_双相机可见性与瞄准.png`
- `03_距离窗口与闭合速度.png`
- `04_状态机速度与姿态.png`
- `05_轨迹正交投影.png`

原始`data/`、ULog和失败记录均不会移动或覆盖。

第二张图明确区分三个量：紫色为深度相机实测的机体`-Z`与LOS误差，灰色为经过
制导、限速和加速度平滑后真正发送给PX4的期望攻击轴与LOS误差，青色为实际
机体`-Z`跟踪期望攻击轴的误差。打击星标同时绘制在搜索、双目和IMM有效性行，
便于核验事件当帧的数据链完整性；旧版21列状态CSV仍可读取，但无法补造当时未
记录的真实攻击轴误差。

第一张三维图改用两机全球真值统一转换到同一局部NED，不再把两份独立本地轨迹
归零后按名义生成偏移拼接。在每次打击位置增加完整姿态坐标架：红色为机体`+X`、绿色为机体
`+Y`、紫色为实际攻击轴`-Z`，并标注该时刻的Roll/Pitch/Yaw。细紫线仍表示飞行
过程中稀疏采样的连续攻击轴，二者不应混淆。第五张图给出北-东俯视、东-高度
和北-高度三种正交投影，并连接两次打击时的真实LOS。

所有 PX4 飞行状态与控制指令均采用本地 **NED 坐标系**：

- x：北（North）
- y：东（East）
- z：下（Down），因此飞行高度 5 m 通常对应本地 z 约为 -5 m

## 设计结构

- `OffboardVehicle`：负责 PX4 DDS 话题、Offboard 指令预热、模式切换、解锁和状态超时保护。
- `StartupSearch`：可选的起始搜索模块；起飞稳定后保持位置并原地偏航扫描，稳定发现目标后提前停止并交给双相机接管链。
- `TrackingController`：不依赖 ROS、Gazebo 或 UE 的速度跟踪基线算法。
- `AccelerationGuidance`：由目标和追踪者状态生成期望 NED 加速度及期望 yaw。
- `MpcGuidance`：负责双目深度确认后的三维中段接近。
- `TerminalGuidance`：水平距离10 m内使用N/E/D三轴APN末段制导。
- `GeometricControl`：将期望加速度转换为姿态/推力或机体系角速度/推力。
- `TargetTrajectory`：生成静止、直线和小圆三种与仿真后端无关的目标轨迹。
- `CommonFrameTransform`：使用 WGS84 椭球高建立共享原点，并将双机全局位置转换为公共 NED。
- `CameraModel`：与 ROS/Gazebo 无关的理想针孔投影及 body FRD 到 optical frame 变换。
- `target_observation`：组合公共相对位置和 Pursuer 姿态，发布理想相机目标观测。
- `RelativeEkf`：公共 NED 下的相对位置、相对速度六状态恒速 EKF。
- `relative_estimator`：相机反投影、观测协方差传播、姿态时间同步和 EKF 发布节点。
- `RelativeKinematics`：计算距离、接近速度、LOS 角速度、TCPA 和预测脱靶距离。
- `role=hold`：单机起飞并悬停，用于验证最小 Offboard 控制链路。
- `role=target`：控制 Target 沿圆周飞行，并发布统一的目标状态。
- `role=pursuer`：真值只用于生成模拟相机观测；控制器只接受有效的 EKF 相对估计。

Pursuer 支持三个控制层级：

| `control_level` | PX4 输入消息 | PX4 内部保留的控制环 |
|---|---|---|
| `velocity` | `TrajectorySetpoint` | 速度、姿态、角速度和控制分配 |
| `attitude` | `VehicleAttitudeSetpoint` | 姿态、角速度和控制分配 |
| `body_rate` | `VehicleRatesSetpoint` | 角速度和控制分配 |

双机 launch 默认使用 `attitude`，先满足姿态推力控制并降低首次测试风险。确认推力标定、
坐标方向和姿态响应正确后，再显式切换到 `body_rate`。

起始搜索默认关闭，日常快速仿真不会增加等待时间。展示时可在原启动命令后增加：

```bash
startup_search_enabled:=true \
startup_search_yaw_rate_rad_s:=0.5 \
startup_search_turns:=1.0 \
startup_search_settle_time_s:=0.5 \
startup_search_stop_on_acquisition:=true \
startup_search_repeat_until_acquired:=true \
startup_search_acquisition_hold_time_s:=0.2
```

偏航角速度正值为逆时针、负值为顺时针，圈数支持小数。搜索仅作用于 Pursuer：
达到起飞高度后保持水平位置和高度；搜索相机连续发现目标后进入只使用单位视线
的姿态接管，双目连续确认0.2 s且至少5帧后才启动IMM-EKF和MPC。两个相机均
丢失时重新扫描；关闭搜索后，原有快速仿真链保持不变。

目标先起飞、启用搜索的两种专用测试：

```bash
cd ~/UAV_ws
./scripts/run_search_takeoff_test.sh stationary
./scripts/run_search_takeoff_test.sh straight
```

脚本只接受静止和匀速直线场景；直线默认1.2 m/s。每轮完成后会自动绘制三维
姿态、双目视野、搜索视野和打击误差四张图，并把CSV和日志移入该轮的
`原始数据/`。外部生成高度默认0.01 m；如果自研模型地面初始化持续出现航向
估计错误，只按既定小步规则重试：

```bash
SPAWN_Z_M=0.02 ./scripts/run_search_takeoff_test.sh straight
```

仿真目标真值接口为：

```text
话题：/tracking/target/state
类型：nav_msgs/msg/Odometry
坐标系：map_ned
```

当前 Target 节点将 PX4 融合后的 WGS84 全局位置转换为公共 NED，作为仿真真值。
该话题只用于构造相机仿真输入，不再直接进入 Pursuer 控制器。

双机公共坐标接口为：

```text
公共原点：/tracking/common_origin（sensor_msgs/msg/NavSatFix，transient-local）
Target公共状态：/tracking/target/state（nav_msgs/msg/Odometry）
仿真真值相对状态：/tracking/relative/state（nav_msgs/msg/Odometry）
EKF 相对估计：/tracking/relative/estimate（RelativeEstimate）
相对运动学：/tracking/relative/kinematics（RelativeKinematics）
坐标系：map_ned
```

Target 使用第一帧有效 `VehicleGlobalPosition` 作为公共 WGS84 原点。两架飞机均使用
`alt_ellipsoid` 经 ECEF 转换到同一个局部 NED；本地位置只用于各自 PX4 的轨迹设定，
不再参与双机位置相减。NED 速度轴不受平移原点影响，速度继续取各自有效的
`VehicleLocalPosition`。

## 当前验证路线

```text
Gazebo Harmonic
      ↓ 仿真传感器与动力学
PX4 SITL（Target x500 + Pursuer 22001_gz_my_quad_real）
      ↓ uXRCE-DDS
Micro XRCE-DDS Agent
      ↓ ROS 2 话题
OffboardVehicle → TrackingController
```

UE 仿真暂不参与当前阶段，以降低 CPU、GPU 和内存占用。已有 UE 包不会被本包调用，
也不需要在运行 gzsim 时启动。

## 编译与算法测试

```bash
cd ~/UAV_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to px4_offboard_tracking
source install/setup.bash
colcon test --packages-select px4_offboard_tracking
colcon test-result --verbose
```

预期结果：`px4_msgs` 和 `px4_offboard_tracking` 编译成功；速度制导、加速度制导、
悬停姿态推力、加速方向和角速度限幅测试全部通过。

## 历史测试：单架 x500 起飞悬停（当前正式验收不再执行）

先关闭不需要的 UE、旧 PX4、旧 Gazebo 和重复 Agent 进程，避免端口或话题冲突。

终端 1，启动 DDS Agent：

```bash
MicroXRCEAgent udp4 -p 8888
```

终端 2，启动 PX4 SITL 与 Gazebo Harmonic x500：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```

终端 3，先确认 PX4 数据已经进入 ROS 2：

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
ros2 topic list | grep '^/fmu/'
ros2 topic hz /fmu/out/vehicle_local_position
```

确认位置话题持续更新后，启动单机 Offboard。启动前应打开 QGroundControl，并确保
仿真区域安全：

```bash
ros2 launch px4_offboard_tracking single_x500.launch.py takeoff_height_m:=5.0
```

检查 PX4 状态：

```bash
ros2 topic echo /fmu/out/vehicle_status --once
ros2 topic echo /fmu/out/vehicle_local_position
```

预期结果：

- `nav_state: 14`，表示 Offboard 模式；
- `arming_state: 2`，表示已解锁；
- x、y 保持在起飞点附近；
- z 逐渐收敛到约 -5 m。

停止 ROS 2 launch 会中断 Offboard 设定值。此时 PX4 将执行参数中配置的 Offboard
丢失保护动作。本节点不会在飞行中直接强制反解锁。

## 测试二：双机 Target-Pursuer

双机 gzsim 需要两个 PX4 SITL 实例：

- 实例 0：Target，MAVLink system ID 为 1，DDS 话题通常位于 `/fmu/...`；
- 实例 1：Pursuer，MAVLink system ID 为 2，DDS 话题通常位于 `/px4_1/fmu/...`。

不同 PX4 构建环境的 Gazebo 多实例启动命令可能不同，因此 ROS launch 不负责创建
仿真器，而是通过参数接收两个 DDS 话题前缀。启动控制前必须确认两个位置话题均存在：

```bash
ros2 topic list | grep -E '^/(fmu|px4_1/fmu)/out/vehicle_local_position$'
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /px4_1/fmu/out/vehicle_local_position
```

两个 PX4 实例都正常发布位置后，首先使用默认姿态推力模式启动追踪：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py
```

该模式发布 `VehicleAttitudeSetpoint.q_d` 和 FRD 机体系推力。多旋翼推力使用
`thrust_body = [0, 0, -thrust]`。当前 gz_x500 实测悬停推力约为 0.725，配置使用
`hover_thrust=0.725`、`max_thrust=0.9`。更换机型、质量或动力模型后必须重新标定。
确认以下项目后才能继续：

- 静止悬停时 roll、pitch 接近 0；
- 向北加速时产生正确的低头姿态；
- 高度没有持续发散；
- 推力没有长时间触及 0 或 `max_thrust`；
- QGroundControl 中没有姿态、角速度或 Offboard failsafe。

姿态推力测试稳定后，切换到角速度推力：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py control_level:=body_rate
```

角速度模式发布 `VehicleRatesSetpoint`，ROS 2 几何控制器负责姿态误差到机体系角速度
的转换，PX4 继续负责角速度 PID、控制分配和电机输出。初次测试应保持默认的
`max_body_rate_rad_s:=3.0` 或进一步降低，不要一开始提高角速度及最大推力。

检查目标状态接口：

```bash
ros2 topic hz /tracking/target/state
ros2 topic echo /tracking/target/state --once
```

默认预期行为：

- Target 在约 12 m 高度以 4 m 半径做小圆运动；
- Target 水平速度约为 1 m/s；
- Pursuer 独立保持约 5 m 高度，因此 Target 初始位于其上方约 7 m；
- Pursuer 最大水平速度为 5 m/s；
- Pursuer 接近 Target 后保持约 3 m 水平距离；
- EKF 估计无效、协方差非法或数据超过 0.5 s 时，Pursuer 水平指令归零并保持高度。

可通过 QGroundControl 或以下话题观察两机轨迹：

```bash
ros2 topic echo /fmu/out/vehicle_local_position
ros2 topic echo /px4_1/fmu/out/vehicle_local_position
```

双机水平制导使用公共 `map_ned` 下的 EKF 相对位置和速度。若公共本机位置或 EKF
估计超过 `state_timeout_s`，或估计的 `valid` 为 false，Pursuer 禁止使用旧状态，
水平指令归零并保持自身高度。

### 公共 NED 与真实相对距离验收

确认 PX4 全局位置与三个 tracking 话题持续更新：

```bash
ros2 topic hz /fmu/out/vehicle_global_position
ros2 topic hz /px4_1/fmu/out/vehicle_global_position
ros2 topic echo /tracking/common_origin --once
ros2 topic hz /tracking/target/state
ros2 topic hz /tracking/relative/state
```

查看公共 NED 下 Target 相对 Pursuer 的位置和速度：

```bash
ros2 topic echo /tracking/relative/state
```

其中 `pose.pose.position` 是 `[north, east, down]` 相对位置。静止场景稳定后应满足：

- `z` 约为 `-7 m`，表示 Target 位于 Pursuer 上方；
- 水平分量与 Gazebo 中两机实际水平间距一致；
- 三维距离为 `sqrt(x*x + y*y + z*z)`；
- 不允许再用两个 `/vehicle_local_position` 的 x/y/z 直接相减做验收。

### 理想向上相机观测

Milestone 3 使用标准 optical frame：

- `+Z`：相机前方，默认与 body FRD 的 `-Z` 重合；
- `+X`：图像向右，默认与 body FRD 的 `+Y` 重合；
- `+Y`：图像向下，默认与 body FRD 的 `+X` 重合。

默认 body FRD 到 optical frame 的 Hamilton 四元数为：

```text
[w, x, y, z] = [0, 0.70710678, 0.70710678, 0]
```

相机平移外参表示“相机原点相对机体原点的位置”，单位为米并使用 body FRD。旋转和平移
均通过 ROS 参数提供，不依赖 Gazebo 模型中的固定传感器名称或接口。

理想观测话题：

```text
话题：/tracking/camera/observation
类型：px4_offboard_tracking/msg/TargetObservation
frame_id：pursuer/camera_optical
```

消息包含：

- `header.stamp`：输入公共相对状态的时间戳；
- `position_camera`：目标相对相机的 optical frame 三维位置；
- `pixel_u`、`pixel_v`：理想针孔像素坐标；
- `depth`：optical `+Z` 深度；
- `visible`：深度为正且像素位于图像边界内。

默认双机 launch 会同时启动理想观测节点。可显式关闭：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  enable_camera_observation:=false
```

配置相机内参与平移外参示例：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=stationary \
  camera_width:=640 camera_height:=480 \
  camera_fx:=320.0 camera_fy:=320.0 \
  camera_cx:=319.5 camera_cy:=239.5 \
  camera_translation_body_z_m:=-0.10
```

运行后检查：

```bash
ros2 interface show px4_offboard_tracking/msg/TargetObservation
ros2 topic hz /tracking/camera/observation
ros2 topic echo /tracking/camera/observation
```

静止且 Pursuer 基本水平、Target 位于正上方时，预期：

- `position_camera.x/y` 接近 0；
- `depth` 接近两机垂直间距；
- `pixel_u/v` 接近 `(319.5, 239.5)`；
- `visible: true`。

若目标位于相机后方、深度小于阈值、像素越界、相对状态过期或姿态过期，则
`visible: false`。Milestone 3 不添加像素噪声、深度噪声、延迟、丢帧或异常点。

### 相机帧率与观测故障注入

早期版本的 Offboard 定时器固定为 50 ms，相对状态和相机观测因此只有约 20 Hz。
现在两个频率分别参数化：

- `offboard_rate_hz`：Offboard 设定值和公共相对状态频率，默认 50 Hz；
- `camera_rate_hz`：相机独立采样频率，默认 50 Hz；
- `offboard_warmup_s`：进入 Offboard/解锁前的设定值预热时间，默认仍为 1 s。

检查频率：

```bash
ros2 topic hz /tracking/relative/state
ros2 topic hz /tracking/camera/observation
```

无丢帧且系统负载正常时，两者应接近 50 Hz。可按算力调整，例如：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  offboard_rate_hz:=60.0 camera_rate_hz:=60.0
```

Milestone 4 支持以下可独立开关的观测退化：

| 功能 | 开关 | 主要参数 |
|---|---|---|
| 像素高斯噪声 | `enable_pixel_noise` | `pixel_noise_stddev_px` |
| 深度高斯噪声 | `enable_depth_noise` | `depth_noise_stddev_m` |
| 固定发布延迟 | `enable_latency` | `latency_ms` |
| 随机丢帧 | `enable_dropout` | `dropout_probability` |
| 随机异常观测 | `enable_outliers` | `outlier_probability`、深度范围 |

综合故障场景示例：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=circle \
  camera_rate_hz:=50.0 \
  enable_pixel_noise:=true pixel_noise_stddev_px:=1.5 \
  enable_depth_noise:=true depth_noise_stddev_m:=0.10 \
  enable_latency:=true latency_ms:=100.0 \
  enable_dropout:=true dropout_probability:=0.10 \
  enable_outliers:=true outlier_probability:=0.02 \
  observation_random_seed:=42
```

行为约定：

- 延迟队列保留采集时刻 `header.stamp`，不会把发布时间伪装成采集时间；
- 丢帧时不发布该帧，因此输出频率约为 `camera_rate_hz * (1-dropout_probability)`；
- 像素或深度加噪后会重新反投影 `position_camera`，避免三维位置绕过噪声；
- 异常点在图像范围和配置深度范围内随机生成；
- `outlier_injected` 仅用于仿真记录和测试，不得作为后续 EKF/质量门的输入真值标签；
- 所有故障开关默认关闭，固定随机种子使测试可重复。

### 六状态相对 EKF（Milestone 5）

状态向量为公共 NED 下的：

```text
x = [relative_north, relative_east, relative_down,
     relative_vn,    relative_ve,   relative_vd]
```

滤波器采用恒速状态转移和白噪声加速度模型。观测节点根据针孔反投影的 Jacobian，
将像素和深度噪声传播为三维观测协方差，再结合与采集时间最接近的 Pursuer 姿态，
把 optical frame 观测转换到公共 NED。异常观测使用三自由度 NIS 门限拒绝；节点不会
读取 `outlier_injected` 真值标签。时间倒退、不可见、非有限值和姿态不同步的观测也会
被拒绝。

估计接口：

```text
话题：/tracking/relative/estimate
类型：px4_offboard_tracking/msg/RelativeEstimate
坐标系：map_ned
```

消息包含六状态、6×6 协方差、三维创新、创新 Mahalanobis 距离、最近一次被接受观测的
数据年龄和 `valid`。`valid` 同时检查数据年龄、位置方差和数值有限性。控制器再次检查
时间戳、`valid`、数据年龄和协方差对角线；任一检查失败都会停止水平追踪并保持高度。

查看估计频率和内容：

```bash
ros2 topic hz /tracking/relative/estimate
ros2 topic echo /tracking/relative/estimate
```

正常可见且系统负载允许时，频率应接近 `estimator_rate_hz`（默认 50 Hz），`valid` 应为
true，`data_age` 接近 0。可用以下场景检查异常拒绝和超时：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=stationary \
  enable_outliers:=true outlier_probability:=0.10 \
  enable_dropout:=true dropout_probability:=0.20 \
  observation_random_seed:=42
```

异常点出现时可看到创新或 NIS 增大，但估计节点不得根据 `outlier_injected` 字段作弊；
长时间无有效观测后 `data_age` 增大并最终令 `valid: false`。

### 相对运动学（Milestone 6）

`relative_kinematics` 仅消费有效 EKF 估计并发布：

```text
话题：/tracking/relative/kinematics
类型：px4_offboard_tracking/msg/RelativeKinematics
```

定义如下，其中 `r` 和 `v` 分别为公共 NED 下的相对位置和相对速度：

- `range = |r|`；
- `closing_speed = -(r·v)/|r|`，接近为正、远离为负；
- `los_rate = (r×v)/|r|²`，单位 rad/s；
- `time_to_closest_approach = max(0, -(r·v)/|v|²)`；
- `predicted_miss_distance = |r + v * time_to_closest_approach|`。

检查命令：

```bash
ros2 topic hz /tracking/relative/kinematics
ros2 topic echo /tracking/relative/kinematics
```

静止上下对准场景中，`range` 应接近垂直间距，`closing_speed` 和 `los_rate` 应接近 0；
圆轨迹场景中应出现有限的横向 LOS 角速度。零距离或非有限输入会令运动学结果无效。
相对运动学模块只提供估计和预测量；下游接近走廊单独由以下模块实现。

### 向上相机接近走廊（Milestone 7）

attitude/body-rate 低层制导现在先根据 EKF 相对状态构造预测接近走廊，而不是继续使用
原有的 3 m 水平 standoff 直接追踪当前目标位置：

```text
predicted_relative = relative_position + limited(relative_velocity * prediction_time)
desired_relative   = [0, 0, -vertical_separation]
corridor_error     = predicted_relative - desired_relative
```

默认预测时间 0.8 s、预测提前量最大 2 m、期望垂直间距 7 m。水平加速度由走廊位置误差
和相对速度阻尼共同生成并限幅为 2 m/s²。现阶段垂直通道继续保持 Pursuer 自身 5 m
高度，B5/B6 高度子模式将在后续任务状态机阶段处理。

如果估计失效、过期或目标不在 Pursuer 上方，低层控制不再简单恢复水平姿态，而是使用
Pursuer 自身速度生成限幅的水平阻尼刹车，同时保持高度。这一回退不会继续使用旧目标
状态。

关键参数：

- `corridor_vertical_separation_m`；
- `corridor_prediction_time_s`；
- `corridor_maximum_lead_distance_m`；
- `corridor_horizontal_position_gain`；
- `corridor_horizontal_velocity_gain`；
- `corridor_maximum_horizontal_acceleration_mps2`；
- `invalid_estimate_velocity_damping`。

### body -Z 预测 LOS 对准（Milestone 8）

attitude 模式使用标准 optical `+Z = body -Z`。`ZAxisAlignment` 计算预测 LOS 和当前
body `-Z` 方向，并将 LOS 倾角限制在 `alignment_maximum_tilt_rad` 内。

为避免远距离时将全部集体推力直接指向 LOS 导致欠阻尼漂飞，对准采用分阶段权重：

- 水平距离大于 2 m，或相对水平速度大于 1 m/s：走廊平移和速度阻尼优先；
- 进入走廊且速度下降后：连续增加预测 LOS 对准权重；
- 接近目标正下方并稳定后：body `-Z` 收敛到预测 LOS。

最终姿态由 `AttitudeSetpointGenerator` 生成。它执行：

- Hamilton 四元数符号连续处理；
- 最短弧 SLERP；
- 最大姿态变化率限制；
- 推力随姿态使用同一平滑权重；
- 根据相机轴的垂直分量进行倾角推力补偿；
- `minimum_thrust`/`max_thrust` 限幅。

关键参数：

- `alignment_maximum_tilt_rad`；
- `alignment_entry_horizontal_distance_m`；
- `alignment_entry_relative_speed_mps`；
- `attitude_smoothing_time_constant_s`；
- `maximum_attitude_slew_rate_rad_s`。

低风险验收命令：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=stationary \
  control_level:=attitude \
  enable_trajectory_recorder:=true
```

验收时应同时检查相机像素到主点 `(319.5, 239.5)` 的误差、Pursuer 水平轨迹是否有界、
高度是否保持和 EKF `valid`。Milestone 8 只修改 attitude 路径；body-rate 外环仍保留为
后续 Milestone 15。

### 双机轨迹记录与可视化

轨迹工具不订阅 Gazebo 接口，只使用稳定的 tracking 数据契约：

- `/tracking/target/state`：Target 的公共 NED 真值；
- `/tracking/relative/state`：Target 相对 Pursuer 的公共 NED 真值；
- `/tracking/camera/observation`：相机像素、深度和可见性；
- `/tracking/relative/estimate`：EKF 六状态、完整 6×6 协方差、创新量、马氏距离、数据年龄和 `valid`；
- `/tracking/relative/kinematics`：距离、相对速度、闭合速度、LOS 角速度、最近接时间和预测脱靶量；
- `/tracking/alignment/feasibility`：倾角、推力裕度、角速度、高度损失、距离、对准时长和可达性；
- `/px4_1/fmu/out/vehicle_attitude`：Pursuer 实际四元数，据此计算机体 +X 和机体 -Z 在公共 NED 中的方向。

Pursuer 公共位置由 `target_common - target_relative_to_pursuer` 重建，不会直接相减
两架 PX4 各自的原始 `VehicleLocalPosition`。CSV 中保存 North、East、Down；三维图将
纵轴显示为直观的 `Altitude = -Down`。

当前 CSV 共 130 列，完整保留以下数据，而不只保存绘图所需的精简字段：

- Target/Pursuer 公共 NED 位置、速度和姿态四元数；
- Pursuer 航向、机体 +X（机头）和机体 -Z（相机/攻击轴）；
- 相机三维坐标、像素、深度、可见性、异常注入和观测时间戳；
- EKF 相对位置/速度、36 项协方差、创新量、马氏距离、年龄和有效性；
- 完整相对运动学、对准可达性、任务状态、质量门、捕获窗口和安全状态；
- 捕获连续保持时间、不同观测时间戳帧数、各子条件以及累计打击次数。

启动控制时同时记录：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=stationary \
  control_level:=velocity \
  enable_trajectory_recorder:=true
```

默认输出到：

```text
~/UAV_ws/flight_records/tracking_YYYYmmdd_HHMMSS.csv
```

也可以指定目录和文件名：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  enable_trajectory_recorder:=true \
  trajectory_output_directory:=/tmp/tracking_test \
  trajectory_filename:=stationary.csv
```

停止 launch 后生成两张图：

```bash
ros2 run px4_offboard_tracking plot_trajectories \
  ~/UAV_ws/flight_records/tracking_YYYYmmdd_HHMMSS.csv
```

输出文件：

- `trajectory_3d.png`：公共 NED 中 Target 与 Pursuer 的三维轨迹；
- `camera_track.png`：标准 optical frame 图像平面上的像素轨迹，颜色表示经过时间。

有桌面显示环境时可追加 `--show` 打开图窗；WSL 无图形显示时仍会使用无界面后端正常
保存 PNG。相机图的 v 轴向下，与标准图像坐标一致；不可见帧保留在 CSV 中，但不会作为
有效像素点画入相机轨迹。

### Milestone 1 目标场景

`target_scenario` 支持以下值：

| 场景 | 行为 |
|---|---|
| `stationary` | Target 在起飞点上方 12 m 静止悬停 |
| `straight` | Target 按 `target_heading_rad` 航向以恒定速度直线飞行 |
| `circle` | Target 以配置的半径和速度做圆周运动，默认场景 |

静止目标：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=stationary target_height_m:=12.0 pursuer_height_m:=5.0
```

向北直线目标（NED 航向 0 rad）：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=straight target_heading_rad:=0.0 target_speed_mps:=1.0
```

默认小圆目标：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  target_scenario:=circle target_radius_m:=4.0 target_speed_mps:=1.0
```

直线场景当前没有航程边界，会持续沿指定航向生成位置设定值；只应在受控的短时仿真
测试中使用。

## 参数调整

常用参数位于 `config/tracking.yaml`：

- `position_gain`：位置误差比例增益；
- `velocity_feedforward`：Target 速度前馈系数；
- `max_horizontal_speed_mps`：最大水平追踪速度；
- `max_vertical_speed_mps`：最大垂直速度；
- `standoff_distance_m`：期望水平跟踪距离；
- `state_timeout_s`：目标或本机状态超时时间。
- `target_scenario`：`stationary`、`straight` 或 `circle`；
- `target_heading_rad`：直线场景的 NED 航向角；
- `target_radius_m`、`target_speed_mps`：小圆半径与目标速度；
- `altitude_hold_position_gain`、`altitude_hold_velocity_gain`：Pursuer 高度保持增益；
- `acceleration_position_gain`：距离误差到期望加速度的比例增益；
- `acceleration_velocity_gain`：相对速度阻尼增益；
- `max_horizontal_acceleration_mps2`：最大水平期望加速度；
- `max_vertical_acceleration_mps2`：最大垂直期望加速度；
- `hover_thrust`：归一化悬停推力，必须针对 x500 实际悬停值校准；
- `max_thrust`：归一化集体推力上限；
- `max_tilt_rad`：最大倾角；
- `attitude_gain`：姿态误差到角速度设定值的增益；
- `max_body_rate_rad_s`：三轴角速度设定值绝对上限。

建议先在低速、大间距下验证，再逐渐提高 Target 和 Pursuer 速度。

## 后续替换为真实视觉输入

视觉节点需要完成以下工作：

1. 从相机图像中检测 Target；
2. 估计 Target 相对位置和速度；
3. 按 `TargetObservation` 契约发布像素、深度、采集时间和可见性；
4. 使用与当前仿真相同的 `relative_estimator` 和 `RelativeEkf`；
5. 停止启动真值观测生成节点 `target_observation`。

只要观测话题契约、相机外参和时间同步规则保持一致，EKF、控制算法与 PX4 Offboard
通信层都无需改动。

## Milestone 9：对准可达性

姿态模式会发布：

```text
/tracking/alignment/feasibility
```

消息 `AlignmentFeasibility` 分别报告倾角、剩余推力、机体角速度、高度损失、最小距离和
最长对准时间是否满足限制。当前 PX4 DDS 配置未发布 `VehicleAngularVelocity` 时，节点会
根据连续姿态四元数估计角速度模长；如果以后该 PX4 话题可用，则会直接采用它。

默认调参值包括：最大可达倾角 0.30 rad、最小推力余量 0.08、最大角速度 2.0 rad/s、
最大高度损失 1.2 m、最小距离 2.0 m、单次最长对准时间 4.0 s。任何输入无效或过期都会
令 `feasible=false`，不会用旧数据继续判定。

验收：

```bash
ros2 topic echo /tracking/alignment/feasibility
```

应能看到各子条件，而不是只有一个无法定位原因的总布尔量。

## Milestone 10：B0-B12 任务状态机

任务状态发布到：

```text
/tracking/mission/status
```

当前已实现 B0 至 B12 的状态和转移框架。B5 高度纠偏与 B6 侧向纠偏作为 B3/B4 的临时
子模式，完成后返回原父状态。M11 质量门和 M12 捕获窗口已接入状态机：B7 只有在所有质量
条件通过时才进入 B8，B8 只有在捕获窗口连续保持后才进入 B9。

进入 B4 瞄准不要求 Pursuer 位于 Target 正下方。默认条件为：水平距离不大于 2.5 m、
三维距离不大于 8.5 m、相对速度不大于 1.2 m/s，且 M9 判定可达。退出侧向阈值为 3.2 m，
形成滞环。B3 至少保持 1 s 后才重新进入 B4；强 LOS 对准只在 B4/B7/B8 生效，返回 B3 后
立即恢复接近走廊控制。

启动和观察：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude enable_trajectory_recorder:=true

ros2 topic echo /tracking/mission/status
```

圆轨迹调参试飞中，状态可重复完成 `B1 -> B2 -> B3 -> B4 -> B7`。状态门控后相机可见率
为 87.2%，EKF 有效率为 87.1%，试飞结束时水平误差约 0.80 m、像素误差约 55 px。
验收记录位于：

```text
~/UAV_ws/flight_records/m10_state_gated.csv
~/UAV_ws/flight_records/m10_state_gated_plots/trajectory_3d.png
~/UAV_ws/flight_records/m10_state_gated_plots/camera_track.png
```

“保持一定距离进入瞄准”和“满足捕获窗口”现已分别由 B4 入口、B7 质量门和 B8 连续保持
判定负责。进入 B9 后会发布纯 ROS 2 `virtual_capture_event` 状态快照，随后立即进入恢复；
该事件不会连接或触发任何真实硬件输出。

## Milestone 11：跟踪质量门

Pursuer 发布：

```text
/tracking/quality_gate
```

`TrackingQuality` 会分别报告 EKF 有效性、最大位置方差、创新马氏距离、相机可见性、观测与
估计数据年龄、姿态是否饱和以及 M9 对准可达性。所有子条件同时满足时 `passed=true`，状态机
才允许从 B7 进入 B8；任一数据过期或无效都不会继续使用旧状态。

默认门限为：最大位置方差 4.0、最大创新马氏距离 11.345、观测和估计最大年龄 0.2 s。
验收时运行：

```bash
ros2 topic echo /tracking/quality_gate
ros2 topic echo /tracking/mission/status
```

应能看到各子条件以及 `B7_QUALITY_GATE -> B8_CAPTURE_WINDOW` 转移。若未通过，可直接根据
消息中的独立布尔量定位失败条件。

## Milestone 12：虚拟捕获窗口

Pursuer 发布：

```text
/tracking/capture_window
```

`CaptureWindowStatus` 同时检查质量门、像素误差、机体 -Z/LOS 对准误差、距离、LOS 角速度、
相对速度、最近接时间和预测脱靶距离。默认门限为 60 px、0.12 rad、2.0--8.5 m、
0.25 rad/s、1.2 m/s、预测脱靶距离 8.5 m、最近接时间 4.0 s。一次虚拟捕获必须同时满足：

- 所有瞬时条件连续成立至少 0.4 s；
- 窗口内至少收到 8 个不同观测时间戳的有效相机帧。

控制循环重复处理同一相机帧不会增加帧计数。任何条件失败、观测无效或时间戳倒退都会立即
清零计时和帧计数。若实际相机只能稳定在约 20 Hz，可显式使用
`capture_required_hold_time_s:=0.3 capture_minimum_distinct_frames:=6`；不建议降低到 3--4 帧。

启动与验收：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude enable_trajectory_recorder:=true

ros2 topic echo /tracking/capture_window
ros2 topic echo /tracking/mission/status
```

2026-07-13 的圆轨迹双机验收依次达到：

```text
B2_TAKEOFF_HOLD -> B3_APPROACH -> B4_ALIGN -> B7_QUALITY_GATE
-> B8_CAPTURE_WINDOW -> B9_VIRTUAL_CAPTURE
```

B8 在窗口连续满足约 1 s 后进入 B9。该记录是旧门限基线；当前默认验收以 0.4 s/8 个不同
时间戳为准。验收文件为：

```text
~/UAV_ws/flight_records/m12_capture_window.csv
~/UAV_ws/flight_records/m12_capture_window_plots/trajectory_3d.png
~/UAV_ws/flight_records/m12_capture_window_plots/camera_track.png
```

M13/M14 已实现，因此 B9 会发布一次虚拟事件并进入恢复；消息仍然不会产生真实硬件输出。

## Milestone 13：虚拟捕获事件与快照

每次进入 B9 时，Pursuer 只发布一次：

```text
/tracking/virtual_capture_event
```

`VirtualCaptureEvent` 包含单调递增的 `event_id`、公共 NED 下 Pursuer 和估计 Target 位置、
相对位置/速度、像素与对准误差、距离、LOS 角速度、相对速度、预测结果、EKF 方差与创新量、
窗口保持时间。事件发布后状态按 `B9 -> B10 -> B12` 推进，不会停留在 B9 重复发布。

验收：

```bash
ros2 topic echo /tracking/virtual_capture_event
ros2 topic echo /tracking/mission/status
```

应看到每次捕获只有一个事件编号，随后立即进入 `B10_POST_CAPTURE` 和 `B12_RECOVERY`。

## Milestone 14：安全中止、恢复与重新进入

独立 `SafetyManager` 发布：

```text
/tracking/safety/status
```

默认检查有效本机状态、有效姿态、最小距离 1.5 m、实际倾角 0.45 rad、速度 4.0 m/s 和
高度误差 2.0 m。任一安全条件失败会进入 B11；B10/B11/B12 期间停止追击，使用水平制动、
定高和回正姿态。恢复需满足倾角不大于 0.12 rad、速度不大于 0.5 m/s、高度误差不大于
0.6 m，并连续保持 1.0 s，才从 B12 回到 B1 重新评估目标。B12 内安全条件再次恶化会返回
B11，而不会强行重入跟踪。

默认场景启动：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py control_level:=attitude

ros2 topic echo /tracking/safety/status
ros2 topic echo /tracking/mission/status
```

安全分支仿真验收可以临时收紧高度门限；这不会修改默认参数：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude \
  safety_maximum_altitude_error_m:=0.2 \
  recovery_maximum_altitude_error_m:=0.1
```

2026-07-13 默认圆轨迹试飞在 80 s 内发布 6 个独立虚拟事件，每次均完成
`B9 -> B10 -> B12 -> B1` 并重新进入跟踪。严格高度门限场景成功触发
`B3 -> B11 -> B12`。默认场景记录位于：

```text
~/UAV_ws/flight_records/m14_recovery_reentry.csv
~/UAV_ws/flight_records/m14_recovery_reentry_plots/trajectory_3d.png
~/UAV_ws/flight_records/m14_recovery_reentry_plots/camera_track.png
```

## Milestone 15：角速度与推力控制

`control_level:=body_rate` 使用独立的姿态外环，将连续姿态目标转换成
`VehicleRatesSetpoint`。滚转/俯仰和偏航使用独立增益与限幅；推力仍由姿态/推力生成器给出。
`attitude` 模式保留为对照组，二者共用同一套观测、EKF、走廊、质量门、捕获窗口和状态机。

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=body_rate target_scenario:=s_curve
```

## Milestone 16：两次虚拟打击最终验收

默认目标为 1.2 m/s 的有界 S 形轨迹；也可设置 `target_scenario:=rectangle`。Pursuer 使用固定
偏航减少无意义的机头持续旋转，同时通过机体 -Z 对准预测 LOS。任务默认需要两次成功：首次
事件后执行 `B9 -> B10 -> B12 -> B1` 恢复和重入，第二次事件后保持 B10 终态并发布
`/tracking/mission/complete=true`。启用自动结束后，launch 延迟 2 s 干净关闭 ROS 节点。

最终 attitude 验收命令：

```bash
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude target_scenario:=s_curve target_speed_mps:=1.2 \
  required_virtual_captures:=2 shutdown_on_mission_complete:=true \
  enable_trajectory_recorder:=true \
  trajectory_output_directory:=$HOME/UAV_ws/flight_records \
  trajectory_filename:=m16_attitude_s_curve_final.csv
```

body-rate 对照只需将 `control_level` 改为 `body_rate`。2026-07-14 实测结果：

| 控制层 | 两次窗口保持时间 | 不同观测帧数 | B8 像素误差中位数 | B8 对准误差中位数 | 任务结果 |
|---|---:|---:|---:|---:|---|
| attitude | 0.412 / 0.409 s | 23 / 23 | 42.13 px | 0.1040 rad | 两次成功，自动退出 |
| body_rate | 0.405 / 0.405 s | 22 / 22 | 45.33 px | 0.0749 rad | 两次成功，自动退出 |

attitude 该次总时长 92.6 s，主要包含一次外部 PX4/DDS 状态流长时间中断后的安全等待；
body-rate 干净试飞总时长 23.1 s。时长差不能直接解释为控制器性能差，几何误差指标更适合
本次对照。记录和图位于：

```text
~/UAV_ws/flight_records/m16_attitude_s_curve_final.csv
~/UAV_ws/flight_records/m16_attitude_s_curve_final_plots/trajectory_3d.png
~/UAV_ws/flight_records/m16_attitude_s_curve_final_plots/camera_track.png
~/UAV_ws/flight_records/m16_body_rate_s_curve_retry.csv
~/UAV_ws/flight_records/m16_body_rate_s_curve_retry_plots/trajectory_3d.png
~/UAV_ws/flight_records/m16_body_rate_s_curve_retry_plots/camera_track.png
```

直接验收捕获证据：

```bash
ros2 topic echo /tracking/capture_window
ros2 topic echo /tracking/virtual_capture_event
ros2 topic echo /tracking/mission/status
```

`CaptureWindowStatus` 中应同时看到 `hold_elapsed >= 0.4`、
`distinct_observation_frames >= 8` 和 `held=true`；每次 `VirtualCaptureEvent` 也保存对应帧数。

### 打击位置可视化

`plot_trajectories` 会把 `capture_held` 的 false-to-true 上升沿识别为独立激发信号，避免同一
窗口的连续 CSV 行被重复标记：

- 三维图用红色星标和蓝色叉标分别标出每次打击时的 Target/Pursuer 位置；
- 黑色虚线连接同一次事件的两机位置，绿色箭头表示 Pursuer 机体 +X（机头）；
- 紫色箭头表示 Pursuer 机体 -Z（相机/攻击轴），并标注它与 LOS 的实际夹角；
- 相机图用金色星标和 `T1/T2` 标出激发时目标的像素位置；
- 新录制的 CSV 使用完整实际四元数绘制两个机体轴；旧 CSV 没有四元数时仅对机头使用固定航向回退。

```bash
ros2 run px4_offboard_tracking plot_trajectories \
  ~/UAV_ws/flight_records/full_s_curve_qos.csv \
  --output-directory ~/UAV_ws/flight_records/full_s_curve_plots
```

新版绘图工具会在试飞外层目录直接生成四张便于查看的图片：

- `01_三维轨迹与攻击轴.png`
- `02_双目相机视野_YOLO中心.png`
- `03_搜索相机视野_YOLO中心.png`
- `04_打击轨迹与误差.png`

默认会把 CSV 移入 `原始数据/CSV/`，把日志移入 `原始数据/日志/`，图片
留在外层。只想绘图而不整理目录时增加 `--no-organize`。改造前工具完整
保存在 `scripts/archive/plotting_v1/`。

双目和搜索相机使用各自视场对应的针孔内参。相机图中的轨迹可解释为理想
YOLO 检测框中心；搜索相机图只使用像素中心和可见性，不向控制算法输出
深度、距离或三维位置。

### 22001 实时姿态和平滑性监视器

`realtime_attitude_monitor` 是独立的只读可视化工具，不发布控制指令，也不替代现有离线三维
轨迹图和相机轨迹图。它订阅 Pursuer 的 PX4 姿态与角速度，在一个实时窗口中显示：

- roll、pitch、yaw 的滚动历史；
- 三轴机体角速度及警告线；
- 角加速度曲线、最近 2 s RMS、峰值角速度和 `SMOOTH/WARNING` 状态；
- 机体 X/Y/Z 轴在公共 NED 方向中的实时三维姿态，以及数据超时提示。

双机仿真和 DDS 话题正常后，在支持图形界面的终端运行：

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
ros2 run px4_offboard_tracking realtime_attitude_monitor
```

默认监视 `/px4_1/fmu/out/vehicle_attitude` 和
`/px4_1/fmu/out/vehicle_angular_velocity`，显示最近 15 s，界面刷新 10 Hz。可按仿真或实机
安全限制调整告警阈值，例如：

```bash
ros2 run px4_offboard_tracking realtime_attitude_monitor \
  --window 20 --refresh-rate 10 \
  --rate-warning 1.5 --acceleration-warning 6.0
```

橙色虚线与红色 `WARNING` 只用于调试提示，不会修改安全状态机或触发控制动作。关闭图形窗口
或按 `Ctrl+C` 即可退出监视器，不影响正在运行的 Offboard 任务。

### S 形、圆形、矩形完整记录验收

三类场景均采用 attitude 控制、Target 速度 1.2 m/s、首次打击后恢复重入、累计两次后自动结束。
每次打击必须连续满足窗口至少 0.4 s，并包含至少 8 个不同观测时间戳。2026-07-14 实测如下：

| 场景 | CSV | 记录行/列 | 两次保持时间 | 两次不同观测帧 | 打击时 -Z/LOS 夹角 | 结果 |
|---|---|---:|---|---|---|---|
| S 形 | `full_s_curve_qos.csv` | 3851 / 130 | 0.405 / 0.407 s | 22 / 22 | 5.00° / 3.79° | 两次成功，自动结束 |
| 圆形 | `full_circle.csv` | 1405 / 130 | 0.408 / 0.404 s | 22 / 22 | 1.10° / 5.86° | 两次成功，自动结束 |
| 矩形 | `full_rectangle.csv` | 1327 / 130 | 0.406 / 0.404 s | 22 / 22 | 7.20° / 5.28° | 两次成功，自动结束 |

对应 CSV、三维轨迹和相机轨迹均位于 `~/UAV_ws/flight_records/`。S 形测试中外部
PX4/Gazebo 状态流曾短时中断，安全管理器执行 B11/B12 后重新获取目标并完成第二次打击；
这验证了恢复重入路径，但也说明 WSL 下仍需关注仿真状态流的偶发停顿。

### 2°中心激发规则

当前常规激发规则为：目标相对相机主光轴误差不超过2°，连续保持至少0.2 s，
并包含至少5个不同观测时间戳；严格验收档为0.3 s/8帧。当前双目逻辑相机
水平焦距为12847.020 px，因此2°水平等效像素门约为448.628 px。相机图显示
红色光心、红色2°虚线区域以及T1/T2激发点。

接近走廊仍使用 0.8 s 前视预测，但进入 B4/B7/B8 后，相机对准使用当前 LOS，且只有在
水平误差不超过 0.8 m、相对速度不超过 0.8 m/s 后才进入完整光轴对准，避免把较大的 LOS
倾角变成持续水平加速。

2026-07-14 严格窗口实测：

| 场景 | 速度 | 两次保持时间 | 独立帧 | 激发像素误差 | 对准误差 | 结果 |
|---|---:|---|---|---|---|---|
| S 形 | 1.2 m/s | 0.311 / 0.312 s | 17 / 18 | 8.57 / 8.57 px | 1.40° / 1.43° | 两次激发 |
| 圆形 | 1.2 m/s | 0.320 / 0.320 s | 17 / 17 | 10.57 / 4.39 px | 1.76° / 0.69° | 两次激发 |
| 矩形 | 1.0 m/s | 0.307 / 0.301 s | 17 / 14 | 11.13 / 7.65 px | 1.93° / 1.24° | 两次激发 |

矩形 1.2 m/s 直角速度跳变压力测试未通过：最长保持 0.200 s。该结果没有通过放宽 2°门限
规避；矩形验收速度保持为 1.0 m/s。三维图的 `async truth` 是记录器异步订阅状态拼接得到的
诊断角，激发条件以同一捕获状态中的 `trigger center` 和 `capture_alignment_error_rad` 为准。

### 远距离与4K相机新默认配置

当前运行配置已把Target/Pursuer高度调整为35 m/5 m，接近走廊期望垂直间距为30 m，
虚拟捕获允许距离为25--35 m。B4进入条件中的1.0 m是公共NED水平距离
`sqrt(relative_n^2 + relative_e^2)`，不是单独的纵坐标限制；退出阈值为1.5 m。

双目逻辑相机默认内参升级为3840x2160、`fx=12847.020`、
`fy=6124.984`、主点`(1919.5,1079.5)`，有效视场为水平17°、垂直20°。
2°水平等效像素门相应换算为约448.628 px。进入瞄准走廊只表示几何条件适合开始对准，
最终虚拟捕获仍必须同时通过2度像素/光轴、LOS角速度、相对速度、预测和连续帧门。

S形轨迹改为周期连续的8字/S形参数曲线，不再在端点瞬间反向；矩形改为定速直线与
四分之一圆弧组成的圆角矩形。两者的Target yaw均由轨迹切线计算，因此目标机机头方向
与瞬时运动方向一致。上面的历史CSV仍对应旧的7 m近距离配置，不能直接作为新配置验收结果。

2026-07-15使用理想真值投影、理想深度且关闭噪声、延迟、丢帧和异常值，对30 m保守走廊
重新验收：

| 场景 | 速度 | 两次距离 | 两次像素误差 | 两次对准误差 | 保持时间/独立帧 | 结果 |
|---|---:|---|---|---|---|---|
| S形 | 1.2 m/s | 29.164/29.746 m | 28.412/22.406 px | 0.789°/0.613° | 0.320 s、17帧/0.320 s、17帧 | 两次成功 |
| 圆形 | 1.2 m/s | 29.304/29.652 m | 26.654/20.946 px | 0.754°/0.556° | 0.312 s、18帧/0.318 s、18帧 | 两次成功 |
| 圆角矩形 | 1.2 m/s | 29.047/29.418 m | 25.801/21.278 px | 0.753°/0.589° | 0.309 s、17帧/0.307 s、17帧 | 两次成功 |

三场均未出现安全中止，均完成首次捕获、恢复重入和第二次捕获。记录文件名分别为
`range30_conservative_s_curve.csv`、`range30_conservative_circle.csv`和
`range30_conservative_rectangle.csv`，CSV仍作为本地实验数据，不纳入Git提交。

随后在1.2 m/s圆角矩形、理想真值相机/理想深度条件下，仅扩大“开始瞄准”的水平走廊，
下面这批历史试验使用严格2度/0.3 s/8帧捕获门：

| B4水平入口 | 两次距离 | 两次像素误差 | 两次对准误差 | 保持时间/独立帧 | 结果 |
|---:|---|---|---|---|---|
| 2 m | 29.503/29.757 m | 22.127/35.624 px | 0.640°/1.003° | 0.312 s、17帧/0.317 s、17帧 | 两次成功 |
| 3 m | 29.619/29.684 m | 30.475/11.327 px | 0.885°/0.322° | 0.304 s、16帧/0.320 s、17帧 | 两次成功 |
| 5 m | 29.613/30.341 m | 29.269/9.005 px | 0.844°/0.267° | 0.320 s、17帧/0.320 s、17帧 | 两次成功 |

三轮均无安全中止。5 m约等于30 m距离下9.5度LOS偏角，仍低于当前0.28 rad
（约16度）对准倾角限制。该结论只说明可更早进入B4对准，不代表把最终打击窗口放宽到5 m；
最终事件依然在严格2度中心窗口内产生。launch现支持
`aim_entry_horizontal_distance_m`、`aim_exit_horizontal_distance_m`和
`alignment_entry_horizontal_distance_m`覆盖，默认值仍保守保持1.0/1.5/1.5 m。

以下是旧B状态链的历史试验配置，不是当前完整入口默认值：当时任务时序设置
`attack_enable_delay_s=30.0`，Target在
`target_motion_delay_s=20.0`后才开始轨迹运动，Pursuer可提前起飞、观测和进行B3走廊跟踪，
但必须等Target沿轨迹运行约10 s后才允许从B3进入B4/B7/B8。这样不会再以“刚达到足够高度”
作为立即开始虚拟打击的条件。该时间门独立于高度、距离、相对速度和质量门，所有条件仍需
同时满足。

首次虚拟打击后还设置`reattack_cooldown_s=3.0`。恢复、重新估计和B3走廊跟踪可在冷却期
继续进行，但距离上次`virtual_capture_event`不足3 s时，`approach_ready`保持false，
禁止再次进入B4/B7/B8。因此第二次虚拟打击事件与第一次事件至少间隔3 s；该门只约束任务
节奏，不虚构当前模型中不存在的后坐力动力学。

最终联合验收采用圆角矩形1.2 m/s、30 m间距、5 m瞄准入口、理想真值相机和理想深度。
首次事件距离Pursuer本地任务计时起点约30.57 s；两次事件间隔3.720 s。两次触发距离为
30.168/30.919 m，像素误差24.019/30.174 px，对准误差0.677°/0.839°，保持时间和独立帧为
0.300 s、16帧/0.320 s、17帧，且没有安全中止。

本地实验文件统一整理在`flight_records`：根目录只保留S形、圆形和圆角矩形各自的
`best_*_trajectory_3d.png`与`best_*_camera_track.png`，共6张最佳图；三种最终CSV和
其余历史运行统一放入`flight_records/archive/`的分批子目录，实验数据不纳入Git。

三种最终统一条件记录如下：

| 场景 | 两次距离 | 两次像素误差 | 两次对准误差 | 两事件间隔 | 安全恢复 |
|---|---|---|---|---:|---|
| S形 | 29.488/30.434 m | 32.557/35.344 px | 0.914°/1.065° | 8.420 s | 无安全中止 |
| 圆形 | 30.185/30.402 m | 16.437/13.431 px | 0.512°/0.425° | 7.240 s | 一次状态短失效，B11/B12恢复后完成 |
| 圆角矩形 | 30.168/30.919 m | 24.019/30.174 px | 0.677°/0.839° | 3.720 s | 无安全中止 |

### 两机机头随轨迹转向

Target的yaw始终由轨迹切线计算，圆形、S形和圆角矩形都会让机头沿瞬时运动方向转动。
Pursuer默认从固定yaw改为`pursuer_yaw_mode:=velocity`：水平速度不小于0.3 m/s时，
yaw取`atan2(v_east,v_north)`；低速或悬停时保持上一次有效yaw，避免速度噪声造成机头抖动。
仍可用`fixed`保持固定航向，或用`los`让机头水平投影指向目标。

该修改主要改善飞行方向一致性、可视化和未来前向传感器/机载设备的朝向。上视相机的主光轴
是机体-Z轴，绕该轴改变yaw理论上不会改变目标是否位于光心，因此它不直接放宽或提升2度
捕获窗口；快速yaw还会占用部分角速度裕度，所以仍需通过联合仿真确认。

### PD与线性MPC制导对照

Pursuer的平移制导可通过`guidance_controller`选择：`pd`保留为接近走廊位置/速度PD回归
基线；`mpc`使用六状态相对位置/速度中的水平四状态，在有限预测时域内最小化
位置误差、相对速度和控制代价。MPC采用目标短时匀速假设，并显式限制水平加速度范数；
垂直方向继续由独立高度保持器控制，姿态/推力、质量门和任务状态机保持不变。

```bash
# PD基线
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude guidance_controller:=pd

# MPC对照
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude guidance_controller:=mpc

# 实验性三维MPC（水平与垂直相对运动均进入预测控制）
ros2 launch px4_offboard_tracking dual_tracking.launch.py \
  control_level:=attitude guidance_controller:=mpc_3d
```

默认MPC参数为`dt=0.1 s`、预测25步，位置/速度/加速度权重分别为
`1.0/4.0/1.5`，终端权重倍率为`4.0`，最大水平加速度为`1.3 m/s²`。

首次圆角矩形1.2 m/s对照中，PD与MPC均完成两次虚拟打击且无安全中止。MPC将两次
打击间隔从13.92 s缩短到7.70 s，水平误差RMS从2.502 m小幅降到2.469 m；但首次
打击更晚，相对速度RMS从1.847 m/s升至2.198 m/s，触发角误差也由0.792°/0.617°
升至0.960°/0.999°。

随后进行了三轮MPC调参。综合最佳参数为位置/速度/控制权重`1.0/4.0/1.5`、最大水平
加速度`1.3 m/s²`：首次触发33.43 s、两次事件间隔3.38 s、水平误差RMS 2.688 m、
相对速度RMS 2.028 m/s，两次触发角误差0.794°/0.162°，无安全中止。相比首版MPC，
首次触发提前约10.2 s、重入缩短4.32 s，速度RMS降低约7.7%，触发精度明显提高；但
水平RMS和速度RMS仍未全面超过PD。因此二维`mpc`不作为默认方案，只保留为快速重入对照；
后续通过三维`mpc_3d`完成的正式验收结果见下文。

`mpc_3d`在同一预测器中增加垂直相对位置、相对速度和加速度代价，目标是保持目标机位于
追踪机上方约30 m。为防止相对高度估计瞬态直接造成掉高，其垂直输出与原绝对高度保持器
按`0.3/0.7`融合，并限制垂直加速度为`0.5 m/s²`。1.2 m/s圆形第二轮试验完成两次
打击，间隔7.30 s、最大指令倾角0.2804 rad且无安全中止；但最大高度损失约4.99 m，
过程中进入过一次高度修正，表现尚未达到二维MPC基线。因此该模式只用于后续调参验证，
该模式完成三场景正式验收后已设为仿真默认控制器；迁移实机前仍需重新辨识质量、悬停推力、
加速度与姿态限幅，并从更保守的垂向输出开始验证。

三维模式还使用入口稳定门：垂直间距误差不超过1.0 m、估计相对垂直速度不超过0.5 m/s，
且任务已越过30 s禁打击窗口，并连续保持2.0 s后才接通B3和三维MPC。目标机仍在起飞时，
追踪机保持自身起飞高度并停留在等待状态。该门只作用于`mpc_3d`首次进入；第一次打击后的
恢复重入不额外等待，也不改变PD、二维MPC或任务安全阈值。

加入稳定入口门后的圆形第七轮为当前最佳三维MPC结果：两次打击间隔5.38 s，B3--B9
垂直间距误差RMS 0.801 m、最大误差1.009 m，最大垂直速度0.206 m/s、最大指令倾角
0.1389 rad，跟踪阶段高度损失为0，且无安全中止。启动阶段不再进入B5；仍观察到一次短暂
估计重捕获，后续优化重点应放在相对估计连续性，而不是继续增大垂向控制增益。

1.2 m/s三场景正式验收中，三维MPC的两次打击间隔分别为圆形5.68 s、S形4.52 s、
矩形5.26 s；对应PD基线为7.86 s、6.56 s、29.82 s，分别缩短27.7%、31.1%和
82.4%。三轮均完成两次打击、无安全中止、启动阶段无B5。S形水平误差RMS由2.518 m
降至1.493 m、相对速度RMS由1.796 m/s降至0.978 m/s；矩形分别由2.714 m降至
1.337 m、由1.531 m/s降至0.753 m/s。圆形B4--B9稳态水平误差由0.826 m降至
0.741 m。因此项目仿真默认制导现为`mpc_3d`，使用`guidance_controller:=pd`可显式回退。

### 后续迁移到PX4内部的边界

迁移时不需要把本项目的自动起飞和降落流程写入新跟踪模块。起飞、就位、任务结束后的返航或
降落继续交给PX4已有飞行模式；跟踪模块只在飞行器已经解锁、处于允许高度并由上层任务明确
激活后运行。应迁移的范围是公共坐标/相对状态接口之后的相对EKF、运动学、PD/MPC接近制导、
走廊、-Z轴对准、质量门、两次虚拟打击与恢复/重入状态机，以及安全中止判定。

ROS 2或伴随计算机不应继续向迁移后的模块发送姿态、角速度或推力Offboard指令，而只发送带
时间戳、有效标志和协方差的视觉观测，例如像素`u/v`、深度（或尺度信息）、相机内参标识和
参数化外参。PX4内部先把观测转换为相对状态并完成质量检查，再调用跟踪状态机和制导器，最后
通过PX4内部setpoint/uORB接口交给已有姿态或角速度控制环。仅传“相机参数”不够，必须同时
持续传入每帧目标观测；相机内外参属于配置，目标像素/深度/时间戳属于实时数据。

推荐数据链为：

```text
ROS视觉检测/深度
  -> 时间戳观测 + 相机配置（DDS/uORB桥）
  -> PX4相对状态估计与质量门
  -> 跟踪/恢复/重入状态机
  -> PD或MPC平移加速度
  -> -Z轴姿态对准与推力生成
  -> PX4姿态或角速度内环
```

在完成PX4内部迁移后，ROS Offboard节点只保留为仿真对照工具，不再作为实机控制链的一部分。
