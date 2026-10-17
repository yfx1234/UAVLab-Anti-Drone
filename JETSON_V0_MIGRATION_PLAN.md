# Jetson ROS 2 V0 项目主台账

> 最后核验：2026-08-14（Asia/Shanghai）  
> 本文件是项目唯一管理台账、交接入口和当前状态依据。历史方案、阶段总结与重复验收文档已经合并到此处；源码包内 README、接口说明和操作脚本属于技术资料，不属于平行台账。

## 1. 一页交接结论

项目当前已经实现并验证软件层完整闭环，并已在真实目标实飞中到达 INTERCEPT 阶段：

`搜索 → 双目测距 → 相对状态估计 → 拦截制导 → 姿态/加速度控制输出 → FIRE → COMPLETE → 制动 → 稳定悬停 → 飞手人工退出 Offboard`

当前结论必须按以下边界理解：

| 项目 | 状态 | 结论 |
| --- | --- | --- |
| 三相机采集与 DeepStream 检测 | PASS | 生产默认 CSI 25 FPS、搜索 USB 解码前抽帧 15 FPS（2026-08-13 修复 USB 时间戳积压） |
| 相机物理布局 | PASS | **前后双目**：left=前相机（机头侧，CAM1/sensor-id 1）、right=后相机（机尾侧，CAM0/sensor-id 0），双镜头+搜索均朝天；实物与上游规范系差 90° 滚转，由桥接层补偿参数吸收（2026-08-13 定案） |
| 双目标定与测距 | PASS（带修正） | 2026-08-04 棋盘标定 + 08-05 RTK 验证；08-13 发现支架机械漂移，已用实飞数据拟合修正（垂直残差 66→8.6px，距离影响 1.4%）；**正式重标定 + 修正归零是 pending 项** |
| 异步测量、KF 和搜索/深度桥 | PASS | V1 接口、时间戳、rectified solver、安装滚转补偿、距离速率门均已实飞验证 |
| anti-drone 搜索/拦截决策迁移 | PASS | 直接运行上游 `pure_interceptor_controller`，不是复制状态机 |
| 上游 setpoint 等价 | PASS | mode/trajectory/attitude 三类允许消息共 25,678 组 CDR 二进制零差异 |
| FIRE→COMPLETE→稳定悬停 | PASS（Shadow） | 恰好一次 FIRE，完成后稳定悬停并置 `safe_to_exit_offboard=true` |
| 真实目标实飞（8-13/8-14 五架次） | PROGRESS | Offboard 切入、搜索捕获、方位跟踪、双目交接、深度捕获、INTERCEPT **均已真实到达**；INTERCEPT 稳定保持与虚拟开火待下一架次验证（目标须布置在 8-12 m） |
| 真实目标现场 observation | WAIVED | 2026-08-12 项目负责人明确决定不作为本轮 Live 前置 |
| S2 故障注入 | ACCEPTED | 2026-08-12 项目负责人确认既有断流/TTL/接管结果通过并接受复用 |
| S3 无桨台架 | PASS（确认复用） | 逐轴姿态、推力范围、Offboard 退出、RC 接管、断流 TTL 很早已完成，项目负责人再次确认通过 |
| S4/S5 分级实飞 | PASS（虚拟目标） | 搜索/跟踪、接近和完整闭环此前已用虚拟目标分级通过，项目负责人确认复用 |
| 真实 PX4 Live 输出 | AUTHORIZED | 需每次显式 `live + allow_live_control=true + calibration_ready=true` 三确认；仍由飞手人工切入/退出 Offboard |

一句话交接：

> 软件完整闭环与控制载荷等价已通过；2026-08-13/14 真实目标实飞五架次已推进到 INTERCEPT。当前系统处于"全链路可走通、参数为外场调优态"：双目为数据拟合修正（非正式标定）、搜索新鲜度门 0.45 s、EKF 固定延迟 0.20 s、检出置信度 0.35、垂直门 25 px——这些外场特调值在第 16 节有完整记录与恢复指引。剩余硬前置：双目重标定、8-12 m 目标距离下的稳定 INTERCEPT 与虚拟开火验证。安全契约不变：人工起飞/切 Offboard/退出、永久拦截 VehicleCommand、异常立即人工接管。

## 2. 不可破坏的安全契约

1. 人工起飞，人工切入 Offboard，任务完成后飞手人工退出 Offboard。
2. 软件不得自动解锁、自动起飞、自动切模式或自动 LAND。
3. 生产上游只允许以下 PX4 输入载荷进入安全门：
   - `OffboardControlMode`
   - `TrajectorySetpoint`
   - `VehicleAttitudeSetpoint`
4. `VehicleCommand`、body-rate、direct actuator、thrust-and-torque 永久拦截。
5. `px4_upstream_adapter_node` 默认保持：
   - `mode=shadow`
   - `allow_live_control=false`
   - `calibration_ready=false`
6. Shadow 模式不创建真实 `/fmu/in/*` publisher。
7. 相机、ROS launch 和录包必须用单次 Ctrl-C/SIGINT 干净退出；不要用 `kill -9` 释放 Argus/GStreamer。
8. Live 只能在当次飞手/场地确认后显式同时给出三个参数；每架次留下 bag、日志和飞手确认。

## 3. 权威位置与版本关系

### 3.1 板端生产基准

- 主机：`uavlab-desktop`
- 用户：`uavlab`
- IP（2026-08-11）：`192.168.1.44`
- 项目根：`/home/uavlab/Jetson_ROS2_V0`
- ROS 工作区：`/home/uavlab/Jetson_ROS2_V0/ros2_ws`
- 生产源码：`/home/uavlab/Jetson_ROS2_V0/ros2_ws/src`

板端项目根外仍有一个当前运行依赖：`/home/uavlab/px4_bridge_ws`。`micro-xrce-agent.service` 通过项目内 `deploy/start_micro_xrce_agent.sh` source 该工作区的 `install/setup.bash`，因此不得当作遗留目录删除；若后续迁移，必须同步修改启动脚本并重启、复核服务。

板端是实际构建、测试和运行过的生产事实基准。本地 `D:\Jetson_ROS2_V0\ros2_ws\src` 已于 2026-08-11 从板端回同步；仅保留三个不参与板端运行的主机侧辅助文件：

- `falcon_intercept/config/camera/camera_frames_solidworks_raw.csv`
- `falcon_intercept/test/fixtures/timestamp_sync_events.csv`
- `falcon_intercept_deepstream/tools/postprocess_stereo_training_video.ps1`

### 3.2 上游 anti-drone

- 本地参考仓库：`D:\Jetson_ROS2_V0\anti-drone`
- 分支：`feat/predictive-shot-referee-v1`
- 上游基线：`645a365`（2026-08-12 由 `e601ff2` 推进；含实物 Offboard 心跳安全门禁与 3v1 协同混合仿真完善）
- 状态：基线上叠加未提交实机修复；不要用 `git reset --hard` 或直接覆盖 `ros2_ws/src/px4_offboard_tracking`。
- 2026-08-12 心跳门禁（重要行为变化）：切入 Offboard 前控制器只发布 `OffboardControlMode` 心跳，位置/速度/姿态设定值在收到新鲜 Offboard VehicleStatus 后才放行（`setpoint_streaming()` 新语义）。起飞前正确状态变为 `setpoint_streaming=false` + 心跳存在 + 轨迹/姿态话题无消息，与 08-06 前旧手册"setpoint_streaming=true"口径相反；板端 `/dryrun/fmu` POSCTL 替身实测：心跳 FLOWING、轨迹/姿态 SILENT、状态字段一致。
- 2026-08-12 限速器对齐：上游新速度限制器默认软区 `2.5 m/s`，与实机虚拟靶入口旧默认命令限速 `2.0 m/s` 冲突（软区≥限速时控制器构造拒绝启动）。按项目负责人指示全部采用上游统一包络 `12.0/14.0` 与新限速器默认（软区 2.5、刹车增益 4.0、刹车加速度 5.0）；对上游唯一 launch 偏离为 `coordinate_camera_manual_real.launch.py` 的 `2.0/3.0→12.0/14.0`。显式收紧速度时命令限速必须大于 `2.5`。

当前生产搜索、任务决策、预测射击、姿态控制和完成态语义来自上游 `px4_offboard_tracking`。Falcon 新代码只负责感知适配、输出隔离和 Shadow/Live 安全门。

### 3.3 文档权威

- 唯一项目台账：根目录本文件。
- 包内 README：只解释单包接口或使用方法。
- `ros2_ws/docs/交付`：保留上游实验手册与仿真资料，不作为当前板端状态依据。
- 原始 bag、标定照片、录像和日志是证据，不是台账。

## 4. 板端系统基线

| 项 | 当前值 |
| --- | --- |
| OS | Ubuntu 22.04.5 LTS |
| Jetson BSP | L4T R36.4.3 |
| Kernel | 5.15.148-tegra, aarch64 |
| ROS | ROS 2 Humble |
| 功耗档 | 25W（`nvpmodel -q` 显示 mode 1） |
| 工作盘 | NVMe 233G；2026-08-11 二次清理后使用约 30G、可用约 193G |
| Argus | `nvargus-daemon.service` active |
| PX4 DDS Agent | `micro-xrce-agent.service` enabled + active |
| 串口 | `/dev/ttyTHS1`, 921600 baud |
| 时钟保护 | `falcon-clock-guard.service` enabled + active |

板端当前常驻进程只有 Micro XRCE Agent 和 ROS daemon；没有残留 DeepStream、GStreamer 或闭环控制 launch。

## 5. SSH、环境与构建

### 5.1 SSH

```powershell
ssh -i C:\Users\ASUS\.ssh\jetson_access_ed25519 -o IdentitiesOnly=yes uavlab@192.168.1.44
```

同事电脑应使用自己的私钥；不要复制不受控的私钥文件。连接后：

```bash
cd /home/uavlab/Jetson_ROS2_V0/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

`ROS_LOCALHOST_ONLY=1` 是本机生产运行约束，避免局域网其他 DDS 节点误入。

### 5.2 构建

普通增量构建：

```bash
cd /home/uavlab/Jetson_ROS2_V0/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

闭环相关最小构建：

```bash
colcon build --symlink-install --packages-up-to \
  falcon_intercept falcon_intercept_deepstream px4_offboard_tracking
```

板端时钟曾在断电后回到 1970；时钟保护现已部署。如果文件时间异常导致增量构建未刷新，应先确认时钟，不要盲目删除源码。

### 5.3 回归测试

```bash
python3 -m pytest src/falcon_intercept/test -q
colcon test --packages-select px4_offboard_tracking
colcon test-result --all
```

当前基线：

- `falcon_intercept`：97 passed。
- `px4_offboard_tracking`：43 个测试目标，335 tests，0 errors、0 failures、0 skipped（2026-08-12 `e601ff2` 基线）。
- 闭环完成态自动审计：`passed=true`。

## 6. 目录结构与保留策略

```text
/home/uavlab/Jetson_ROS2_V0/
├── JETSON_V0_MIGRATION_PLAN.md       # 唯一主台账
├── deploy/                           # systemd/时钟/XRCE 部署资产
├── weights/                          # 当前与回退模型资产
├── test_results/                     # 模型、SITL、硬件 Shadow 历史证据
├── flight_yolo_records/              # 唯一现场检测录像
└── ros2_ws/
    ├── src/                          # 生产源码
    ├── build/ install/               # 当前可运行构建；可重建但暂保留
    ├── bags/                         # 最终回放与 RTK 证据
    ├── calibration_records/          # 棋盘格、相机与已知距离原始数据
    ├── calibration_work/             # 标定中间产物
    ├── docs/                         # 上游交付技术资料
    └── scripts/                      # 实验批处理与工具
```

以下大目录虽然占空间，但属于唯一实验资产，不按“缓存”删除：

- `ros2_ws/calibration_records`：约 5.0G。
- `ros2_ws/bags/rtk_field_test`：约 907M。
- `flight_yolo_records`：约 1.1G。
- `test_results`：约 892M。
- `weights`：约 439M。

可以随时重建或清理的内容：

- `ros2_ws/log`
- `__pycache__`、`.pytest_cache`、`*.pyc`
- 明确标为 `invalid`、`superseded` 的临时 bag
- 已被最终 `completion_shadow_v4` 替代的 completion v1/v2/v3
- 零字节 TensorRT `*.lock`

## 7. 相机、模型与标定事实

### 7.1 相机映射与模式

| 逻辑相机 | 物理/设备 | 机体位置（2026-08-13 定案） | 生产模式 |
| --- | --- | --- | --- |
| stereo left | CAM1 / Argus sensor-id 1 | **前相机（机头侧）**，镜头朝天 | 3840×2160，源 25 FPS |
| stereo right | CAM0 / Argus sensor-id 0 | **后相机（机尾侧）**，镜头朝天 | 3840×2160，源 25 FPS |
| search | USB | 镜头朝天，与双目同板同向 | 2560×1440，抽帧 15 FPS |

**关键认知**：代码/话题里的 left/right 只是立体标定对的标签，实物是**前后双目**——基线 0.0991 m 沿机体前后轴（杆臂：左目光心在 RTK 参考点机头方向 +133 mm）。图像水平轴对应机体前后、垂直轴对应机体左右（实物映射：图像左=机头、图像上=机体左侧）。上游规范系假设基线沿机体左右轴，两者相差 90° 滚转，由 `perception_bridge_node` 的安装补偿参数吸收（见 2026-08-13 晚间条目）。不要根据 `sensor-id 0/1` 的数字直觉交换左右，也不要把 left/right 当成机体左右。

### 7.2 DeepStream 模型

生产默认：

- Engine：`weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine`
- ONNX 回退：`weights/candidates/0728/uav_0728_1280x736_fp16.onnx`
- 输出格式：`end_to_end`
- 置信度：`0.50`
- Engine SHA-256：`12e125e571510bd0ddb35e0c83dff259890d560b226c86a770a6faa8acbc3142`

不要把 raw parser 配置与 end-to-end engine 混用。

### 7.3 双目标定

生产文件：

`ros2_ws/src/falcon_intercept/config/camera/stereo_calib.yaml`

当前值来自 2026-08-04 刚性背板 9×6 内角点、25 mm 方格的 36 对照片，最终使用 32 对：

- stereo RMS：0.315 px
- pair error median：0.310 px
- pair error p95：0.628 px
- baseline：0.099061 m
- 左 fx/fy：9941.890 / 9932.893
- 右 fx/fy：9958.926 / 9956.278
- rectified solver 现场修正：`rectified_right_u_offset_px=-1.65`

2026-08-05 RTK 外场验证后，该文件已是 production 默认，不再是“候选未启用”。

搜索相机需特别注意：

- `stereo_calib.yaml` 的 search 段仍是厂家 FOV 估计。
- 当前真实闭环 bridge 实际使用 `perception_bridge_node.py` 的搜索内参默认值：fx=4059.6413、fy=1247.0766、cx=1279.6667、cy=719.6667（2560×1440）。
- 后续若重新标定搜索相机，应统一这两处来源并补逐轴目标验收，不能只改 YAML 注释。

## 8. ROS 2 完整闭环结构

```text
DeepStream detections
  ├─ search detections ─> search_bearing_node ─> CameraBearingObservation
  └─ stereo detections ─> stereo_target_solver ─> AsyncStereoMeasurement
                                      │
                                      └─ target_kf_node

CameraBearingObservation + AsyncStereoMeasurement
  └─ perception_bridge_node
       └─ SearchObservation + DepthObservation (anti_drone_interfaces V1)
            └─ upstream relative_estimator + pure_interceptor_controller
                 └─ /upstream/fmu/in/{offboard_control_mode,
                                      trajectory_setpoint,
                                      vehicle_attitude_setpoint}
                      └─ upstream_gate_node
                           └─ px4_upstream_adapter_node (Shadow 默认)
```

核心接口：

| Topic | 类型 | 用途 |
| --- | --- | --- |
| `/search/detections_json` | `std_msgs/String` | 搜索相机检测 |
| `/stereo/left/detections_json` | `std_msgs/String` | 左目检测 |
| `/stereo/right/detections_json` | `std_msgs/String` | 右目检测 |
| `/perception/search/bearing` | `CameraBearingObservation` | 生产搜索 bearing |
| `/perception/async_stereo_measurement` | `AsyncStereoMeasurement` | 生产双目测量 |
| `/perception/search/observation` | `SearchObservation` | anti-drone V1 搜索输入 |
| `/perception/depth/observation` | `DepthObservation` | anti-drone V1 深度输入 |
| `/tracking/relative/estimate` | `RelativeEstimate` | 上游估计输出 |
| `/tracking/pure_interceptor/status` | `PureInterceptorStatus` | 上游任务/安全状态 |
| `/tracking/virtual_shot_event` | `VirtualShotEvent` | FIRE 证据 |
| `/tracking/mission/complete` | `std_msgs/Bool` | COMPLETE |
| `/intercept/upstream_px4_shadow` | `std_msgs/String` | adapter Shadow 判据 |

坐标约定：

- 相机消息：REP-103 optical（+X 右、+Y 下、+Z 前）。
- PX4/相对状态：NED。
- bridge 不做额外机体系旋转；三维姿态转换仍由上游估计与相机外参契约负责。

## 9. 正式运行入口

### 9.1 每个新终端先执行

```bash
cd /home/uavlab/Jetson_ROS2_V0/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_LOCALHOST_ONLY=1
```

SSH 登录命令见第 5.1 节。以下命令均在板端执行；凡标注“终端 1/2/3”的操作，必须打开独立终端并在每个终端先执行上面的四行。相机是独占资源，原始录像、YOLO 和真实闭环三者不得同时启动两套相机管线。

### 9.2 每次运行前检查

```bash
systemctl is-active nvargus-daemon
systemctl is-active micro-xrce-agent
systemctl is-active falcon-clock-guard
pgrep -af 'deepstream_three_camera_node|gst-launch|real_strike|pure_interceptor'
ros2 topic hz /fmu/out/vehicle_status
df -h /home/uavlab
```

预期三个服务均输出 `active`，`vehicle_status` 有数据，且没有旧相机/闭环进程。没有连接 PX4 时可忽略最后一个 ROS topic 超时，但不得据此进行实机控制。若 Argus 曾异常，确认所有相机进程已退出后执行：

```bash
sudo systemctl restart nvargus-daemon
```

所有相机、ROS launch 和录包都用一次 `Ctrl-C` 正常结束，并等待文件收尾和进程退出；不要直接关闭终端，不要用 `kill -9`。

### 9.3 原始三相机预览与录像

正式入口：

```bash
REC=/home/uavlab/search_camera_test/tools/three_camera_preview.sh
SAFE_REC=/home/uavlab/search_camera_test/tools/three_camera_preview_power_safe.sh
```

先检查设备、模式、插件和磁盘，再录制 8 秒验证样本：

```bash
bash "$REC" check
bash "$REC" verify
```

带预览录像；必须在 Jetson 桌面运行，SSH 时先指定桌面显示：

```bash
export DISPLAY=:0
bash "$REC" record-preview --search-exposure 5 \
  --save-dir /home/uavlab/search_camera_test/recordings
```

无预览/SSH 录像，适合正式采集：

```bash
bash "$REC" record --search-exposure 5 \
  --save-dir /home/uavlab/search_camera_test/recordings
```

定时 10 分钟、CSI 15 FPS 录像：

```bash
bash "$REC" record --duration 600 --csi-fps 15 --search-exposure 5 \
  --save-dir /home/uavlab/search_camera_test/recordings
```

有突然断电风险时使用 5 秒分段版；已闭合分段可保留，断电时通常只损失每路最后一个分段：

```bash
bash "$SAFE_REC" verify --duration 8
bash "$SAFE_REC" record --duration 1800 --segment-sec 5 \
  --search-exposure 5 --save-dir /home/uavlab/search_camera_test/recordings_power_safe
```

户外搜索画面过曝时把 `--search-exposure 5` 依次改为 `2`、`1`；太暗时改为 `10`、`20`。室内自动曝光可使用 `--search-auto`。输出目录会自动生成时间戳子目录、`session.info`、媒体检查结果和 `SHA256SUMS`。

### 9.4 YOLO 三相机检测、预览与标注录像

生产默认已经固定为 left=sensor 1、right=sensor 0、CSI 25 FPS、USB 25 FPS、confidence 0.50 和 0728 FP16 end-to-end engine。

只运行检测、桌面预览、不录像：

```bash
export DISPLAY=:0
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  display:=true record:=false stereo_roi_enabled:=false
```

只运行检测、无预览、不录像，适合给后续 ROS 节点提供检测话题：

```bash
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  display:=false record:=false stereo_roi_enabled:=false
```

正式三路 YOLO 标注录像，无预览；默认一直运行到 `Ctrl-C`：

```bash
bash src/falcon_intercept_deepstream/tools/record_three_yolo_annotated.sh
```

定时 10 分钟录像或修改置信度：

```bash
DURATION_SEC=600 CONFIDENCE=0.50 \
  bash src/falcon_intercept_deepstream/tools/record_three_yolo_annotated.sh
```

输出位于 `/home/uavlab/Jetson_ROS2_V0/flight_yolo_records/flight-yolo-fp16_0728_rect-<时间>`，包含三路分段 MKV、`manifest.txt`、检测统计、媒体探测和 ROS 日志。

同时预览并录制 YOLO 叠加画面：

```bash
export DISPLAY=:0
RUN_DIR=/home/uavlab/Jetson_ROS2_V0/flight_yolo_records/manual_$(date +%Y%m%d_%H%M%S)
mkdir -p "$RUN_DIR"
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  display:=true display_sync:=false stereo_roi_enabled:=false \
  record:=true record_output_dir:="$RUN_DIR" record_segment_seconds:=5
```

运行中可在另一终端检查：

```bash
ros2 topic hz /search/detections_json
ros2 topic hz /stereo/left/detections_json
ros2 topic hz /stereo/right/detections_json
```

### 9.5 双目测距与 RTK 精度计算

终端 1 启动 9.4 的无预览 YOLO。终端 2 启动测距链并录制现场证据：

```bash
bash src/falcon_intercept/tools/rtk_field_test/record_field_truth_test.sh \
  handoff_accuracy --with-chain --with-roi --listen 0.0.0.0:14560
```

目标机 RTK MAVLink 必须由地面电脑/MAVProxy转发至 `udp:192.168.1.44:14560`。该命令会创建：

`ros2_ws/bags/rtk_field_test/handoff_accuracy_<时间>/`

其中包括 `enemy_rtk.jsonl`、`measurements.jsonl`、`evidence_bag` 和日志。停止后取最新一次目录：

```bash
RUN_DIR=$(find bags/rtk_field_test -maxdepth 1 -type d \
  -name 'handoff_accuracy_*' -printf '%T@ %p\n' | sort -n | tail -1 | cut -d' ' -f2-)
echo "$RUN_DIR"
```

离线计算精度。`RIG_YAW_DEG` 是试验时机体 X 轴从正北顺时针量取的航向角，必须按现场填写；天线到左相机的机体系 FLU 杠杆臂为 `0.133,0,0.0462 m`：

```bash
RIG_YAW_DEG=0
python3 src/falcon_intercept/tools/rtk_field_test/compare_rtk_stereo_truth.py \
  --enemy "$RUN_DIR/enemy_rtk.jsonl" \
  --measurements "$RUN_DIR/measurements.jsonl" \
  --lever-body 0.133,0,0.0462 --rig-yaw-deg "$RIG_YAW_DEG" \
  --time-mode utc --source async \
  --output "$RUN_DIR/comparison.json"
```

若两架飞机都飞，Jetson 侧使用 `--airborne` 只录己方；敌机电脑单独保存 `enemy_rtk.jsonl`，落地后复制到同一目录再执行上述比较。UTC 模式要求双方 RTK fixed，默认 `fix_type >= 6`。

### 9.6 搜索/决策/打击逻辑与虚拟目标

不接相机、不控制飞机的四级 Shadow 冒烟测试可直接运行：

```bash
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh search
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh tracking
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh strike
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh full
```

这四条用于分层诊断 Falcon 旧控制链，不是当前生产上游等价性验收。当前上游完整软件闭环应运行：

```bash
ros2 launch falcon_intercept real_strike_completion_replay.launch.py
```

launch 正常退出后审计：

```bash
python3 src/falcon_intercept/tools/audit_completion_replay_bag.py \
  bags/completion_shadow_v4
```

预期最后输出 `passed=true`。

实物拦截机追坐标虚拟目标的上游外场入口如下。它会直接发布真实 `/fmu/in/*`，不是 Shadow；只有完成 S2、S3、获得当次飞手/场地授权并核对参数后才能运行：

```bash
ros2 launch px4_offboard_tracking coordinate_camera_manual_real.launch.py \
  hover_thrust:=0.42 maximum_thrust:=0.70 \
  target_scenario:=straight target_speed_mps:=2.0 \
  virtual_target_initial_north_m:=0.0 virtual_target_initial_east_m:=0.0 \
  command_maximum_speed_mps:=12.0 safety_maximum_speed_mps:=14.0 \
  maximum_acceleration_mps2:=1.0 maximum_tilt_rad:=0.35 \
  required_captures:=1
```

外场操作顺序：

1. 飞手确认围界、接管开关和口令；操作者启动上述 launch，节点不会自动解锁、起飞或切模式。
2. 飞手人工解锁、起飞并稳定悬停，至少高于 `minimum_flight_height_m=3.0`。
3. 飞手人工切入 Offboard；虚拟目标此时相对当前机位生成，上游开始搜索、跟踪和拦截。
4. 另一终端监控 `ros2 topic echo /tracking/pure_interceptor/status` 和 `ros2 topic echo /tracking/virtual_shot_event`。
5. COMPLETE 后等待 `safe_to_exit_offboard=true` 和稳定悬停；飞手人工退出 Offboard，再人工返航/降落。
6. 任何异常立即由飞手退出 Offboard；软件不得自动恢复接管。

`hover_thrust=0.42` 是 T6143 机体当前 2.76 kg 配置（2200 电池）的实测悬停推力（2026-08-12 项目负责人提供的同事既有实测值），`maximum_thrust=0.70` 为首架次上限；更换电池或载荷配置后必须重新标定。复飞后首个切入窗口顺带复核：切入头两秒若出现持续缓慢下沉，说明真值高于 0.42，按相同流程修正。

### 9.7 完整真实视觉闭环

#### 外场四终端一键入口（推荐）

2026-08-12 已把通用初始化和各终端参数封装为四个专用 launch，并提供同名 wrapper。ROS 在找到 launch 之前必须先加载环境，因此 wrapper 负责 `cd` 工作区、source ROS/overlay 和设置 `ROS_LOCALHOST_ONLY=1`；外场无需再手工执行通用初始化。按以下顺序在四个独立终端运行：

终端 1，相机/DeepStream：

```bash
bash /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/field_terminal_1_camera.sh
```

终端 2，自动时间戳证据 bag：

```bash
bash /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/field_terminal_2_evidence.sh
```

终端 3，真实视觉完整闭环 Live：

```bash
bash /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/field_terminal_3_live.sh
```

该 wrapper 显式传入 `field_authorized:=true`；底层专用 launch 默认是 false，直接运行 launch 而不显式授权会在创建任何节点前报 `REAL FLIGHT REFUSED`。专用 launch 内部再同时传入 `adapter_mode=live`、`allow_live_control=true`、`calibration_ready=true`，原三确认没有取消。

终端 4，只读精简监控：

```bash
bash /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/field_terminal_4_monitor.sh
```

监控只在 mission state、adapter gate reason 发生变化时打印，并单独打印虚拟开火及首次 `safe_to_exit_offboard=true`，不会被 50 Hz 重复状态刷屏。正常停止仍按“飞手先退出 Offboard并落地解除武装 → 终端 3 → 终端 2 → 终端 1 → 终端 4”顺序，各终端只按一次 Ctrl-C。

以下分解命令保留用于调试和交接理解。

当前可直接执行的是“真实三相机 + 真实 YOLO/测距 + 上游决策 + PX4 输出隔离”的 Shadow 链。

终端 1：启动真实三相机检测：

```bash
export DISPLAY=:0
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  display:=true record:=false stereo_roi_enabled:=true
```

终端 2：启动真实完整闭环 Shadow：

```bash
ros2 launch falcon_intercept real_strike_v0.launch.py adapter_mode:=shadow
```

终端 3：监控关键状态：

```bash
ros2 topic echo /tracking/pure_interceptor/status
```

需要留证时，终端 3 改为录包：

```bash
RUN_DIR=bags/real_target_shadow_$(date +%Y%m%d_%H%M%S)
ros2 bag record -o "$RUN_DIR" \
  /search/detections_json /stereo/left/detections_json /stereo/right/detections_json \
  /perception/search/observation /perception/depth/observation \
  /tracking/relative/estimate /tracking/pure_interceptor/status \
  /tracking/virtual_shot_event /tracking/mission/complete \
  /intercept/upstream_px4_shadow
```

目标进入搜索和双目视野后，应检查：

```bash
ros2 topic hz /perception/search/observation
ros2 topic hz /perception/depth/observation
ros2 topic echo /intercept/upstream_px4_shadow
ros2 topic info /fmu/in/vehicle_attitude_setpoint -v
ros2 topic info /fmu/in/trajectory_setpoint -v
```

该入口使用当前 solver/KF/bridge，并直接包含上游 `pure_interceptor_manual_real.launch.py`；`required_captures=1`、`auto_engage=false`、`takeoff_enabled=false`，完成后语义是制动、稳定悬停、等待飞手人工退出 Offboard。

2026-08-12 起，真实视觉 Live 入口已改为显式三确认：三个参数默认仍分别为 `shadow/false/false`，缺少任一 Live 确认都会拒绝；完整放行命令为：

```bash
ros2 launch falcon_intercept real_strike_v0.launch.py \
  adapter_mode:=live allow_live_control:=true calibration_ready:=true
```

该命令只创建 mode/trajectory/attitude 三类 PX4 输入 publisher；`VehicleCommand` 仍永久 remap 到 `/upstream/blocked/vehicle_command`，无订阅者。节点不自动解锁、起飞、切模式或 LAND。启动后先保持飞机未解锁，核对 9.7 节监控与唯一发布者；飞手稳定悬停且高于 3 m 后再人工切入 Offboard。COMPLETE 后必须等待 `safe_to_exit_offboard=true`，再由飞手人工退出 Offboard。

### 9.8 正常停止与故障恢复

停止顺序：先在闭环/录包终端各按一次 `Ctrl-C`，确认退出后，再在 DeepStream/录像终端按一次 `Ctrl-C`。随后检查：

```bash
pgrep -af 'deepstream_three_camera_node|gst-launch|real_strike|pure_interceptor'
systemctl is-active nvargus-daemon micro-xrce-agent falcon-clock-guard
```

正常情况下第一条无输出，三个服务仍为 `active`。若相机管线异常退出且下次无法启动，确认没有残留进程后再 `sudo systemctl restart nvargus-daemon`。严禁在飞行中重启 Agent、Argus 或整机。

## 10. 已关闭的软件证据

### 10.1 上游载荷二进制等价

证据：

- `ros2_ws/bags/replay_shadow_v1`（5× 压力）
- `ros2_ws/bags/replay_shadow_v2_1x`（1× 原速）
- 输入 bag：`ros2_ws/bags/replay_field01`

1× 原速结果：

| 消息 | 上游/relay | CDR mismatch |
| --- | ---: | ---: |
| OffboardControlMode | 12,839 / 12,839 | 0 |
| TrajectorySetpoint | 3,441 / 3,441 | 0 |
| VehicleAttitudeSetpoint | 9,398 / 9,398 | 0 |

field01 覆盖搜索、bearing、depth handover、depth acquire、intercept、IMM coast 和回搜；原始数据本身不满足 shot hard gate，因此 FIRE 验收由下一项单独完成。

### 10.2 FIRE→COMPLETE→稳定悬停

最终证据：

`ros2_ws/bags/completion_shadow_v4`

启动：

`ros2 launch falcon_intercept real_strike_completion_replay.launch.py`

审计：

```bash
python3 src/falcon_intercept/tools/audit_completion_replay_bag.py \
  bags/completion_shadow_v4
```

结果：

- 1064 条状态。
- 有序状态链：`TAKEOFF_SAFE → SEARCH → BEARING_TRACK → DEPTH_HANDOVER → DEPTH_ACQUIRE → INTERCEPT → EGRESS_REATTACK → COMPLETE`。
- 恰好一次 `shot_id=1`。
- `mission/complete=true`。
- 最大 capture count=1。
- 出现 `safe_to_exit_offboard=true`。
- 1046 条 Shadow 状态全部 `would_forward=false`、`vehicle_command_forwarded=false`。
- bag 中无真实 `/fmu/in/*`。
- 六节点单次 Ctrl-C 后 clean exit。

合成节点只产生 17 m 正前方 V1 搜索/深度观测，不直接发布 `RelativeEstimate`；估计、制导、预测射击和全部开火硬门仍由原上游节点执行。

## 11. 现场证据与数据归档

最重要的当前数据：

| 数据 | 路径 | 用途 |
| --- | --- | --- |
| 最终 RTK field01 | `ros2_ws/bags/rtk_field_test/field01_20260805_160105` | 当前测距/时序外场证据 |
| ROI 实测 | `ros2_ws/bags/rtk_field_test/roitest_20260805_122351` | 搜索→双目 ROI |
| 棋盘原始照片 | `ros2_ws/calibration_records/checkerboard_stills_20260804` | 当前双目标定 |
| 新右相机验收 | `ros2_ws/calibration_records/new_right_camera_20260804` | 右目更换后硬件证据 |
| 完整闭环 final bag | `ros2_ws/bags/completion_shadow_v4` | 软件任务闭环 |
| 原速等价 bag | `ros2_ws/bags/replay_shadow_v2_1x` | setpoint 等价 |

旧 RTK、棋盘、模型和飞行录像目前保留，是因为其中包含唯一原始数据。后续若需释放空间，应先复制到校验过的离板归档，再按“路径、大小、SHA-256、删除日期”登记，不能现场直接批量删除。

## 12. Live 放行前的唯一后续顺序

### S0：真实目标 observation

目标放入搜索相机和双目视野，Shadow 启动完整真实链。必须记录：

- 三路检测 FPS 与颜色错误计数。
- SearchObservation/DepthObservation 合法率、时间戳和 sequence。
- `PureInterceptorStatus` 搜索→bearing→depth 迁移。
- 上游隔离输出与 relay 一致。
- `/upstream/blocked/vehicle_command` 订阅者为 0。
- 真实 `/fmu/in/*` 无本链 publisher。

### S2：故障注入

依次验证并录包：

- bridge kill
- upstream controller kill
- adapter kill
- DDS/状态超时
- setpoint TTL
- PX4 failsafe
- RC/人工退出 Offboard
- 接管锁存后不得自动恢复

### S3：无桨台架

- 明确拆桨。
- 核对姿态四元数、推力符号、NED/FRD 轴和 50 Hz setpoint。
- 检查断流、状态过期和人工接管。
- 检查 `/fmu/in/*` 唯一发布者。
- 只在台账留下完整 bag/日志后讨论 live 门。

### S4/S5：分级实飞

1. SEARCH/TRACK，不接近。
2. 限速 approach。
3. 完整闭环。

每级必须有飞手、观察员、地理围界、最小高度/速度限制、RC 接管口令和独立证据。任何一级失败都回退 Shadow。

## 13. 当前已知风险

1. ~~搜索相机内参存在"双来源"~~ 已于 2026-08-14 解决：bridge 默认改为厂商标称 FOV 对应值（2217/2284），与 YAML 一致；正式相机标定后应替换为实测值。
2. **双目支架机械漂移（最高优先级硬件项）**：8-04 标定后支架扭转 ~0.36°，当前靠数据拟合修正运行（残差 8.6px、垂直门 25px）。重新紧固支架 + 棋盘格重标定后，修正归零、门改回 8px。
3. **飞行中 USB 搜索路滞后平台 ~280ms**（地面全栈仅 73ms）：疑似供电/热节流，当前以 0.45s 新鲜度门兼容。建议遮阳/风道/检查 BEC 余量；若反弹可将搜索降至 10 FPS。
4. **15-20m 远距离双目检出率不足**：当前链路验证要求目标 8-12m；更远距离需 ROI 高分辨率或模型增补样本，属独立改进项。
5. `anti-drone` 参考仓库有未提交实机修复；继续开发前应先形成受控提交或补丁，不要强制 reset。
6. TensorRT engine 与 JetPack/TensorRT/设备相关；换板或升级 JetPack 后应从 ONNX 重建，不要盲目复制 engine。
7. Argus 在多次异常退出后可能停帧；每次启动相机前固定 `sudo systemctl restart nvargus-daemon`（已写入 1.md）。
8. 历史录像和 RTK 数据有一部分目录名为 1970，这是旧 RTC/NTP 问题，不代表文件无效。
9. `ROS_LOCALHOST_ONLY=1` 会阻止跨机 ROS 发现；敌机真值链按 MAVLink/文件工具走，不要为方便随意关闭隔离。
10. 上游估计器内参（camera_fx=12847/fy=6125）与生产标定（9942/9933）不一致，仅影响协方差回退路径，待核。

## 14. 2026-08-11 整理结果

本轮交接整理遵循“生产源码、唯一证据、当前模型/标定保留；重复文档、缓存、失败回放和旧根副本删除”。

最终执行结果：删除旧平行方案、旧台账备份、Codex 临时目录、根目录旧源码副本、completion v1-v3、明确标记 invalid/superseded 的 bag、181 MB ROS 历史日志、Python 缓存和零字节 TensorRT 锁文件；项目根现在只有本主台账这一份 Markdown。大型唯一实验数据和最终 v4 证据未删除。删除前的材料恢复包位于板端项目外 `/tmp/jetson_cleanup_recovery_20260811.tar.gz`，SHA-256 为 `cd426fd54a4a66570025f31a17b7962a251f470f5982465e475b823384646445`；该 `/tmp` 文件是临时、易失的，交接确认后可人工删除。

整理后回归：`falcon_intercept` 原为 97/97；2026-08-12 增加 Live 双确认与四终端入口契约后为 100/100；`px4_offboard_tracking` 332/332；`CAMERA_MAPPING_CONTRACT_OK left=CAM1/sensor1 right=CAM0/sensor0 search=USB`；最终闭环 bag 审计 `passed=true`。三个系统服务 `nvargus-daemon`、`micro-xrce-agent`、`falcon-clock-guard` 均 active，无相机或闭环控制残留进程。

已合并进本台账的旧主题：

- 异步双目 V0 验收和相机同步总结。
- 真实完整闭环实施方案。
- Shadow replay 与 FIRE/COMPLETE 审计。
- 2026-07 至 2026-08 的阶段性工作流水。

板端整理后的原则：

- 根目录只保留本主台账，不保留平行方案或旧台账备份。
- 生产源码只在 `ros2_ws/src` 维护。
- 日常相机部署入口只用指向规范源码的符号链接。
- 最终闭环只保留 `completion_shadow_v4`，失败/过渡 v1-v3 删除。
- ROS log、pytest/pycache 和零字节锁文件清理。
- 唯一原始实验数据保留，不用“目录大”作为删除理由。

### 14.1 用户主目录二次清理

2026-08-11 经逐项盘点和用户明确确认，删除项目根外的旧 PX4 SITL staging、已安装软件的离线安装包、2026-08-07 旧版直控虚拟靶实验 Bag、旧相机色彩诊断录像、历史桥接/ROS 日志、相机脚本备份、可重建缓存、Code 缓存以及五个未使用的 VS Code Server 版本。NVMe 使用量由约 44G 降至约 30G，可用空间由约 180G 增至约 193G。

二次清理后明确保留：

- `/home/uavlab/px4_bridge_ws`：当前 Micro XRCE Agent 运行依赖。
- `/home/uavlab/search_camera_test/tools`：只保留指向项目源码的正式入口。
- `/home/uavlab/.local`：Torch、ONNX Runtime、Ultralytics 等当前 Python 环境。
- 当前使用的 VS Code Server、开发扩展及用户/SSH/桌面配置。

原先因 root 权限遗留的 `/home/uavlab/nvcam-settings-color-ab-20260717-101531`（约 1.6M）已由用户删除并在板端确认不存在，本轮清理清单已经闭合。

## 15. 精简历史里程碑

- 2026-07-07：Falcon V0 ROS 骨架与决策闭环建立。
- 2026-07-13 至 07-20：三相机、GPU/DeepStream、专用模型、ROI 和性能基线完成。
- 2026-07-21 至 07-27：PX4 安全 adapter、相机装配契约、实物目标与 Shadow 硬件验证。
- 2026-07-28 至 07-30：0728 FP16 模型定版；异步双目状态估计和 RTK 工具链落地。
- 2026-07-31：首次 15 m 外场测距，暴露机架/标定问题。
- 2026-08-01：物理左右映射修正为 left=sensor1、right=sensor0。
- 2026-08-02：断电时钟保护与 NTP 恢复闭环。
- 2026-08-04：更换故障右目；重新完成刚性背板双目标定。
- 2026-08-05：rectified solver、-1.65 px 修正、25 FPS 定档和 RTK 外场验收。
- 2026-08-06：anti-drone 基线推进到 `f4f1bf4` 并完成板端回归。
- 2026-08-10：姿态/轨迹安全门、真实闭环组装、CDR 等价与 FIRE→COMPLETE Shadow 全序列通过。
- 2026-08-11：以板端生产事实回同步本地，重写唯一主台账并清理重复资产。
- 2026-08-12：项目负责人确认既有无桨台架、断流/接管与虚拟目标分级实飞全部通过，决定豁免重复验收；真实视觉入口改为默认关闭、必须显式三确认的 Live 参数，Falcon 98/98 回归通过并授权外场使用。
- 2026-08-12：新增相机、证据录包、Live 闭环、状态监控四个外场专用 launch 及自动初始化 wrapper；未授权 Live 启动拒绝契约和入口完整性测试加入回归，Falcon 更新为 100/100。
- 2026-08-12：anti-drone 基线推进到 `e601ff2`（全速域参数解耦+3v1 纯虚拟仿真包）；按项目负责人指示实机入口对齐上游统一 12/14 包络与新限速器默认，修复软区 2.5≥限速 2.0 的启动拒绝冲突；手册与契约测试口径同步更新；板端 218 文件哈希一致、构建 6 包通过、回归 335 tests 0 失败、双实机入口 `/dryrun/fmu` 隔离冒烟通过（真实 `/fmu/in/*` 零发布者）。
- 2026-08-12：外场首次 Live 切入尝试（bag `real_closed_loop_live_20260812_094954`）分析：解锁后适配器 50 Hz 预流真实 `/fmu/in/*`（位置保持+NaN 速度）属设计行为，Position 段 PX4 不消费、无 EKF reset/failsafe/模式闪变，切入前手感与链路无关；切入窗口 0.84 s（161.18→162.02 s）指令温和（近水平姿态、yaw 保持、推力 0.49–0.535），飞机 5–6° roll 正常回正，但出现 −0.68 m/s 缓慢下沉，指认 `hover_thrust=0.535` 低于实际悬停需求；飞行员紧急切出正确，人工接管链实测有效。下一硬前置：用当次 ULog 的 HTE 中位数完成 G02 复核替换 0.535 再复飞；切入前杆量回中、地速<0.5 m/s。
- 2026-08-12：G02 悬停标定通道就绪——飞控侧确认 `MPC_USE_HTE=1`（反解锁自动回写 `MPC_THR_HOVER`）；`MPC_THR_HOVER=0.5000` 为未保存的出厂默认（无 `+` 标记），非实测值不予采用。板端部署免照看脚本 `falcon_intercept/tools/capture_hover_reference.sh`：起飞前一次启动、自后台脱离 SSH、定时 SIGINT 关包、自动选取最稳 20 s 空中窗口输出悬停质量统计（vz/倾角/电压电流）并 sync 落盘，落地后可直接断电。待外场悬停架次完成后取回写值全局替换 0.535，并按 2200 电池配置标注。
- 2026-08-12：悬停推力定档为 `0.42`（T6143 机体当前 2.76 kg 配置 + 2200 电池，同事既有实测值，项目负责人提供并确认刻度口径；较历史 3.1 kg 配置减重 344 g，质量相关表述已同步修正），全局替换 6 处代码默认、契约测试断言与手册/外场命令，原计划的外场悬停测量取消（脚本保留备用）；下次切入窗口以"头两秒是否持续缓慢下沉"作为内置复核，若出现则按同流程修正。
- 2026-08-12：anti-drone 基线推进到 `645a365`（实物 Offboard 心跳安全门禁 + 3v1 协同混合仿真完善）。门禁使切入前仅 `OffboardControlMode` 心跳上线、全部控制设定值 withhold 至确认 Offboard，从机制上消除当日外场"切入前疑似发路径点"的顾虑；板端构建 6 包通过、回归 335 tests 零失败、`/dryrun/fmu` POSCTL 替身验证心跳 FLOWING/设定值 SILENT、真实 `/fmu/in/*` 全程零发布。注意：期间发现 SSH 非交互 shell 缺少 `ROS_LOCALHOST_ONLY=1` 会导致话题发现失败（假性 topic-absent），板端验证脚本已统一加上该环境变量。
- 2026-08-13：真实闭环外场（续）：**切入 Offboard 死锁根因查明并现场热修**。根因：上游 `645a365` 心跳门禁在切入前只发 OffboardControlMode、扣住全部 setpoint（`pure_interceptor_controller.cpp:876-883`，`setpoint_streaming()` 要求 PX4 已进入 Offboard）；而我方适配器转发 OCM 的前提恰是收到有效上游 setpoint（`px4_upstream_adapter_node.py` 原 322 行 + 门禁 `command_present` 检查）→ 三方互等死锁，OCM 永远 NO MESSAGE。8-12 dryrun 用假 Offboard 状态替身，恰好绕过真实切入前状态，未暴露。热修：`_on_timer` 新增 preheat 路径——armed、未落地、未进 Offboard、上游 OCM 心跳新鲜且 failsafe/接管锁存/姿态/位置/状态新鲜度全部通过时，只转发 OCM 心跳、不要求 setpoint；切入后走原字节级转发逻辑。现场经 heredoc 补丁打入（备份 `.bak_20260813`），Python symlink 安装免编译，重启终端 3 生效。
- 2026-08-13：搜索相机室外过曝现场调参：大晴天下搜索 USB 相机全白，默认 `exposure_time_absolute=5 / brightness=-8` 不够。现场 v4l2-ctl 实时调至 `exposure_time_absolute=1`、`brightness=-100` 后画面可用（注意该控制项为整数，`0.5` 会被拒绝/钳位到 1；brightness 驱动范围为 -64~64，**-100 实际被静默钳位到 -64，真实生效值即 -64**，launch 默认值已按真实值写为 -64）。新默认值已写入 `deepstream_three_camera_v0.launch.py`（`search_exposure_time=1`、`search_brightness=-64`），launch 参数接口现成、免编译生效；C++ 源码默认值待回归时一并补齐。**该值为大晴天特调，阴天/傍晚需现场回调（v4l2-ctl 实时调整步骤已写入 1.md 终端 1 节）**。
- 2026-08-13：真实闭环外场前终检通过（三服务 active、无残留、PX4 三路话题正常、四终端脚本 src=install 一致、节点齐全、生产标定/视差修正 -1.65/25 档/ROI VIC 保护/引擎全部确认）。**录包清单修复**：`field_terminal_2_evidence.launch.py` 原录 `/left|/right/detections_json`，与 DeepStream 实际发布的 `/stereo/left|/right/detections_json` 不符（rosbag2 对不存在话题静默不录、无外显报错），并补录感知核心话题 `/perception/search/bearing`、`/perception/async_stereo_measurement`（双目测距原始输出）、`/perception/target/camera_position`（KF 输出，RTK 真值对比所用）；修复后 24 个话题，本地与板端 MD5 一致（`e8fe93cb...f70af3`），symlink-install 即刻生效无需重建。
- 2026-08-13：飞行中同步录像尝试**失败并已完整回退**。发现：① 本机 GStreamer 无 NVIDIA 硬编插件（`nvv4l2h264enc` 不存在，仅 openh264enc/x264 软编）；② 更根本的是 display 与 record 支路共存时录像支路完全不写文件且三路 FPS 25→~20（拓扑问题，与编码器无关，互斥 throw 是合理保护）。源码与二进制已回退至改动前 MD5（`23312522...df9726`）并重建验证，今日飞行按原配置（仅 rosbag，无视频）。后续正确方向：display/record 用 tee 做平行支路 + 安装 `gst-nvvideo4linux2` 硬编插件，台架充分验证后再上飞。
- 2026-08-13：**搜索链路零推进根因查明并修复（搜索相机时间戳积压）**。下午"必中"架次 SEARCH 仍不推进：bag 分析（`real_closed_loop_live_20260813_114428`，已用 `ros2 bag reindex` 恢复 metadata）显示感知其实正常——搜索检出 284 帧、最高置信度 0.943，bearing/observation 各 284 条；但上游控制器 `search_visible()` 恒假。根因：控制器用消息 `header.stamp` 做新鲜度门（`observation_timeout_s=0.25`，`pure_interceptor_controller.cpp:573/1028-1040`），而搜索 USB 相机路的 PTS 滞后随时间线性增长（10 秒桶中位 233→684 ms，约 +6.4 ms/s 无界；同 bag 双目 CSI 两路稳定 ~28 ms）——MJPEG 解码支路略慢于 25 fps 产出且源端无丢帧机制，帧在 v4l2 驱动环内无限积压。修复：`append_search` 在 `v4l2src` 与 `jpegparse` 之间插入 `queue leaky=2 max-size-buffers=2`（陈旧帧就地丢弃，永不积压）；地面 60 s 实测：滞后中位 110 ms、p95 113 ms、max 119 ms、全程零增长、25 fps 满帧无丢帧，远低于 250 ms 门。残留 ~110 ms 为 USB 解码链固有延迟，对 20°/s 搜索扫描约 2° 方位滞后，可接受。另注：该架次未切入 Offboard（nav_state 全程 PosCTL）且终端 3 未按架次重启（任务状态残留上一架次的 SEARCH、控制器已 stopped）——每架次必须按 SOP 完整重启四终端。
- 2026-08-13：当日外场经验汇总：① bag 损坏（无/空 metadata.yaml）可用 `ros2 bag reindex <dir>` 恢复后正常回放；② 无 sqlite3 CLI 时可用 python3 标准库 sqlite3 直读 db3 做话题计数与 JSON 载荷分析（std_msgs/String 的 CDR 载荷 = 4 字节头 + uint32 长度 + 字符串）；③ 订阅 detection 类话题须用 sensor_data（best-effort）QoS，默认 reliable 会静默收不到。
- 2026-08-13（晚间，回办公室）：**追击反向根因定案并修复——相机安装方向与上游硬编码规范系相差 90° 滚转**。证据链：项目负责人预览示意图（图像左=机头、图像上=机身左侧）+ 绕圈轨迹图像顺时针 -719° 与 90° 旋转预测一致 + 地面持机采样（物理左偏尾→图像右上）互相印证；非镜像（刚性旋转不产生镜像，v4l2 也无 hflip 控制项）。同事坐标系文档（相机参数.md，对应上游 `645a365`）确认：控制器/估计器的相机外参**硬编码无参数入口**，规范系为 optical+X=机体右、+Y=机头（搜索链含 10° 机头俯仰）、+Z=朝上，且架构上应由视觉侧适配节点完成实物→规范系转换。修复落在 `perception_bridge_node.py`：新增 `search_optical_to_canonical` / `depth_fru_to_canonical` 及协方差旋转，参数 `search_mount_roll_deg=-90`、`search_canonical_tilt_deg=10`、`depth_mount_roll_deg=-90`（实物若重新按规范安装则归零）；像素坐标同步重映射到规范图像。**附带发现并修复一个潜伏 bug：旧桥接把 solver 的 FRU 输出直接当 optical 发布（depth_z_m 取的是 up 分量而非镜头轴向距离）**。验证：单元测试 105/105（含"经上游硬编码矩阵往返还原机体系真值"用例）；bag `162119` 回放两条链规范系前后轴 42/0 全一致，侧向因双目漂移数据不可信留待标定后复验。**遗留**：① 估计器内参 camera_fx=12847/fy=6125 与生产标定（9942/9933）不符（仅影响协方差回退路径，待核）；② 搜索链 fx=4059/fy=1247 比例异常（影响角度精度，待与厂商 FOV 核对）；③ 双目几何已由数据拟合修正（见下条），正式重标定后归零复验。
- 2026-08-14：**深度链帧约定勘误与修复（前一晚桥接修复的深度部分有误）**。当日架次（bag `20260814_111328`）SEARCH→BEARING_TRACK 正常但永不进 DEPTH_HANDOVER：搜索链读数完美（h≈0°、v≈-10.2°，精确命中正上方签名，**方向修复对搜索链彻底验证**），但深度角误差中位 87°（交接门 8°，过门率 0%）。根因：`AsyncStereoMeasurement.position_camera` 的契约是 **REP-103 optical**（+X 图像右/+Y 图像下/+Z 镜头，`stereo_target_solver.py:812-816` 有明确注释），而前一晚我按 FRU 理解做了轴重排 → 深度方向被打乱。solver 输出数据本身一直健康（样本：正上方目标读出 z=14.7m 镜头向、x/y 仅 ~0.7m）。修复：深度链改用与搜索链同一滚转变换（`depth_optical_to_canonical = search_optical_to_canonical(tilt=0)`），协方差旋转矩阵同步改为纯滚转。**同时撤回昨晚"旧桥接 FRU 当 optical 是潜伏 bug"的错误结论——旧桥接按 optical 解释本就是对的，唯一缺的是安装滚转补偿。** 验证：单测 105/105；bag 回放深度角误差中位 87°→**6.4°**（p90 8.4°），交接门过门率 0%→**77%**，深度距离中位 14.8m 合理，与搜索链轴向读数互相印证。
- 2026-08-14（第五架次 bag `20260814_142432`）：捕获 0.2s、交接准入 <1s、深度时延中位 237ms（修复生效），但深度流在 18.3m 中位距离上断续（78-105s 连续 27 秒无检出，目标仅 ~25px 宽），DEPTH_ACQUIRE 的"0.2s 连续+4 帧"无法完成，停留 BEARING_TRACK 一分钟后目标移出视野。**结论：系统侧已正常，限制项是远距离检出率；链路验证应在 8-12m 目标距离进行，15-20m 远距离检出作为后续独立改进项**（候选：ROI 放大输入分辨率、更低置信度、模型增补远距离样本）。
- 2026-08-14（晚间修正）：**搜索相机内参修正**。桥接 `search_fx/fy` 原默认 4059.6/1247.1 源自 FOV 横竖写反（35×60）的上游标定缩放值，导致水平角偏小 1.83 倍、垂直角偏大 1.83 倍（居中零位不受影响，故此前飞行未暴露；离轴制导增益错误）。已改为厂商标称 FOV 60×35 对应值 2217.025/2283.984（与 stereo_calib.yaml search 段一致），cx/cy 同步取 1279.5/719.5。回归 105/105，板端同步。
- 2026-08-14（交接封版）：**全部改动落库、双侧一致、回归全绿**。本轮（8-13/14）改动面：`perception_bridge_node.py`（安装滚转补偿 + 搜索内参修正 + 深度链帧约定）、`stereo_target_solver.py`（距离速率门/上限）、`rectified_stereo.py` + `stereo_calib.yaml`（漂移拟合修正）、`real_strike_v0.launch.py`（垂直门 25、fixed_lag 0.20）、`field_terminal_1_camera.launch.py`（置信度 0.35）、`field_terminal_2_evidence.launch.py`（录包话题修正+感知话题补录）、`px4_upstream_adapter_node.py`（预热补丁）、`deepstream_three_camera_node.cpp`（搜索路漏队列+解码前抽帧 15fps）、`deepstream_three_camera_v0.launch.py`（搜索曝光默认 1/-64）、`pure_interceptor_real.launch.py`（observation_timeout_s 0.45）、`1.md`（外场 SOP 全量更新）。13 个关键文件本地/板端 MD5 全一致；板端重建通过；回归 Falcon 105/105、上游 gtest 226/226 零失败（`offboard_route_offset` 的 7-22 陈旧失败与本项目无关）；无残留进程、三服务 active。台账第 1 节状态表、第 13 节风险表、交接前特调参数一览表已同步刷新。
- 2026-08-17（离线像素级核验）：**搜索相机 10°安装倾角被 bridge 与上游重复处理，根因定案并在仓库修复**。实物关系重新确认：双目前/后相机光轴均为 Body −Z，搜索相机成像面机头侧低、机尾侧高，其朝上光轴向 Body +X 偏 10°；这正是上游 `R_body_from_search_optical` 已编码的物理外参。原 bridge 又以 `search_canonical_tilt_deg=10` 施加一次规范化旋转，两级矩阵端到端相消，错误地把搜索光轴当作与双目平行。对 8-14 bags 直接解包检测像素/CDR复算：历史旧内参可逐项复现两路 8.42°/9.03°/9.84°/9.10°夹角；以晚间已部署的当前内参重算为 11.02°/11.08°/11.11°/11.13°，其中当前前向差稳定约 10.7°、侧向约 3.1°。修复为 bridge 仅保留 −90°安装滚转、附加倾角默认及生产/replay launch 显式值均改为 0°，上游10°保持不变；14:24预测前向残差由10.70°降至约0.70°，总残差约3.2°。板子已不在本地，本次通过 Gitee `Jetson部署` 分支交付，待板端拉取、重建并做静态五方向复核。
- 2026-08-14（第四架次 bag `20260814_131159`）：**首次进入 INTERCEPT**，但 INTERCEPT↔IMM_COAST 以 5-10Hz 高频振荡，62s 进 FOV_RECOVERY 后退回 SEARCH。数据归因：测量流本身健康（拦截窗口内间隔中位 62ms），问题是**深度观测端到端时延中位 367ms（p90 471ms、max 608ms）**——构成 = 相机管道 ~30ms + solver 异步 EKF 固定延迟平滑 `async_fixed_lag_s=0.30` + 处理；观测年龄在 0.45s 门上下抖动 → `depth_visible` 闪烁 → 振荡。修复：`async_fixed_lag_s` 0.30→**0.20**（real_strike_v0.launch.py 参数，免编译），预期时延中位降至 ~270ms；保留约 4 帧平滑深度，EKF 质量损失可忽略。链路参数地面冒烟验证：async_fixed_lag_s=0.2、max_vertical_error_px=25、observation_timeout_s=0.45、max_range_rate_mps=40、max_range_m=80 全部按预期加载。**当日时延预算台账**：搜索路 ~76ms（15fps 抽帧后地面值，飞行平台 ~280ms）、深度路 = fixed_lag + ~70ms；新鲜度门 0.45s 对两路均有余量。
- 2026-08-14（第三架次 bag `20260814_122511`）：**搜索未捕获归因——飞行中搜索相机滞后反弹**。该架次全程停在 SEARCH：bearing/observation 各 245 条（YOLO 有检出，中位置信度 0.69），但搜索帧 PTS 滞后在地面段 106ms、起飞后爬升并稳定在 ~250-280ms 平台，47.7% 的帧越过 250ms 新鲜度门 → `search_visible` 抖动、0.2s 捕获保持无法满足。地面用**完整四终端全栈**复测只有 73ms，且 bag 里每帧检测数 0-1 个（排除检测负载），随飞行进程爬升后平台化——指向飞行状态下的供电/热节流（电机负载电压跌落或日晒升温），地面无法复现。处置（板端已生效）：上游控制器 `observation_timeout_s` 0.25→**0.45**（`pure_interceptor_real.launch.py` 参数覆盖，免编译；280ms 平台下仍有 170ms 新鲜窗口，15fps 帧间隔 66ms 保证连续覆盖）。**硬件侧建议**：给 Jetson 遮阳/留风道、检查机载 BEC 供电余量；若后续仍反弹，下一杠杆是搜索降到 10fps。
- 2026-08-14（第二架次 bag `20260814_115621`）：**DEPTH_HANDOVER 首次到达**（5 次），但每次 0.2-1.1s 即因深度流中断退回 BEARING_TRACK，且 101.4s 出现 SAFETY_ABORT。数据归因：① 误配对产生垃圾短距（14.5m→0.77m 跳变，3 条 <3m）触发上游 `safety_minimum_depth_m=3.0` 安全中止；② 深度观测流太稀（~5Hz 且有 >0.25s 缺口），无法满足 DEPTH_ACQUIRE 的"0.2s 连续可见 + 最少 4 帧"要求，瓶颈在双目检出率（仅 ~16% 帧有检出）。修复（板端已生效，回归 105/105）：a) solver 新增**距离速率门**（`max_range_rate_mps=40`、余量 0.5m、静默 1s 后视为重新捕获不拦截）——2 倍速回放验证 <3m 垃圾清零；b) 新增**绝对上限** `max_range_m=80`（0.1m 基线下 >80m 必为伪影）；c) 终端 1 检测置信度 0.50→**0.35** 提升双目检出连续性（搜索链由 bearing 节点自有 0.5 门兜底，垃圾配对由速率门拦截）。回放失真提示：bag 倍速回放会压缩 wall-clock 门限，验证时须按真实速率折算。
- 2026-08-13（晚间）：**外场 SOP（1.md）按当日改动更新**：① 搜索相机预期帧率 25→15 FPS；② 切入前预流判据改写——未解锁时 OCM 无消息为正常、切入前 setpoint 话题静默是心跳门禁设计行为（当日上午现场误判的根源）；③ nvargus-daemon 重启固化为每次相机启动前的固定步骤；④ 新增"地面距离快检"（已知距离 vs `range_m`，5-10% 内正常）；⑤ 终端 1 画面确认项改用前/后双目命名。
- 2026-08-13（晚间）：**双目机械漂移免标定数据修正**。。用当日全部 bag 的 3824 对左右目同名点，在 rectified 域拟合右目微旋转漂移修正 `drift_rotation_right_rad=[-0.006294, 0, -0.002337]` rad（ωx=-0.361°、ωz=-0.134°；**ωy 与视差简并不可观测，强制为 0**——3 参数自由拟合会给出 ωy=-2.2° 导致距离系统性偏差 83% 的垃圾解，已识别并排除）。效果：垂直误差中位 65.8→8.6px（留出验证 9.0px，非过拟合）；对视差影响中位仅 +0.77px，距离变化中位 1.4%（12.4→12.5m）。25px 门过门率从 2% 恢复到 94%。落地：`rectified_stereo.py` 的 `rectify_right` 增加旋转修正（yaml 加载、默认零兼容旧格式）、生产 `stereo_calib.yaml` 写入修正值、`real_strike_v0.launch.py` 垂直门从临时 120 收紧为 25.0（残差 p90=21px 留有余量；正式棋盘格重标定后修正归零、门改回 8.0）。回归 105/105 通过。**注意：该修正只保证几何自洽与方向正确，绝对距离精度（1.4% 量级修正前偏差）需用卷尺地面实测复核后才能在精度场合引用。**
- 2026-08-13：**地面核验矩阵（修复后每次改动相机/标定后必跑）**：飞机平放、标记机头，目标无人机依次置于 正上方 / 机头侧上方 / 机尾侧上方 / 机体左侧上方 / 机体右侧上方，读取 `/perception/search/observation` 与 `/perception/depth/observation`，对照同事文档 §3/§4 的规范表：正上方→搜索 vertical_angle≈-10°（LOS≈(0,-0.174,0.985)）、机体右上→搜索图像右侧（horizontal>0）、机头上方→图像下侧（vertical>0）；深度链 position_optical 各分量同号对应。全部符合才算方向链闭合。
- 2026-08-13：搜索相机滞后**二次修复（解码前抽帧 15 fps）**。首轮漏队列修复后飞行架次（bag `151012`）实测：进 Offboard ✓、进 SEARCH ✓、搜索检出率升至 68%（1972 帧），但滞后中位仍 290 ms 且缓慢增长（漏队列只压住驱动环，压不住解码器 V4L2 内部环）→ `SEARCH→BEARING_TRACK` 仍被 250 ms 新鲜度门挡住。二次修复：`jpegparse` 后、`nvv4l2decoder` 前加 `videorate drop-only=true ! image/jpeg,framerate=15/1`（MJPEG 帧独立可安全丢弃；1440p MJPEG 只支持 25 fps，无法从 caps 降档；注意 caps 名是 `image/jpeg` 不是 `video/x-jpeg`，后者协商失败报 Internal data stream error）。地面 60 s 实测：搜索 15.0 fps、滞后中位 76 ms / max 88 ms、全程零增长，CSI 双路 25 fps 不受影响。新鲜度余量：每 66 ms 一帧新鲜观测 vs 250 ms 门。
- 2026-08-13：**SEARCH→BEARING_TRACK 首次打通**（bag `153554`，30.2s 进 SEARCH、30.4s 进 BEARING_TRACK，0.2s 保持门一次通过）。BEARING_TRACK 段航向保持是上游设计行为（滚转/俯仰执行相机交接，yaw 锁定避免混控瞬态，`pure_interceptor_controller.cpp:1160-1163`），不是故障。
- 2026-08-13：**双目几何机械漂移确诊**（当日最深一层根因）。bag `153554` 离线配对分析：rectified 视差中位 +67.8px 健康、三角化距离中位 14.6m 合理，但**垂直误差中位 74.9px（p90 103、max 212）vs 门 8px，112/114 对被拒**→ solver 全天仅 4 条有效输出 → 永远无法 DEPTH_HANDOVER。垂直差随 u 带斜率（左半 67.5px→右半 46.1px）是相机相对扭转特征；早上 bag（`112007`）垂直误差已达 106px，证明漂移发生在 8-05 验收之后、本日之前，与当日全部代码改动无关（早上 bag 跑的是与 8-05 相同二进制）。**现场处置：经项目负责人批准，`real_strike_v0.launch.py` 给 solver 加 `max_vertical_error_px=120.0`（launch 参数免编译），仅供完整闭环链路验证使用，距离精度在该配置下不作验收；恢复标定后必须改回 8.0。** 后续正道：回室内用棋盘格重新标定（8-04 流程），并检查支架机械松动。另观察到搜索相机与双目之间存在 ~10° 轴向不一致（搜索 -14° vs 双目 -3.6°），标定时一并核查。

---

## 当前外场特调参数一览（2026-08-14 交接态）

以下为本轮真实目标外场期间偏离上游/原始默认的全部参数与补丁，交接后如需恢复基线请按此表逐项核对：

| 项 | 当前值 | 原始默认 | 位置 | 引入原因 | 恢复正常值的条件 |
| --- | --- | --- | --- | --- | --- |
| 搜索相机抽帧 | 15 FPS | 25 FPS | `deepstream_three_camera_node.cpp` 的 `append_search`（videorate，C++ 需重建） | USB 解码积压导致 PTS 滞后无界增长 | 确认解码链在飞行负载下能跟上 25 FPS 后 |
| 搜索曝光/亮度 | 1 / -64 | 5 / -8 | `deepstream_three_camera_v0.launch.py` | 大晴天抗过曝（电子 ND） | 随光照现场调（1.md 终端 1 节有步骤） |
| 检测置信度 | 0.35 | 0.50 | `field_terminal_1_camera.launch.py` | 提升双目检出连续性 | 远距离检出改进后回调 |
| 垂直误差门 | 25 px | 8 px | `real_strike_v0.launch.py` | 漂移修正残差 p90≈21 px | 棋盘格重标定后改回 8 |
| 双目漂移修正 | [-0.006294, 0, -0.002337] rad | 无（全零） | `stereo_calib.yaml` 的 `drift_rotation_right_rad` | 支架机械扭转的数据拟合修正 | 重标定后归零 |
| EKF 固定延迟 | 0.20 s | 0.30 s | `real_strike_v0.launch.py` 的 `async_fixed_lag_s` | 深度观测时延 367ms 顶到新鲜度门 | 长期可保留；若管道提速可回到 0.30 |
| 观测新鲜度门 | 0.45 s | 0.25 s | `pure_interceptor_real.launch.py` 的 `observation_timeout_s` | 飞行中搜索滞后 ~280ms 平台（疑似供电/热节流） | 供电/散热整改后可回收 |
| 距离速率门/上限 | 40 m/s、80 m | 无 | `stereo_target_solver.py` 参数默认 | 误配对垃圾距离触发安全中止 | 长期保留（安全改进，无需恢复） |
| 搜索内参 | fx=2217.0、fy=2284.0 | 4059.6/1247.1（FOV 横竖写反） | `perception_bridge_node.py` 默认 | 离轴角度比例错误 1.8 倍 | 正式相机标定后换成实测值 |
| 安装滚转补偿 | -90°（搜索+深度） | 0° | `perception_bridge_node.py` 默认 | 实物安装与上游规范系差 90° | 相机物理转正到规范安装后归零 |
| 搜索附加倾角补偿 | 0° | 10°（错误重复补偿） | `perception_bridge_node.py` 默认、`real_strike_v0.launch.py` | 实物10°倾角已由上游 `R_body_from_search_optical` 编码，只能保留一次 | 仅在重新测量出相对上游规范的残余 pitch 后调整 |
| 适配器预热补丁 | 已合入 | 无 | `px4_upstream_adapter_node.py` | 上游心跳门禁与适配器互等死锁 | 长期保留（上游不动则必须保留） |

---

交接人开始任何工作前，只需要完成四件事：

1. 阅读本台账第 1、2、3、12、13 节与第 15 节 2026-08-13/14 全部条目，以及上面的特调参数表。
2. SSH 后核对服务、残留进程和 `ROS_LOCALHOST_ONLY=1`。
3. 运行 105 项 Falcon 测试（`python3 -m pytest src/falcon_intercept/test/ -q`）与 226 项上游 gtest（`colcon test --packages-select px4_offboard_tracking`，当前全绿）。
4. Shadow 仍是默认；仅按 9.7 节在当次飞手/场地确认后显式给出三个 Live 参数。外场操作以 `1.md`（已同步板端 `/home/uavlab/Jetson_ROS2_V0/1.md`）为准。
