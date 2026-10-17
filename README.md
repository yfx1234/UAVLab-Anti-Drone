# UAVLab 反无人机系统（完整板上代码）

Jetson Orin NX 板载无人机感知 + 网枪拦截完整系统：ZED X Mini 双目 →
YOLO/TensorRT 检测（多模型可选、置信度/目标尺寸可调）→ 相机坐标观测 →
感知桥 → IMM 目标估计 → 前馈制导（站位保持 + 主动瞄准 AIM）→ 角度门火控
（2° 即打 / 4° 兜底，单发闩锁）→ PX4 offboard → 网枪拦截。
PC 工作台提供实时监看、素材管理、检测预览与拦截链控制。

## 系统数据链

```
ZED X Mini (GMSL)
  └─ zed_perception_standalone   board/zed_tracking   多目标观测（合同 v2）
       ├─ PC 8767 工作台         panel/               监看/录像/素材/检测预览/拦截链控制
       └─ /perception/observations_json
            └─ zed_perception_bridge   src/zed_perception_bridge   v2 → DepthObservation V1
                 └─ relative_estimator       src/px4_offboard_tracking   IMM CV/CA/CT + 姿态
                      ├─ netgun_guidance_node  src/netgun_intercept  制导（rc_gated 安全门控）
                      └─ netgun_strike_node    src/netgun_intercept  火控五门+单发闩锁
                           └─ PX4 v1.15 (offboard, MicroXRCE 串口)
```

## 目录结构

| 目录 | 内容 |
|---|---|
| `board/zed_tracking/` | 感知核心：zed_perception_standalone（YOLO/TensorRT 推理、深度质检、观测发布）、supervisor RPC、检测预览标注工具 |
| `board/zed_perception_interfaces/` | 感知观测 ROS 消息（合同 v2） |
| `panel/` | PC 感知与采集工作台（8767）：实时监看、板上录像、场次管理、素材下载、检测预览、拦截链启停与状态 |
| `src/netgun_intercept/` | 拦截核心：制导（搜索自旋/跟踪/AIM/SETTLE/SPIN）、火控、网枪模型、实机 launch |
| `src/px4_offboard_tracking/` | IMM 相对估计器（bearing-only 退化支持） |
| `src/zed_perception_bridge/` | 感知桥：PerceptionFrame v2 → DepthObservation V1 |
| `src/anti_drone_interfaces/` 等 | 接口与上游包（见 `VENDORED_DEPENDENCIES.md`） |
| `models/` | 感知模型资产登记（pt/onnx 哈希；引擎不入库，板上构建） |
| `deploy/` | 拦截链板端脚本（启停/状态、时钟守护、XRCE 服务）、感知状态读取 |
| `scripts/` | 感知板端部署与构建脚本 |
| `tools/` | 拦截链仓库审计与无硬件全链测试（tools/board_test） |
| `docs/` | 双方文档：感知接口/工作台 + 网枪架构/交接/实机部署记录 |
| `tests/` `board/zed_tracking/test/` | Python 与 C++ 测试 |
| `environment/` `archive/` `board_misc/` | 硬件参考、归档区 |

## 分支与仓库约定

- **GitHub `main`** 与 **Gitee `master`**：完整板上代码（感知 + 拦截链），发布时指向同一个提交，源码与历史一致。
- Gitee `perception`：仅感知副本；`netgun_intercept`：拦截链历史分支。
- 后续改动须同步两个默认分支，并核对远端提交 SHA；`origin` 在不同本地副本中指向不同平台，使用前必须核对 URL。操作见 [仓库同步约定](docs/仓库同步约定.md)。
- 模型权重、TensorRT 引擎、录像素材不入库；来源与哈希登记在 `models/` 与 `weights/README.md`。

## 环境与部署（系统依赖不随仓库分发，见下）

**板上（Jetson Orin NX 16GB，P3767-0000）**

| 依赖 | 版本 |
|---|---|
| JetPack / L4T | 6.2 / 36.4.3 |
| ZED SDK | 5.5（ZED X Mini，GMSL 驱动已适配） |
| TensorRT / CUDA | 10.3 / 12.6 |
| ROS 2 | Humble |
| PX4 + MicroXRCE-DDS | v1.15，串口 `/dev/ttyTHS1` @921600 |

感知构建与测试（相机须停止）：

```bash
bash scripts/build_board.sh /home/uavlab/Jetson_ROS2_V0
```

拦截链构建（板上 `~/interceptor_ws`，px4_msgs 等底座来自 `Jetson_ROS2_V0/ros2_ws/install`）：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select netgun_intercept px4_offboard_tracking \
  zed_perception_bridge anti_drone_interfaces --cmake-args -DBUILD_TESTS=OFF
```

启动顺序：接电（相机与板子同时上电）→ PC 打开 `panel/`（8767）→ 连接板子
（自动校时）→ 启动相机 → 开启 YOLO → 启动拦截链（面板按钮）→ 飞手解锁起飞 →
切 offboard 交由拦截链接管（任意时刻切出立即回到手动）。

模型管理：pt/onnx 放 `weights/candidates/<目录>/`，`trtexec --fp16
--builderOptimizationLevel=3` 构建引擎，`tools/build_model_manifest.py` 生成
边车后即可在面板 YOLO 输入中选择（现役自训模型 + GLAD 公开 MAV 模型）。

PC 工作台依赖：Python 3.10+、paramiko、ffmpeg（打包版即可），入口
`panel/dashboard.py`（127.0.0.1:8767）。

## 文档索引

| 文档 | 内容 |
|---|---|
| `docs/观测接口与设备状态.md` | 感知观测合同 v2（消费端必读） |
| `docs/GLAD模型接入与检测预览_2026-10-16.md` | 公开模型接入、检测预览与阈值口径 |
| `docs/实机部署记录_2026-10-15.md`（拦截侧 `docs/`） | 感知↔拦截接口、板上部署与全链验证 |
| `docs/网枪完整架构v2.md` | 拦截链架构（唯一真源） |
| `docs/交接/网枪拦截项目交接文档.md` | 拦截链状态总览 + 下一步清单 |
| `docs/项目开发手册.md` | 红线/参数真源/环境配方 |
| `docs/仓库维护指南.md` | 目录职责与审计工具 |
| `docs/仓库同步约定.md` | GitHub/Gitee 默认分支同步、远端名称与验收方式 |

## 安全红线（摘录，全文见拦截侧项目开发手册）

不自动解锁（仿真除外）/ 三类 setpoint 白名单 / VehicleCommand 永久拦截
（实机 rc_gated）/ 实机切入 offboard 前零 setpoint、切出即停 / 飞控数据
断链自动关火 / 开火单发闩锁。

## 仿真（拦截链，WSL Ubuntu 22.04）

```bash
source /opt/ros/humble/setup.bash && source ~/UAV_ws/install/setup.bash
colcon build --packages-select netgun_intercept px4_offboard_tracking \
  virtual_interception_referee --cmake-args -DBUILD_TESTS=OFF
ros2 launch netgun_intercept netgun_sim_12m.launch.py target_scenario:=straight target_speed_mps:=3.0
```

完整环境配方见 `docs/交接/网枪拦截项目交接文档.md`。
