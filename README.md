# anti-drone-interceptor

Jetson 板载视觉拦截平台：单副 4mm 双目（竖直向上安装）→ IMM CV/CA/CT
目标估计 → 前馈制导（保持下方站位 + 主动瞄准 AIM）→ 角度门火控
（2° 即打 / 4° 兜底，单发闩锁）→ 网枪拦截 → 3×3 覆盖裁判。
仿真验证链运行于 ROS 2 Humble + PX4 SITL (v1.15) + Gazebo Harmonic。

## 仓库结构

| 目录 | 内容 |
|---|---|
| `src/netgun_intercept/` | **网枪拦截核心包**：制导节点（三状态机+AIM+SPIN）、火控节点、网枪模型、launch、Python 数值镜像 |
| `src/px4_offboard_tracking/` | 相对估计器（IMM 三模+bearing-only）、仿真传感器/真值桥、目标源 |
| `src/virtual_interception_referee/` | 3×3 即时覆盖裁判 |
| `src/anti_drone_interfaces/` 等 | 接口与上游包（见 `VENDORED_DEPENDENCIES.md`） |
| `docs/` | 架构与方案文档（入口见下） |
| `docs/交接/` | 交接文档与工作日志（最新状态在此） |
| `deploy/` | 板端部署脚本（时钟守护、XRCE 服务） |
| `tools/` | 仓库审计工具（不依赖 ROS） |
| `weights/` | 模型资产（大二进制不入库，见其 README） |
| `archive/` `board_misc/` | 归档区与板端历史参考（不参与构建） |

## 分支约定

- `netgun_intercept`：网枪拦截链开发分支（以核心 ROS 包命名）。

## 快速开始（仿真，WSL Ubuntu 22.04）

```bash
# 依赖：ROS 2 Humble、PX4 v1.15 (~/PX4-Autopilot)、上游覆盖层 ~/UAV_ws
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
cd <工作区>
colcon build --packages-select netgun_intercept px4_offboard_tracking \
  virtual_interception_referee --cmake-args -DBUILD_TESTS=OFF
source install/setup.bash

# 双机仿真（先起 PX4 双实例 + MicroXRCEAgent，再 launch）
ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=3.0
```

完整环境配方、坑表与矩阵运行方法见 `docs/交接/网枪拦截项目交接文档.md`。

## 文档索引

| 文档 | 内容 |
|---|---|
| `docs/网枪完整架构v2.md` | 面向实机的完整架构（架构唯一真源） |
| `docs/双目8m网枪拦截方案.md` | 方案定案记录（v1→v4） |
| `docs/项目开发手册.md` | 红线/参数真源/环境配方 |
| `docs/网枪联仿结果.md` | 联仿矩阵与决定性实验 |
| `docs/交接/网枪拦截项目交接文档.md` | 状态总览 + 下一步清单 |
| `docs/交接/主动瞄准开发交接.md` | AIM+SPIN 设计与实现记录 |
| `docs/交接/网枪拦截项目工作日志.md` | 按日工作日志 |
| `docs/仓库维护指南.md` | 目录职责与审计工具用法 |

## 当前状态（2026-10-15）

- 仿真侧：20 格全几何矩阵 19/20 格有解（唯一无解格 headon@5 待预测
  拦截项）；40 轮矩阵触发 77.5-82.5%、命中 100%、miss 均值 0.64m；
  @1 m/s 弱档由 AIM 主动瞄准覆盖（8/8）。
- 实机适配：rc_gated 门控已就绪，感知源/GPIO/内参/实机 launch/
  systemd 为剩余前置项（见交接文档 §八-9）。

## 安全红线（摘录，全文见项目开发手册）

不自动解锁（仿真除外）/三类 setpoint 白名单/VehicleCommand 永久拦截
（实机 rc_gated 模式）/实机切入 offboard 前零 setpoint、切出即停。
