# netgun_intercept — 单双目 8m 网枪打击拦截包（骨架版）

方案与决策记录：`docs/双目8m网枪拦截方案.md`（v4）。本包是"相机输入 →
完成打击"完整链路中的**开火决策环节**；感知/估计/制导复用既有节点，
接口与 **PX4 v1.15.4**（px4_msgs release/1.15）对应。

## 完整链路骨架（新包在其中的位置）

```
[虚拟相机/真实相机]
  仿真：logical_camera_simulator（真值 + 姿态 → 安装坐标变换 → 4mm 内参
        投影 + 噪声 + 10m 深度截断；扩展 maximum_depth_m 参数）
  实机：falcon_intercept 双目链（stereo_target_solver / search_bearing）
    │  anti_drone V1 观测
    ▼
  relative_estimator（IMM-EKF；CV+CA 现役，CT 模型为方案 WP-D 待加）
    │  /tracking/relative_estimate（目标-本机相对位置/速度/协方差）
    ├──────────────────────────────┐
    ▼                              ▼
  pure_interceptor_controller    netgun_strike_node（本包）
  （制导/瞄准/安全门，参数改 12m 档）  轴误差/距离/LOS率 → 择时火控
    │                              │  VirtualShotEvent（开火信号）
    ▼                              ▼
  upstream_gate → px4_upstream   virtual_interception_referee
  （三类 setpoint 白名单，shadow 默认）  （3×3 覆盖面判定，WP-C 待改）
```

## 组件

| 组件 | 状态 | 说明 |
|---|---|---|
| `netgun_model` | 实现 | V5-C 简单模型（tof/lead/covered），接口可替换全模型 |
| `netgun_fire_control` | 实现 | 1° 即打 / 2°+0.2s 兜底 / 蒙射接口默认关 / 单发闩锁 / 无去抖 |
| `netgun_strike_node` | 骨架 | 匀速外推预测；授权门简化为 armed+offboard |
| 裁判 3×3 / IMM CT / 12m 参数档 | 待做 | 方案 WP-C / WP-D / WP-A 剩余项 |

## 骨架边界（TODO 清单）

- 授权门完善：接入高度门 / 任务时间授权 / attack_enable_delay（上游主控状态）；
- IMM 概率加权预测（现匀速外推）；
- 与上游 `enable_native_fire` 的开火权交接参数（方案 §1，待上游加）；
- 仿真传感器 `maximum_depth_m` 与 4mm 内参（`logical_camera_simulator`）；
- 真值隔离：本节点不订阅真值话题，验收由 truth-isolation 测试保障。

## 构建与测试

```bash
colcon build --packages-select netgun_intercept
colcon test --packages-select netgun_intercept
```
