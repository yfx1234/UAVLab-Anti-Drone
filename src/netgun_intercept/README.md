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

## 开火模式与实物击发（I-3，2026-10-21）

`netgun_strike_node` 参数 `fire_mode`：

| 取值 | 行为 | 使用场景 |
|---|---|---|
| `auto`（节点默认） | 火控门满足即自动单发 | 仿真矩阵/联仿 |
| `auto_manual`（`netgun_real.launch.py` 默认） | 自动单发 + `/netgun/fire_command` 手动兜底，两路共享单发闭锁 | 实机 |
| `manual` | 只认按钮（I-1 口径） | 特殊测试 |

- 手动兜底门（`manual_fire_block()` 纯函数，单测覆盖）按顺序报首因：
  `DISABLED → LATCHED → FCU_STALE → NOT_ARMED → LOW_HEIGHT → TILT →
  (NOT_AUTHORIZED)`。默认 `manual_fire_require_authorization=false`：
  只看物理安全门（飞控姿态/状态/位置新鲜、已解锁、高度有效（z_valid）且 ≥ `minimum_height_m`、
  枪管轴偏离竖直 ≤ `fire_max_tilt_rad`=45°），不要求 offboard/稳定跟踪/
  有效估计——自动一直不发或遥控接管后操作手仍可发射；设 true 恢复 I-1 的
  "授权+有效估计"口径。
- 倾角门同时约束自动开火（自动另需 `t=1`）；对外发布的武器授权
  `/tracking/weapon_authorization` 不含倾角（避免 AIM 进出振荡）。
- 手动请求逐拍消费不积压：门不通过立即拒绝并记日志
  `manual fire rejected: <首因>`；已发射后再按记 `already fired`。
- 无估计时手动兜底仍可发射：事件 `solution_valid=false`、方向记枪管轴；
  手动发射一律记枪管实际指向（自动沿用预测射击方向）。
- 实物击发钩子 `trigger_command`（launch 参数，默认空=仅虚拟事件）：开火瞬间
  在独立线程 `posix_spawn("/bin/sh -c <cmd>")` 异步执行，不阻塞 50Hz 火控环；
  环境变量 `NETGUN_SHOT_ID`、`NETGUN_SHOT_REASON`；退出码 0 记
  `trigger command done rc=0`，否则 `trigger command FAILED`。单发闭锁保证
  每任务至多执行一次。干跑替身见 `tools/board_test/netgun_trigger_dryrun.sh`。
  实机实现（2026-10-23）：`deploy/netgun_pwm_link.py fire` → 常驻服务
  `netgun-pwm-link.service` → MAVProxy → 飞控 FMU PWM OUT 5，`run_interceptor_stack.sh start`
  默认装订。2026-10-24 起按网枪触发模块：平时锁存待命低位（默认 1000 µs），开火时确认低位后
  `DO_SET_ACTUATOR` 击发高位（默认 2000 µs）保持 1.0 s 再回低位；未解锁拒绝且不发高位。
  脉宽/时长存 `~/netgun_config/netgun_pwm.json`（面板「击发参数」或 `netgun_pwm_link.py params`）；
  接线、参数、台架与解锁实测见 `指令.md` 0.8。
- 制导收到开火事件：TRACK/AIM → RETREAT→（spin 开）SETTLE→SPIN→DONE；TAKEOFF 中 →
  RETREAT→DONE 保持；WAITING（未切 offboard）→ 闩锁，之后进 offboard 直接 DONE。
  RETREAT（2026-10-24，安全后撤）：网弹竖直上抛后落回机体附近，击发后先沿机体后向
  水平退 `post_shot_retreat_m`（默认 3 m，launch 可调，0=旧行为），到位 0.5 m 内或
  `retreat_timeout_s`（默认 4.5 s）兜底后接后续流程；后撤目标锚定 hold_target，
  SETTLE/SPIN/DONE 自动保持在新点，yaw 冻结不转向。
- 面板按钮的板上入口（2026-10-23）：`deploy/netgun_fire_button.py serve` 是常驻发射
  通道（节点 `netgun_fire_server`，随 `run_interceptor_stack.sh start` 启动，launch 退出即停）。
  火控进程运行满 3 s、端点进入图后才建状态订阅/发射发布端，提前完成匹配。
  `fire`/`status` 经 `~/netgun_logs/netgun_fire_d<域>.sock`（0600）受理：状态超过
  1.5 s 未更新或尚未匹配时回复退回，由助手改走直连。门控与直连路径相同：
  `fready=1` 才发布一次；以 `/tracking/virtual_shot_event` 或 shots 增加作为确认。
  受理后超时不重发。

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
