# 虚拟目标分层测试

本工具把实物拦截机追虚拟目标链拆成四种可重复的 shadow 测试。四种模式都复用
`real_virtual_target_v0.launch.py` 和生产控制节点，发布合成 `/fmu/out/*` 状态，
但绝不发布 `/fmu/in/*`。它们不启动或修改 RTK 外场工具。

```bash
cd /home/uavlab/Jetson_ROS2_V0/ros2_ws
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh search
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh tracking
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh strike
bash src/falcon_intercept/tools/run_virtual_target_stage_test.sh full
```

| 模式 | 虚拟相机输入 | 必须通过的断言 |
|---|---|---|
| `search` | 仅搜索相机 | SEARCH扫描有效、目标提示有效、无三维跟踪、无开火 |
| `tracking` | 仅异步双目 | 三维跟踪有效、经过TRACK和APPROACH、无开火 |
| `strike` | 仅异步双目、静止靶 | 经过FIRE/EGRESS/RECOVER/DONE，一发一中 |
| `full` | 搜索＋异步双目、低速直线靶 | 搜索到完整任务状态，两发两中 |

每种模式还会检查三个关键 PX4 输入话题的 ROS 发布者总数为零。失败时脚本返回非零，
并打印本轮计数、经过的状态、最后一帧跟踪/捕获窗/安全门和 launch 日志。

正式入口支持以下场景参数：

- `target_scenario`：`stationary`、`straight`、`circle`、`s_curve`、
  `figure_eight`、`rectangle`；
- `target_speed_mps`、`target_radius_m`、`target_path_width_m`、
  `target_path_length_m`；
- `initial_forward_m`、`initial_left_m`、`initial_up_m`；
- `target_motion_delay_s`、`target_motion_ramp_s`；
- `detection_mode`：`all`、`search_only`、`stereo_only`；
- `detection_start_delay_s`、`detection_dropout_start_s`、
  `detection_dropout_duration_s`；
- `attack_enable_delay_s`、`required_shots`、`recoil_duration_s`、
  `recoil_opening_speed_mps`、`reattack_cooldown_s`。

专项模式只用于定位软件链问题，不替代真实飞控、真实相机、Offboard或飞行验收。
