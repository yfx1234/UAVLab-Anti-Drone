# velocity_path Gazebo Sim 验收报告

日期：2026-07-20 至 2026-07-21。环境：PX4 v1.15.4、ROS 2 Jazzy、Gazebo Sim Harmonic 8.14.0、
`gz_x500`、`default.sdf`、Micro XRCE-DDS Agent 2.4.0。

## 基线与根因结论

修改前 `RUNNING_TRAJECTORY` 使用 PX4 位置控制，同时填写移动位置目标、速度/加速度前馈和 yaw。
源码中的 `PathProgressGovernor` 根据 `s_ref-s_vehicle` 连续降速，并把参考进度硬限制为
`s_vehicle+max_reference_lead_distance_m`；发生裁剪时还同步 `SpeedPlanner`。因此只要实机跟踪持续落后，
就具备形成“参考推进—触及 lead—参考减速/暂停—追上—恢复”的条件，同时 PX4 位置误差速度修正会
叠加在速度前馈上。

Gazebo x500 的 3 m/s 加速基线确实触及 0.50 m lead 上限，规划目标降至 2.348 m/s，实际速度峰值
3.270 m/s（相对 3 m/s 为 9.0% 超调）。稳定 1 m/s 巡航时仿真机跟踪较好，没有复现持续周期循环；
因此结论是“代码和数据确认触发机制存在，但稳定默认 x500 未复现实机那样的反复触发”，不能把所有
实机振荡都未经验证地归因于同一机制。

## 增量测试

长直线公平巡航窗口采用：实际投影超过 5 m、剩余超过 5 m、规划速度超过 0.98 m/s、规划加速度
绝对值小于 0.05 m/s²。

| 配置 | 命令均值 | 实际均值 | 实际标准差 | 实际范围 | 速度误差标准差 |
|---|---:|---:|---:|---:|---:|
| 修改前 position+FF | 1.000 | 1.045 | 0.0357 | 0.967--1.144 | 0.0357 |
| velocity_path，无加速度前馈 | 1.000 | 1.007 | 0.0112 | 0.987--1.043 | 0.0112 |
| velocity_path，有加速度前馈 | 1.000 | 0.999 | 0.0088 | 0.979--1.017 | 0.0088 |

纯速度后端把巡航速度波动降低约 68.5%，加入一致的路径加速度前馈后降低约 75.3%。速度 setpoint 的
位置三轴均为 NaN；第一阶段 acceleration 三轴均为 NaN，第二阶段正式运行段为有限前馈。

其他验收：

| 测试 | 结果 |
|---|---|
| 终端捕获 | 直线、圆形、正弦均先连续减速再切位置保持；未在进入容差时跳位置目标 |
| 正弦 12 m/1.5 m/2 周期 | 曲率 -1.60--1.60 1/m，曲率限速最低 0.967 m/s，横向误差绝对均值 0.104 m |
| 圆形固定 yaw | yaw setpoint 恒定 0，实际 heading 仅小幅波动 |
| 圆形切线 yaw | 最终代码 CW 连续圆稳定段命令 1.000±0.000 m/s、实际 0.973±0.0129 m/s；横向误差绝对均值 0.131 m、最大 0.248 m |
| 往返固定 yaw | setpoint `vx` 从正变负，yaw 始终 0；返程不原地转头 |
| 往返切线 yaw | 第一腿停稳后约 1.86 s 平滑转 180°，再反向加速；yaw rate 不超过 45 deg/s |
| racetrack | 4 m 直线、2 m 半径，正式段命令速度标准差近零，实际标准差 0.0183 m/s |
| square rounded | 正确闭合长度 14.28 m；20 s 跨圈运行无顶点停车，命令始终 1.0 m/s |
| square stop_turn | 四个顶点均停车，每个安全条件满足后约 0.5 s 即开始下一边；固定 yaw 不转头 |
| Offboard 退出 | Auto Loiter 后 4 s 订阅窗口内 trajectory_setpoint 为 0 字节 |
| Offboard 重入 | 重新进入 APPROACH_START，再重建投影/规划/yaw 状态并开始轨迹 |
| Land/disarm | 进入 LANDING_OR_DISARMED，停止心跳和轨迹 setpoint |

修改前后总览图：

```text
test_logs/velocity_path_upgrade/plots/before_after_overview.png
```

关键 CSV 位于：

```text
test_logs/velocity_path_upgrade/before/
test_logs/velocity_path_upgrade/phase1/
test_logs/velocity_path_upgrade/phase2/
test_logs/velocity_path_upgrade/circle_fixed/
test_logs/velocity_path_upgrade/circle_tangent/
test_logs/velocity_path_upgrade/sine_fixed/
test_logs/velocity_path_upgrade/reciprocal_fixed/
test_logs/velocity_path_upgrade/reciprocal_tangent/
test_logs/velocity_path_upgrade/racetrack/
test_logs/velocity_path_upgrade/square_rounded/
test_logs/velocity_path_upgrade/square_stop_turn/
```

最终 yaw-rate 去重修正后的圆形 tangent 专项文件为
`circle_tangent/circle_velocity_tangent_final_continuous_1mps.csv`。901 个去重正式轨迹样本中，位置
setpoint 三轴全为 NaN；CW 曲率为正（稳定值约 0.50 1/m），稳定 yaw-rate 为正，绝对值不超过
0.7854 rad/s（45 deg/s）。yaw 跟踪误差绝对均值 0.0257 rad（1.48 deg），最大 0.1195 rad。

## 仿真基础设施观察

Gazebo Sim 冷启动时首次 PX4 model create 可能在 server 就绪前超时；第二次相同 `gz_x500` 命令能
连接已运行的 `default` world。长时间暂停对话并让 SITL 全量 ULog 持续运行后，VM 出现 timesync
跳变、传感器 timeout 和磁盘快速增长；相关失败飞行已作废，重新冷启动后再测试。这个现象属于当前
VM/Gazebo 实时性限制，不应通过调低 PX4 速度环增益掩盖。

最后一次圆形 tangent 复测在起飞前通过 PX4 shell 停止 `logger=all`，避免 VM 时间跳变再次放大全量
ULog；ROS CSV 完整保留。此前同一轮产生的单个 5.6 GiB 异常膨胀 ULog 已删除并释放磁盘，不属于
验收数据集。

## 尚存限制

- `z_shape` 已改为统一圆滑多段路径，但没有完成本轮专项飞行验收，仍为实验功能。
- 圆角正方形采用直线加圆弧，位置和切线连续，但连接处曲率不连续；更高速度需要 clothoid 或五次样条。
- 投影是局部离散段搜索；复杂自交路径依靠窗口避免跳支路，但没有全局拓扑标识。
- `square/stop_turn` 为兼容保留的精确位置控制模式，不代表连续速度后端。
- Tangent yaw 会占用姿态/混控余量；本次圆形数据也显示固定 yaw 的横向跟踪更好，因此默认保持 fixed。
- 本报告是 SITL 低速结果，不替代实机无桨检查、定位质量和动力饱和评估。
