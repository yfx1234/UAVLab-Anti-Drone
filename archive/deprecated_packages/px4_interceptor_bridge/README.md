# px4_interceptor_bridge（Legacy，停止使用）

> 本包属于旧的 PX4 内部拦截模式链路，当前重构分支不再构建或启动。正式链路使用
> `px4_offboard_tracking` 通过标准 PX4 v1.15.4 Offboard 消息完成控制。此目录仅用于
> 阅读历史实现，不得重新加入活动 launch、依赖或部署清单。

目标信息中继与PX4拦截模式协调包。当前Milestone 1实现数据中继，不包含PD/MPC或飞行控制输出。

## 数据链

```text
/perception/target/camera_position
  → 输入契约、时间戳、sequence、置信度和协方差检查
  → 采样时刻VehicleAttitude匹配/SLERP
  → optical → FRD → relative NED
  → 位置协方差旋转
  → /px4_1/fmu/in/target_relative_measurement
```

PX4 v1.15的uXRCE-DDS桥会自动将ROS/Agent时间戳转换到PX4 HRT时间域。本节点把原始曝光时间写入`timestamp_sample`，不得再手工叠加`TimesyncStatus.estimated_offset`。

## 启动

```bash
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
ros2 launch px4_interceptor_bridge interceptor_bridge.launch.py
```

默认仅向实例1拦截机发布。相机外参、输入frame、超时、姿态同步容差、置信度和协方差阈值均在`config/interceptor_bridge.yaml`中配置。

相机帧会先进入短时等待队列，姿态消息随后到达时再进行采样时刻匹配。节点保留DDS同步状态用于诊断。PX4 uXRCE客户端已经在建立正常通信前等待时间同步收敛，因此ROS侧默认不再使用单个RTT状态作为重复的硬门控；如特定集成需要，可通过`require_timesync`重新启用。

时间偏移使用短窗口内的低延迟姿态样本估计。孤立的DDS排队尖峰不会清空姿态缓冲，只有连续多个样本确认时钟域改变后才重置。节点按PX4 `timestamp_sample`过滤重复或回退的姿态样本，并周期输出发布数、拒绝数、成功率、延迟异常数、待处理队列和估计偏移。

连续性验收建议：

```bash
ros2 topic hz /px4_1/fmu/out/vehicle_attitude
ros2 topic hz /px4_1/fmu/in/target_relative_measurement
ros2 topic echo /px4_1/fmu/in/target_relative_measurement --once
```

20 Hz相机输入下，输出应稳定在约20 Hz，通常最大间隔不超过0.1 s；连续30 s不应出现`synchronized attitude timeout`，通信诊断成功率应接近100%，待处理队列应回到0。

姿态消息同时使用DDS时间戳和ROS接收时间估计时钟映射。若WSL/SITL中的DDS offset发生跳变，节点会清空旧姿态缓存并立即重建映射，再用校正后的采样时间与相机曝光时间匹配。

节点拒绝以下输入：无效标志、错误frame、过期/未来时间、重复或倒退sequence、低置信度、非有限/非正深度、非对称或非半正定协方差、DDS时间同步长期失效、等待超时后仍缺少采样时刻姿态。

本包不发布`TrajectorySetpoint`、姿态、角速度、推力或持续Offboard控制设定值。
