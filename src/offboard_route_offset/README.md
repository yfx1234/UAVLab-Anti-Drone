# offboard_route_offset

PX4 v1.15.4、ROS 2 Jazzy 与 Gazebo Sim 的人工接管型 Offboard 轨迹节点。3.0 默认使用
`velocity_path` 后端和固定机头，以 Frenet 速度跟踪取代正式轨迹阶段的前方位置目标追赶。

节点不发布 `VehicleCommand`，不会主动解锁、起飞、切换模式、降落或上锁。飞手切出
Offboard，或 PX4 进入 Land/RTL/Descend、接地、上锁状态后，节点停止发布会干扰飞手的
`TrajectorySetpoint`；重新进入 Offboard 时先从当前状态平滑接近固定任务起点。

主要文件：

- `src/offboard_route_offset_node.cpp`：安全状态机、路径、投影、速度规划和 PX4 setpoint。
- `config/route_offset.yaml`：默认参数。
- `launch/offboard_route_offset.launch.py`：节点 launch。
- `scripts/tune_route_offset.sh`：主要启动和调参入口。
- `scripts/collect_offset_metrics.py`：ROS CSV 数据采集。
- `scripts/plot_velocity_path_comparison.py`：修改前后对比图。
- `docs/offboard_route_offset_usage.md`：完整构建、Gazebo Sim、调参和实机低速流程。
- `docs/velocity_path_test_report.md`：基线、增量仿真与限制。

快速构建与启动节点：

```bash
cd /home/danyu/offboard_guojun_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select offboard_route_offset --symlink-install
source install/setup.bash
ros2 run offboard_route_offset tune_route_offset.sh
```

推荐默认值为 `trajectory_control_backend=velocity_path`、
`trajectory_setpoint_rate_hz=50`、`yaw_mode=fixed` 和
`square_corner_mode=rounded`。`position_feedforward` 与 `square/stop_turn` 保留用于回归和精确
顶点任务；`z_shape` 的圆滑路径仍属于实验功能，完成专项 Gazebo Sim 验收前不建议实机使用。
