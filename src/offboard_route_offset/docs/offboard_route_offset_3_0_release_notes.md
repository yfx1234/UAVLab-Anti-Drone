# offboard_route_offset 3.0

3.0 将正式轨迹默认后端改为 `velocity_path`，保留 `position_feedforward` 对照模式，并新增：

- 实际位置局部投影、闭合路径解缠和低通进度；
- Frenet 切向/法向速度跟踪与横向误差连续降速；
- NED-z 速度外环及垂直速度、加速度、jerk 限制；
- `TERMINAL_CAPTURE`，将端点运动生成和完成判定解耦；
- fixed/tangent yaw 模式，默认 fixed；
- 50 Hz 默认 setpoint、切线/法向路径加速度前馈；
- 圆角正方形、跑道形连续掉头和实验性圆滑 Z 路径；
- 36 项轨迹调试数组、CSV 采集和对比绘图脚本；
- `tune_route_offset.sh` 后端/yaw/速度路径参数入口和重复节点保护。

所有原有人工解锁、人工起飞、人工模式切换、Land/RTL 静默和 Offboard 重入安全语义保持不变。
