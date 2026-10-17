# offboard_route_offset 2.1 发布说明

发布日期：2026-07-20

版本：`2.1.0`

## 版本定义

2.1 是当前“偏移原点”程序的稳定版本，完整继承 2.0 的安全边界：

```text
不自动解锁
不自动起飞
不自动切换 Offboard
不自动降落或上锁
切出 Offboard 后立即停止发布 trajectory setpoint
再次进入 Offboard 时重新前往本次起飞点对应的固定轨迹起点
```

2.1 在此基础上增加闭环轨迹进度、jerk 受限速度规划、正弦路径优化和端点稳定控制。

## 主要变化

### 1. 闭环轨迹进度

直线、圆形、正方形和正弦不再只按固定时间推进参考点。程序实时把飞机位置投影到路径，计算：

```text
s_vehicle = 飞机实际路径进度
s_ref     = 参考路径进度
lead      = s_ref - s_vehicle
```

飞机跟踪正常时，参考点按规划速度推进；飞机落后时，参考速度逐渐降低；领先达到上限时暂停推进，待飞机追上后自动恢复。发布的位置、速度和加速度由同一轨迹状态生成。

### 2. 速度与加速度规划

- 使用实际单调时钟和实际 `dt`，不依赖循环次数；
- 限制最大加速度、最大减速度和最大 jerk；
- 接近速度目标前提前卸除加速度，避免规划速度自身超调；
- 使用实际速度、实际剩余距离和响应延迟提前制动；
- 终点增加两帧预测余量，避免参考位置到达终点后再硬钳制速度；
- 路径进度被限制时，将实际限制结果同步回速度规划器，避免残留速度和加速度尖峰。

### 3. 正弦轨迹

正弦轨迹保留原有几何定义，新增累计弧长投影、实际进度限制、解析切向量和曲率、曲率限速、向心加速度前馈以及最终端点平滑减速。

### 4. 直线端点

直线使用 `ACCEL -> CRUISE -> BRAKE -> SETTLE` 阶段。端点完成需要实际位置和实际速度持续满足容差。往返路径必须完成：

```text
减速到零 -> 端点稳定 -> 原地转向 -> yaw 稳定 -> 反向加速
```

### 5. 正方形顶点

每个顶点执行：

```text
BRAKE -> VERTEX_SETTLE -> YAW_TURN -> YAW_SETTLE -> ACCEL
```

转向期间固定位置，速度和加速度设为零。下一条边不会与机头转向同时加速。

### 6. 终点容差

水平路径完成判据拆分为水平和垂向容差，避免实机高度估计轻微波动反复清零水平端点的稳定计时器：

```yaml
endpoint_position_tolerance_m: 0.12
endpoint_speed_tolerance_mps: 0.20
endpoint_altitude_tolerance_m: 0.25
endpoint_vertical_speed_tolerance_mps: 0.30
endpoint_settle_time_s: 0.50
```

实机定位噪声较大时，应根据无桨测试日志逐步放宽，不应直接设置过大的容差。

## 仿真验证摘要

测试环境：PX4 v1.15.4、ROS 2 Jazzy、Gazebo `gz_x500`。

| 路径 | 请求速度 | 规划峰值 | 实际峰值 | SETTLE 速度 | 最大参考领先 |
| --- | ---: | ---: | ---: | ---: | ---: |
| 正弦 | 1 m/s | 1.000 | 1.165 | 0.085 | 0.131 m |
| 正弦 | 3 m/s | 1.233 | 1.276 | 0.077 | 0.196 m |
| 直线 | 1 m/s | 1.000 | 1.220 | 0.052 | 0.269 m |
| 直线 | 3 m/s | 3.000 | 3.155 | 0.101 | 0.377 m |
| 正方形 | 1 m/s | 1.000 | 1.292 | 等待后停稳 | 0.383 m |
| 正方形 | 3 m/s | 1.873 | 2.125 | 0.030 | 0.349 m |

直线 1 m/s 和 3 m/s 的终点超调分别为 0.142 m 和 0.173 m。正方形转向阶段横滚和俯仰峰值约在 1.2 度以内。

3 m/s 正弦和边长 3 m 的正方形受曲率、路径长度及动力学参数限制，无法达到 3 m/s。该行为由参数和路径计算得出，不是隐藏速度硬编码。

完整测试说明见：

```text
src/offboard_route_offset/docs/test_report.md
test_logs/sine_endpoint_round/REPORT.md
test_logs/sine_endpoint_round/metrics.json
```

## 已验证与未验证范围

已完成 SITL 和日志验证：

```text
固定偏移起点
line
line_reciprocating
circle
sine
square
切出 Offboard 后停止 setpoint
空中重新进入 Offboard 后任务复位
QGC/PX4 Land 静默
```

尚未完成 2.1 同等级闭环优化和最终日志验收：

```text
z_shape
```

因此 `z_shape` 不应直接用于首次实机测试。

## 从 2.0 升级到 2.1

建议把整个包复制到实机，不要只替换二进制或单个 CPP：

```bash
rsync -av ~/offboard_guojun_ws/src/offboard_route_offset/ \
  USER@REAL_HOST:~/offboard_guojun_ws/src/offboard_route_offset/
```

在实机重新编译：

```bash
cd ~/offboard_guojun_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select offboard_route_offset --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
```

启动日志应包含：

```text
offboard_route_offset 2.1.0 started
```

确认安装的包版本：

```bash
ros2 pkg xml offboard_route_offset | grep '<version>'
```

预期输出：

```xml
<version>2.1.0</version>
```

## 实机使用限制

1. 首次测试必须移除螺旋桨，确认 PX4 状态、位置数据和 setpoint 发布逻辑；
2. 首次带桨只使用低高度、短距离、低速度直线；
3. 同一时间只能运行一套 Offboard 控制节点；
4. 飞手必须保持随时切出 Offboard 的能力；
5. 曲线速度可能受 `max_safe_lateral_acceleration_mps2` 限制；
6. 仿真通过不等同于实机放飞许可。
