# anti_drone_interfaces

`anti_drone_interfaces` 是仿真感知、实机感知与拦截控制之间唯一共享的 ROS 2 接口包。
它不包含识别算法、控制算法、Gazebo 真值或裁判逻辑。

接口版本：V1
ROS 版本：ROS 2 Humble

## 1. 固定数据边界

```text
仿真：
Gazebo真值 -> logical_camera_simulator
                       ├─ /perception/search/observation
                       └─ /perception/depth/observation

实机：
搜索相机图像 -> 协作组搜索识别/适配节点
                       └─ /perception/search/observation
双目相机图像 -> 协作组双目识别/深度适配节点
                       └─ /perception/depth/observation

两种输入之后完全共用：
相机消息 -> relative_estimator -> pure_interceptor_controller -> PX4 Offboard
```

仿真和实机必须使用相同的消息类型、话题、坐标定义、时间戳语义和 QoS。控制器不根据
消息来源选择分支，也不得订阅 Gazebo、目标机 PX4 状态或裁判真值。

## 2. 正式话题

| 话题 | 类型 | 频率建议 | QoS |
|---|---|---:|---|
| `/perception/search/observation` | `anti_drone_interfaces/msg/SearchObservation` | 20–60 Hz | SensorDataQoS |
| `/perception/depth/observation` | `anti_drone_interfaces/msg/DepthObservation` | 20–60 Hz | SensorDataQoS |

SensorDataQoS 在本项目中表示 `best_effort + volatile + keep_last(5)`。发布端不得使用
transient-local 保存旧观测。

协作组可以另外发布标准的 `sensor_msgs/msg/Image` 和
`sensor_msgs/msg/CameraInfo`，供识别、标定和可视化使用；纯控制链不直接订阅原始图像，
只订阅上述两个目标观测话题。

## 3. 公共时间、帧和序号规则

- `header.stamp`：原始图像曝光中点或硬件采集时刻，转换到 ROS 时钟域；
- 禁止填写推理结束时间、消息发布时间或接收时间；
- `header.frame_id`：产生该观测的规范化逻辑相机 optical frame；
- `sequence`：每个相机流独立单调递增，包含未检测帧，`uint32` 回绕允许；
- `track_id`：感知节点维护的目标轨迹 ID；单目标初始实现可固定为 `1`，无目标为 `0`；
- `interface_version`：必须填写消息常量 `INTERFACE_VERSION_V1`；
- 搜索相机与双目相机不要求同帧发布，估计器按各自采集时间与机体姿态对齐；
- 同一次发布中的像素、角度、LOS、深度和三维点必须对应同一个目标和同一个采集时刻。

规范 optical 坐标统一为：

```text
Optical +X：图像向右
Optical +Y：图像向下
Optical +Z：相机光轴向前
```

## 4. 搜索相机消息

类型：`anti_drone_interfaces/msg/SearchObservation`

搜索消息只表达单目可获得的无尺度信息：

- 是否检测到目标；
- 置信度和目标轨迹 ID；
- 图像尺寸；
- 目标参考像素和可选检测框尺寸；
- 横、纵视线角；
- 角度协方差；
- optical frame 下单位 LOS。

搜索消息在结构上不得出现：

- 深度；
- 斜距；
- 三维位置；
- 目标真值；
- 仿真故障或“是否注入离群点”标签。

字段约束：

```text
0.0 <= confidence <= 1.0
0.0 <= pixel_u < image_width
0.0 <= pixel_v < image_height
norm(unit_los_optical) = 1
unit_los_optical.z > 0
```

并满足：

```text
horizontal_angle_rad = atan2(unit_los_optical.x, unit_los_optical.z)
vertical_angle_rad   = atan2(unit_los_optical.y, unit_los_optical.z)
```

`angle_covariance` 是 `[horizontal, vertical]` 的 2×2 行优先协方差，单位为
`rad²`。

## 5. 双目深度相机消息

类型：`anti_drone_interfaces/msg/DepthObservation`

除二维检测信息外，双目消息必须明确给出：

```text
depth_z_m = position_optical.z
range_m   = norm(position_optical)
```

两者不可混用。`position_covariance` 是
`[x_optical, y_optical, z_optical]` 的 3×3 行优先协方差，单位为 `m²`。

深度测量必须同时满足：

```text
depth_valid = true
depth_z_m > 0
range_m >= depth_z_m
position_optical.z > 0
```

`angle_covariance` 的定义与搜索消息相同。

## 6. 无检测和无效深度

相机节点应持续发布帧状态，不能仅在检测成功时发布：

- 无检测：`visible=false`、`track_id=0`、`confidence=0`；
- 无检测时所有不可用的浮点测量填写 `NaN`；
- `bounding_box_valid=false` 时框宽和框高填写 `0`；
- `angles_valid=false` 时角度和角度协方差填写 `NaN`；
- `unit_los_valid=false` 时单位 LOS 各分量填写 `NaN`；
- 双目检测到目标但深度失败时可以保持 `visible=true`，但必须
  `depth_valid=false`，深度、斜距、三维点和位置协方差填写 `NaN`；
- 不得用 `0 m` 表示无深度，也不得沿用上一帧深度冒充当前测量。

控制器和估计器会独立检查消息版本、有效标志、时间新鲜度和数值有限性。

## 7. 当前逻辑相机标定

### 7.1 深度相机

规范逻辑 frame：`pursuer/depth_camera_optical`

```text
Optical +X = Body +Y
Optical +Y = Body +X
Optical +Z = Body -Z
```

```text
R_body_from_depth_optical =
[ 0  1  0
  1  0  0
  0  0 -1 ]
```

保守视场：

- 水平半视场 `8.5°`；
- 垂直半视场 `10°`。

### 7.2 搜索相机

规范逻辑 frame：`pursuer/search_camera_optical`

搜索光轴相对 Body `-Z` 朝 Body `+X` 偏转 `10°`：

```text
R_body_from_search_optical =
[ 0  0.984807753  0.173648178
  1  0            0
  0  0.173648178 -0.984807753 ]
```

视场：

- 水平半视场 `17.5°`；
- 垂直半视场 `30°`。

若实物相机的原始 optical frame 与上述逻辑 frame 不同，协作组适配节点必须使用标定外参
把检测结果转换到上述逻辑 frame 后再发布。本阶段不要求修改现有 TF 树。

## 8. 实机发布端验收

协作组发布端至少应通过：

1. 接口版本为 V1；
2. 采集时间戳严格递增且处于 ROS 时钟域；
3. 目标静止于图像中心时，横纵角接近零；
4. 目标向图像右侧移动时，`pixel_u`、横向角和 LOS.x 同号增大；
5. 目标向图像下方移动时，`pixel_v`、纵向角和 LOS.y 同号增大；
6. 深度点满足 `z=depth_z_m` 和 `norm(point)=range_m`；
7. 丢检和深度失效按第 6 节发布，不复用旧值；
8. 消息中不存在目标真值、Gazebo 状态或仿真故障标签；
9. 断开仿真相机、启动实机节点后，控制和估计节点无需改代码或重映射类型。

接口检查命令：

```bash
source /opt/ros/humble/setup.bash
source /home/key/UAV_ws/install/setup.bash

ros2 interface show anti_drone_interfaces/msg/SearchObservation
ros2 interface show anti_drone_interfaces/msg/DepthObservation
ros2 topic info -v /perception/search/observation
ros2 topic info -v /perception/depth/observation
ros2 topic hz /perception/search/observation
ros2 topic hz /perception/depth/observation
```

## 9. 历史接口

`TargetCameraPosition` 仅用于历史 PX4 内部中继分支。新的纯 Offboard 控制链和协作组实机
感知节点不得使用该消息。
