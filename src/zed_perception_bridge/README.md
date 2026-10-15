# zed_perception_bridge

感知桥：把板上 ZED 感知（`zed_tracking`/`zed_perception_standalone`，main 侧部署）
的观测转换成网枪拦截链 `relative_estimator` 需要的 V1 接口。

## 为什么需要

- 感知侧发布 `/perception/observations`（`zed_perception_interfaces/PerceptionFrame`，
  合同 v2，含 boot_id/新鲜标志/处理耗时）及同帧 JSON 对照
  `/perception/observations_json`（`std_msgs/String`）。
- 拦截链估计器消费 `/perception/depth/observation`
  （`anti_drone_interfaces/DepthObservation`，V1 稳定接口）。
- 两者此前无桥接（交接文档 §八-9 第一项待办的实质）。

本包订阅 **JSON 通道**而非自定义消息通道：拦截工作区（`~/interceptor_ws`）构建期
无需依赖 `zed_perception_interfaces`（消费端免重建消息包），运行期只依赖
`anti_drone_interfaces`。

## 接口

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `/perception/observations_json` | `std_msgs/String` | SensorData |
| 发布 | `/perception/depth/observation` | `anti_drone_interfaces/DepthObservation` | SensorData |
| 发布 | `/debug/bridge_state`（1Hz） | `std_msgs/String` | 可靠 10 |

字段映射与降级规则见 `zed_perception_bridge/mapping.py` 模块注释。要点：
- `header.stamp` 透传 `stamp_ns`（原图曝光时刻），不重打时间戳；
- 仅实测深度（`depth_measured_valid`）置 `depth_valid`；`size_estimate` 估距降级
  为 bearing-only（估计器原生支持，`bearing_only_nominal_range_m=12`）；
- 默认只认 YOLO 类别确认目标（`class_confirmed`），generic 轨道默认排除；
- `intrinsics_fx/fy/cx/cy` 在 `/debug/bridge_state` 透传播报的相机内参，用于核对
  估计器 `camera_fx` 等 launch 参数（初版为 1274 假设值）。

## 参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `selection` | `confidence` | 目标选择策略：`confidence` / `nearest` / `largest` |
| `allow_generic` | `false` | 是否允许 generic（非类别确认）轨道 |
| `min_confidence` | `0.0` | 额外置信度门（0=关闭） |
| `input_json_topic` / `output_topic` / `debug_topic` | 见上表 | 话题重映射 |

## 测试

```bash
python3 -m pytest test/ -q
```

（纯逻辑测试，不依赖 ROS 环境；`mapping.py` 不导入 rclpy。）
