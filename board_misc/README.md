# board_misc — 板子上散落在主目录的零散脚本

这些文件原本散落在 `/home/uavlab/` 主目录和 `~/search_camera_test/tools/` 下，
备份时复制到此处归档（**原文件仍在原位，未移动**）。

| 文件 | 原位置 | 用途 |
|---|---|---|
| `analyze_gate_bag.py` | `~/analyze_gate_bag.py` | bag 包门/目标数据分析 |
| `capture_timestamps.py` | `~/capture_timestamps.py` | 相机标定采集时间戳记录 |
| `synthetic_dropout_node.py` | `~/synthetic_dropout_node.py` | 合成丢帧测试节点 |

2026-10-07 去重说明：以下三份副本与仓库内权威文件字节完全相同（SHA-256
配对校验后删除，`git checkout 96b44f9 -- <路径>` 可恢复；板端原位置文件未动）：

| 已删除副本（原位置） | 仓库内权威位置 |
|---|---|
| `controller_p1fixed_20260905.cpp`（`~/controller_p1fixed_20260905.cpp`） | `ros2_ws/src/px4_offboard_tracking/src/pure_interceptor_controller.cpp` |
| `search_camera_test_tools/three_camera_preview.sh`（`~/search_camera_test/tools/`） | `ros2_ws/src/falcon_intercept_deepstream/tools/three_camera_preview.sh` |
| `search_camera_test_tools/three_camera_preview_power_safe.sh`（同上） | `ros2_ws/src/falcon_intercept_deepstream/tools/three_camera_preview_power_safe.sh` |

相关说明：

- 主目录软链 `~/flight.sh` → `ros2_ws/src/falcon_intercept/tools/field_flight.sh`
- 主目录软链 `~/select_yolo_model.sh` → `ros2_ws/src/falcon_intercept_deepstream/tools/select_yolo_model.sh`
- `.bashrc` 中含 px4-ros2-runtime 环境改动，等价的幂等环境脚本见 `deploy/px4_ros2_environment.sh`
