# 上游依赖版本锁定（未纳入本仓库的部分）

本仓库只包含自研代码与训练模型。以下依赖来自上游开源项目，重建环境时按
下方 commit 重新克隆即可，因此不进仓库。

## ROS 2 消息接口

| 依赖 | 上游 | 分支 / Commit | 说明 |
|---|---|---|---|
| px4_msgs | https://github.com/PX4/px4_msgs.git | release/1.15 @ `a1045ec4feb6d709bdecaf3895f1d5b43a5dabb8` | 原 `ros2_ws/src/px4_msgs`。本地与上游的差异仅为 README.md、package.xml 两处小改动，其余 220 个"改动"文件均为行尾差异，无实质内容 |

## PX4–ROS2 桥接（原 `~/px4_bridge_ws/src/`）

| 依赖 | Commit |
|---|---|
| Micro-XRCE-DDS-Agent | `57d0862` |
| Micro-XRCE-DDS-Client | `0c6743f` |
| Micro-CDR | `3d1b177` |

## TensorRT engine（`weights/*.engine`）

`.engine` 文件绑定本机 Jetson 的 TensorRT/GPU 版本，换机器或重刷系统后无效。
重新生成方式：用仓库中对应的 `.onnx` 通过 `trtexec` 重编，例如：

```bash
trtexec --onnx=uav_best_1280_int8.onnx --fp16 --saveEngine=uav_best_1280_int8_opt3.engine
```

## 本仓库未包含的大体积数据（重要！）

以下数据**不进 git**，目前仅存在于这块板子上，请务必另用移动硬盘备份：

- `flight_yolo_records/`（约 26G，外场飞行记录）
- `ros2_ws/bags/`（约 3.3G）
- `ros2_ws/calibration_records/`（约 2.7G）
- `test_results/`（约 1.1G，含测试视频/图片）
- `weights/int8_calibration_provisional_1280/`（约 30M，INT8 标定图）
