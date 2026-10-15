# weights/ — 模型资产说明

2026-10-07 结构整理：本目录原有大量字节完全相同的副本（同一份权重以多个
分辨率/量化命名并存），已按 SHA-256 去重。**删除的只是别名副本，每一份
内容都保留了唯一权威位置**；下表删除前的文件名如有仓库外脚本仍在使用，
按"等价于"列取对应文件，或用
`git checkout 96b44f9 -- weights/<原文件名>` 恢复原路径。

## 当前保留

| 文件 | 说明 |
|---|---|
| `uav_best.pt` | 当前一代 uav_best 权重（Torch 源，SHA-256 前缀 `d673dae6`） |
| `uav_best_1280.onnx` / `uav_best_1280_int8.onnx` / `uav_best_1280x736_fp16.onnx` | uav_best 的 ONNX 导出；`deepstream_preview.sh`、`record_three_yolo_annotated.sh` 引用 |
| `uav_best_1280_int8_opt3.metadata.json` | int8_opt3 engine 构建元数据（`VENDORED_DEPENDENCIES.md` 的 trtexec 命令对应） |
| `uav_best_1440x2560.onnx`、`yolo26n_*.onnx` | 各分辨率独立导出的 ONNX（内容互不相同，非重复） |
| `candidates/0728/` | 生产回退代（engine 由板上编译，`uav_0728_1280x736_fp16.pt/.onnx`） |
| `candidates/platform_exp_20260909/` | 当前实验代（`exp.pt/.onnx`，`field_flight.sh`/launch 引用其 engine） |
| `archive/20260720_67f84a66/` | 上一代归档（SHA-256 前缀 `67f84a66`） |
| （包内）`ros2_ws/src/falcon_intercept/models/yolo26n.pt` | yolo26n Torch 源权威副本，launch 实际引用这一份 |

## 已删除的重复别名（等价关系）

| 已删除文件名 | 等价于 |
|---|---|
| `uav_best_1280.pt`、`uav_best_1280_int8.pt`、`uav_best_1280x736_fp16.pt`、`uav_best_candidate_20260728.pt`、`uav_best_candidate_20260728_1280.pt`、`uav_best_candidate_20260728_1280_int8.pt` | `uav_best.pt`（candidate_20260728 当年已被提升为 best，文件为同内容拷贝） |
| `uav_best_1280_int8_ultralytics.pt`、`uav_best_1440x2560.pt` | `archive/20260720_67f84a66/` 内对应文件（旧代内容） |
| `yolo26n.pt`、`yolo26n_1280.pt`、`yolo26n_1440x2560.pt`、`yolo26n_2176x3840.pt` | 包内 `ros2_ws/src/falcon_intercept/models/yolo26n.pt`（.pt 与推理分辨率无关，命名是别名） |
| `incoming/0728.pt` | `candidates/0728/uav_0728_1280x736_fp16.pt`（暂存目录未清理） |
| `uav_best_candidate_20260728_1280.onnx` | `uav_best_1280.onnx` |
| `uav_best_candidate_20260728_1280_int8.onnx` | `uav_best_1280_int8.onnx` |
| `uav_best_1280_int8_ultralytics.onnx` | `archive/20260720_67f84a66/uav_best_1280_int8_ultralytics.onnx` |
| `uav_best_1280_int8_provisional.metadata.json` | `archive/20260720_67f84a66/` 内同名文件 |
| `uav_best_candidate_20260728_1280_int8.metadata.json` | `uav_best_1280_int8_opt3.metadata.json` |

注意：`.engine` 文件是板端本机编译产物（见根 `.gitignore`），不在本仓库；
新引擎一律放 `candidates/<名称>/`，不要在根目录另起别名文件。
