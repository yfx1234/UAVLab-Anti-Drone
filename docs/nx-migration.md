# Orin NX 16GB 迁移

2026-10-13 已换装 P3767-0000 / NX16GB，原载板与NVMe保留。已部署保留原载板接线的 NX 候选设备树、正确的 NX 25W 配置、NX 原生 TensorRT 引擎，并恢复硬件编码器。现行结果见 [修复与优化记录](NX修复与优化_2026-10-13.md) 和 `environment/current-nx16.json`；[完整检查](NX完整检查_2026-10-13.md) 是修复前状态。以下步骤供重新移植参考，不代表现机仍未适配。

## 历史 Nano 实测基线

Orin Nano 8GB，DAMIAO ORIN NX V2 系列载板，ZED X Mini（用户称 4 mm）、ZED Link Mono、H100；Ubuntu 22.04、L4T 36.4.3、5.15.148-tegra、CUDA 12.6、TensorRT 10.3、ZED SDK 5.5、ROS2 Humble。详细版本与当前文件哈希在 `environment/current-nano.json`。

载板小版本没有从系统独立读出。镜像源为 Seeed J401，不等于当前实物载板是 J401。当前驱动适配是现机验证结果，不是厂商对该组合的通用兼容承诺。

## 迁移顺序

1. 保存现有系统/数据与启动配置备份。根据已核实的NX P3767-0000与载板获取匹配的BSP；当前实际仍使用Nano p3767-0003基础设备树，需完成适配并保留回退入口。
2. 核对 NX 供电、散热、NVMe、CSI0 接线、ZED Link Mono 独立供电。建立和当前一致的用户 `uavlab`、项目目录 `/home/uavlab/Jetson_ROS2_V0` 和图传地址。
3. 按实际 L4T/内核安装相机驱动、ZED SDK 与权限。参考 `environment/hardware/README.md` 和适配记录。确认 SDK 能枚举相机、读取左右图与深度，其他相机程序退出后再启动平台。
4. 安装 ROS2 Humble、C++17/CMake、colcon、CUDA 开发文件、TensorRT 开发库、OpenCV core/imgproc/imgcodecs/videoio、OpenSSL 开发库及 ament 的 GTest/PyTest 支持。软件版本先对照环境清单，不直接升级整个 BSP。
5. 在 NX 克隆仓库，执行 `scripts/stage_board.py --dry-run`，核对目标目录后正式复制源码/模型。脚本对改动文件留备份，不删除录像，不部署硬件启动配置。
6. 释放相机后运行 `scripts/build_board.sh`。默认只构建两个感知包，关闭历史滤波/飞控工具安装与对应测试；6 组原生感知测试应通过。
7. 在 NX 上从 ONNX 重建引擎，存入 `weights/candidates/platform_exp_20260909/exp_opt3.engine`，保持 batch=1、输入 FP32 NCHW `[1,3,736,1280]`、输出 FP32 `[1,300,6]` 与端到端 xyxy/score/class 合同。允许内部 FP16，但不能擅自改输入/输出类型或加入另一种 NMS 格式。
8. 运行 `build_model_manifest.py` 生成新模型身份；相机关闭时用 `--check-model` 验证合同。以新引擎 SHA256 作为 model_id，不能沿用 Nano 的 f5ca… 模型 ID。保留源 PT/ONNX 哈希，张量合同通过不代表识别精度验收。
9. 电脑核对 NX 的 SSH 主机公钥指纹后更新 `panel/board_client.py`。固定图传电脑/板子 IP 和项目目录若保持一致，其余连接路径可继续复用。若换用户/根目录，还须更新 RPC 和 `session_manager.py` 中的固定数据路径。
10. 按需安装 `board/zed_tracking/tools/install_clock_sync.sh`。它只开放固定校时程序；电脑必须自身校时正确。RTC 完全断电保时仍需实测与载板备份供电确认。

示例引擎构建（仅在 NX 上、相机释放时执行，实际参数先由 `trtexec --help` 核对）：

```bash
cd /home/uavlab/Jetson_ROS2_V0/weights/candidates/platform_exp_20260909
/usr/src/tensorrt/bin/trtexec --onnx=exp.onnx --saveEngine=exp_opt3.engine --fp16 --builderOptimizationLevel=3
python3 /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/zed_tracking/tools/build_model_manifest.py "$PWD/exp_opt3.engine"
```

TensorRT 引擎对硬件和运行版本有兼容约束；JetPack 不提供通用硬件兼容模式保障，因此在 NX 重建并验证。[NVIDIA TensorRT 10.x 兼容说明](https://docs.nvidia.com/deeplearning/tensorrt/10.x.x/inference-library/version-compatibility.html)

## NX 验收项目

先 1080p/15 fps，再 1080p/30 fps；分别测 RGB、RGB+无损录像、YOLO+深度、YOLO+深度+录像及同时图传。每组记录独立帧数、SDK时间戳、实际持续时间、掉帧、可用内存、温度、磁盘写入与错误；检查导出视频时长及场次状态。MP4补帧后的标称30fps不算采集达标。

相机坐标测试用约30cm实物四旋翼、人工量距和静置不同方位完成，分别评估深度有效/无效两种定位；暂未进行。当前 Nano 的30fps无损短测仅约18.2fps，高帧率默认标为实验，不承诺 NX 到位后自动达标。

现机已完成启动/功耗、原生引擎与编码链路修复。默认仍为1080p15原图无损；30fps、真实天空/移动相机和实物定位按修复记录的各项结果判断，不能仅凭 NX 型号或 MP4 标称帧率验收。
