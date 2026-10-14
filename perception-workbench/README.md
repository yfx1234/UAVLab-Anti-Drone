# ZED 感知、录像与电脑工作台

本目录是独立感知与数据采集模块的源码副本：ZED X Mini 相机采集、YOLO/TensorRT 检测、深度与相机坐标观测、板上 SVO 录像、电脑实时监看和素材管理。源码版本及逐文件 SHA256 见 `SOURCE_MANIFEST.json`。

默认入口为 `zed_perception_standalone` 与 `tools/perception_supervisor.py`，默认采集 RGB；YOLO 可独立开关。当前操作页已停用通用检测，保留相关历史感知实现供复核。默认构建关闭 `UAVLAB_BUILD_LEGACY_TRACKING`，本副本不包含历史飞行、控制、滤波、射击或闭环仿真工具，不随开机启动飞控或控制链路。

## 目录

| 路径 | 内容 |
| --- | --- |
| `board/zed_tracking` | C++ 感知、相机采集、录像、导出与板端管理 |
| `board/zed_perception_interfaces` | ROS2 观测消息与服务合同 |
| `panel` | 电脑本地服务、网页、下载与素材管理 |
| `scripts` | 启动、构建、环境准备与仓库检查工具 |
| `tests` | 回归测试及明确手动执行的硬件验收脚本 |
| `docs` / `environment` | 操作、接口、验证边界与设备环境参考 |
| `DESIGN.md` / `PRODUCT.md` | 当前工作台设计约定 |

## 电脑启动

在本目录中执行：

```powershell
python -m pip install -r requirements-panel.txt
powershell -ExecutionPolicy Bypass -File scripts/start_panel.ps1
```

打开 <http://127.0.0.1:8767/>。启动服务不会自动启动相机。可用 `-DataDirectory` 指定本机素材目录。连接板子前检查 `panel/board_client.py` 的设备地址、项目目录和 SSH 主机指纹；认证信息由本机 SSH 环境或连接界面提供，不写入 Git。

## Jetson 构建与模型

参考环境为 Jetson Orin NX 16GB、Ubuntu 22.04 / L4T 36.4.3、ZED SDK 5.5、ROS2 Humble。先按实际 BSP 安装兼容的相机驱动、ZED SDK、CUDA、TensorRT、OpenCV 和 ROS2；`environment` 是环境参考，不是整机镜像。

本副本只发布源码，**不含 PT/ONNX 模型权重、TensorRT 引擎、模型索引、SVO/视频、采集日志、密钥或编译产物**。YOLO 需要另外提供经过授权的模型，在目标 Jetson 上构建并验证匹配输入尺寸和张量布局的 TensorRT 引擎；不能直接跨模块复用其他设备的引擎。板端配置中的既有路径应按实际部署调整。

将两个 `board` 包以实文件复制到目标工作区后，可构建源码：

```bash
mkdir -p /home/uavlab/Jetson_ROS2_V0/ros2_ws/src
cp -a board/zed_perception_interfaces board/zed_tracking /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/
bash scripts/build_board.sh /home/uavlab/Jetson_ROS2_V0
```

构建前释放相机。上述构建脚本不启动相机；运行时按实际部署设置 supervisor 与引擎路径。`scripts/stage_board.py` 是原完整仓库的部署助手，要求 `models/model-index.json` 及对应模型原件；本源码副本未提供这些文件，不能直接运行该助手。历史工具选项保持 `OFF`，本副本不支持开启它。

## 使用与验证

日常流程是启动相机、按需开启 YOLO、开始板上录像、停止录像并等待收尾，再下载 RGB/深度展示视频或原始双目 SVO。关闭网页不会停止板上录像。原始无损素材、图传预览和导出的压缩视频用途不同；深度展示视频也不等同于逐像素米制深度数据。

观测坐标为相机光学坐标：X 向右、Y 向下、Z 向前，单位米。深度位置表示有深度支持的目标表面；尺寸估距是近似值。真实目标定位误差、长期负载与新场地表现需要单独验收。接口见 `docs/观测接口与设备状态.md`，现有硬件复核见 `docs/NX性能与外场复核_2026-10-14.md`。其他历史记录反映各自日期的状态；关于当前入口、源码副本范围和模型缺失，以本 README 为准。

```powershell
python -m unittest discover -s tests -v
python -m unittest discover -s panel -p test_dashboard.py -v
python scripts/check_repository.py --history
```

前端回归测试使用 Node.js；电脑媒体回归使用 FFmpeg。名称带 `live` / `board` 的硬件脚本须在人工准备好设备后显式执行；普通 unittest discovery 不启动硬件验收。电脑回归通过不代表 Jetson 构建、相机或真实目标精度已在当前设备验收。

本次副本不增加或变更软件许可；保留原源码中的许可声明。
