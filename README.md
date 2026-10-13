# UAVLab Anti-Drone

私有仓库：<https://github.com/yfx1234/UAVLab-Anti-Drone>，主分支 `main`。

ZED X Mini 四旋翼感知与数据采集平台。当前运行在 Jetson Orin Nano 8GB，准备迁移到 Orin NX 16GB；仓库重点为感知、相机、录像、电脑监看与素材管理。

## 当前链路

ZED 左目原图 → 通用天空候选检测 / YOLO TensorRT（独立或联合）→ 框内深度质量判断 → 相机坐标输出；无可靠深度时，按约 0.30 m 的目标尺寸估距。小目标、触边或无可信估距时只输出方向。坐标为 X 向右、Y 向下、Z 向前，单位米；深度位置对应有深度支持的目标表面，不能直接视为四旋翼几何中心。

通用检测使用多尺度亮/暗局部对比度、图像时刻 IMU 旋转补偿和连续帧确认，不需要训练模型，也不进行四旋翼类别确认。适用于天空背景的目标候选；云边、鸟、污点和室内物品仍可能成为候选。具体流程、限制及实物测试见 `docs/generic-sky-field-test.md`，简单操作见 `docs/通用检测_简单使用版.md`。

默认相机 **1920×1080 / 15 fps**；当前模型实际输入 **NCHW [1,3,736,1280]**，即宽 1280、高 736。采集、YOLO 输入、图传预览是不同尺寸：图传预览通常为 640×360、最多 8 fps，不作为训练原片。

板上单一进程拥有相机，默认 RGB；启用任一种检测同时启用深度计算。LOSSLESS 双目 SVO 与检测共用采集帧，切换通用检测和 YOLO 不打断同场录像。通用模式单独运行时不加载 TensorRT。深度展示视频按需离线生成，采用 ZED SDK 原生 VIEW::DEPTH；首次转换需要释放相机。

## 文件入口

| 路径 | 内容 |
| --- | --- |
| `board/zed_tracking` | C++ 采集、推理、定位、录像、离线导出与 SSH RPC |
| `board/zed_perception_interfaces` | ROS2 消息和服务接口 |
| `panel` | 电脑本地服务与中文网页，默认端口 8767 |
| `models/platform_exp_20260909` | 现用模型 PT、ONNX 与 Nano 引擎的来源/张量清单 |
| `environment` | 板上实测环境、当前硬件配置参考、相机适配记录 |
| `scripts` | 可携带的电脑启动、源码部署、构建和同步检查工具 |
| `tests` | 电脑侧回归测试；名称带 live/board 的脚本需人工在硬件上执行 |
| `docs/nx-migration.md` | NX 迁移步骤与待验收项 |
| `docs/development-workflow.md` | 后续改动、验证、部署与 GitHub 同步约定 |
| `docs/operator-guide.md` | 日常使用说明 |

历史滤波/飞行工具仍保留在源码和旧提交中，默认构建关闭 `UAVLAB_BUILD_LEGACY_TRACKING`；日常面板不调用这些工具。早期跟踪方案见 `docs/archive/zed-tracking-before-perception-only.md`。

## 电脑启动

```powershell
python -m pip install -r requirements-panel.txt
powershell -ExecutionPolicy Bypass -File scripts/start_panel.ps1
```

服务已运行时只需打开 <http://127.0.0.1:8767/>。数据默认保存到仓库下 `data/perception_data`，可用启动参数 `-DataDirectory` 指向现有素材目录。原电脑的旧启动脚本和素材目录继续可用；新启动脚本不会迁移已有数据。

当前图传配置固定为电脑 `192.168.50.101/24`、板子 `192.168.50.100`。板子 SSH 用户为 `uavlab`，项目根为 `/home/uavlab/Jetson_ROS2_V0`。换板后必须核验并更新 `panel/board_client.py` 的 SSH 主机指纹；不关闭主机身份检查。密码/私钥通过本机 SSH 环境或连接界面使用，不上传仓库。

## 板上构建

安装与当前 BSP 匹配的相机驱动、ZED SDK、CUDA、TensorRT、ROS2 Humble 和 OpenCV 后执行：

```bash
python3 scripts/stage_board.py --project-root /home/uavlab/Jetson_ROS2_V0 --dry-run
python3 scripts/stage_board.py --project-root /home/uavlab/Jetson_ROS2_V0
bash scripts/build_board.sh /home/uavlab/Jetson_ROS2_V0
```

这些命令只部署源码与构建，不停止或启动相机，不刷机，不修改驱动。supervisor 依赖实际工作区目录，因此源码使用实文件复制，不能用指向仓库的软链接替代。

NX 必须从 ONNX 在目标模块重建 TensorRT 引擎。当前 Nano `.engine` 不作为跨模块通用文件上传；原文件仍在板上。模型哈希见 `models/model-index.json`。SDK、CUDA、ROS 和驱动二进制从官方渠道安装，仓库中的环境清单不是整机镜像。

## 验证范围

- 当前 Nano 的 1080p/15 fps、同场 RGB→YOLO→RGB 录像及 SVO 回放已实测；实物四旋翼识别/坐标精度尚待用户安排。
- 新增通用检测/联合YOLO的同场SVO短测503帧完整，原片实际约13.74Hz；通用检测约13.7Hz、联合约7.9Hz。当前Nano不宣称满15Hz或30Hz，真实天空与移动相机测试待做。摘要见 `environment/generic-detection-validation.json`。
- 1080p/30 fps 的无损录像短测实际约 18.2 个独立图像/秒，尚未达标。SVGA/30 与 HD1200/15 只通过短时纯预览测试；60/120 fps 未验收。
- 2026-10-13 已换装 NX16GB；现有采集/检测/录像/导出/图传短测可运行，但仍加载Nano启动/功耗配置与旧引擎。1080p30纯RGB约30Hz；30fps无损录像约23.3Hz，叠加YOLO约20.2Hz、联合检测约18.5Hz，尚未达标。详见 [NX完整检查](docs/NX完整检查_2026-10-13.md) 与 `environment/current-nx16.json`。
- 默认构建保留 6 组当前感知原生测试；历史滤波/控制的 2 组测试需开启旧工具选项及另外安装原 `falcon_intercept`/飞控相关依赖。

提交前运行 `python -m unittest discover -s tests -v` 和 `python scripts/check_repository.py --history`。每次完成项目改动后提交并推送 GitHub；同步未成功时明确报告。

2026-10-12 迁移工具验收：电脑74项回归（73通过、1跳过）；当前Nano独立目录中成功复制源码与模型、从头构建两个感知包，5组原生测试全部通过。该验证没有覆盖当前运行目录，也不代表NX已验收。摘要见 `environment/migration-tool-validation.json`。
