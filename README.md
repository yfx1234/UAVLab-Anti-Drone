# UAVLab Anti-Drone

私有仓库：<https://github.com/yfx1234/UAVLab-Anti-Drone>，主分支 `main`。

ZED X Mini 四旋翼感知与数据采集平台。当前运行在 Jetson Orin NX 16GB（P3767-0000），原 DAMIAO ORIN NX V2 载板与 NVMe 保留；仓库重点为感知、相机、录像、电脑监看与素材管理。已适配 NX 启动、正确功耗配置（现用 MAXN_SUPER）、原生 TensorRT 引擎与编码器，最新实测边界见 [结项复核](docs/NX结项复核_2026-10-14.md)。

## 当前链路

ZED 左目原图 → YOLO TensorRT → 框内深度质量判断 → 相机坐标输出；无可靠深度时，按约 0.30 m 的目标尺寸估距。小目标、触边或无可信估距时只输出方向。坐标为 X 向右、Y 向下、Z 向前，单位米；深度位置对应有深度支持的目标表面，不能直接视为四旋翼几何中心。

2026-10-14 按用户要求停用通用检测，操作页入口已移除，RPC 拒绝再次开启；历史实现和素材记录保留。YOLO 输入可选原来的 1280×736 或 1080p 对应的 1920×1088（上下补边），原输入仍为默认。两次外场同帧对比及限制见本轮复核记录。

默认相机 **1920×1080 / 15 fps**；当前模型实际输入 **NCHW [1,3,736,1280]**，即宽 1280、高 736。采集、YOLO 输入、图传预览是不同尺寸：图传预览通常为 640×360、最多 8 fps，不作为训练原片。

板上单一进程拥有相机，默认 RGB；开启 YOLO 同时启用深度计算，暂停 YOLO 可释放检测与深度负载。默认训练采集使用像素一致的 LOSSLESS 双目 SVO；已验证 NX 可选高速压缩录像（H264_LOSSLESS，但本机实测 RGB 颜色有少量变化）。录像中可开关 YOLO，模型输入在同场录像期间锁定。备注修改自动保存并显示在场次列表。RGB/深度下载自动合并整场，原分段保留；深度展示使用 SDK VIEW::DEPTH，黑色区域表示无效或超距深度，首次转换需要释放相机。

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
