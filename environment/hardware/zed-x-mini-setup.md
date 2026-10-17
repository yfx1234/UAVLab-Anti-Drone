# ZED X Mini 配置与验证记录

日期：2026-10-07（Asia/Shanghai）

## 结果

已完成当前硬件组合的本机适配，重启后 ZED SDK 能识别并打开 1 台 ZED X Mini，左右图像、深度和 IMU 均已实测成功。目标机桌面和 SSH 均恢复正常。

当前实体载板依据用户提供的商品图、实物接口布局及达妙官方资料，识别为 **达妙 DAMIAO ORIN NX V2 系列**，具体硬件小版本未从板上读出。核心模组仍为 Orin Nano 8GB。此前 Seeed J401 包装和系统镜像反映原配置，不能证明更换后的实体载板是 J401；仅凭 22 针接口也不能判断驱动兼容性。

这次是针对当前机器完成并验证的适配，不能视为达妙或 Stereolabs 对该组合的官方支持承诺。

## 当前环境

| 项目 | 实测结果 |
|---|---|
| 目标机 | uavlab-desktop，172.20.10.3 |
| 核心模组 | Jetson Orin Nano 8GB，EEPROM 699-13767-0003-301 L.1 |
| 实体载板 | DAMIAO ORIN NX V2 系列，CSI0 为 22 针接口 |
| 相机连接 | ZED Link Mono 接 CSI0，用户确认独立 12 V 供电 |
| 相机 | ZED X Mini，序列号 52175314，固件 2001 |
| 系统 | Ubuntu 22.04.5，原 Seeed J401 镜像 |
| BSP / 内核 | L4T 36.4.3-20250107174145 / 5.15.148-tegra |
| CUDA / TensorRT | 12.6 / 10.3.0.30 |
| ZED SDK / 驱动来源 | SDK 5.5.0 / ZED Link Mono 1.4.4 |
| 视频节点 | /dev/video0、/dev/video1 |
| 最终验证启动 ID | 87c03e85-0022-4ea7-91fe-78d8bb524ae4 |

JetPack 元包显示 6.2.1+b38，但相机驱动选择依据实际 L4T 36.4.3，不能只依据元包标签。

## 实测证据

| 测试 | 结果 |
|---|---|
| SDK 枚举和打开 | 1 台，OPEN_STATUS=SUCCESS |
| RGB 模式 | 960×600，配置 30 FPS，深度关闭 |
| 左右图像 | 读取与 PNG 保存均成功 |
| RGB 连续采集 | 90 帧，0 错误，约 27.6 FPS；包含写图开销 |
| 深度模式 | PERFORMANCE，960×600，配置 30 FPS，最大距离参数 10 m |
| 深度连续采集 | 180 帧，0 错误，约 30.04 FPS |
| 深度数据 | 320×200 输出，32169 / 64000 个有限正值像素，约 50.3% |
| IMU | 可用，加速度与角速度读取成功；单次加速度约 (0.05, -9.81, 0.06) m/s² |
| 系统服务 | zed_x_daemon、IMU_Daemon、nvargus-daemon 均运行，退出状态正常 |
| 驱动加载服务 | driver_zed_loader 为一次性任务，退出码 0；执行后 inactive 属正常行为 |
| 包管理 | dpkg --audit 无异常输出 |
| 测试后进程 | 未保留 test_zed 或 gst-launch 持续采集进程 |
| 最近 10 分钟内核日志 | 未发现所检索的采集超时、符号版本冲突或 OOM 错误 |

左右图像已目视核验，内容为现场桌面与房间；深度伪彩图也已保存和核验。深度图有较多无效区域，不能将“有深度数据”理解为当前整个画面都有可靠距离。

SDK 提示当前场景纹理少或存在遮挡，跳过了自标定；测距精度、运动场景和长时间稳定性尚未验证。深度测试采用轻量 PERFORMANCE 模式，SDK 5.5 已标记该模式弃用并建议 NEURAL；这次未优化或实测 NEURAL，后续算法项目应另行选择并验证。

## 配置内容与原因

1. 安装 ZED SDK 5.5.0，跳过 Python API 和自动驱动安装；配置 video、zed、imu 权限。安装器还更新了相关 Ubuntu 依赖并触发 initramfs 重建。
2. 官方相机模块与原 Seeed tegra-camera.ko 的 8 个导出符号 CRC 不匹配。读取运行模块与文件证据后，取得 NVIDIA **相同 L4T 36.4.3** 的 OOT 模块包，审核相关模块的符号依赖，并替换相机链路需要的模块集合。没有强行忽略模块版本校验。
3. 手动部署 Mono 1.4.4 的驱动模块、用户态驱动程序、服务和对应版本的 ISP/采集补丁。此为手动文件部署，未登记为完整 stereolabs 驱动 DEB 包。
4. 首次设备树合并在文件上正确，但启动后仍被固件的 IMX219 配置覆盖。改为在 extlinux 中显式追加最终 Mono DTBO，并禁用冲突的 IMX219 节点。
5. 原镜像的 GPIO 复用导致 Mono 复位线未正确释放。为 CAM0 的 PH6 增加 GPIO hog，持续输出高电平。最终 Mono MAX9296 在 I²C 总线 10 读到芯片 ID 0x94，相机和视频节点正常注册。
6. 执行两次重启。最终启动配置、左右图像、深度、IMU 均在第二次重启后核验。

未刷整机镜像，保留原内核 Image。相机模块、设备树和启动配置已修改；不能将这次操作描述为“只安装 SDK”。未使用的 CAM1 或空 GMSL 端口可能仍有未探测到相机的启动日志，不代表当前 CSI0 相机故障。

## 持久配置与回退

- 当前默认启动项：`ZED_DAMIAO`。
- 启动文件：`/boot/extlinux/extlinux.conf`。
- 基础 FDT：`/boot/dtb/kernel_tegra234-j401-p3768-0000+p3767-0003-recomputer.dtb`。
- 最终覆盖：`/boot/damiao-zed-mono.dtbo`。
- DTBO SHA256：`b7948a270fe6fd9444ee818efac549b00b9271325f9de8d87bd81b10558c005c`。
- 原始文件、清单、回退脚本与适配源码：`/var/backups/zed-damiao-20261007`，仅 root 可访问。
- 原 initrd 另存：`/boot/initrd.pre-zed-damiao`。

需要撤回相机适配时，在目标机执行：

```bash
sudo python3 /var/backups/zed-damiao-20261007/rollback.py
sudo reboot
```

脚本停用相关 ZED 服务，按清单恢复相机适配文件和启动配置，并重新生成模块依赖。这不是整盘快照回滚：SDK、用户组及安装过程中更新的系统依赖会保留。

此适配依赖当前内核、BSP 和手动部署文件。以后升级 L4T、重装驱动或换用其他 CSI 相机前，应先保存上述备份并重新核验兼容性；现有其他厂商的部分相机模块未纳入本次验证。

## 复测与文件

目标机已保存测试源码和二进制。测试图像输出目录位于 /tmp，重启后可能清空，复测先创建目录：

```bash
mkdir -p /tmp/zed-setup-20261007
/home/uavlab/test_zed_damiao
/home/uavlab/test_zed_depth
```

两个测试依次执行，每个程序结束后释放相机。左右图像测试和深度测试不能同时占用相机。

本地交付文件（与本记录同目录）：

- `zed_left.png`、`zed_right.png`：左右实拍。
- `zed_depth.png`：深度伪彩图。
- `ZED_DAMIAO测试日志.txt`：RGB 编译与运行日志。
- `ZED_DAMIAO深度测试日志.txt`：深度和 IMU 编译与运行日志。

## 官方资料

- [达妙载板资料仓库](https://github.com/dmBots/orin-board)：用于核对实际载板与接口；仓库的通用 CSI 示例不能直接证明官方 ZED 支持。
- [达妙 CSI 相机说明](https://github.com/dmBots/orin-board/blob/master/dm_orin_board_public_information/Jetson_Orin_CSI_Camera_Support/README_zh-CN.md)。
- [Stereolabs 驱动下载](https://www.stereolabs.com/developers/drivers)。
- [ZED Link Mono 的 Orin Nano 接线说明](https://docs.stereolabs.com/docs/products/embedded/zed-link-capture-card/gmsl2/zed-link-mono/jetson-orin-nano-devkit-setup)。
- [Stereolabs 第三方设备适配说明](https://docs.stereolabs.com/docs/products/embedded/zed-link-capture-card/gmsl2/third-party-devices)。
- [NVIDIA 同版本 OOT 模块包](https://repo.download.nvidia.com/jetson/t234/pool/main/n/nvidia-l4t-kernel-oot-modules/nvidia-l4t-kernel-oot-modules_5.15.148-tegra-36.4.3-20250107174145_arm64.deb)。

## 后续：图传实时预览

2026-10-07 已新增浏览器四格实时预览，当前可显示左右图、深度、可靠性与 IMU。基础测试程序已结束，但图传预览现在按页面控制运行；停止按钮或关闭页面约两分钟后释放相机。详情见同目录 `ZED图传预览使用说明.md`，本机地址为 http://127.0.0.1:8766。

## 色彩配置补全

后续图像质量检查发现首次手动适配漏部署官方 AR0234 ISP 文件，导致灰紫色、黑位抬升。2026-10-07 已补上 `zedx_ar0234.isp`、`zedx_ar0234.nito`，备份并清理旧 ISP 缓存、重启相机服务，未重启整机。新文件和缓存备份已纳入同一回退清单。正常场景原始图的绿色与对比度已恢复；详见 `ZED图传预览使用说明.md`。

预览控制也已由 Wi-Fi SSH 改为图传网口 SSH；视频与控制均经 192.168.50.100 ↔ 192.168.50.101。
