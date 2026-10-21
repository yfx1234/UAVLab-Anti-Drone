# Jetson USB 摄像头网页预览

适用环境：USB 摄像头接在 Jetson（Ubuntu），Windows 通过浏览器查看。2026-10-21 已在 Orin NX 上接通 LRCP 0G05B10 的原生 2592×1944 画面，实际部署、SSH 转发和验证边界见 [500 万像素部署记录](../docs/USB五百万相机工作台部署_2026-10-21.md)。

本包参考 Gitee 项目 `buaalab/anti-drone-interceptor` 的 `panel/index.html` 监看样式，以及其“元数据 + JPEG”显示方式，提供独立的 USB 预览入口。它是精简的监看页面，包含实时画面、连接状态、实际画面尺寸、预览帧率、全屏和保存截图；不是原项目完整工作台的原位改造，不包含原来的素材库、ZED 深度或检测功能。

原项目版本：`master`，提交 `a568f052756301219db502b5a89e8965c7b33526`。

## 1. 将文件放到 Jetson

将整个 `usb-camera-preview` 文件夹复制到 Jetson，例如桌面或用户主目录。`usb_preview.py` 与 `usb_index.html` 必须在同一目录。

若使用 Windows PowerShell，已配置好 SSH 时可运行（先替换用户名、IP 和本地路径）：

```powershell
scp -r "C:\你的下载目录\usb-camera-preview" 用户名@Jetson的IP:~/
```

## 2. 在 Jetson 终端安装依赖、确认设备

以下命令均在 Jetson 上执行：

```bash
sudo apt update
sudo apt install python3-opencv v4l-utils
v4l2-ctl --list-devices
ls -l /dev/v4l/by-id/
hostname -I
```

从 `v4l2-ctl --list-devices` 找到 USB 摄像头对应节点，不能只凭编号认定 `/dev/video0`。一个摄像头可能有多个节点；选择支持 Video Capture 的节点：

```bash
v4l2-ctl -d /dev/video0 --all
v4l2-ctl -d /dev/video0 --list-formats-ext
```

如果 `by-id` 下存在稳定路径，建议后续用该路径代替 `/dev/video0`，避免重启后编号变化。`hostname -I` 输出中，选 Windows 能访问的局域网地址。

## 3. 启动网页

```bash
cd ~/usb-camera-preview
python3 usb_preview.py --device /dev/video0 --width 1280 --height 720 --fps 15
```

如文件夹放在其他位置，修改 `cd` 路径。将 `/dev/video0` 替换为实际 USB 摄像头节点。采集尺寸、帧率和格式必须从相机支持的组合中选择；程序的默认请求为 1280×720、15 fps、MJPG，驱动可能调整参数，页面显示实际收到的画面尺寸与预览帧率。

如果只支持 YUYV，可从较低分辨率开始，例如：

```bash
python3 usb_preview.py --device /dev/video0 --fourcc YUYV --width 640 --height 480 --fps 15
```

保持终端运行；`Ctrl+C` 结束服务并释放摄像头。多个浏览器共用同一个采集线程。拔插导致读取失败后会释放并重试原路径；若节点编号变化，需使用新路径重新启动，或改用稳定的 `by-id` 路径。

## 4. Windows 打开网页

在 Windows 的 Edge 或 Chrome 地址栏输入：

```text
http://Jetson的IP:8767
```

例如：Jetson 的局域网 IP 若是 `192.168.1.50`，则打开 `http://192.168.1.50:8767`。

Windows 不需要安装 Python，也不需要浏览器摄像头权限。Windows 的 `localhost` 指向 Windows 自己，应使用 Jetson 的 IP。此服务用于可信局域网，没有登录认证，不要把端口映射到公网。

## 常见问题

- **打不开网页**：先在 Jetson 自己打开 `http://127.0.0.1:8767`。若本机能打开，Windows PowerShell 执行 `Test-NetConnection Jetson的IP -Port 8767`，检查双方网络及已有防火墙规则。页面不要求固定 IP 网段，也不改动网络配置。
- **端口已占用**：加 `--port 8768`，浏览器改访问 `http://Jetson的IP:8768`。
- **页面提示无法打开相机**：确认节点；运行 `fuser /dev/video0` 检查是否已有采集程序。一次只让一个程序采集同一摄像头。
- **权限不足**：运行 `ls -l /dev/video0` 和 `id`。如果设备属组为 `video` 且当前用户不在该组，可执行 `sudo usermod -aG video "$USER"`，然后注销并重新登录。不要长期用 root 运行服务。
- **黑屏、卡顿或读取失败**：对照 `--list-formats-ext` 选择支持的格式；先试 640×480、15 fps，再提高分辨率；检查 USB 连接和供电。
- **缺少 cv2**：使用 Ubuntu 自带的 `python3` 配合 `sudo apt install python3-opencv`。Conda 或隔离虚拟环境可能看不到系统包。
- **为什么不能直接运行原来的 dashboard.py**：原程序专门连接 ZED 板端服务，且其 `panel/board_client.py` 与 `panel/frame_receiver.py` 固定了原设备的 IP、用户名、RPC 路径和 SSH 主机指纹。这个 USB 预览入口直接在 Jetson 提供网页，绕开这些专用依赖。

## 验证范围

此前本地通过模拟相机的 HTTP 预览、共享采集、断线后恢复、过期帧拒绝等测试。2026-10-21 实机已验证 USB 3.0 枚举、2592×1944 JPEG 解码和电脑经 SSH 转发显示；请求 15 fps 时，当前 OpenCV 解码与 JPEG 重编码链路的预览约 7.5 fps。该结果不等于相机的最高采集帧率，也不是长期稳定性验收。页面的预览帧率是 JPEG 发布速率，最近帧间隔是 Jetson 本地的帧新鲜度，不是端到端延迟。

参考：[原项目](https://gitee.com/buaalab/anti-drone-interceptor)、[OpenCV VideoCapture](https://docs.opencv.org/4.x/d8/dfe/classcv_1_1VideoCapture.html)。
