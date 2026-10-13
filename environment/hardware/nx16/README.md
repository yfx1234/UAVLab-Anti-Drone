# 已换装NX16GB的启动修复

本板为P3767-0000、原DAMIAO ORIN NX V2载板、ZED Link Mono/CSI0，L4T36.4.3。已安装的NVIDIA NX与Nano Super原始DTB比对，模块差异为根compatible/model、DLA/PVA状态及三个CPU簇的1510.4MHz OPP；当前载板原树已有DLA/PVA，保留所有载板外设节点并从NX参考补齐SKU/OPP。

`scripts/prepare_nx_boot.py` 从已验证载板DTB生成 `/boot/dtb/uavlab-damiao-nx16-36.4.3.dtb`，继续合并原 `damiao-zed-mono.dtbo`，不替换载板为NVIDIA开发套件，不改写QSPI/内核/相机驱动。

2026-10-13受控重启后：活动compatible为P3767-0000-super，8个CPU在线，NX功耗配置文件正确，当前为25W/ID3；video0/video1及三个相机服务正常。新启动项为UAVLAB_NX16，原ZED_DAMIAO仍可在启动菜单选择。

备份：板上 `/var/backups/uavlab-nx16-20261013T115705Z/`，包含原extlinux、载板DTB、相机overlay、功耗状态和旧配置链接目标。恢复时先释放相机，将该目录的extlinux.conf恢复到 `/boot/extlinux/extlinux.conf`、nvpmodel-status恢复到 `/var/lib/nvpmodel/status`，并把 `/etc/nvpmodel.conf` 链接恢复到 nvpmodel-link.txt 指定文件，再受控重启。原DTB/overlay/initrd从未覆盖。

原树SHA256 `fa7df121ae7235bd8e1de429193744cc95acaacbe35a6e3cb2c712612c1db59a`；NX参考 `89546ca76ebbe404bfc37fa6a2d28bf073f0bb91f7d6378558e6099978572975`；Mono overlay `b7948a270fe6fd9444ee818efac549b00b9271325f9de8d87bd81b10558c005c`；候选 `0c065f10d7920dc8393d9e78cce82d9b94b0d57e51205fccffe6ff853108b191`。

旧GStreamer注册缓存仍只有解码器。使用新的独立缓存可注册H264/H265编码器，1080p30/90个合成帧实际NVENC测试通过；备份并更新用户的 `~/.cache/gstreamer-1.0/registry.aarch64.bin` 后默认缓存也可用。未升级混用的多媒体软件包，因为当前编码问题已在原环境修复，后续升级须另做匹配验收。

本记录仅适用于该现机，不能作为任意DAMIAO载板/其他L4T的通用刷机文件。TPC掩码查询仍显示实际15/配置240，但CUDA可见8个SM；不要凭这个数字擅自写GPU掩码。持续录像、实际深度/识别和功耗温度验收见最终修复报告。
