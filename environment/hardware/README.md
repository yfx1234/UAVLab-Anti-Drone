# 相机硬件适配参考

当前 Nano 使用 DAMIAO ORIN NX V2 系列载板的 CSI0，ZED Link Mono 独立12V供电。驱动与设备树适配经过当前机器重启/RGB/深度/IMU验证；换NX需重新核对BSP、SKU和接线。

`current-nano` 保存实际 extlinux、合并覆盖DTBO以及由实际DTBO反编译的DTS参考。扩展名 `.reference` 表示资料，不是自动部署配置。DTS含合并节点/符号，不能当作独立、跨SKU的通用覆盖源码。

现机关键处理：

- 对齐 L4T 36.4.3 的 NVIDIA 相机 OOT 模块与ZED模块导出符号；未强制忽略符号版本校验。
- Mono 1.4.4驱动手动部署，包含采集补丁、服务和 ISP；dpkg清单不能独立证明这些文件完整。
- extlinux明确选择最终Mono覆盖，禁用冲突IMX219节点。
- CAM0 PH6复位线用GPIO hog保持高电平，使MAX9296在I²C总线10可探测。

原适配说明在 `zed-x-mini-setup.md`，原板root备份在 `/var/backups/zed-damiao-20261007`。SDK、驱动模块和ISP二进制不作为仓库安装器；从Stereolabs/NVIDIA取得与新系统对应的版本。

- [DAMIAO载板资料](https://github.com/dmBots/orin-board)
- [Stereolabs驱动](https://www.stereolabs.com/developers/drivers)
- [第三方设备适配说明](https://docs.stereolabs.com/docs/products/embedded/zed-link-capture-card/gmsl2/third-party-devices)
