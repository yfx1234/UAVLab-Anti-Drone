# Micro XRCE-DDS Agent boot service

The deployment starts the serial Micro XRCE-DDS Agent and a ROS 2 CLI discovery
daemon with one shared environment. It does not start an Offboard controller,
publish a setpoint, arm the vehicle, or change PX4 mode. It sets
`ROS_DOMAIN_ID=0`. Whether the PX4 ROS graph is limited to the onboard
computer is decided by the flight-controller parameter `UXRCE_DDS_PTCFG`
(0 = all interfaces, 1 = localhost only); the Agent itself does not read
`ROS_LOCALHOST_ONLY`. Verify connectivity from the interceptor stack logs
(`attmiss` not tracking `obs`, `fcu=1` in `netgun diag`) — not from a login
shell's `ros2 topic hz`, which may run with a different
`ROS_LOCALHOST_ONLY` value. QGC/MAVLink is unaffected.

Install on the Jetson:

```bash
cd /home/uavlab/Jetson_ROS2_V0
chmod +x deploy/*.sh
./deploy/install_px4_ros2_runtime.sh
```

Verify before flight:

```bash
systemctl is-enabled micro-xrce-agent.service
systemctl is-active micro-xrce-agent.service
systemctl is-enabled ros2-px4-daemon.service
systemctl is-active ros2-px4-daemon.service
journalctl -u micro-xrce-agent.service -b --no-pager -n 100
pgrep -af MicroXRCEAgent
ros2 topic hz /fmu/out/vehicle_status
ros2 topic hz /fmu/out/vehicle_local_position
```

There must be exactly one `MicroXRCEAgent` process. Do not manually start a
second Agent while this service is active. Interactive Bash shells source
`/etc/profile.d/px4-ros2.sh`, preventing a CLI daemon from being created with
discovery settings that differ from the boot services.

After updating the unit or the start/stop scripts, deploy from the repo and
restart (G-3 2026-10-18): the installer **copies** the unit to
`/etc/systemd/system/` and the environment file to `/etc/profile.d/px4-ros2.sh`,
so `daemon-reload + restart` alone leaves those copies stale. On the board the
systemd-facing copy root is `/home/uavlab/Jetson_ROS2_V0` (the unit's
`ExecStart` points there) while the source repo is `/home/uavlab/interceptor_ws`:

```bash
git -C /home/uavlab/interceptor_ws pull
cp /home/uavlab/interceptor_ws/deploy/*.sh \
   /home/uavlab/Jetson_ROS2_V0/deploy/
cp /home/uavlab/interceptor_ws/deploy/systemd/*.service \
   /home/uavlab/Jetson_ROS2_V0/deploy/systemd/
/home/uavlab/Jetson_ROS2_V0/deploy/install_px4_ros2_runtime.sh
# 安装器内部已 sudo install 两个拷贝 + daemon-reload + restart
```

Acceptance now has one extra step (G-3): a **newly opened** login shell must
carry the shared environment:

```bash
echo $ROS_LOCALHOST_ONLY    # 应输出 0（旧拷贝是 1）
```

Connectivity acceptance for the CLI daemon (F-5): a login shell must see both
the PX4 and interceptor topics, and the blocked command topic must have no
publishers:

```bash
ros2 topic list    # /fmu/out/* 与 /tracking/relative/estimate 同时可见
ros2 topic info /fmu/in/vehicle_command
# Publisher count: 0, Subscription count: >= 1（rc_gated 红线，agent 侧订阅）
```

Stop or disable it:

```bash
sudo systemctl disable --now micro-xrce-agent.service
sudo systemctl disable --now ros2-px4-daemon.service
```
