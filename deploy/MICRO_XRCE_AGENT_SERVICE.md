# Micro XRCE-DDS Agent boot service

The deployment starts the serial Micro XRCE-DDS Agent and a ROS 2 CLI discovery
daemon with one shared environment. It does not start an Offboard controller,
publish a setpoint, arm the vehicle, or change PX4 mode. It sets
`ROS_DOMAIN_ID=0` and `ROS_LOCALHOST_ONLY=1`, so the PX4 ROS graph is not
advertised to other computers on the LAN. QGC/MAVLink is unaffected.

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

Stop or disable it:

```bash
sudo systemctl disable --now micro-xrce-agent.service
sudo systemctl disable --now ros2-px4-daemon.service
```
