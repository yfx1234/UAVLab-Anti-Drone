# Falcon Jetson clock guard

The carrier currently exposes `nvvrs-pseq-rtc` and `tegra_rtc`, but both return
to 1970 after complete power loss. With no Internet or RTC backup supply, no
software can reconstruct the time spent powered off exactly.

This package provides three layers:

1. `systemd-timesyncd` corrects the clock whenever an NTP server is reachable.
2. `falcon-clock-guard` restores the newest trustworthy persisted epoch instead
   of allowing the board to remain in 1970.
3. A timer saves the clock every five minutes, and the service saves it again
   during a clean shutdown.

Install once from the Jetson. Files must be copied with `scp -p`; if the board
is still in 1970, the installer uses the preserved workstation modification
time as a trustworthy initial lower bound. An explicit `SEED_EPOCH` overrides
that automatic seed:

```bash
cd /home/uavlab/Jetson_ROS2_V0
sudo -v
bash deploy/install_falcon_clock_guard.sh
# Optional exact seed:
# SEED_EPOCH=<current-unix-epoch> bash deploy/install_falcon_clock_guard.sh
```

Check status with:

```bash
/usr/local/sbin/falcon-clock-guard status
timedatectl status
systemctl status falcon-clock-guard.service falcon-clock-save.timer systemd-timesyncd.service
```

The persisted time is a safe lower bound, not an exact offline RTC. For exact
wall time after arbitrary power-off intervals, fit the carrier's supported RTC
backup battery or provide a reachable LAN NTP/GPS time source.
