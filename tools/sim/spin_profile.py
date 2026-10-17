#!/usr/bin/env python3
"""SPIN 漂移时间剖面：每 0.2s 输出相对开火点的水平位移。"""
import re
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition


def log_epoch(path, pat):
    for line in open(path, errors="ignore"):
        m = re.search(pat, line)
        if m:
            return float(m.group(1))
    return None


bag_dir, log_path = Path(sys.argv[1]), sys.argv[2]
t_fire = log_epoch(log_path, r"\[(\d+\.\d+)\].*NETGUN FIRE")
t_spin = log_epoch(log_path, r"\[(\d+\.\d+)\].*spin done")
db = sqlite3.connect(str(next(bag_dir.glob("*.db3"))))
tid = [i for i, n in db.execute("SELECT id,name FROM topics") if n.endswith("vehicle_local_position")][0]
rows = db.execute("SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)).fetchall()

x0 = y0 = None
print(f"fire={t_fire:.1f} spin_done={t_spin:.1f}")
last_t = 0
for ts, blob in rows:
    t = ts / 1e9
    if t < t_fire - 0.5 or t > t_spin + 0.5:
        continue
    m = deserialize_message(bytes(blob), VehicleLocalPosition)
    if not (m.xy_valid and m.z_valid):
        continue
    if x0 is None:
        x0, y0 = m.x, m.y
    if t - last_t >= 0.2:
        last_t = t
        print(f"t+{t - t_fire:5.2f}s  dx={m.x - x0:+7.3f} dy={m.y - y0:+7.3f} "
              f"r={((m.x - x0)**2 + (m.y - y0)**2) ** 0.5:6.3f}  vx={m.vx:+.2f} vy={m.vy:+.2f}")
