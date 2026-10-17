#!/usr/bin/env python3
"""SPIN 质量 bag 复盘：开火点→spin done 窗口内的水平漂移与高度保持。

用法: analyze_spin_bag.py <bag_dir> <round_log>
对齐方式：节点日志时间戳（epoch 秒）↔ bag 落盘时间戳（epoch 纳秒）。
"""
import re
import sqlite3
import sys
from pathlib import Path

from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition


def log_epoch_seconds(path):
    """取 FIRE 与 spin done 两条日志的 epoch 秒。"""
    t_fire = t_spin = None
    pat_fire = re.compile(r"\[(\d+\.\d+)\].*NETGUN FIRE")
    pat_spin = re.compile(r"\[(\d+\.\d+)\].*spin done")
    for line in open(path, errors="ignore"):
        m = pat_fire.search(line)
        if m and t_fire is None:
            t_fire = float(m.group(1))
        m = pat_spin.search(line)
        if m and t_spin is None:
            t_spin = float(m.group(1))
    return t_fire, t_spin


def main():
    bag_dir = Path(sys.argv[1])
    log_path = sys.argv[2]
    t_fire, t_spin = log_epoch_seconds(log_path)
    if t_fire is None:
        print("SPIN CHECK: no FIRE in log — skip")
        return
    win_end = (t_spin if t_spin else t_fire + 5.0) + 0.5

    db_file = next(bag_dir.glob("*.db3"))
    db = sqlite3.connect(str(db_file))
    topic_id = None
    for tid, name in db.execute("SELECT id, name FROM topics"):
        if name.endswith("vehicle_local_position"):
            topic_id = tid
            break
    if topic_id is None:
        print("SPIN CHECK: local_position topic not found")
        return

    rows = db.execute(
        "SELECT timestamp, data FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,)).fetchall()
    if not rows:
        print("SPIN CHECK: 0 messages")
        return

    samples = []
    for ts_ns, blob in rows:
        t = ts_ns / 1e9
        if t_fire - 0.2 <= t <= win_end:
            m = deserialize_message(bytes(blob), VehicleLocalPosition)
            if m.xy_valid and m.z_valid:
                samples.append((t, m.x, m.y, m.z))
    if not samples:
        print("SPIN CHECK: no samples in window")
        return

    x0, y0, z0 = samples[0][1], samples[0][2], samples[0][3]
    max_h = max((abs(x - x0) + abs(y - y0)) / 2.0 for _, x, y, _ in samples)  # 包络半宽
    max_e = max((x - x0) ** 2 + (y - y0) ** 2 for _, x, y, _ in samples) ** 0.5
    alt_span = max(-z for *_, z in samples) - min(-z for *_, z in samples)
    print(
        f"SPIN CHECK: n={len(samples)} win=[{t_fire:.1f},{win_end:.1f}] "
        f"drift_euclid={max_e:.3f}m drift_box_half={max_h:.3f}m alt_span={alt_span:.3f}m "
        f"{'PASS' if max_e < 0.5 else 'FAIL(>0.5m)'}")


if __name__ == "__main__":
    main()
