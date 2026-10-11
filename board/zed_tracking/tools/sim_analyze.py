#!/usr/bin/env python3
"""sim_analyze:闭环仿真结果分析。

用法: sim_analyze.py <bag_dir> <motion> <cx> <cy> <cz> <radius> <angular_rate>

从 bag 读取机体状态与控制状态,按目标轨迹公式重建真值距离,
统计收敛时间、稳态保持误差、指向误差与射击事件。
"""

from __future__ import annotations

import glob
import json
import math
import sqlite3
import sys


def target_position(motion, center, radius, rate, elapsed):
    if motion == "hover":
        return list(center)
    if motion == "circle":
        angle = rate * elapsed
        return [center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle), center[2]]
    if motion == "line":
        period = max(1.0, 2.0 * radius / max(0.1, abs(rate)))
        phase = (elapsed % period) / period
        return [center[0] + (2.0 * phase - 1.0) * radius, center[1], center[2]]
    raise ValueError(motion)


def load_topic(db, name):
    # topics 表主键列是 id(messages.topic_id 外键指向它);
    # 写成 select topic_id 会被 SQLite 解析为外层列,条件恒真。
    rows = db.execute(
        "select data from messages where topic_id="
        "(select id from topics where name=?) order by timestamp",
        (name,)).fetchall()
    decoder = json.JSONDecoder()
    result = []
    for (blob,) in rows:
        text = decode_cdr_string(blob)
        if text is None:
            continue
        try:
            result.append(decoder.raw_decode(text, 0)[0])
        except json.JSONDecodeError:
            continue
    return result


def decode_cdr_string(blob):
    """按 CDR 小端布局解析 String:4B encapsulation + 4B 长度 + 内容。"""
    if len(blob) < 9 or blob[1] != 1:
        text = blob.decode("utf-8", "ignore")
        start = text.find("{")
        return text[start:] if start >= 0 else None
    length = int.from_bytes(blob[4:8], "little")
    if length <= 0 or 8 + length > len(blob) + 4:
        text = blob.decode("utf-8", "ignore")
        start = text.find("{")
        return text[start:] if start >= 0 else None
    return blob[8:8 + length].decode("utf-8", "replace")


def main():
    if len(sys.argv) < 7:
        print(__doc__)
        return 2
    bag = sys.argv[1]
    motion = sys.argv[2]
    center = [float(v) for v in sys.argv[3:6]]
    radius = float(sys.argv[5 + 1])
    rate = float(sys.argv[6 + 0])
    files = sorted(glob.glob(f"{bag}/**/*.db3", recursive=True))
    if not files:
        print("no db3")
        return 2
    db = sqlite3.connect(files[0])
    vehicles = load_topic(db, "/sim/vehicle_state")
    statuses = load_topic(db, "/zed_tracking/track_hold_status")
    shots = load_topic(db, "/zed_tracking/shot_event")
    truths = load_topic(db, "/sim/target_truth")
    if len(vehicles) < 50:
        print(f"vehicle_state 样本不足: {len(vehicles)}")
        return 1
    t0 = vehicles[0]["stamp_s"]
    ranges = []
    for item in vehicles:
        elapsed = item["stamp_s"] - t0
        pos = item["pos_ned"]
        if truths:
            truth = min(truths, key=lambda s: abs(s["stamp_s"] - item["stamp_s"]))
            target = truth["pos_ned"]
        else:
            target = target_position(motion, center, radius, rate, elapsed)
        ranges.append((elapsed, math.dist(target, pos)))
    converged_at = None
    window = int(len(ranges) * 0.9)
    for index in range(len(ranges)):
        if all(6.5 <= r <= 8.5 for _, r in ranges[index:index + window]) and \
                len(ranges[index:index + window]) == window:
            converged_at = ranges[index][0]
            break
    tail = ranges[-150:] if len(ranges) >= 150 else ranges
    errors = [abs(r - 7.5) for _, r in tail]
    rms = math.sqrt(sum(e * e for e in errors) / max(1, len(errors)))
    print(f"样本 {len(ranges)} 条,时长 {ranges[-1][0]:.1f}s")
    if converged_at is not None:
        print(f"保持收敛时间: {converged_at:.1f}s(进入 6.5~8.5 m 并保持)")
    print(f"稳态(|d-7.5|) RMS: {rms:.2f} m  "
          f"min={min(r for _, r in tail):.2f}  max={max(r for _, r in tail):.2f}")
    aims = [abs(s["aim_error_deg"]) for s in statuses
            if s.get("aim_error_deg") is not None]
    if aims:
        tail_aim = aims[-150:]
        rms_aim = math.sqrt(sum(a * a for a in tail_aim) / len(tail_aim))
        print(f"指向误差 RMS(尾段): {rms_aim:.2f} deg")
    print(f"射击事件: {len(shots)} 次")
    for shot in shots[:6]:
        print(f"  shot #{shot.get('sequence')} range={shot.get('range_m'):.2f} m "
              f"aim={shot.get('aim_error_deg'):.1f} deg t={shot['stamp_s']-t0:.1f}s")
    if not shots:
        print("FAIL: 无射击事件")
        return 1
    if rms > 0.8:
        print("FAIL: 稳态误差超限")
        return 1
    print("SIM_CHECK_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
