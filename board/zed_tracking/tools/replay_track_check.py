#!/usr/bin/env python3
"""replay_track_check:离线检查回放 bag 中滤波输出的质量。

用法:
  python3 replay_track_check.py /path/to/bag_dir

检查项:
  1. track_json 频率与有效率;
  2. 相邻输出位置跳变分布(95 分位应远小于 1 m);
  3. 原因(reason)分布,定位失效环节;
  4. 观测模式分布(深度/伪测距)。
"""

from __future__ import annotations

import glob
import json
import sqlite3
import statistics
import sys


def load_topic(db_path, topic_name):
    db = sqlite3.connect(db_path)
    # topics 主键列是 id;用 topic_id 会被 SQLite 解析为外层列导致过滤失效。
    rows = db.execute(
        "select data from messages where topic_id="
        "(select id from topics where name=?) order by timestamp",
        (topic_name,),
    ).fetchall()
    decoder = json.JSONDecoder()
    payloads = []
    for (blob,) in rows:
        text = blob.decode("utf-8", "ignore")
        start = text.find("{")
        if start < 0:
            continue
        try:
            payloads.append(decoder.raw_decode(text, start)[0])
        except json.JSONDecodeError:
            continue
    return payloads


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    bag = sys.argv[1]
    files = sorted(glob.glob(f"{bag}/**/*.db3", recursive=True))
    if not files:
        print(f"no .db3 under {bag}")
        return 2
    tracks = load_topic(files[0], "/zed_tracking/track_json")
    targets = load_topic(files[0], "/zed_perception_node/target_json")
    if not tracks:
        print("no track_json messages; filter output missing")
        return 1
    stamps = [float(item.get("stamp_s", 0.0)) for item in tracks]
    duration = max(stamps) - min(stamps) if len(stamps) > 1 else 0.0
    print(f"track 条数 {len(tracks)}  时长 {duration:.1f}s  "
          f"输出频率 {len(tracks)/max(duration,1e-6):.1f} Hz")
    valid_flags = [bool(item.get("valid")) for item in tracks]
    print(f"valid 率 {sum(valid_flags)/len(valid_flags)*100:.0f}%")
    reasons = {}
    for item in tracks:
        reasons[item.get("reason")] = reasons.get(item.get("reason"), 0) + 1
    print(f"reason 分布 {reasons}")
    modes = {}
    for item in targets:
        modes[item.get("mode")] = modes.get(item.get("mode"), 0) + 1
    print(f"观测模式分布 {modes}")
    positions = [
        item.get("pos_ned") for item in tracks
        if item.get("pos_ned") and len(item["pos_ned"]) == 3
    ]
    if len(positions) > 2:
        jumps = []
        for first, second in zip(positions, positions[1:]):
            delta = statistics.dist(first, second) if hasattr(statistics, "dist") else (
                (first[0]-second[0])**2 + (first[1]-second[1])**2 +
                (first[2]-second[2])**2) ** 0.5
            jumps.append(delta)
        jumps.sort()
        p50 = jumps[len(jumps)//2]
        p95 = jumps[int(len(jumps)*0.95)]
        print(f"相邻位置跳变: 中位 {p50*100:.1f} cm / 95分位 {p95*100:.1f} cm")
        if p95 > 1.0:
            print("FAIL: 95分位跳变超过 1 m")
            return 1
    ages = [item.get("age_s") for item in tracks if item.get("age_s") is not None]
    if ages:
        print(f"观测年龄: 中位 {statistics.median(ages)*1000:.0f} ms / "
              f"最大 {max(ages)*1000:.0f} ms")
    print("REPLAY_CHECK_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
