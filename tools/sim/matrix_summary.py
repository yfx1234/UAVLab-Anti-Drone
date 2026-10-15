#!/usr/bin/env python3
"""矩阵结果聚合：按格/按族统计触发与命中。用法: matrix_summary.py <results.txt>"""
import sys
from collections import defaultdict

rows = []
for line in open(sys.argv[1], errors="ignore"):
    parts = [p.strip() for p in line.strip().split("|")]
    if len(parts) >= 9 and parts[0] == "RESULT":
        rows.append(parts)

cells = defaultdict(lambda: [0, 0, 0, []])
for p in rows:
    name, spd = p[2], p[3]
    fired = p[4] == "fire=1"
    hit = p[6] == "hit=1"
    miss_val = p[7].replace("miss=", "") if len(p) > 7 else ""
    miss = float(miss_val) if miss_val else None
    c = cells[(name, spd)]
    c[0] += 1
    c[1] += fired
    c[2] += hit
    if miss is not None:
        c[3].append(miss)

tf = th = 0
misses = []
print(f"{'cell':<22}{'fire':>6}{'hit':>5}  miss")
for (name, spd), c in sorted(cells.items(), key=lambda kv: (kv[0][0], float(kv[0][1]))):
    tf += c[1]
    th += c[2]
    misses += c[3]
    m = f"{sum(c[3])/len(c[3]):.2f}" if c[3] else "-"
    print(f"{name}@{spd:<10}{c[1]}/{c[0]:<4}{c[2]}/{c[1] if c[1] else 0:<4}{m}")

n = len(rows)
avg = sum(misses) / len(misses) if misses else 0
print(f"\nTOTAL rounds={n} fired={tf} ({100*tf/n:.1f}%) "
      f"hit={th}/{tf} ({100*th/tf if tf else 0:.1f}%) miss_avg={avg:.2f}m")
