#!/usr/bin/env python3
"""AIM 介入统计：每轮 AIM 进入次数、进入→开火时延、命中/miss。

用法: aim_stats.py <matrix_results_dir>
判据：AIM 进入时间戳记 t_enter；若 FIRE 在 t_enter 后 1.0s 内（timeout
0.8s + 余量）且中间无 aim timeout/est lost 退出，计为 AIM 开火。
"""
import re
import sys
from pathlib import Path

pat_enter = re.compile(r"\[(\d+\.\d+)\].*→ AIM")
pat_exit = re.compile(r"\[(\d+\.\d+)\].*aim (timeout|est lost)")
pat_fire = re.compile(r"\[(\d+\.\d+)\].*NETGUN FIRE.*reason=([A-Z_]+)")
pat_result = re.compile(r"REFEREE RESULT shot=\d+ hit=(\d) miss=([0-9.]+)")

rounds = []
for f in sorted(Path(sys.argv[1]).glob("r*.log")):
    enters, exits, fire = [], [], None
    for line in open(f, errors="ignore"):
        m = pat_enter.search(line)
        if m:
            enters.append(float(m.group(1)))
            continue
        m = pat_exit.search(line)
        if m:
            exits.append(float(m.group(1)))
            continue
        if fire is None:
            m = pat_fire.search(line)
            if m:
                fire = (float(m.group(1)), m.group(2))
    hit = miss = None
    for line in open(f, errors="ignore"):
        m = pat_result.search(line)
        if m:
            hit, miss = int(m.group(1)), float(m.group(2))
    # AIM 开火判定：fire 落在某次进入后 1.0s 内且无退出间隔
    aim_fire = None
    if fire:
        for t0 in enters:
            if t0 <= fire[0] <= t0 + 1.0:
                if not any(t0 < tx <= fire[0] for tx in exits):
                    aim_fire = fire[0] - t0
    rounds.append((f.name, len(enters), fire, aim_fire, hit, miss))

n_aim = sum(1 for r in rounds if r[1] > 0)
n_fire = sum(1 for r in rounds if r[2])
n_aimfire = sum(1 for r in rounds if r[3] is not None)
aimfires = [r for r in rounds if r[3] is not None]
print(f"{'round':<34}{'ent':>4}{'fire':>6}{'aim→fire_s':>11}{'hit':>5}{'miss':>7}")
for name, ne, fire, af, hit, miss in rounds:
    fs = f"{fire[1][:7]}" if fire else "-"
    afs = f"{af:.3f}" if af is not None else ("geo" if fire else "-")
    print(f"{name:<34}{ne:>4}{fs:>6}{afs:>11}"
          f"{('-' if hit is None else hit):>5}{('-' if miss is None else f'{miss:.2f}'):>7}")

lat = [r[3] for r in aimfires]
hits = [r[4] for r in aimfires if r[4] is not None]
misses = [r[5] for r in aimfires if r[5] is not None]
print(f"\n总轮数={len(rounds)}  有AIM进入={n_aim}  开火={n_fire}  其中AIM开火={n_aimfire}")
if lat:
    print(f"AIM进入→开火时延: min={min(lat):.3f}s max={max(lat):.3f}s "
          f"均值={sum(lat)/len(lat):.3f}s (n={len(lat)})")
if hits:
    print(f"AIM开火命中 {sum(hits)}/{len(hits)}  miss 均值={sum(misses)/len(misses):.2f}m "
          f"max={max(misses):.2f}m")
geo = [r for r in rounds if r[2] and r[3] is None]
gh = [r[4] for r in geo if r[4] is not None]
gm = [r[5] for r in geo if r[5] is not None]
if gh:
    print(f"几何开火（无AIM）命中 {sum(gh)}/{len(gh)}  miss 均值={sum(gm)/len(gm):.2f}m")
