#!/usr/bin/env python3
"""5-10 m/s 交战可行性推定（几何/门限解析，2026-10-15）。

锚点参数（均为项目定案/实测值）：
  垂直间距 h=7.4m（火控实测 range 反推）；LOS 门 0.5 rad/s；
  距离窗 [4.0, 9.8]；倾角上限 36°（MPC，AIM 自钳 20°）；
  平台 速度 10 m/s / 加速度 6.5 m/s²；网 55 m/s、展开 0.055s、半幅 1.5m。
锚点实测（40 轮矩阵 @5 m/s）：pursuit 2/2、circle 1/2、s_curve 1/2、
headon 0/2。
"""
import math

H = 7.4
LOS_GATE = 0.5
R_MIN, R_MAX = 4.0, 9.8
TILT_GEO = math.radians(36.0)
TILT_AIM = math.radians(20.0)
V_PLAT, A_PLAT = 10.0, 6.5
NET_V, DEPLOY, HALF = 55.0, 0.055, 1.5


def los_rate(v_rel, d):
    """横向相对速度 v_rel、水平偏移 d 下的瞄准线角速度。"""
    return v_rel * H / (d * d + H * H)


def theta(d):
    return math.atan2(d, H)


def r_slant(d):
    return math.hypot(d, H)


print("=" * 72)
print("1) headon 等待姿态（悬停/预置点不动）：三约束联立的可行偏移区间")
print("   约束：LOS率<0.5 ∧ 距离∈[4,9.8] ∧ 倾角≤上限")
print("=" * 72)
d_rmax = math.sqrt(R_MAX ** 2 - H ** 2)
for v in range(5, 11):
    d_los = math.sqrt(max(0.0, H * (v / LOS_GATE - H)))
    for tilt, tag in ((TILT_GEO, "几何36°"), (TILT_AIM, "AIM20°")):
        d_tilt = H * math.tan(tilt)
        lo, hi = d_los, min(d_rmax, d_tilt)
        ok = lo < hi
        print(f"  v={v:2d} [{tag}] d_los≥{d_los:4.1f} d_tilt≤{d_tilt:4.1f} "
              f"d_window≤{d_rmax:4.1f} → 区间[{lo:4.1f},{hi:4.1f}] "
              f"{'可行' if ok else '空=无解'}")

print()
print("=" * 72)
print("2) headon 速度匹配需求：fire 点处相对速度上限 v_rel_max（0.5 门）")
print("   所需同向速度 u = v − v_rel_max（平台上限 10）")
print("=" * 72)
for v in range(5, 11):
    cells = []
    for d in (2.5, 4.0, 5.0):
        v_rel_max = LOS_GATE * (d * d + H * H) / H
        u = max(0.0, v - v_rel_max)
        cells.append(f"d={d:3.1f}(θ={math.degrees(theta(d)):4.1f}°):u≥{u:4.1f}")
    print(f"  v={v:2d}  " + "  ".join(cells))

print()
print("=" * 72)
print("3) circle：生成器 R=2v（ω≡0.5）与固定半径的 ω / 向心加速度")
print("=" * 72)
for v in range(5, 11):
    parts = []
    for R in (2.0 * v, 10.0, 15.0, 20.0):
        w = v / R
        a = v * v / R
        parts.append(f"R={R:4.0f}:ω={w:.2f}{'超门' if w > LOS_GATE else '    '}"
                     f" a={a:4.1f}{'超6.5' if a > A_PLAT else ''}")
    print(f"  v={v:2d}  " + "  ".join(parts))

print()
print("=" * 72)
print("4) pursuit：ramp 瞬态间隙（响应滞后 0.4s 估）vs 宽轴视场 ±5.6m")
print("=" * 72)
for v in range(5, 11):
    gap = 0.4 * v
    margin = V_PLAT - v
    print(f"  v={v:2d}  速度裕度={margin:4.1f}  瞬态间隙≈{gap:4.1f}m "
          f"{'出视场风险' if gap > 5.6 else '视场内'}"
          f"{'  ★零裕度(不可跟)' if margin <= 0.5 else ''}")

print()
print("=" * 72)
print("5) 网命中预算：tof 内目标行程（速度补偿吃掉）与机动附加位移")
print("=" * 72)
for v in range(5, 11):
    r_fire = 8.0
    tof = r_fire / NET_V + DEPLOY
    travel = v * tof
    acc_add = A_PLAT * tof * tof / 2
    # tof 内 LOS 转角（若恰在门限 0.5 开火）
    los_drift = LOS_GATE * tof
    lateral = los_drift * r_fire
    print(f"  v={v:2d}  tof={tof:.3f}s 行程={travel:4.2f}m(已补偿) "
          f"机动附加≤{acc_add:.2f}m 门限横移={lateral:.2f}m  vs 半幅{HALF}")
