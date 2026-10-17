#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""拦截轨迹可视化：追踪机/目标 3D+侧视，相机轴（机体-Z）指向箭头，开火点。"""
import sys, math, re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa
import rosbag2_py
from rclpy.serialization import deserialize_message
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from nav_msgs.msg import Odometry

def cjk():
    import subprocess
    try:
        out = subprocess.run(["fc-list", ":lang=zh", "family"], capture_output=True, text=True).stdout
        fams = {l.strip() for l in out.splitlines() if l.strip()}
        for f in ["Noto Sans CJK SC", "WenQuanYi Micro Hei", "WenQuanYi Zen Hei"]:
            if f in fams:
                plt.rcParams["font.family"] = f
                return True
    except Exception:
        pass
    return False
ZH = cjk()
def T(zh, en): return zh if ZH else en

def read_bag(uri):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=uri, storage_id="sqlite3"),
           rosbag2_py.ConverterOptions("", ""))
    loc, att, tru = [], [], []
    while r.has_next():
        t, d, ts = r.read_next()
        try:
            if t.endswith("vehicle_local_position"):
                m = deserialize_message(d, VehicleLocalPosition)
                if m.xy_valid and m.z_valid:
                    loc.append((ts/1e9, m.x, m.y, m.z))
            elif t.endswith("vehicle_attitude"):
                m = deserialize_message(d, VehicleAttitude)
                att.append((ts/1e9, list(m.q)))
            elif t.endswith("relative_truth"):
                m = deserialize_message(d, Odometry)
                p = m.pose.pose.position
                tru.append((ts/1e9, p.x, p.y, p.z))
        except Exception:
            pass
    return loc, att, tru

def body_minus_z(q):
    w, x, y, z = q
    # 旋转矩阵作用于 (0,0,-1)：第三列取负
    return np.array([
        -(2*(x*z + y*w)),
        -(2*(y*z - x*w)),
        -(1 - 2*(x*x + y*y))])

def fire_epoch(logpath):
    try:
        for line in open(logpath, encoding="utf-8", errors="ignore"):
            m = re.search(r"\[(\d{10})\.(\d+)\].*NETGUN FIRE.*range=([0-9.]+)m", line)
            if m: return float(m.group(1)) + float("0." + m.group(2)), float(m.group(3))
    except Exception: pass
    return None, None

def plot(tag, title, outpng):
    loc, att, tru = read_bag(f"/home/key/netgun_logs/vis/{tag}")
    if not loc: print(f"{tag}: no data"); return False
    L = np.array([(t,x,y,z) for t,x,y,z in loc])
    A = att; Tr = np.array([(t,x,y,z) for t,x,y,z in tru]) if tru else np.zeros((0,4))
    t0 = L[0,0]
    tt = L[:,0]-t0; X, Y, Z = L[:,1], L[:,2], L[:,3]
    # 截到开火后 4s（开火后制导脱离，避免长尾掩盖拦截过程）
    fe0, _ = fire_epoch(f"/home/key/netgun_logs/vis/{tag}.log")
    if fe0:
        cut = np.searchsorted(L[:,0], fe0 + 4.0)
        if cut < len(L): L = L[:cut]; tt = L[:,0]-t0; X, Y, Z = L[:,1], L[:,2], L[:,3]
    # 目标绝对位置 = 追踪机 + 相对真值（按接收时间对齐）
    TX = TY = TZ = None
    if len(Tr):
        idx = np.searchsorted(Tr[:,0], L[:,0])
        idx = np.clip(idx, 0, len(Tr)-1)
        TX = X + Tr[idx,1]; TY = Y + Tr[idx,2]; TZ = Z + Tr[idx,3]
    fe, frange = fire_epoch(f"/home/key/netgun_logs/vis/{tag}.log")
    fig = plt.figure(figsize=(16,7))
    # ---- 3D ----
    ax = fig.add_subplot(121, projection="3d")
    ax.plot(X, Y, -Z, color="#1f77b4", lw=2, label=T("追踪机","Pursuer"))
    if TX is not None:
        ax.plot(TX, TY, -TZ, color="#d62728", lw=2, ls="--", label=T("目标","Target"))
    # 相机轴箭头（每 ~2.5s）
    arrow_len = 2.5
    last = -10
    for t, x, y, z in L:
        if t-t0-last >= 2.5 and A:
            i = min(range(len(A)), key=lambda k: abs(A[k][0]-t))
            v = body_minus_z(A[i][1])
            ax.quiver(x, y, -z, v[0]*arrow_len, v[1]*arrow_len, v[2]*arrow_len,
                      color="#2ca02c", lw=1.2, arrow_length_ratio=0.18)
            last = t-t0
    if fe:
        i = int(np.argmin(np.abs(L[:,0]-fe)))
        ax.scatter([X[i]],[Y[i]],[-Z[i]], marker="*", s=380, color="gold",
                   edgecolors="k", zorder=5, label=T("开火点","FIRE"))
        if TX is not None:
            ax.scatter([TX[i]],[TY[i]],[-TZ[i]], marker="X", s=200, color="magenta", zorder=5)
            ax.plot([X[i],TX[i]],[Y[i],TY[i]],[-Z[i],-TZ[i]], color="gold", ls=":", lw=2)
    ax.set_xlabel(T("北 x (m)","North x (m)")); ax.set_ylabel(T("东 y (m)","East y (m)"))
    ax.set_zlabel(T("高度 (m)","Altitude (m)"))
    ax.set_title(title + (T(f"  开火距离 {frange:.1f}m", f"  fire@{frange:.1f}m") if fe else T("  未触发","  no fire")))
    ax.legend(loc="upper left")
    # ---- 侧视 (北-高) ----
    ax2 = fig.add_subplot(122)
    ax2.plot(X, -Z, color="#1f77b4", lw=2, label=T("追踪机","Pursuer"))
    if TX is not None:
        ax2.plot(TX, -TZ, color="#d62728", lw=2, ls="--", label=T("目标","Target"))
    last = -10
    for t, x, y, z in L:
        if t-t0-last >= 2.5 and A:
            i = min(range(len(A)), key=lambda k: abs(A[k][0]-t))
            v = body_minus_z(A[i][1])
            ax2.annotate("", xy=(x+v[0]*2.0, -z+v[2]*2.0), xytext=(x, -z),
                         arrowprops=dict(arrowstyle="->", color="#2ca02c", lw=1.2))
            last = t-t0
    if fe:
        i = int(np.argmin(np.abs(L[:,0]-fe)))
        ax2.scatter([X[i]],[-Z[i]], marker="*", s=380, color="gold", edgecolors="k", zorder=5, label=T("开火点","FIRE"))
        if TX is not None:
            ax2.scatter([TX[i]],[-TZ[i]], marker="X", s=200, color="magenta", zorder=5)
    ax2.set_xlabel(T("北 x (m)","North x (m)")); ax2.set_ylabel(T("高度 (m)","Altitude (m)"))
    ax2.set_title(T("侧视图（北-高）","Side view (North-Altitude)"))
    ax2.grid(alpha=0.3); ax2.legend()
    ax2.set_aspect("equal", adjustable="datalim")
    fig.tight_layout()
    fig.savefig(outpng, dpi=130)
    plt.close(fig)
    print(f"saved {outpng} fired={fe is not None}")
    return True

import os
OUT = "/mnt/g/Users/23103/Desktop"
ok1 = plot("pursuit3", T("追击-直线 3m/s","Pursuit straight 3m/s"), f"{OUT}/网枪拦截轨迹_追击直线3m每秒.png")
ok2 = plot("circle3", T("追击-圆周 3m/s r6m","Pursuit circle 3m/s r6m"), f"{OUT}/网枪拦截轨迹_追击圆周3m每秒.png")
ok3 = plot("headon3", T("迎敌-直线 3m/s","Head-on straight 3m/s"), f"{OUT}/网枪拦截轨迹_迎敌3m每秒.png")
print("PLOTS_DONE", ok1, ok2, ok3)
