"""协同拦截结果绘图。

控制、CSV 原始量保持 NED；所有供人观察的垂向坐标一律显示为
``h = -z_NED``，绝不直接展示向下为正的 NED-z。
"""

from pathlib import Path
import shutil

import matplotlib
matplotlib.use('Agg')

import matplotlib.pyplot as plt
import numpy as np

# Matplotlib 在本机将同一套 Noto CJK TTC 注册为 JP family；该字库完整覆盖中文。
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK JP', 'WenQuanYi Micro Hei',
                                   'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

COLORS = {'target': '#d62728', 'p1': '#1565c0',
          'p2': '#2e7d32', 'p3': '#7b1fa2'}
LABELS = {'target': '敌机', 'p1': 'P1（原单机闭环）',
          'p2': 'P2（PX4僚机）', 'p3': 'P3（PX4僚机）'}
ALTITUDE_LABEL = '高度 h = -z_NED / m'

FORMAL_FIGURES = (
    '01_三机三维轨迹_高度向上.png',
    '02_XY平面等边三角形包围.png',
    '03_高度曲线与敌机垂直间距.png',
    '04_P1敌机距离与开火事件.png',
    '05_P2P3敌机距离.png',
    '06_三架拦截机两两安全距离.png',
    '07_包围质心误差与方位角误差.png',
    '08_关键时刻三机包围快照.png',
)


def _save(fig, path):
    fig.tight_layout()
    fig.savefig(str(path), dpi=180, bbox_inches='tight')
    plt.close(fig)


def _shot_index(t, event):
    return min(int(np.searchsorted(t, float(event['time_s']))), len(t) - 1)


def _shot_label(sequence, event):
    actual = event.get('actual_range_m')
    estimated = event.get('estimated_range_m')
    result = event.get('result', 'PENDING')
    actual_text = '—' if actual is None else f'{actual:.2f} m'
    return f'第{sequence}次开火\n实际 {actual_text}\n估计 {estimated:.2f} m · {result}'


def plot_all(results_dir, series, summary, cfg, shot_events=None):
    results_dir = Path(results_dir)
    shot_events = list(shot_events or [])
    if cfg.get('scenario_name') == 'hybrid_2v1':
        _plot_hybrid_formal(results_dir, series, summary, shot_events)
        _write_compatibility_figures(results_dir, series)
    else:
        _plot_generic(results_dir, series, summary, shot_events)


def _plot_hybrid_formal(out, series, summary, shots):
    t = series['t']
    traj = series['traj']
    best = int(series.get('best_formation_index', len(t) - 1))

    # 01 三维总览：显示高度 h=-z_NED，z 轴视觉方向向上。
    fig = plt.figure(figsize=(12, 8.2))
    ax = fig.add_subplot(111, projection='3d')
    for key in ('target', 'p1', 'p2', 'p3'):
        p = traj[key]
        ax.plot(p[:, 0], p[:, 1], -p[:, 2], lw=1.8,
                color=COLORS[key], label=LABELS[key])
        ax.scatter(p[0, 0], p[0, 1], -p[0, 2], color=COLORS[key],
                   marker='o', s=35)
        ax.scatter(p[-1, 0], p[-1, 1], -p[-1, 2], color=COLORS[key],
                   marker='s', s=35)
    for seq, event in enumerate(shots[:2], 1):
        idx = _shot_index(t, event)
        p = traj['p1'][idx]
        ax.scatter(p[0], p[1], -p[2], marker='*', s=180,
                   color='#ff9800' if seq == 1 else '#e65100',
                   edgecolor='black', linewidth=0.7,
                   label=f'P1第{seq}次开火（{event.get("result", "PENDING")}）')
        q = traj['target'][idx]
        ax.plot([p[0], q[0]], [p[1], q[1]], [-p[2], -q[2]],
                color='#ff9800', ls=':', lw=1.2)
    snap = [traj[k][best] for k in ('p1', 'p2', 'p3')]
    ax.plot([p[0] for p in snap + [snap[0]]],
            [p[1] for p in snap + [snap[0]]],
            [-p[2] for p in snap + [snap[0]]], 'k--', alpha=0.55, lw=1.1,
            label=f'最佳包围截面 t={t[best]:.2f}s')
    ax.set_xlabel('x_NED / m')
    ax.set_ylabel('y_NED / m')
    ax.set_zlabel(ALTITUDE_LABEL, labelpad=10)
    ax.set_title('三机协同拦截三维轨迹（显示高度向上，控制量仍为NED）')
    ax.view_init(elev=24, azim=-58)
    ax.set_box_aspect((1.8, 1.2, 0.9))
    ax.legend(loc='upper left', fontsize=8)
    ax.grid(True, alpha=0.25)
    _save(fig, out / FORMAL_FIGURES[0])

    # 02 全程 + 最佳时刻局部几何。
    fig, axes = plt.subplots(1, 2, figsize=(14, 6.3))
    for key in ('target', 'p1', 'p2', 'p3'):
        p = traj[key]
        axes[0].plot(p[:, 0], p[:, 1], color=COLORS[key], lw=1.5,
                     label=LABELS[key])
        axes[0].scatter(p[-1, 0], p[-1, 1], color=COLORS[key], s=25)
    axes[0].set_title('全程XY轨迹')
    axes[0].set_xlabel('x_NED / m'); axes[0].set_ylabel('y_NED / m')
    axes[0].axis('equal'); axes[0].grid(True, alpha=0.3); axes[0].legend(fontsize=8)
    target_xy = traj['target'][best, :2]
    points = {k: traj[k][best, :2] - target_xy for k in ('p1', 'p2', 'p3')}
    axes[1].scatter(0, 0, marker='X', s=120, color=COLORS['target'], label='敌机（中心）')
    for key in ('p1', 'p2', 'p3'):
        axes[1].scatter(*points[key], s=95, color=COLORS[key], label=LABELS[key])
        axes[1].annotate(key.upper(), points[key], xytext=(5, 5),
                         textcoords='offset points', fontsize=10)
    polygon = np.array([points['p1'], points['p2'], points['p3'], points['p1']])
    axes[1].plot(polygon[:, 0], polygon[:, 1], 'k--', lw=1.2)
    axes[1].axhline(0, color='gray', lw=0.6); axes[1].axvline(0, color='gray', lw=0.6)
    axes[1].set_title(f'最佳包围局部图 t={t[best]:.2f} s\n敌机坐标平移至原点')
    axes[1].set_xlabel('相对 x / m'); axes[1].set_ylabel('相对 y / m')
    axes[1].axis('equal'); axes[1].grid(True, alpha=0.3); axes[1].legend(fontsize=8)
    fig.suptitle('以P1实时方位为基准的近似等边三角形包围', fontsize=14)
    _save(fig, out / FORMAL_FIGURES[1])

    # 03 绝对高度与敌机下方垂直间距。
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    for key in ('target', 'p1', 'p2', 'p3'):
        axes[0].plot(t, -traj[key][:, 2], color=COLORS[key], lw=1.5,
                     label=LABELS[key])
    axes[0].set_ylabel(ALTITUDE_LABEL)
    axes[0].set_title('高度曲线（显示量 h=-z_NED，正方向向上）')
    axes[0].grid(True, alpha=0.3); axes[0].legend(ncol=2, fontsize=8)
    for key in ('p1', 'p2', 'p3'):
        axes[1].plot(t, series['vertical_below_target_m'][key],
                     color=COLORS[key], lw=1.5, label=f'{key.upper()}在敌机下方距离')
    axes[1].axhspan(15, 25, color='#2e7d32', alpha=0.10,
                    label='P2/P3规定垂直层 15–25 m')
    axes[1].set_xlabel('时间 / s'); axes[1].set_ylabel('敌机下方垂直间距 / m')
    axes[1].grid(True, alpha=0.3); axes[1].legend(ncol=2, fontsize=8)
    _save(fig, out / FORMAL_FIGURES[2])

    # 04 P1 远程打击细节图。
    fig, ax = plt.subplots(figsize=(12, 6.2))
    ax.plot(t, series['dist']['p1'], color=COLORS['p1'], lw=2.0,
            label='P1—敌机实际三维距离')
    ax.axhspan(15, 35, color='#43a047', alpha=0.12, label='有效开火区间 15–35 m')
    ax.axhline(17, color='#e65100', ls='--', lw=1.5, label='17 m参考线')
    for seq, event in enumerate(shots[:2], 1):
        idx = _shot_index(t, event)
        distance = event.get('actual_range_m')
        if distance is None:
            distance = float(series['dist']['p1'][idx])
        color = '#2e7d32' if event.get('result') == 'HIT' else '#c62828'
        ax.axvline(event['time_s'], color=color, ls=':', lw=1.2)
        ax.scatter(event['time_s'], distance, marker='*', s=170,
                   color=color, edgecolor='black', zorder=5)
        ax.annotate(_shot_label(seq, event), (event['time_s'], distance),
                    xytext=(12, 15 if seq == 1 else -58), textcoords='offset points',
                    fontsize=9, arrowprops={'arrowstyle': '->', 'color': color})
    if not shots:
        ax.text(0.5, 0.88, '本轮未收到P1虚拟开火事件', transform=ax.transAxes,
                ha='center', color='#c62828')
    ax.set_xlabel('时间 / s'); ax.set_ylabel('实际三维距离 / m')
    ax.set_title('P1远程打击细节：实际距离、估计距离与独立裁判结果')
    ax.grid(True, alpha=0.3); ax.legend(fontsize=9)
    _save(fig, out / FORMAL_FIGURES[3])

    # 05 僚机与敌机距离。
    fig, ax = plt.subplots(figsize=(12, 5.8))
    for key in ('p2', 'p3'):
        ax.plot(t, series['dist'][key], color=COLORS[key], lw=1.8,
                label=f'{key.upper()}—敌机三维距离')
    ax.axhline(15, color='#c62828', ls='--', lw=1.2, label='最小安全距离 15 m')
    ax.axhspan(20, 37, color='#43a047', alpha=0.10,
               label='水平/垂直合成三维包围距离 20–37 m')
    ax.set_xlabel('时间 / s'); ax.set_ylabel('距离 / m')
    ax.set_title('P2/P3与敌机距离（安全门控不得穿越敌机）')
    ax.grid(True, alpha=0.3); ax.legend()
    _save(fig, out / FORMAL_FIGURES[4])

    # 06 两两安全距离。
    fig, ax = plt.subplots(figsize=(12, 5.8))
    pair_labels = {'p1_p2': 'P1—P2', 'p2_p3': 'P2—P3', 'p3_p1': 'P3—P1'}
    pair_colors = {'p1_p2': '#00838f', 'p2_p3': '#6a1b9a', 'p3_p1': '#ef6c00'}
    for key in ('p1_p2', 'p2_p3', 'p3_p1'):
        ax.plot(t, series['inter_distance'][key], lw=1.7,
                color=pair_colors[key], label=pair_labels[key])
    ax.axhspan(0, 15, color='#c62828', alpha=0.12, label='禁止区 <15 m')
    ax.axhline(15, color='#c62828', ls='--', lw=1.2)
    ax.set_xlabel('时间 / s'); ax.set_ylabel('两两距离 / m')
    ax.set_title(f'三架拦截机两两安全距离（全程最小 {summary.get("min_inter_pursuer_distance", 0):.2f} m）')
    ax.grid(True, alpha=0.3); ax.legend()
    _save(fig, out / FORMAL_FIGURES[5])

    # 07 质心/方位误差。
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    axes[0].plot(t, series['centroid_error_xy'], color='black', lw=1.7,
                 label='三机水平质心—敌机误差')
    axes[0].axhline(8, color='#757575', ls='--', label='质量参考线 8 m（非硬门槛）')
    axes[0].set_ylabel('水平质心误差 / m'); axes[0].grid(True, alpha=0.3)
    axes[0].legend()
    axes[1].plot(t, series['bearing_error_deg']['p2'], color=COLORS['p2'],
                 lw=1.5, label='P2相对 +120°误差')
    axes[1].plot(t, series['bearing_error_deg']['p3'], color=COLORS['p3'],
                 lw=1.5, label='P3相对 −120°误差')
    axes[1].axhspan(-35, 35, color='#43a047', alpha=0.10,
                    label='优选参考带 ±35°（非硬门槛）')
    axes[1].set_xlabel('时间 / s'); axes[1].set_ylabel('方位角误差 / °')
    axes[1].grid(True, alpha=0.3); axes[1].legend(ncol=2)
    fig.suptitle('动态近似等边三角形的中心性与方位角误差', fontsize=14)
    _save(fig, out / FORMAL_FIGURES[6])

    # 08 第一次/第二次开火与最佳包围三张快照。
    snapshot_indices = [_shot_index(t, event) for event in shots[:2]]
    snapshot_titles = [f'第{i+1}次开火 · {shots[i].get("result", "PENDING")}'
                       for i in range(min(2, len(shots)))]
    while len(snapshot_indices) < 2:
        fallback = int((len(t) - 1) * (0.45 + 0.15 * len(snapshot_indices)))
        snapshot_indices.append(fallback)
        snapshot_titles.append('无开火事件 · 过程快照')
    snapshot_indices.append(best); snapshot_titles.append('最佳包围几何')
    fig, axes = plt.subplots(1, 3, figsize=(16, 5.2))
    for ax, idx, title in zip(axes, snapshot_indices, snapshot_titles):
        center = traj['target'][idx, :2]
        rel = {k: traj[k][idx, :2] - center for k in ('p1', 'p2', 'p3')}
        ax.scatter(0, 0, marker='X', s=110, color=COLORS['target'], label='敌机')
        for key in ('p1', 'p2', 'p3'):
            ax.scatter(*rel[key], s=85, color=COLORS[key], label=key.upper())
            ax.annotate(key.upper(), rel[key], xytext=(4, 5), textcoords='offset points')
        poly = np.array([rel['p1'], rel['p2'], rel['p3'], rel['p1']])
        ax.plot(poly[:, 0], poly[:, 1], 'k--', lw=1.0)
        ax.set_title(f'{title}\nt={t[idx]:.2f} s')
        ax.set_xlabel('相对 x / m'); ax.set_ylabel('相对 y / m')
        ax.axis('equal'); ax.grid(True, alpha=0.3); ax.legend(fontsize=8)
    fig.suptitle('关键时刻三机水平包围快照（敌机位于各子图原点）', fontsize=14)
    _save(fig, out / FORMAL_FIGURES[7])


def _write_compatibility_figures(out, series):
    """保留旧自动化入口所需名称；正式展示只使用连续编号图。"""
    mapping = {
        FORMAL_FIGURES[0]: 'trajectory_3d.png',
        FORMAL_FIGURES[1]: 'trajectory_xy.png',
        FORMAL_FIGURES[3]: 'distance_vs_time.png',
        FORMAL_FIGURES[5]: 'separation_safety.png',
        FORMAL_FIGURES[6]: 'cooperative_metric.png',
    }
    for source, destination in mapping.items():
        shutil.copy2(out / source, out / destination)
    fig, ax = plt.subplots(figsize=(10, 5))
    for key in ('target', 'p1', 'p2', 'p3'):
        ax.plot(series['t'], series['v_mag'][key], color=COLORS[key],
                lw=1.4, label=LABELS[key])
    ax.axhline(8, color=COLORS['target'], ls='--', lw=1, label='敌机8 m/s参考')
    ax.axhline(16, color=COLORS['p1'], ls='--', lw=1, label='P1安全上限16 m/s')
    ax.set_xlabel('时间 / s'); ax.set_ylabel('速度 / (m/s)')
    ax.set_title('四机速度曲线（兼容诊断图）'); ax.grid(True, alpha=0.3); ax.legend()
    _save(fig, out / 'speed_vs_time.png')


def _plot_generic(out, series, summary, shots):
    """纯算法仿真的兼容绘图，同样只以 h=-z_NED 展示垂向。"""
    fig = plt.figure(figsize=(10, 7))
    ax = fig.add_subplot(111, projection='3d')
    for key in ('target', 'p1', 'p2', 'p3'):
        p = series['traj'][key]
        ax.plot(p[:, 0], p[:, 1], -p[:, 2], color=COLORS[key], label=LABELS[key])
    ax.set_xlabel('x_NED / m'); ax.set_ylabel('y_NED / m'); ax.set_zlabel(ALTITUDE_LABEL)
    ax.set_title('三维轨迹（高度向上）'); ax.legend(); ax.view_init(24, -58)
    _save(fig, out / 'trajectory_3d.png')
    fig, ax = plt.subplots(figsize=(9, 7))
    for key in ('target', 'p1', 'p2', 'p3'):
        p = series['traj'][key]
        ax.plot(p[:, 0], p[:, 1], color=COLORS[key], label=LABELS[key])
    ax.axis('equal'); ax.grid(True, alpha=0.3); ax.legend()
    ax.set_xlabel('x_NED / m'); ax.set_ylabel('y_NED / m'); ax.set_title('XY轨迹')
    _save(fig, out / 'trajectory_xy.png')
    fig, ax = plt.subplots(figsize=(10, 5))
    for key in ('p1', 'p2', 'p3'):
        ax.plot(series['t'], series['dist'][key], color=COLORS[key], label=LABELS[key])
    ax.set_xlabel('时间 / s'); ax.set_ylabel('距离 / m'); ax.grid(True, alpha=0.3); ax.legend()
    _save(fig, out / 'distance_vs_time.png')
