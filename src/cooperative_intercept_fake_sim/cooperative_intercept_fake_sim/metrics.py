"""协同指标收集与统计（Metrics）。

每步收集：
    - 各机速度 / 加速度（检验 8 m/s 与 16 m/s 速度限制）
    - 三架 Pursuer 到目标的距离（最小距离、进入打击窗口时间）
    - 拦截机两两之间的距离（防碰撞统计）
    - 虚拟预测射击事件（每机射击/命中次数、首次命中、双命中完成）
    - 协同指标时间序列：
        escape_coverage ：前向逃逸半球角度覆盖度（目标速度方向 ±90 deg 内，
                          三机方位角空隙的最大空隙越小、覆盖越高）
        range_factor    ：距离因子 = clamp(1 - |d_mean - range_ref|/tolerance, 0, 1)
                          以 20 m 包围圈为参考
        cooperative_score = w_cov * escape_coverage + w_r * range_factor

成功判据（协同式 + 枪打击方案）：
    至少 minimum_strikers 架 Pursuer 在 15~35 m 打击窗口内各完成 2 次命中，
    且三机距离目标均在 [encircle_min, encircle_max] 内并持续 hold_s。
"""

import numpy as np

from .utils import clamp_f


class MetricsCollector:
    def __init__(self, cfg_score, cfg_success, cfg_shooting, entry_range_m):
        self.cov_w = float(cfg_score['coverage_weight'])
        self.range_w = float(cfg_score['range_weight'])
        self.range_ref = float(cfg_score['range_ref_m'])
        self.range_tol = float(cfg_score.get('range_tolerance_m',
                                             self.range_ref * 0.5))
        self.half_cone = float(cfg_score['escape_half_cone_rad'])
        self.formation_thr = float(cfg_score['formation_threshold'])
        self.formation_hold = float(cfg_score['formation_hold_s'])

        self.min_strikers = int(cfg_success['minimum_strikers'])
        self.enc_min = float(cfg_success['encircle_min_range_m'])
        self.enc_max = float(cfg_success['encircle_max_range_m'])
        self.success_hold = float(cfg_success['hold_s'])
        self.entry_range = entry_range_m

        # 时间序列（每步追加，仿真结束转 numpy 数组）
        self.t = []
        self.v_mag = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.a_mag = {'target': [], 'p1': [], 'p2': [], 'p3': []}
        self.dist = {'p1': [], 'p2': [], 'p3': []}
        self.inter_dist = []       # 三机两两距离
        self.score = []
        self.escape_cov = []
        self.range_factor = []
        self.max_forward_gap = []
        self.pred_error = []
        self.hits_log = {'p1': 0, 'p2': 0, 'p3': 0}  # 每机累计命中（步进记录用）

        # 事件时间
        self.first_entry_time = None
        self.first_success_time = None
        self.stable_success_time = None
        self.formation_time = None
        self.intercept_point = None   # 目标在稳定成功时刻的位置
        self.success = False
        self._success_run = 0.0
        self._formation_run = 0.0
        self._encircle_run = 0.0      # 包围圈保持计时（供成功判据）

    # ------------------------------------------------------------------
    def update(self, t, dt, target, vehicles, pred_point, strikers=None,
               active=True):
        """每步更新。

        strikers: dict {'p1': shooter, 'p2': shooter, 'p3': shooter}
        active:   接战已启用（False = 接战延迟期间，不统计进入窗口/判据）
        """
        p_t, v_t, a_t = target
        p, v, a, q = {}, {}, {}, {}
        for k, st in vehicles.items():
            p[k], v[k], a[k], q[k] = st

        self.t.append(t)
        for k in ('target', 'p1', 'p2', 'p3'):
            vv = v_t if k == 'target' else v[k]
            aa = a_t if k == 'target' else a[k]
            self.v_mag[k].append(float(np.linalg.norm(vv)))
            self.a_mag[k].append(float(np.linalg.norm(aa)))

        # ---- 距离 ----
        d = {k: float(np.linalg.norm(p[k] - p_t)) for k in ('p1', 'p2', 'p3')}
        for k in ('p1', 'p2', 'p3'):
            self.dist[k].append(d[k])
        self.inter_dist.append([
            float(np.linalg.norm(p['p1'] - p['p2'])),
            float(np.linalg.norm(p['p2'] - p['p3'])),
            float(np.linalg.norm(p['p3'] - p['p1'])),
        ])

        # ---- 前向逃逸半球角度覆盖 ----
        v_t_mag = float(np.linalg.norm(v_t))
        bearings = []
        for k in ('p1', 'p2', 'p3'):
            rel = p[k] - p_t
            bearings.append(np.arctan2(rel[1], rel[0]))
        if v_t_mag > 0.3:
            # 目标高速飞行：只统计前向逃逸半球（速度方向 ±90 deg），
            # 半球边界作为锚点，最大空隙越大覆盖越低。
            fwd = np.arctan2(v_t[1], v_t[0])
            lo, hi = fwd - self.half_cone, fwd + self.half_cone
            inside = sorted([b for b in bearings if lo <= b <= hi])
            anchors = [lo] + inside + [hi]
            gaps = [anchors[i + 1] - anchors[i]
                    for i in range(len(anchors) - 1)]
            max_gap = max(gaps) if gaps else 2.0 * self.half_cone
            escape_cov = clamp_f(1.0 - max_gap / (2.0 * self.half_cone))
        else:
            # 目标停止/低速：所有方向都是逃逸方向，改用全向覆盖
            sorted_b = sorted(bearings)
            sorted_b = sorted_b + [sorted_b[0] + 2.0 * np.pi]
            gaps = [sorted_b[i + 1] - sorted_b[i]
                    for i in range(len(sorted_b) - 1)]
            max_gap = max(gaps)
            escape_cov = clamp_f(1.0 - max_gap / (2.0 * np.pi))

        # ---- 距离因子：以 20 m 包围圈为参考（太远或太近都降低）----
        d_mean = sum(d.values()) / 3.0
        range_factor = clamp_f(
            1.0 - abs(d_mean - self.range_ref) / self.range_tol)

        # ---- 协同得分 ----
        score = self.cov_w * escape_cov + self.range_w * range_factor

        self.escape_cov.append(escape_cov)
        self.range_factor.append(range_factor)
        self.max_forward_gap.append(float(np.degrees(max_gap)))
        self.score.append(score)

        # ---- 预测截击点误差 ----
        self.pred_error.append(
            float(np.linalg.norm(p_t - pred_point)) if pred_point is not None
            else 0.0)

        # ---- 命中累计（供 CSV/绘图；仅统计配置了射击器的飞机）----
        if strikers:
            for k in strikers:
                self.hits_log[k] = strikers[k].hit_count

        # ---- 进入打击窗口（仅接战后统计）----
        if (active and self.first_entry_time is None
                and min(d.values()) <= self.entry_range):
            self.first_entry_time = t

        # ---- 成功判据：≥2 机双命中 + 包围圈保持 ----
        # 只统计配置了射击器的飞机（fake_sim 为三机；hybrid 2V1 为两架僚机）
        n_complete = sum(1 for k in (strikers or {})
                         if strikers[k].complete)
        in_encircle = (active and
                       all(self.enc_min <= d[k] <= self.enc_max
                           for k in ('p1', 'p2', 'p3')))
        if in_encircle:
            self._encircle_run += dt
        else:
            self._encircle_run = 0.0
        ok = (n_complete >= self.min_strikers
              and self._encircle_run >= self.success_hold)
        if ok:
            if self.first_success_time is None:
                self.first_success_time = t
            self._success_run += dt
            if self._success_run >= self.success_hold and \
                    self.stable_success_time is None:
                self.stable_success_time = t
                self.success = True
                self.intercept_point = p_t.copy()
        else:
            self._success_run = 0.0

        # ---- 包围形成（几何）----
        if score >= self.formation_thr:
            self._formation_run += dt
            if self._formation_run >= self.formation_hold and \
                    self.formation_time is None:
                self.formation_time = t
        else:
            self._formation_run = 0.0

    # ------------------------------------------------------------------
    def finalize(self):
        """汇总最终统计指标，返回 dict。"""
        t = np.asarray(self.t)

        def max_of(dct, k):
            return float(np.max(dct[k])) if dct[k] else 0.0

        min_d = {k: float(np.min(self.dist[k])) for k in ('p1', 'p2', 'p3')}
        inter_arr = np.asarray(self.inter_dist).reshape(-1, 3)
        min_inter = float(np.min(inter_arr)) if inter_arr.size else 0.0
        idx_min_inter = int(np.argmin(inter_arr.min(axis=1))) if inter_arr.size else 0
        t_min_inter = float(t[idx_min_inter]) if t.size else 0.0

        success_time = self.stable_success_time
        if success_time is not None:
            idx = int(np.searchsorted(t, success_time))
            pred_err_at_success = float(self.pred_error[min(idx, len(t) - 1)])
        else:
            pred_err_at_success = None

        # 速度限制校验（超限视为实现 bug）
        v_checks = {
            'target_max_speed': max_of(self.v_mag, 'target'),
            'p1_max_speed': max_of(self.v_mag, 'p1'),
            'p2_max_speed': max_of(self.v_mag, 'p2'),
            'p3_max_speed': max_of(self.v_mag, 'p3'),
        }

        return {
            'success': self.success,
            'capture_range_m': self.enc_max,
            'formation_threshold': self.formation_thr,
            'target_max_speed': v_checks['target_max_speed'],
            'p1_max_speed': v_checks['p1_max_speed'],
            'p2_max_speed': v_checks['p2_max_speed'],
            'p3_max_speed': v_checks['p3_max_speed'],
            'target_max_accel': max_of(self.a_mag, 'target'),
            'p1_max_accel': max_of(self.a_mag, 'p1'),
            'p2_max_accel': max_of(self.a_mag, 'p2'),
            'p3_max_accel': max_of(self.a_mag, 'p3'),
            'min_distance_p1': min_d['p1'],
            'min_distance_p2': min_d['p2'],
            'min_distance_p3': min_d['p3'],
            'first_entry_time': self.first_entry_time,
            'first_success_time': self.first_success_time,
            'intercept_time': success_time,
            'min_inter_pursuer_distance': min_inter,
            'min_inter_pursuer_time': t_min_inter,
            'formation_time': self.formation_time,
            'prediction_error_mean': float(np.mean(self.pred_error)),
            'prediction_error_at_success': pred_err_at_success,
            'final_cooperative_score': float(self.score[-1]),
            'max_cooperative_score': float(np.max(self.score)),
            'intercept_point': self.intercept_point,
            'speed_limit_violation': (
                v_checks['target_max_speed'] > 8.0 + 1e-9 or
                v_checks['p1_max_speed'] > 16.0 + 1e-9 or
                v_checks['p2_max_speed'] > 16.0 + 1e-9 or
                v_checks['p3_max_speed'] > 16.0 + 1e-9),
        }

    def series(self):
        """返回时间序列（供 CSV / 绘图）。"""
        return {
            't': np.asarray(self.t),
            'v_mag': {k: np.asarray(self.v_mag[k]) for k in self.v_mag},
            'a_mag': {k: np.asarray(self.a_mag[k]) for k in self.a_mag},
            'dist': {k: np.asarray(self.dist[k]) for k in self.dist},
            'score': np.asarray(self.score),
            'escape_cov': np.asarray(self.escape_cov),
            'range_factor': np.asarray(self.range_factor),
            'max_forward_gap': np.asarray(self.max_forward_gap),
            'pred_error': np.asarray(self.pred_error),
        }

