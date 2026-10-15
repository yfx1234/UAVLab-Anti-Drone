"""目标轨迹生成器（Target Generator）。

所有场景统一使用"路径表 + 弧长参数化"框架：
    1. 由场景参数解析构造路径（直线 / S 形 / 转弯 / 组合），
       以细间距采样为稠密点列 p(s)；
    2. 预计算累计弧长 S(s)、单位切向 t̂(s)、曲率 κ(s) 与法向 n̂(s)
       （中心差分，采样足够密所以光滑精确）；
    3. 运行时给定时刻 t：先由速度剖面（加速 - 巡航 - 减速，raised-cosine
       平滑）算出已走过的弧长 L(t)，再反查 s，得到 p / v / a：
           v = t̂ * L̇          （速度严格等于 L̇，不超过 max_speed）
           a = t̂ * L̈ + n̂ * κ * L̇²
       因此目标速度在巡航段精确等于 max_speed = 8 m/s，数学上保证 <= 8。

路径构造（NED 水平面，z = altitude_m 恒定）：
    straight       ：沿 heading 方向直线
    s_curve        ：沿 heading 方向前进 + 正弦横向摆动
    turn           ：直线 + 定半径圆弧转弯 + 直线
    encirclement   ：直线 + S 形 + 转弯 + 长直线（展示用组合路径）
"""

import numpy as np

from .utils import safe_atan2

_PATH_SAMPLE_DM = 0.05  # 路径采样间距（米），足够密以保证差分光滑


def _rotate_z(v, deg):
    """绕 NED z 轴旋转（正角 = 顺时针 / 右转）。"""
    rad = np.deg2rad(deg)
    c, s = np.cos(rad), np.sin(rad)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1], v[2]])


class TargetGenerator:
    def __init__(self, cfg_target, scenario_cfg, origin):
        self.alt = float(cfg_target['altitude_m'])
        self.v_max = float(cfg_target['max_speed'])
        self.ramp_t = float(cfg_target['ramp_time_s'])
        self.decel_t = float(cfg_target['decel_time_s'])
        self.origin = np.asarray(origin, dtype=float)

        # ---- 1. 构造路径采样点 ----
        pts = self._build_path_points(scenario_cfg['path'])
        self.pts = np.asarray(pts, dtype=float)  # (N, 3)

        # ---- 2. 弧长 / 切向 / 曲率表 ----
        d = np.diff(self.pts, axis=0)
        seg_len = np.linalg.norm(d, axis=1)
        self.s_cum = np.concatenate([[0.0], np.cumsum(seg_len)])  # 累计弧长 (N,)
        self.total_len = float(self.s_cum[-1])
        # 中心差分切向
        tangent = np.zeros_like(self.pts)
        tangent[1:-1] = (self.pts[2:] - self.pts[:-2]) / \
            (self.s_cum[2:] - self.s_cum[:-2])[:, None]
        tangent[0] = tangent[1]
        tangent[-1] = tangent[-2]
        self.tangent = tangent / np.linalg.norm(tangent, axis=1)[:, None]
        # 曲率 = |d t̂ / ds|，法向 = (d t̂ / ds) / κ
        dt_ds = np.zeros_like(self.pts)
        dt_ds[1:-1] = (self.tangent[2:] - self.tangent[:-2]) / \
            (self.s_cum[2:] - self.s_cum[:-2])[:, None]
        dt_ds[0] = dt_ds[1]
        dt_ds[-1] = dt_ds[-2]
        self.curv = np.linalg.norm(dt_ds, axis=1)
        normal = np.zeros_like(self.pts)
        nz = self.curv > 1e-6
        normal[nz] = dt_ds[nz] / self.curv[nz, None]
        self.normal = normal

        # ---- 3. 速度剖面解析式 ----
        # 加速段：v = V * 0.5*(1-cos(pi*t/T_ramp))，加速段弧长 = V*T_ramp/2
        self.ramp_len = self.v_max * self.ramp_t / 2.0
        self.decel_len = self.v_max * self.decel_t / 2.0
        cruise_len = max(0.0, self.total_len - self.ramp_len - self.decel_len)
        self.cruise_t = cruise_len / self.v_max
        # 轨迹路径运动总时间（之后目标原地悬停等待仿真结束）
        self.path_time = self.ramp_t + self.cruise_t + self.decel_t

        # 运行时弧长反查的索引缓存（t 单调递增，线性推进即可）
        self._idx = 0

    # ------------------------------------------------------------------
    # 路径构造
    # ------------------------------------------------------------------
    def _build_path_points(self, pcfg):
        ptype = pcfg['type']
        if ptype == 'straight':
            return self._seg_straight(float(pcfg['length_m']),
                                      float(pcfg['heading_deg']))
        if ptype == 's_curve':
            n_bumps = int(pcfg.get('n_bumps', 5))
            return self._seg_s_curve(n_bumps,
                                     float(pcfg['amplitude_m']),
                                     float(pcfg['wavelength_m']),
                                     float(pcfg['heading_deg']))
        if ptype == 'turn':
            return self._seg_turn(float(pcfg['leg1_m']),
                                  float(pcfg['turn_deg']),
                                  float(pcfg['turn_radius_m']),
                                  float(pcfg['leg2_m']))
        if ptype == 'encirclement':
            return self._seg_encirclement(pcfg)
        raise ValueError(f"未知场景路径类型: {ptype}")

    def _seg_straight(self, length, heading_deg):
        u = _rotate_z(np.array([1.0, 0.0, 0.0]), heading_deg)
        s = np.arange(0.0, length + _PATH_SAMPLE_DM, _PATH_SAMPLE_DM)
        p = np.outer(s, u)
        p[:, 2] = self.alt
        return p

    def _seg_s_curve(self, n_bumps, amp, wavelength, heading_deg):
        """S 形路径：交替正负的 (1-cos) 半波凸包。

        每个凸包在起点/终点处横向位置与横向速度均为 0
        （1-cos 剖面起点斜率 = 0），因此与前后直线段切向连续，
        不会产生方向突变（保证加速度有限，目标速度平滑）。
        """
        u = _rotate_z(np.array([1.0, 0.0, 0.0]), heading_deg)
        n = _rotate_z(np.array([0.0, 1.0, 0.0]), heading_deg)
        total = n_bumps * wavelength
        s = np.arange(0.0, total + _PATH_SAMPLE_DM, _PATH_SAMPLE_DM)
        lat = np.zeros_like(s)
        sign = 1.0
        for b in range(n_bumps):
            local = s - b * wavelength
            mask = (local >= 0.0) & (local < wavelength)
            lat[mask] += sign * amp * 0.5 * \
                (1.0 - np.cos(2.0 * np.pi * local[mask] / wavelength))
            sign = -sign
        p = np.outer(s, u) + np.outer(lat, n)
        p[:, 2] = self.alt
        return p

    def _seg_turn(self, leg1, turn_deg, radius, leg2):
        # 直线 + 定半径圆弧 + 直线；turn_deg 为正表示右转（NED 顺时针）
        leg1_pts = self._seg_straight(leg1, 0.0)
        turn_rad = np.deg2rad(turn_deg)
        sign = 1.0 if turn_deg >= 0 else -1.0
        center = np.array([leg1, sign * radius, self.alt])
        n_arc = max(8, int(abs(radius * turn_rad) / _PATH_SAMPLE_DM))
        # 圆弧从 -90 deg（垂直接入直线方向）转到 -90 + turn_rad
        ang = np.linspace(-sign * np.pi / 2,
                          -sign * np.pi / 2 + turn_rad, n_arc)
        arc = np.column_stack([
            center[0] + radius * np.cos(ang),
            center[1] + radius * np.sin(ang),
            np.full(n_arc, self.alt),
        ])
        leg2_pts = self._seg_straight(leg2, turn_deg)
        leg2_pts = leg2_pts + (arc[-1] - leg2_pts[0])
        return np.vstack([leg1_pts[:-1], arc[:-1], leg2_pts])

    def _seg_encirclement(self, pcfg):
        # 直线 + S 形 + 转弯 + 长直线：最后的长直线让包围结构稳定成型
        leg1 = self._seg_straight(float(pcfg['leg1_m']), 0.0)
        s_pts = self._seg_s_curve(int(pcfg.get('n_bumps', 2)),
                                  float(pcfg['amplitude_m']),
                                  float(pcfg['wavelength_m']), 0.0)
        s_pts = s_pts + (leg1[-1] - s_pts[0])
        turn_rad = np.deg2rad(float(pcfg['turn_deg']))
        sign = 1.0 if pcfg['turn_deg'] >= 0 else -1.0
        radius = float(pcfg['turn_radius_m'])
        center = np.array([s_pts[-1][0], s_pts[-1][1] + sign * radius, self.alt])
        n_arc = max(8, int(abs(radius * turn_rad) / _PATH_SAMPLE_DM))
        ang = np.linspace(-sign * np.pi / 2,
                          -sign * np.pi / 2 + turn_rad, n_arc)
        arc = np.column_stack([
            center[0] + radius * np.cos(ang),
            center[1] + radius * np.sin(ang),
            np.full(n_arc, self.alt),
        ])
        leg2 = self._seg_straight(float(pcfg['leg2_m']),
                                  float(pcfg['turn_deg']))
        leg2 = leg2 + (arc[-1] - leg2[0])
        return np.vstack([leg1[:-1], s_pts[:-1], arc[:-1], leg2])

    # ------------------------------------------------------------------
    # 运行时采样
    # ------------------------------------------------------------------
    def _arclen_at(self, t):
        """速度剖面：加速 - 巡航 - 减速，得到 (已走弧长 L, 速度 v, 加速度 a)。"""
        if t <= 0.0:
            return 0.0, 0.0, 0.0
        if t < self.ramp_t:
            # raised-cosine 加速：v = V*0.5*(1-cos(pi t/T))，a = V*pi/(2T) sin(...)
            x = t / self.ramp_t
            v = self.v_max * 0.5 * (1.0 - np.cos(np.pi * x))
            a = self.v_max * np.pi / (2.0 * self.ramp_t) * np.sin(np.pi * x)
            L = self.v_max * 0.5 * (t - self.ramp_t / np.pi * np.sin(np.pi * x))
            return L, v, a
        t_cruise = t - self.ramp_t
        if t_cruise < self.cruise_t:
            return self.ramp_len + self.v_max * t_cruise, self.v_max, 0.0
        t_dec = t_cruise - self.cruise_t
        if t_dec < self.decel_t:
            x = t_dec / self.decel_t
            v = self.v_max * 0.5 * (1.0 + np.cos(np.pi * x))
            a = -self.v_max * np.pi / (2.0 * self.decel_t) * np.sin(np.pi * x)
            # 减速段弧长：L = L_cruise + V/2 * T_decel * (x + sin(pi*x)/pi)
            L = (self.ramp_len + self.v_max * self.cruise_t +
                 self.v_max * 0.5 * self.decel_t *
                 (x + np.sin(np.pi * x) / np.pi))
            return L, v, a
        return self.total_len, 0.0, 0.0

    def sample(self, t):
        """返回 t 时刻目标真值状态 (p, v, a)，NED。"""
        L, v_speed, a_speed = self._arclen_at(t)
        L = min(L, self.total_len)
        # 弧长反查采样点（带索引缓存，t 单调递增时近似线性推进）
        if not (self.s_cum[self._idx] <= L <= self.s_cum[self._idx + 1]):
            self._idx = int(np.searchsorted(self.s_cum, L) - 1)
        self._idx = max(0, min(self._idx, len(self.s_cum) - 2))
        i0, i1 = self._idx, self._idx + 1
        s0, s1 = self.s_cum[i0], self.s_cum[i1]
        f = 0.0 if s1 <= s0 else (L - s0) / (s1 - s0)
        p = self.pts[i0] + f * (self.pts[i1] - self.pts[i0])
        t_hat = self.tangent[i0] + f * (self.tangent[i1] - self.tangent[i0])
        curv = self.curv[i0] + f * (self.curv[i1] - self.curv[i0])
        n_hat = self.normal[i0] + f * (self.normal[i1] - self.normal[i0])
        if np.linalg.norm(n_hat) < 1e-9:
            n_hat = np.array([0.0, 1.0, 0.0])
        else:
            n_hat = n_hat / np.linalg.norm(n_hat)

        # 注意：origin 只提供水平平移（z 已由路径点统一设置为 altitude_m）
        p = p + np.array([self.origin[0], self.origin[1], 0.0])
        v = t_hat * v_speed
        a = t_hat * a_speed + n_hat * (curv * v_speed * v_speed)
        return p, v, a

    def initial_yaw(self):
        """轨迹初始航向（供目标/拦截机初始朝向用）。"""
        d = self.pts[1] - self.pts[0]
        return safe_atan2(d[1], d[0])
