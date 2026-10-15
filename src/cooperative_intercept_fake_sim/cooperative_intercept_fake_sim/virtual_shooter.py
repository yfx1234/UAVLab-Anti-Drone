"""虚拟预测射击模块（Virtual Predictive Shooter）。

体现原项目"枪打击方案"的效果：在 15~35 m 理想打击窗口内，以高速虚拟弹丸
（400 m/s，弹丸飞行时间 < 0.1 s）对目标进行**预测射击**（先导瞄准目标未来
位置），命中半径 0.4 m；每机需完成 2 次命中（两次打击），命中后 3 s 冷却
（对应原项目 required_virtual_captures=2 与 reattack_cooldown_s=3.0）。

射击时序（每步判定）：
    1. 距离门：与目标距离在 [minimum_range, maximum_range] 窗口内才允许射击；
    2. 射速门：距上次发射至少 fire_interval（默认 0.5 s = 2 Hz）；
    3. 瞄准与发射：弹道指向"预测命中点"（目标 t_of_flight 后的位置，
       即先导角瞄准），t_of_flight = range / projectile_speed；
    4. 命中判定：目标在弹丸到达时刻的位置到弹道的最近距离 < hit_radius
       → 命中；命中后进入冷却（3 s），命中计数 +1；
    5. 完成：命中数达到 required_hits（2）后该机停止射击并标记打击完成。

这是纯几何虚拟判定（理想弹道、无重力/气动、无随机散布），仅用于演示
"预测射击 + 双命中"的打击方案效果，不代表真实武器性能。
"""

import numpy as np

from .utils import clamp, normalize


class VirtualShooter:
    def __init__(self, cfg_shooting, name, dt):
        self.name = name
        self.dt = dt
        self.projectile_speed = float(cfg_shooting['projectile_speed_mps'])
        self.min_range = float(cfg_shooting['minimum_range_m'])
        self.max_range = float(cfg_shooting['maximum_range_m'])
        self.hit_radius = float(cfg_shooting['hit_radius_m'])
        self.fire_interval = float(cfg_shooting['fire_interval_s'])
        self.cooldown_after_hit = float(cfg_shooting['cooldown_after_hit_s'])
        self.required_hits = int(cfg_shooting['required_hits'])

        self.shot_count = 0        # 累计射击次数
        self.hit_count = 0         # 累计命中次数
        self.first_hit_time = None
        self.complete_time = None  # 达到 required_hits 的时刻
        self.last_fire_t = -1e9
        self.cooldown_until_t = -1e9

        self.events = []           # (t, 'shot'|'hit', range, aim_error_deg)

    # ------------------------------------------------------------------
    @property
    def complete(self):
        return self.complete_time is not None

    def _try_hit(self, t, p_gun, p_t, v_t):
        """预测射击：瞄准目标在弹丸到达时刻的位置，判定命中。"""
        d = float(np.linalg.norm(p_t - p_gun))
        t_of_flight = d / self.projectile_speed
        p_t_future = p_t + v_t * t_of_flight          # 目标未来位置
        aim = normalize(p_t_future - p_gun)           # 先导角瞄准方向
        if np.linalg.norm(aim) < 0.5:
            return False
        # 弹道最近距离：|cross(p_t_future - p_gun, aim)|
        miss = float(np.linalg.norm(
            np.cross(p_t_future - p_gun, aim)))
        return miss < self.hit_radius

    # ------------------------------------------------------------------
    def update(self, t, p_gun, p_t, v_t):
        """每步更新，返回本步事件列表（供记录与绘图）。"""
        events = []
        d = float(np.linalg.norm(p_t - p_gun))

        # 距离门 + 射速门 + 冷却门 + 完成门
        in_window = self.min_range <= d <= self.max_range
        if (in_window and not self.complete
                and t - self.last_fire_t >= self.fire_interval
                and t >= self.cooldown_until_t):
            self.shot_count += 1
            self.last_fire_t = t
            hit = self._try_hit(t, p_gun, p_t, v_t)
            events.append(('shot', t, d))
            if hit:
                self.hit_count += 1
                self.cooldown_until_t = t + self.cooldown_after_hit
                if self.first_hit_time is None:
                    self.first_hit_time = t
                if self.hit_count >= self.required_hits:
                    self.complete_time = t
                events.append(('hit', t, d))
        return events
