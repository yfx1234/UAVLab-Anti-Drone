"""协同制导层（Cooperative Guidance）：3 对 1 分工与参考轨迹生成。

输入：目标真值 (p_t, v_t, a_t) + 三架拦截机状态（本模块主要用位置做预测距离）
输出：每架拦截机的参考轨迹 (p_ref, v_ref, a_ref, yaw_ref) + 预测截击点/封堵点

三机分工（核心协同逻辑）：

    P1 主追踪机：参考点 = 目标当前位置 - u * behind_offset + n * lateral
        - 不预测，直接追目标当前点，始终保持一个小的"身后"间距，
          持续施加追击压力；目标转弯时跟随目标方向绕到其身后。

    P2 预测截击机：参考点 = 目标预测点 + n * lateral（预测点在目标前方）
        - p_pred = p_t + u * (v * pred_horizon) + 0.5 * a_t * pred_horizon²
        - 预测时域自适应：离预测点远时用长时域（2 s = 前方 16 m），
          提前切向目标未来轨迹，形成明显的"截击"效果；
          接近预测点后收缩到短时域（0.35 s），收拢到目标前侧方贴身包围。

    P3 封堵机：参考点 = 目标位置 + u * forward_offset + n * lateral
        - 始终占据目标前方、与 P2 相反的一侧，封堵目标前方另一侧逃逸方向；
          与 P1（后）、P2（左前）构成后 / 左前 / 右前三面合围。

其中 u = 目标速度单位方向（经低通滤波，低速时保持上一方向），
n = u 的水平左法向。所有偏移量由 YAML 参数控制。

参考速度/加速度取目标当前速度/加速度（运动学匹配），
配合 SE(3) 控制器位置环收敛到参考点 —— 上层协同制导 -> 参考轨迹 ->
SE(3) 控制器 -> 虚拟动力学的完整链路由此建立。
"""

import numpy as np

from .utils import clamp, normalize, safe_atan2


class CooperativeGuidance:
    def __init__(self, cfg_roles, dt):
        self.roles = {
            'p1': cfg_roles['p1'],
            'p2': cfg_roles['p2'],
            'p3': cfg_roles['p3'],
        }
        self.dt = dt
        # 目标速度方向低通滤波（时间常数 0.25 s），
        # 避免目标速度接近零时方向噪声导致参考点抖动
        self._dir_filter_tau = 0.25
        self._dir_alpha = np.exp(-dt / self._dir_filter_tau)
        self._dir = np.array([1.0, 0.0, 0.0])  # 初始朝向 +x（所有场景起点朝 +x）

        self.pred_point = None   # 预测截击点（供指标/绘图）
        self.block_point = None  # 封堵点（供指标/绘图）

    # ------------------------------------------------------------------
    def _velocity_dir(self, v_t, v_mag):
        """低通滤波的目标速度单位方向 u；低速时保持上一方向。"""
        if v_mag > 0.3:
            new_dir = v_t / v_mag
            if np.dot(self._dir, new_dir) < 0.0:  # 大转角直接换向（防滤慢）
                self._dir = new_dir
            else:
                self._dir = normalize(
                    self._dir * self._dir_alpha + new_dir * (1 - self._dir_alpha))
        return self._dir

    # ------------------------------------------------------------------
    def compute(self, t, p_t, v_t, a_t, pursuer_states):
        """计算三架拦截机的参考轨迹。

        pursuer_states: dict {'p1': {'p': ...}, 'p2': ..., 'p3': ...}
        返回: dict {key: {p_ref, v_ref, a_ref, yaw_ref, role}} + 附带的截击/封堵点
        """
        v_mag = float(np.linalg.norm(v_t))
        u = self._velocity_dir(v_t, v_mag)
        # n = u 的水平左法向（保证在水平面内）
        n = np.array([-u[1], u[0], 0.0])
        n = normalize(n)

        refs = {}

        # ---------------- P1：主追踪机（尾追位：目标正后方 20 m） ----------------
        r1 = self.roles['p1']
        # 参考点 = 目标当前点 - u*behind + n*lateral + 高度偏移
        # 对应原项目"枪打击方案"的尾追几何：占据目标后方 20 m 打击窗口位
        p1_ref = (p_t - u * r1['behind_offset_m'] + n * r1['lateral_offset_m']
                  + np.array([0.0, 0.0, r1['z_m'] - p_t[2]]))
        refs['p1'] = {
            'p_ref': p1_ref,
            'v_ref': v_t.copy(),          # 与目标同速，在包围圈上伴飞
            'a_ref': a_t.copy(),
            'yaw_ref': safe_atan2(v_t[1], v_t[0], fallback=0.0),
        }

        # ---------------- P2：预测截击机（提前切入目标未来轨迹） ----------------
        r2 = self.roles['p2']
        horizon = float(r2['pred_horizon_s'])
        d_near = float(r2['near_dist_m'])      # 距目标 d_near 以内开始收拢
        blend_dist = float(r2['horizon_blend_dist_m'])
        # 短时匀加速预测：p_pred = p_t + u*v*T + 0.5*a*T²（u 方向为滤波后的平滑方向）
        p_pred = (p_t + u * (v_mag * horizon) + 0.5 * a_t * horizon * horizon)
        self.pred_point = p_pred.copy()
        # 包围圈位：目标前 10 m + 左侧 17.32 m（方位 -60°），与目标距离 20 m，
        # 与 P1（后 20 m）、P3（前右 20 m）构成 120° 间隔的等边包围圈。
        # n 为 u 的水平左法向，故 lateral_offset_m 取正 = 左侧。
        p_side = (p_t + u * r2['forward_offset_m'] + n * r2['lateral_offset_m']
                  + np.array([0.0, 0.0, r2['z_m'] - p_t[2]]))
        # 混合权重：按 P2 与目标当前距离切换。
        #   距离远（> blend_dist）：飞向完整预测点，提前切向目标未来轨迹；
        #   距离近（< d_near）    ：收拢到包围圈左前位（20 m 打击窗口）。
        d_tgt = float(np.linalg.norm(pursuer_states['p2']['p'] - p_t))
        w = clamp((d_tgt - d_near) / (blend_dist - d_near), 0.0, 1.0)
        p2_ref = w * p_pred + (1.0 - w) * p_side
        refs['p2'] = {
            'p_ref': p2_ref,
            'v_ref': v_t.copy(),
            'a_ref': a_t.copy(),
            'yaw_ref': safe_atan2(v_t[1], v_t[0], fallback=0.0),
        }

        # ---------------- P3：封堵机（前右 20 m 包围位，封堵逃逸方向） ----------------
        r3 = self.roles['p3']
        # 参考点 = 目标前 10 m + 右侧 17.32 m（方位 +60°），与 P2 对称
        p3_ref = (p_t + u * r3['forward_offset_m'] + n * r3['lateral_offset_m']
                  + np.array([0.0, 0.0, r3['z_m'] - p_t[2]]))
        self.block_point = p3_ref.copy()
        refs['p3'] = {
            'p_ref': p3_ref,
            'v_ref': v_t.copy(),
            'a_ref': a_t.copy(),
            'yaw_ref': safe_atan2(v_t[1], v_t[0], fallback=0.0),
        }

        return refs
