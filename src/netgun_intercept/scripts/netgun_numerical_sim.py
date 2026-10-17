#!/usr/bin/env python3
"""netgun 数值冒烟仿真（纯 Python/numpy，无 ROS）。

目的：在 Gazebo 之前用数值实验验证拦截核心逻辑——三模 IMM(CV/CA/CT)
估计、1°即打/2°+0.2s 择时、本机速度补偿、3×3 到达判定——在不同目标
运动（静止→5m/s：悬停/直线/加速/圆周）与姿态抖动（±2–6° 耦合摆动，
见开发手册 §4）下的触发率与命中率。同时对比触发延迟 0s 与 0.5s
（方案 §12.3）的影响。

约定：NED；目标在追踪机上方（相对位置 z 为负）；光轴=body −Z=攻击轴。
IMM 为 C++ relative_ekf 的同构 Python 参照（10 维/3 模/一阶 CT）。
"""

from __future__ import annotations

import math
import sys

import numpy as np

# ---------------- 常量（与 netgun_strike.yaml 单一真源一致） ----------------
MUZZLE_SPEED = 55.0
DEPLOYMENT_S = 0.15
RANGE_MIN, RANGE_MAX = 6.0, 10.0
COVERAGE_HALF = 1.5
PRIMARY_RAD = math.radians(1.0)
WINDOW_RAD = math.radians(2.0)
WINDOW_HOLD_S = 0.2
LOS_RATE_GATE = 0.5
CAM_RATE_HZ = 30.0
DT = 1.0 / CAM_RATE_HZ
END_TIME_S = 40.0


# ---------------- IMM（CV/CA/CT，10 维，同构 C++ relative_ekf） ----------------
class ImmEkf:
    def __init__(
        self,
        acc_noise=2.0,
        turn_noise=0.5,
        p0=1.0, v0=3.0, a0=3.0, w0=0.3,
        cv_cv=0.97, ca_ca=0.95, ct_ct=0.95,
        mu0=(0.6, 0.25, 0.15),
        gate=16.266,
    ):
        self.acc_var = acc_noise ** 2
        self.turn_noise = turn_noise
        self.gate = gate
        self.initial = np.array([p0 ** 2, v0 ** 2, a0 ** 2, w0 ** 2])
        self.tpm = np.array([
            [cv_cv, (1 - cv_cv) / 2, (1 - cv_cv) / 2],
            [(1 - ca_ca) / 2, ca_ca, (1 - ca_ca) / 2],
            [(1 - ct_ct) / 2, (1 - ct_ct) / 2, ct_ct],
        ])
        self.mu = np.array(mu0, dtype=float)
        self.models = None
        self.t_last = None

    def _init_models(self, z):
        self.models = []
        diag = np.concatenate([
            np.full(3, self.initial[0]), np.full(3, self.initial[1]),
            np.full(3, self.initial[2]), [self.initial[3]],
        ])
        for _ in range(3):
            x = np.zeros(10)
            x[:3] = z
            self.models.append({"x": x, "p": np.diag(diag)})

    def _transition(self, model, dt, kind):
        f = np.eye(10)
        f[0:3, 3:6] = np.eye(3) * dt
        w = model["x"][9]
        if kind != 0:
            f[0:3, 6:9] = np.eye(3) * (0.5 * dt * dt)
            f[3:6, 6:9] = np.eye(3) * dt
        if kind == 2:
            f[3, 4] = -w * dt
            f[4, 3] = w * dt
            f[3, 9] = -dt * model["x"][4]
            f[4, 9] = dt * model["x"][3]
        else:
            f[9, 9] = 0.0
            model["x"][9] = 0.0
            model["p"][9, 9] = self.initial[3]
        return f

    def _noise(self, dt, kind):
        q = np.zeros((10, 10))
        v = self.acc_var
        dt2, dt3, dt4, dt5 = dt ** 2, dt ** 3, dt ** 4, dt ** 5
        if kind != 0:
            q[0:3, 0:3] = np.eye(3) * (dt5 * v / 20)
            q[0:3, 3:6] = np.eye(3) * (dt4 * v / 8)
            q[3:6, 0:3] = np.eye(3) * (dt4 * v / 8)
            q[0:3, 6:9] = np.eye(3) * (dt3 * v / 6)
            q[6:9, 0:3] = np.eye(3) * (dt3 * v / 6)
            q[3:6, 3:6] = np.eye(3) * (dt3 * v / 3)
            q[3:6, 6:9] = np.eye(3) * (dt2 * v / 2)
            q[6:9, 3:6] = np.eye(3) * (dt2 * v / 2)
            q[6:9, 6:9] = np.eye(3) * (dt * v)
        else:
            q[0:3, 0:3] = np.eye(3) * (0.25 * dt4 * v)
            q[0:3, 3:6] = np.eye(3) * (0.5 * dt3 * v)
            q[3:6, 0:3] = np.eye(3) * (0.5 * dt3 * v)
            q[3:6, 3:6] = np.eye(3) * (dt2 * v)
        if kind == 2:
            q[9, 9] = self.turn_noise ** 2 * dt
        return q

    def update(self, z, r, t):
        if self.models is None:
            self._init_models(z)
            self.t_last = t
            return True
        dt = t - self.t_last
        if dt <= 0:
            return False
        self.t_last = t
        # 交互
        mixed = []
        for dst in range(3):
            normalizer = float(self.mu @ self.tpm[:, dst])
            x = np.zeros(10)
            p = np.zeros((10, 10))
            for src in range(3):
                w = self.mu[src] * self.tpm[src, dst] / normalizer
                x += w * self.models[src]["x"]
            for src in range(3):
                w = self.mu[src] * self.tpm[src, dst] / normalizer
                d = self.models[src]["x"] - x
                p += w * (self.models[src]["p"] + np.outer(d, d))
            mixed.append({"x": x, "p": p})
        self.models = mixed
        self.mu = self.mu @ self.tpm
        # 预测 + 更新
        h = np.zeros((3, 10))
        h[0:3, 0:3] = np.eye(3)
        likelihoods = np.zeros(3)
        for kind, model in enumerate(self.models):
            f = self._transition(model, dt, kind)
            model["x"] = f @ model["x"]
            model["p"] = f @ model["p"] @ f.T + self._noise(dt, kind)
            innovation = z - h @ model["x"]
            s = h @ model["p"] @ h.T + r
            mah = float(innovation @ np.linalg.solve(s, innovation))
            if mah <= self.gate and np.isfinite(mah):
                likelihoods[kind] = math.exp(-0.5 * min(mah, 100.0)) / math.sqrt(
                    max(np.linalg.det(s), 1e-18) * (2 * math.pi) ** 3
                )
                k = model["p"] @ h.T @ np.linalg.inv(s)
                model["x"] = model["x"] + k @ innovation
                i10 = np.eye(10)
                c = i10 - k @ h
                model["p"] = c @ model["p"] @ c.T + k @ r @ k.T
                model["p"] = 0.5 * (model["p"] + model["p"].T)
            else:
                likelihoods[kind] = 0.0
        norm = float(self.mu @ likelihoods)
        if norm > 1e-300 and np.isfinite(norm):
            self.mu = self.mu * likelihoods / norm
        return bool(np.any(likelihoods > 0))

    def estimate(self):
        x = np.zeros(10)
        for m, w in zip(self.models, self.mu):
            x += w * m["x"]
        return x[:3].copy(), x[3:6].copy(), self.mu.copy()


# ---------------- 抖动模型（姿态-动力学耦合的简化表示） ----------------
def axis_jitter(t, rng, amp_deg=3.0, freq_hz=0.8, noise_deg=0.3):
    """攻击轴绕『当前瞄准方向』的摆动角（弧度，双轴小角）：主频正弦+白噪。

    终端对准段的真实行为：机体轴跟踪预测视线（姿态滞后 τ_att 由
    smoothed_axis 一阶滤波表达），叠加姿态-动力学耦合抖动。
    """
    a = math.radians(amp_deg)
    return (
        a * math.sin(2 * math.pi * freq_hz * t + rng.phase1)
        + math.radians(noise_deg) * rng.gauss,
        a * 0.7 * math.sin(2 * math.pi * freq_hz * 1.37 * t + rng.phase2)
        + math.radians(noise_deg) * rng.gauss * 0.5,
    )


def apply_tilt(direction, tilt_x, tilt_y):
    """把小倾角施加到方向向量上（一阶：沿两个正交切向偏移后归一）。"""
    ref = np.array([0.0, 0.0, 1.0]) if abs(direction[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(direction, ref)
    u /= np.linalg.norm(u)
    v = np.cross(direction, u)
    out = direction + u * tilt_x + v * tilt_y
    return out / np.linalg.norm(out)


def pursuer_jitter_position(t, rng, amp=0.3):
    return np.array([
        amp * math.sin(2 * math.pi * 0.55 * t + rng.phase3),
        amp * 0.8 * math.sin(2 * math.pi * 0.41 * t + rng.phase4),
        0.5 * amp * math.sin(2 * math.pi * 0.63 * t + rng.phase1),
    ])


# ---------------- 场景 ----------------
def target_relative(t, motion, speed, rng):
    """目标相对位置/速度（NED，z 负为上）。"""
    h = -7.0
    if motion == "static":
        return np.array([rng.x0, rng.y0, h]), np.zeros(3)
    if motion == "straight":
        hdg = rng.heading
        return (
            np.array([rng.x0 + speed * t * math.cos(hdg), rng.y0 + speed * t * math.sin(hdg), h]),
            np.array([speed * math.cos(hdg), speed * math.sin(hdg), 0.0]),
        )
    if motion == "accel":
        # 0.5m/s 起步匀加速 1.0m/s²，封顶 5
        v = min(0.5 + 1.0 * t, 5.0)
        hdg = rng.heading
        return (
            np.array([rng.x0 + (0.5 * t + 0.5 * min(t, 4.5) ** 2) * math.cos(hdg),
                      rng.y0 + (0.5 * t + 0.5 * min(t, 4.5) ** 2) * math.sin(hdg), h]),
            np.array([v * math.cos(hdg), v * math.sin(hdg), 0.0]),
        )
    if motion == "circle":
        radius = max(2.0, speed * 2.0)
        w = speed / radius
        ang = rng.phase2
        rel = np.array([radius * (math.cos(w * t + ang) - math.cos(ang)),
                        radius * (math.sin(w * t + ang) - math.sin(ang)), h])
        vel = np.array([-radius * w * math.sin(w * t + ang),
                        radius * w * math.cos(w * t + ang), 0.0])
        return rel, vel
    raise ValueError(motion)


# ---------------- 单轮仿真 ----------------
class Round:
    def __init__(self, motion, speed, seed, delay_assess=(0.0, 0.5),
                 jitter_amp_deg=3.0, pos_noise=0.04, depth_noise=0.10,
                 tau_att=0.3):
        self.motion, self.speed, self.seed = motion, speed, seed
        self.tau_att = tau_att
        self.delays = delay_assess
        self.jitter_amp = jitter_amp_deg
        self.pos_noise = pos_noise  # 相对位置观测噪声（米，≈1.5px/fx·range+深度σ）
        self.depth_noise = depth_noise
        rng = np.random.default_rng(seed)
        self.rng_t = type("R", (), {})()
        self.rng_t.gauss = 0.0
        self.rng_t.x0 = rng.uniform(-1.2, 1.2)
        self.rng_t.y0 = rng.uniform(-1.2, 1.2)
        self.rng_t.heading = rng.uniform(0, 2 * math.pi)
        self.rng_t.phase1 = rng.uniform(0, 2 * math.pi)
        self.rng_t.phase2 = rng.uniform(0, 2 * math.pi)
        self.rng_t.phase3 = rng.uniform(0, 2 * math.pi)
        self.rng_t.phase4 = rng.uniform(0, 2 * math.pi)
        self.np_rng = rng

    def run(self):
        est = ImmEkf()
        hold_start = None
        fired = False
        fire_t = fire_range = fire_axis_err = float("nan")
        fire_dir = None
        fire_axis_smoothed = None
        fire_pursuer = None
        last_dir = None
        last_t = None
        smoothed_axis = None
        t = 0.0
        steps = int(END_TIME_S / DT)
        for i in range(steps):
            t = i * DT
            self.rng_t.gauss = self.np_rng.normal()
            rel, rel_vel = target_relative(t, self.motion, self.speed, self.rng_t)
            p_pos = pursuer_jitter_position(t, self.rng_t)
            # 传感器：真值相对位置 + 噪声（0.2s 延迟以戳平移表达）
            t_obs = max(t - 0.2, 0.0)
            rel_obs_true, _ = target_relative(t_obs, self.motion, self.speed, self.rng_t)
            rng_m = float(np.linalg.norm(rel_obs_true))
            if rng_m > 10.0:
                continue  # 深度无效（>10m），估计器无更新
            z = rel_obs_true + self.np_rng.normal(0.0, self.pos_noise, 3)
            r = (np.eye(3) * (self.pos_noise ** 2 + self.depth_noise ** 2))
            est.update(z, r, t_obs)
            # 本机速度（抖动导数近似）
            dt2 = 1e-3
            p2 = pursuer_jitter_position(t + dt2, self.rng_t)
            v_own = (p2 - p_pos) / dt2
            pos, vel, mu = est.estimate()
            range_est = float(np.linalg.norm(pos))
            if not np.all(np.isfinite(pos)) or range_est > 30.0:
                last_dir, last_t = None, None
                continue
            lead = range_est / MUZZLE_SPEED + DEPLOYMENT_S
            aim = pos + vel * lead - v_own * lead
            aim_dir = aim / np.linalg.norm(aim)
            # 终端对准：机体轴一阶跟踪瞄准方向（姿态滞后 tau_att），叠加抖动
            if smoothed_axis is None:
                smoothed_axis = aim_dir.copy()
            alpha = DT / (self.tau_att + DT)
            smoothed_axis = smoothed_axis + alpha * (aim_dir - smoothed_axis)
            smoothed_axis /= np.linalg.norm(smoothed_axis)
            tilt_x, tilt_y = axis_jitter(t, self.rng_t, self.jitter_amp)
            axis = apply_tilt(smoothed_axis, tilt_x, tilt_y)
            axis_err = math.acos(max(-1.0, min(1.0, float(axis @ aim_dir))))
            los_rate = 0.0
            if last_dir is not None and t - last_t > 1e-4:
                los_rate = math.acos(
                    max(-1.0, min(1.0, float(last_dir @ aim_dir)))
                ) / (t - last_t)
            last_dir, last_t = aim_dir, t
            # ---- 火控（1° 即打 / 2°+0.2s 兜底，单发闩锁） ----
            if fired or not (RANGE_MIN <= range_est <= RANGE_MAX):
                hold_start = None
                continue
            if los_rate > LOS_RATE_GATE:
                hold_start = None
                continue
            if axis_err <= PRIMARY_RAD:
                fired = True
            elif axis_err <= WINDOW_RAD:
                if hold_start is None:
                    hold_start = t
                elif t - hold_start >= WINDOW_HOLD_S - 1e-9:
                    fired = True
            else:
                hold_start = None
            if fired:
                fire_t, fire_range, fire_axis_err = t, range_est, axis_err
                fire_dir = aim_dir
                fire_axis_smoothed = smoothed_axis.copy()
                fire_pursuer = (p_pos, v_own)
                break
        if not fired:
            return {"fired": False}
        # ---- 到达判定（对每个触发延迟分别评估） ----
        results = {}
        for delay in self.delays:
            t_fire = fire_t
            # 发射时刻本机状态（延迟后）
            self.rng_t.gauss = self.np_rng.normal()
            p_dep = pursuer_jitter_position(t_fire + delay, self.rng_t)
            rel_dep_true, _ = target_relative(t_fire + delay, self.motion, self.speed, self.rng_t)
            rng_dep = float(np.linalg.norm(rel_dep_true))
            tof = rng_dep / MUZZLE_SPEED
            t_arr = t_fire + delay + tof
            rel_arr, _ = target_relative(t_arr, self.motion, self.speed, self.rng_t)
            # 发射方向：网沿发射(延迟后)时刻的机体轴飞 —— 用平滑跟踪轴
            # 加当时刻抖动；延迟 0.5s 内瞄准持续，简化用平滑轴+新抖动
            tilt_x, tilt_y = axis_jitter(t_fire + delay, self.rng_t, self.jitter_amp)
            shot_dir = apply_tilt(fire_axis_smoothed, tilt_x, tilt_y)
            p_vel_dep = (pursuer_jitter_position(t_fire + delay + 1e-3, self.rng_t) - p_dep) / 1e-3
            net = p_dep + (p_vel_dep + MUZZLE_SPEED * shot_dir) * tof
            target_abs = p_dep + rel_arr
            miss = target_abs - net
            along = float(miss @ shot_dir)
            lateral = miss - along * shot_dir
            # 覆盖面正交基
            ref = np.array([0.0, 0.0, 1.0]) if abs(shot_dir[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
            u = np.cross(shot_dir, ref)
            u /= np.linalg.norm(u)
            v = np.cross(shot_dir, u)
            lx, ly = abs(float(lateral @ u)), abs(float(lateral @ v))
            results[delay] = {
                "hit": lx <= COVERAGE_HALF and ly <= COVERAGE_HALF,
                "lateral": math.hypot(lx, ly),
                "range_at_depart": rng_dep,
            }
        return {
            "fired": True, "t": fire_t, "range": fire_range,
            "axis_err_deg": math.degrees(fire_axis_err), **results,
        }


def main():
    motions = ["static", "straight", "accel", "circle"]
    speeds = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    rounds = 15
    base_seed = 20261011
    print(f"网枪数值冒烟：{rounds} 轮/格，抖动 ±3°，观测噪声 σ={0.04}m，深度≤10m\n")
    header = (
        f"{'运动':<8}{'速度':>5} {'触发':>6} {'t_fire均值':>9} {'触发误差°':>9} "
        f"{'命中@0s':>8} {'横偏@0s':>8} {'命中@0.5s':>9} {'横偏@0.5s':>9}"
    )
    print(header)
    print("-" * len(header))
    totals = {"fire": 0, "hit0": 0, "hit05": 0, "n": 0}
    for motion in motions:
        for speed in speeds:
            if motion == "static" and speed > 0:
                continue
            fired_n = 0
            t_fire, err_fire = [], []
            hits0, hits05, lat0, lat05 = [], [], [], []
            for k in range(rounds):
                result = Round(motion, speed, base_seed + k * 131 + int(speed * 10)).run()
                totals["n"] += 1
                if result["fired"]:
                    fired_n += 1
                    totals["fire"] += 1
                    t_fire.append(result["t"])
                    err_fire.append(result["axis_err_deg"])
                    r0 = result[0.0]
                    r5 = result[0.5]
                    hits0.append(r0["hit"])
                    lat0.append(r0["lateral"])
                    hits05.append(r5["hit"])
                    lat05.append(r5["lateral"])
                    if r0["hit"]:
                        totals["hit0"] += 1
                    if r5["hit"]:
                        totals["hit05"] += 1
            fire_rate = fired_n / rounds
            mean_t = np.mean(t_fire) if t_fire else float("nan")
            mean_err = np.mean(err_fire) if err_fire else float("nan")
            hit0 = np.mean(hits0) if hits0 else float("nan")
            hit05 = np.mean(hits05) if hits05 else float("nan")
            l0 = np.mean(lat0) if lat0 else float("nan")
            l05 = np.mean(lat05) if lat05 else float("nan")
            print(
                f"{motion:<8}{speed:>5.1f} {fire_rate:>6.0%} {mean_t:>9.1f} "
                f"{mean_err:>9.2f} {hit0:>8.0%} {l0:>8.2f} {hit05:>9.0%} {l05:>9.2f}"
            )
    n = totals["n"]
    print("-" * len(header))
    print(
        f"总计 {n} 轮：触发率 {totals['fire'] / n:.0%}，"
        f"命中率 触发延迟0s {totals['hit0'] / max(totals['fire'], 1):.0%} / "
        f"0.5s {totals['hit05'] / max(totals['fire'], 1):.0%}"
    )
    print("判据（方案 §7 占位）：触发 ≥90%，命中（0s 延迟）≥80%")


if __name__ == "__main__":
    sys.exit(main())
