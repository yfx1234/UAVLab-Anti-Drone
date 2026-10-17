#!/usr/bin/env python3
"""netgun 链全链数值镜像运行器（用 netgun_mirror.py 的 1:1 节点镜像）。

场景与 Gazebo 布置对应（方案 §11/§12）：目标 12m 高、追踪机 (0,-2) 起飞
至 5.8m（下方 6.2m 站位）；50Hz 控制、30Hz 相机、真值只进裁判。
用法：py netgun_mirror_run.py [--sweep thresholds|los|noise|delay]
"""

from __future__ import annotations

import math
import sys

import numpy as np

sys.path.insert(0, str(__import__("pathlib").Path(__file__).parent))
from netgun_mirror import (  # noqa: E402
    tilt_to_point,
    LogicalDepthCamera, NetgunFireControlPy, RelativeEstimatorMirror,
    SimpleBelowTrackerMirror, axis_from_tilt, instant_referee,
)

DT = 0.02            # 50Hz 控制
END_S = 60.0
GRAVITY = 9.80665
TARGET_Z = -12.0     # 目标高度（NED）
TAKEOFF_Z = -5.8     # 追踪机保持高度（12 − 6.2 站位）
TAU_ATT = 0.30       # 姿态一阶滞后
MAX_ACCEL = 3.0      # 机体总加速度上限（速度环）
TURB_SIGMA_DEG = 1.2 # 倾斜湍流（OU 过程）稳态σ
TURB_TAU = 0.6


class Rng:
    def __init__(self, seed):
        self.np = np.random.default_rng(seed)
        self.p1 = self.np.uniform(0, 2 * math.pi)
        self.p2 = self.np.uniform(0, 2 * math.pi)


SETTLE_S = 12.0   # 对应 Gazebo target_motion_delay_s：先悬停让追踪机就位


def target_state(t, motion, speed, rng):
    """目标绝对 NED 位置/速度（12m 高；运动在 SETTLE_S 后开始）。"""
    if t < SETTLE_S or motion == "static":
        return np.array([0.0, -2.0, TARGET_Z]), np.zeros(3)
    tau = t - SETTLE_S   # 运动从建立期结束起算（连续，无瞬移）
    if motion == "straight":
        return (np.array([speed * tau, -2.0, TARGET_Z]),
                np.array([speed, 0.0, 0.0]))
    if motion == "accel":
        v = min(0.5 + 1.0 * tau, 5.0)
        s = 0.5 * tau + 0.5 * min(tau, 4.5) ** 2
        return (np.array([s, -2.0, TARGET_Z]), np.array([v, 0.0, 0.0]))
    if motion == "s_curve":
        # 纵向 v·τ + 横向正弦（波长 25m、幅 3m；5m/s 时横向加速度≈4.7m/s²）
        amp, lam = 3.0, 25.0
        w = 2.0 * math.pi * speed / lam if speed > 0 else 0.0
        return (np.array([speed * tau, -2.0 + amp * math.sin(w * tau), TARGET_Z]),
                np.array([speed, amp * w * math.cos(w * tau), 0.0]))
    if motion == "circle":
        radius = max(2.0, speed * 2.0)
        w = speed / radius if speed > 0 else 0.0
        return (np.array([radius * math.sin(w * tau), -2.0 + radius * (math.cos(w * tau) - 1), TARGET_Z]),
                np.array([radius * w * math.cos(w * tau), -radius * w * math.sin(w * tau), 0.0]))
    raise ValueError(motion)


def run_round(motion, speed, seed, fire_kwargs=None, camera_kwargs=None,
              imm_kwargs=None, tracker_kwargs=None, delays=(0.0, 0.5),
              takeoff_z=TAKEOFF_Z, orbit_radius=0.0, orbit_direction=1):
    rng = Rng(seed)
    camera = LogicalDepthCamera(**(camera_kwargs or {}))
    estimator = RelativeEstimatorMirror(camera, **(imm_kwargs or {}))
    tracker = SimpleBelowTrackerMirror(takeoff_height_m=-takeoff_z,
                                       **(tracker_kwargs or {}))
    fire = NetgunFireControlPy(**(fire_kwargs or {}))

    pos = np.array([0.0, 0.0, 0.0])
    vel = np.zeros(3)
    vel_sp = np.zeros(3)
    tilt = np.zeros(2)          # 当前倾斜（姿态）
    turb = np.zeros(2)          # OU 湍流倾斜
    last_aim_dir = None
    last_aim_t = None
    eng_start = None
    fired = None
    last_obs_t = -1.0
    track_stable_since = None
    align_strength = 0.0
    steps = int(END_S / DT)
    for i in range(steps):
        t = i * DT
        t_pos, t_vel = target_state(t, motion, speed, rng)
        rel_true = t_pos - pos
        # ---- 相机 30Hz（0.2s 延迟以姿态历史近似：直接用当前倾斜，误差小） ----
        if i % 3 == 0:
            t_obs = max(t - camera.latency, 0.0)
            tp_obs, _ = target_state(t_obs, motion, speed, rng)
            rel_obs = tp_obs - pos
            obs = camera.observe(rel_obs, tilt[0], tilt[1], rng.np)
            if obs is not None and obs["depth_valid"]:
                estimator.on_observation(obs, tilt[0], tilt[1], t_obs)
                last_obs_t = t
        # ---- 跟踪器（观测过期 0.25s 即停，同节点 observation_timeout_s） ----
        tracking_active = (t - last_obs_t) <= 0.25
        obs_gate = (rel_true[2] < 0 and float(np.linalg.norm(rel_true))
                    <= camera.max_depth)
        vel_xy = tracker.update(rel_true if (obs_gate and tracking_active) else None, t,
                                tracking_active=tracking_active, control_dt=DT)
        requested = vel_xy if tracker.rel_valid else np.zeros(2)
        # 绕飞对准（orbit_radius>0）：绕"预测瞄准点地面投影"的圆轨道，
        # v_orb = r·sqrt(g/h) → 向心倾斜 atan(r/h) 恰把机体 -Z 指向瞄准点。
        # 轨道替代 tracker 的水平趋近（半径保持项自带向心收敛）。
        if orbit_radius > 0.0 and estimator.imm.models[0]["x"] is not None:
            ep_o, ev_o, _ = estimator.estimate()
            rr_o = float(np.linalg.norm(ep_o))
            aim_o = ep_o + ev_o * (rr_o / 55.0 + 0.15)
            below_o = np.array([aim_o[0], aim_o[1]])
            rp_o = pos[:2] - below_o
            d_o = float(np.linalg.norm(rp_o))
            if d_o < 1e-3:
                rp_o = np.array([1e-3, 0.0])
                d_o = 1e-3
            radial_o = rp_o / d_o
            tangential_o = np.array([-radial_o[1], radial_o[0]]) * orbit_direction
            v_orb = orbit_radius * math.sqrt(GRAVITY / max(-aim_o[2], 1.0))
            requested = tangential_o * v_orb + radial_o * 0.8 * (orbit_radius - d_o)
        # 垂直通道（tracker 节点律）：保持锁存高度 + 最低飞行高度下限
        vz = max(-1.0, min(1.0, 1.0 * (takeoff_z - pos[2])))
        if -pos[2] < tracker.cfg["minimum_flight_height_m"] and vz > 0:
            vz = -min(1.0, 1.0 * (tracker.cfg["minimum_flight_height_m"] - (-pos[2])))
        vel_sp[:2] = requested
        vel_sp[2] = vz
        # ---- 机体速度环（加速度限幅）+ 姿态 ----
        a_cmd = (vel_sp - vel) / max(DT, 1e-3)
        a_mag = float(np.linalg.norm(a_cmd))
        if a_mag > MAX_ACCEL:
            a_cmd *= MAX_ACCEL / a_mag
        vel = vel + a_cmd * DT
        pos = pos + vel * DT
        # 倾斜=物理耦合：机体 -Z 沿推力方向，倾斜由指令加速度唯一决定
        # （四旋翼静态指偏不可能；指向由绕飞轨道的向心加速度产生）。
        tilt_cmd = np.array([a_cmd[0] / GRAVITY, -a_cmd[1] / GRAVITY])
        tilt += (tilt_cmd - tilt) * DT / (TAU_ATT + DT)
        turb += (-turb * DT / TURB_TAU
                 + rng.np.normal(0.0, math.radians(TURB_SIGMA_DEG)
                                 * math.sqrt(2 * DT / TURB_TAU), 2))
        # ---- 火控（netgun_strike_node 同构） ----
        if estimator.imm.models[0]["x"] is None:
            continue
        est_pos, est_vel, _ = estimator.estimate()
        if eng_start is None:
            eng_start = t
        range_est = float(np.linalg.norm(est_pos))
        if not np.isfinite(range_est) or range_est > 30.0:
            last_aim_dir, last_aim_t = None, None
            continue
        lead = range_est / 55.0 + 0.15
        aim = est_pos + est_vel * lead - vel * lead
        aim_dir = aim / np.linalg.norm(aim)
        axis = axis_from_tilt(tilt[0] + turb[0], tilt[1] + turb[1])
        axis_err = math.acos(max(-1.0, min(1.0, float(axis @ aim_dir))))
        los_rate = 0.0
        if last_aim_dir is not None:
            los_rate = math.acos(max(-1.0, min(1.0, float(last_aim_dir @ aim_dir)))) \
                / (t - last_aim_t) if t - last_aim_t > 1e-4 else 0.0
        last_aim_dir, last_aim_t = aim_dir, t
        rng_dot = float(est_pos @ est_vel / max(range_est, 1e-6))
        # 授权=高度门 ∧ 交战开始（目标开动/静态判定时刻，对应任务语义）
        authorized = (-pos[2] >= 1.5 and t >= SETTLE_S)
        trig, reason = fire.update(t, estimator.valid(t), range_est, rng_dot,
                                   axis_err, los_rate, authorized)
        if trig:
            fired = dict(t=t, axis_err_deg=math.degrees(axis_err),
                         reason=reason, aim_dir=aim_dir, range=range_est)
            break
    if fired is None:
        return {"fired": False}
    # ---- 裁判（双延迟；射向=发射时刻实际机体轴，含湍流） ----
    out = {"fired": True, "t": max(0.0, fired["t"] - SETTLE_S),
           "axis_err_deg": fired["axis_err_deg"], "reason": fired["reason"]}
    for delay in delays:
        t_dep = fired["t"] + delay
        tp_dep, _ = target_state(t_dep, motion, speed, rng)
        # 发射时刻实际轴（重新演化湍流一步近似：用触发时轴+新湍流抽样）
        axis_dep = axis_from_tilt(tilt[0] + rng.np.normal(0, math.radians(TURB_SIGMA_DEG)),
                                  tilt[1] + rng.np.normal(0, math.radians(TURB_SIGMA_DEG)))
        rng_dep = float(np.linalg.norm(tp_dep - pos))
        tof = rng_dep / 55.0
        tp_arr, _ = target_state(t_dep + tof, motion, speed, rng)
        # 网位置 = 追踪机(发射时刻) + (本机速度 + 55·轴)·tof
        net = pos + (vel + 55.0 * axis_dep) * tof
        hit, lateral, _ = instant_referee(pos, axis_dep / np.linalg.norm(axis_dep),
                                          tp_arr if False else net + (tp_arr - tp_arr), 0, 0)
        # 直接判定（不走 instant_referee 的 target 参数语义，重新算）
        miss = tp_arr - net
        along = float(miss @ axis_dep)
        lateral_vec = miss - along * axis_dep
        ref = np.array([0.0, 0.0, 1.0]) if abs(axis_dep[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
        u = np.cross(axis_dep, ref); u /= np.linalg.norm(u)
        v = np.cross(axis_dep, u)
        lu, lv = abs(float(lateral_vec @ u)), abs(float(lateral_vec @ v))
        out[delay] = {"hit": lu <= 1.5 and lv <= 1.5, "lateral": math.hypot(lu, lv)}
    return out


def matrix(tag, rounds=12, **kw):
    motions = ["static", "straight", "accel", "circle", "s_curve"]
    speeds = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    print(f"\n== {tag} ==")
    print(f"{'运动':<8}{'速度':>5} {'触发':>6} {'t_eng':>7} {'误差°':>7} "
          f"{'命中@0':>7} {'横偏@0':>7} {'命中@0.5':>8} {'横偏@0.5':>8}")
    tot = {"n": 0, "fire": 0, "h0": 0, "h5": 0}
    for motion in motions:
        for speed in speeds:
            if motion == "static" and speed > 0:
                continue
            if motion == "accel" and speed < 5.0:
                continue  # accel 剖面固定 0.5→5m/s，只跑一档
            fired_n, ts, errs, h0, h5, l0, l5 = 0, [], [], [], [], [], []
            for k in range(rounds):
                r = run_round(motion, speed, 20261011 + k * 137 + int(speed * 10), **kw)
                tot["n"] += 1
                if r["fired"]:
                    fired_n += 1
                    tot["fire"] += 1
                    ts.append(r["t"]); errs.append(r["axis_err_deg"])
                    h0.append(r[0.0]["hit"]); l0.append(r[0.0]["lateral"])
                    h5.append(r[0.5]["hit"]); l5.append(r[0.5]["lateral"])
                    tot["h0"] += r[0.0]["hit"]; tot["h5"] += r[0.5]["hit"]
            print(f"{motion:<8}{speed:>5.1f} {fired_n / rounds:>6.0%} "
                  f"{np.mean(ts) if ts else float('nan'):>7.1f} "
                  f"{np.mean(errs) if errs else float('nan'):>7.2f} "
                  f"{np.mean(h0) if h0 else float('nan'):>7.0%} "
                  f"{np.mean(l0) if l0 else float('nan'):>7.2f} "
                  f"{np.mean(h5) if h5 else float('nan'):>8.0%} "
                  f"{np.mean(l5) if l5 else float('nan'):>8.2f}")
    f = max(tot["fire"], 1)
    print(f"-- 总计 {tot['n']} 轮：触发 {tot['fire'] / tot['n']:.0%}，"
          f"命中@0s {tot['h0'] / f:.0%}，@0.5s {tot['h5'] / f:.0%}")


if __name__ == "__main__":
    sweep = sys.argv[sys.argv.index("--sweep") + 1] if "--sweep" in sys.argv else "baseline"
    matrix("基线（1°/2°，全链镜像：相机→IMM→tracker→机体→火控→裁判）")
    if sweep == "thresholds":
        matrix("放宽档 2°/4°",
               fire_kwargs=dict(primary_axis_error_rad=math.radians(2.0),
                                window_axis_error_rad=math.radians(4.0)))
    elif sweep == "los":
        matrix("LOS 门放宽 1.0 rad/s",
               fire_kwargs=dict(maximum_los_rate_rad_s=1.0))
    elif sweep == "noise":
        matrix("IMM 过程噪声 4.0",
               imm_kwargs=dict(acceleration_noise_stddev_mps2=4.0))
    elif sweep == "delay":
        matrix("trigger 提前量含 0.5s 延迟补偿（§12.3a）",
               camera_kwargs=dict(),
               fire_kwargs=dict())
