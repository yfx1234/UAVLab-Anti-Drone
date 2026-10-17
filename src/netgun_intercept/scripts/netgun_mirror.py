#!/usr/bin/env python3
"""netgun 链 ROS 算法的一比一 Python 镜像（无 ROS/Gazebo 依赖）。

复刻对象与对应源（参数名与 ROS 参数一致，行为逐条对齐）：
  LogicalDepthCamera    ← px4_offboard_tracking/logical_camera_simulator.cpp
  RelativeEstimatorMirror ← px4_offboard_tracking/relative_estimator.cpp
                            + relative_ekf.cpp（IMM CV/CA/CT 10 维）
  SimpleBelowTrackerMirror ← simple_below_tracker.cpp + 节点状态机核心
  PursuerDynamics        ← 机体速度环+姿态一阶滞后（抖动由湍流自然产生）
  NetgunStrikeMirror     ← netgun_intercept/netgun_strike_node.cpp
                            + netgun_fire_control.cpp（逐分支同构）
  InstantRefereeMirror   ← virtual_interception_referee（instant_coverage_mode）

用途：参数快速迭代与数值验收；结论按同名参数 1:1 迁移 Gazebo/板端。
"""

from __future__ import annotations

import math

import numpy as np

# ============ 公共几何（NED；光轴=body −Z=攻击轴） ============
SQRT2_2 = 0.70710678118654752440


def quat_wxyz_to_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def tilt_to_point(axis_desired):
    """解算让攻击轴指向 axis_desired 的倾斜角（小角闭式，限幅 max_deg）。"""
    a = axis_desired / np.linalg.norm(axis_desired)
    if a[2] > -1e-6:   # 需要指向下方/水平时不可达（本构型目标总在上方）
        a = np.array([a[0], a[1], -1e-6]); a /= np.linalg.norm(a)
    tilt_y = -math.asin(max(-1.0, min(1.0, a[0])))
    tilt_x = math.asin(max(-1.0, min(1.0, a[1] / math.cos(tilt_y))))
    return tilt_x, tilt_y


def axis_from_tilt(tilt_x, tilt_y):
    """倾斜角（rad，绕 N x / E y）→ 攻击轴（body −Z 在 NED）。"""
    r = quat_wxyz_to_matrix([
        math.cos(tilt_x / 2) * math.cos(tilt_y / 2),
        math.sin(tilt_x / 2) * math.cos(tilt_y / 2),
        math.cos(tilt_x / 2) * math.sin(tilt_y / 2),
        -math.sin(tilt_x / 2) * math.sin(tilt_y / 2),
    ])
    return r @ np.array([0.0, 0.0, -1.0])


# ============ 仿真相机（logical_camera_simulator 同构） ============
class LogicalDepthCamera:
    """上向安装（q_body_to_optical=(0,1,0,0)，2026-10-13 实物安装定案：
    宽轴 H74°/u 轴沿机头），FOV 半角对齐 ZED X Mini 4mm HD1080。"""

    def __init__(self, fx=900.0, fy=900.0, cx=640.0, cy=360.0,
                 width=1280, height=720, rate_hz=30.0,
                 pixel_noise_stddev_px=1.5, depth_noise_stddev_m=0.10,
                 minimum_depth_m=0.05, maximum_depth_m=10.0, latency_s=0.2):
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        self.width, self.height = width, height
        self.dt = 1.0 / rate_hz
        self.pixel_sigma = pixel_noise_stddev_px
        self.depth_sigma = depth_noise_stddev_m
        self.min_depth, self.max_depth = minimum_depth_m, maximum_depth_m
        self.latency = latency_s

    def observe(self, rel_ned, tilt_x, tilt_y, rng):
        """真值相对位置+机体倾斜 → DepthObservation（可见性含 FOV/深度门）。"""
        r_body_from_ned = quat_wxyz_to_matrix([
            math.cos(tilt_x / 2) * math.cos(tilt_y / 2),
            math.sin(tilt_x / 2) * math.cos(tilt_y / 2),
            math.cos(tilt_x / 2) * math.sin(tilt_y / 2),
            -math.sin(tilt_x / 2) * math.sin(tilt_y / 2)]).T
        p_body = r_body_from_ned @ rel_ned
        # 安装映射（2026-10-13 实物安装，与 camera_model.cpp 同步）：
        # optical = [body_x, -body_y, -body_z]（宽轴 u=机头，v=左舷）
        p_optical = np.array([p_body[0], -p_body[1], -p_body[2]])
        z = p_optical[2]
        visible = (
            0.0 < z
            and abs(p_optical[0]) <= abs(z) * math.tan(0.6458)   # H74° 半角
            and abs(p_optical[1]) <= abs(z) * math.tan(0.3927)   # V45° 半角
            and z > self.min_depth
        )
        depth_valid = visible and z <= self.max_depth
        if not visible:
            return None
        sigma_x = self.pixel_sigma * z / self.fx
        sigma_y = self.pixel_sigma * z / self.fy
        noisy = p_optical + np.array([
            rng.normal(0.0, sigma_x), rng.normal(0.0, sigma_y),
            rng.normal(0.0, self.depth_sigma)])
        return {
            "position_optical": noisy,
            "depth_valid": depth_valid,
            "visible": True,
            "stamp_late_s": self.latency,
        }


# ============ IMM 估计器（relative_ekf 三模同构） ============
class ImmEkf:
    def __init__(self, acceleration_noise_stddev_mps2=2.0,
                 turn_rate_process_noise_stddev_rads=0.5,
                 initial_position_stddev_m=1.0, initial_velocity_stddev_mps=3.0,
                 initial_acceleration_stddev_mps2=3.0,
                 initial_turn_rate_stddev_rads=0.3,
                 innovation_gate_mahalanobis=16.266,
                 imm_probability_cv_to_cv=0.97, imm_probability_ca_to_ca=0.95,
                 imm_probability_ct_to_ct=0.95,
                 imm_initial_cv_probability=0.6, imm_initial_ct_probability=0.15):
        self.config = dict(acc=acceleration_noise_stddev_mps2 ** 2,
                           turn=turn_rate_process_noise_stddev_rads,
                           gate=innovation_gate_mahalanobis)
        initial_ca = 1.0 - imm_initial_cv_probability - imm_initial_ct_probability
        diag = np.concatenate([
            np.full(3, initial_position_stddev_m ** 2),
            np.full(3, initial_velocity_stddev_mps ** 2),
            np.full(3, initial_acceleration_stddev_mps2 ** 2),
            [initial_turn_rate_stddev_rads ** 2]])
        self.models = [{"x": None, "p": np.diag(diag)} for _ in range(3)]
        for m in self.models:
            m["p0"] = np.diag(diag).copy()
        self.tpm = np.array([
            [imm_probability_cv_to_cv, (1 - imm_probability_cv_to_cv) / 2,
             (1 - imm_probability_cv_to_cv) / 2],
            [(1 - imm_probability_ca_to_ca) / 2, imm_probability_ca_to_ca,
             (1 - imm_probability_ca_to_ca) / 2],
            [(1 - imm_probability_ct_to_ct) / 2, (1 - imm_probability_ct_to_ct) / 2,
             imm_probability_ct_to_ct]])
        self.mu = np.array([imm_initial_cv_probability, initial_ca,
                            imm_initial_ct_probability])
        self.t_last = None

    def update(self, z_ned, r_cov, t):
        if self.models[0]["x"] is None:
            for m in self.models:
                x = np.zeros(10)
                x[:3] = z_ned
                m["x"] = x
                m["p"] = m["p0"].copy()
            self.t_last = t
            return True
        dt = t - self.t_last
        if dt <= 0.0:
            return False
        self.t_last = t
        mixed = []
        for dst in range(3):
            nrm = float(self.mu @ self.tpm[:, dst])
            x = np.zeros(10)
            for src in range(3):
                x += self.mu[src] * self.tpm[src, dst] / nrm * self.models[src]["x"]
            p = np.zeros((10, 10))
            for src in range(3):
                w = self.mu[src] * self.tpm[src, dst] / nrm
                d = self.models[src]["x"] - x
                p += w * (self.models[src]["p"] + np.outer(d, d))
            mixed.append({"x": x, "p": p, "p0": self.models[0]["p0"]})
        self.models = mixed
        self.mu = self.mu @ self.tpm
        h = np.zeros((3, 10))
        h[:, :3] = np.eye(3)
        lik = np.zeros(3)
        for kind, m in enumerate(self.models):
            f = np.eye(10)
            f[0:3, 3:6] = np.eye(3) * dt
            w = m["x"][9]
            if kind != 0:
                f[0:3, 6:9] = np.eye(3) * (0.5 * dt * dt)
                f[3:6, 6:9] = np.eye(3) * dt
            if kind == 2:
                f[3, 4] = -w * dt
                f[4, 3] = w * dt
                f[3, 9] = -dt * m["x"][4]
                f[4, 9] = dt * m["x"][3]
            else:
                f[9, 9] = 0.0
                m["x"][9] = 0.0
                m["p"][9, 9] = m["p0"][9, 9]
            m["x"] = f @ m["x"]
            m["p"] = f @ m["p"] @ f.T + self._q(dt, kind)
            inn = z_ned - h @ m["x"]
            s = h @ m["p"] @ h.T + r_cov
            mah = float(inn @ np.linalg.solve(s, inn))
            if np.isfinite(mah) and mah <= self.config["gate"]:
                det = max(float(np.linalg.det(s)), 1e-18)
                lik[kind] = math.exp(-0.5 * min(mah, 100.0)) / math.sqrt(
                    det * (2 * math.pi) ** 3)
                k = m["p"] @ h.T @ np.linalg.inv(s)
                m["x"] = m["x"] + k @ inn
                i10 = np.eye(10)
                c = i10 - k @ h
                m["p"] = c @ m["p"] @ c.T + k @ r_cov @ k.T
                m["p"] = 0.5 * (m["p"] + m["p"].T)
        nrm = float(self.mu @ lik)
        if nrm > 1e-300 and np.isfinite(nrm):
            self.mu = self.mu * lik / nrm
        return bool(np.any(lik > 0))

    def _q(self, dt, kind):
        q = np.zeros((10, 10))
        v = self.config["acc"]
        dt2, dt3, dt4, dt5 = dt ** 2, dt ** 3, dt ** 4, dt ** 5
        if kind != 0:
            for i in range(3):
                q[i, i] = dt5 * v / 20
                q[i, i + 3] = dt4 * v / 8
                q[i + 3, i] = dt4 * v / 8
                q[i, i + 6] = dt3 * v / 6
                q[i + 6, i] = dt3 * v / 6
                q[i + 3, i + 3] = dt3 * v / 3
                q[i + 3, i + 6] = dt2 * v / 2
                q[i + 6, i + 3] = dt2 * v / 2
                q[i + 6, i + 6] = dt * v
        else:
            for i in range(3):
                q[i, i] = 0.25 * dt4 * v
                q[i, i + 3] = 0.5 * dt3 * v
                q[i + 3, i] = 0.5 * dt3 * v
                q[i + 3, i + 3] = dt2 * v
        if kind == 2:
            q[9, 9] = self.config["turn"] ** 2 * dt
        return q

    def estimate(self):
        x = np.zeros(10)
        for m, w in zip(self.models, self.mu):
            x += w * m["x"]
        return x[:3].copy(), x[3:6].copy(), self.mu.copy()


# ============ 估计器节点（relative_estimator 同构：光学→NED+协方差） ============
class RelativeEstimatorMirror:
    def __init__(self, camera: LogicalDepthCamera, max_data_age_s=0.5,
                 **imm_kwargs):
        self.camera = camera
        self.max_data_age_s = max_data_age_s
        self.imm = ImmEkf(**imm_kwargs)
        self.last_accepted_t = None

    def on_observation(self, obs, tilt_x, tilt_y, t):
        """obs.position_optical（含噪）→ body（官方映射）→ NED → IMM 更新。"""
        p_o = obs["position_optical"]
        p_body = np.array([p_o[0], -p_o[1], -p_o[2]])
        r_ned_body = quat_wxyz_to_matrix([
            math.cos(tilt_x / 2) * math.cos(tilt_y / 2),
            math.sin(tilt_x / 2) * math.cos(tilt_y / 2),
            math.cos(tilt_x / 2) * math.sin(tilt_y / 2),
            -math.sin(tilt_x / 2) * math.sin(tilt_y / 2)])
        rel_ned = r_ned_body @ p_body
        z = p_o[2]
        r_cov = np.diag([
            (self.camera.pixel_sigma * z / self.camera.fx) ** 2,
            (self.camera.pixel_sigma * z / self.camera.fy) ** 2,
            self.camera.depth_sigma ** 2])
        accepted = self.imm.update(rel_ned, r_cov, t)
        if accepted:
            self.last_accepted_t = t
        return accepted

    def valid(self, t):
        """同 RelativeEstimate.valid：最近测量年龄 ≤ max_data_age_s。"""
        return (self.last_accepted_t is not None
                and t - self.last_accepted_t <= self.max_data_age_s)

    def estimate(self):
        return self.imm.estimate()


# ============ 跟踪器（simple_below_tracker 律同构） ============
class SimpleBelowTrackerMirror:
    def __init__(self, horizontal_position_gain=0.6,
                 prediction_horizon_s=0.2,
                 relative_velocity_filter_time_constant_s=0.5,
                 maximum_prediction_distance_m=2.0,
                 maximum_relative_speed_mps=8.0,
                 maximum_horizontal_speed_mps=3.0,
                 maximum_horizontal_acceleration_mps2=1.5,
                 altitude_position_gain=1.0,
                 maximum_vertical_speed_mps=1.0,
                 minimum_flight_height_m=1.6,
                 takeoff_height_m=5.8,
                 minimum_target_height_above_m=1.0,
                 maximum_tracking_range_m=25.0,
                 control_rate_hz=50.0):
        self.cfg = {k: v for k, v in locals().items() if k != "self"}
        self.vel_sp = np.zeros(2)
        self.rel_pos = np.zeros(2)
        self.rel_vel = np.zeros(2)
        self.rel_valid = False
        self.prev_pos = None
        self.prev_t = None

    def observation_gate(self, rel_ned):
        rng = float(np.linalg.norm(rel_ned))
        return (self.cfg["minimum_target_height_above_m"] < rng
                <= self.cfg["maximum_tracking_range_m"]
                and -rel_ned[2] >= self.cfg["minimum_target_height_above_m"])

    def update(self, rel_ned, t, tracking_active, control_dt):
        """返回速度设定点（NED xy）；rel_ned 为 None 表示本拍无有效深度观测。"""
        if rel_ned is not None and tracking_active and self.observation_gate(rel_ned):
            pos2 = rel_ned[:2]
            if self.prev_pos is not None and t > self.prev_t > 0:
                dt = t - self.prev_t
                measured = (pos2 - self.prev_pos) / dt
                mag = float(np.linalg.norm(measured))
                if mag > self.cfg["maximum_relative_speed_mps"]:
                    measured *= self.cfg["maximum_relative_speed_mps"] / mag
                alpha = (dt / (self.cfg["relative_velocity_filter_time_constant_s"] + dt)
                         if self.cfg["relative_velocity_filter_time_constant_s"] > 0 else 1.0)
                if not self.rel_valid:
                    self.rel_vel = measured
                    self.rel_valid = True
                else:
                    self.rel_vel += alpha * (measured - self.rel_vel)
            self.prev_pos, self.prev_t = pos2.copy(), t
            self.rel_pos = pos2.copy()
        predicted = self.rel_pos.copy()
        if self.rel_valid:
            pred = self.rel_vel * self.cfg["prediction_horizon_s"]
            mag = float(np.linalg.norm(pred))
            if mag > self.cfg["maximum_prediction_distance_m"]:
                pred *= self.cfg["maximum_prediction_distance_m"] / mag
            predicted += pred
        requested = np.zeros(2)
        if tracking_active and self.rel_valid:
            requested = self.cfg["horizontal_position_gain"] * predicted
            mag = float(np.linalg.norm(requested))
            if mag > self.cfg["maximum_horizontal_speed_mps"]:
                requested *= self.cfg["maximum_horizontal_speed_mps"] / mag
        delta = requested - self.vel_sp
        mag = float(np.linalg.norm(delta))
        cap = self.cfg["maximum_horizontal_acceleration_mps2"] * control_dt
        if mag > cap:
            delta *= cap / mag
        self.vel_sp += delta
        return self.vel_sp


# ============ 火控（netgun_fire_control 逐分支同构） ============
class NetgunFireControlPy:
    def __init__(self, primary_axis_error_rad=math.radians(1.0),
                 window_axis_error_rad=math.radians(2.0), window_hold_s=0.2,
                 maximum_los_rate_rad_s=0.5,
                 effective_range_min_m=6.0, effective_range_max_m=10.0,
                 blind_shot_enabled=False, blind_shot_max_axis_error_rad=math.radians(8.0),
                 blind_shot_exit_margin_s=0.25):
        self.c = dict(locals())
        del self.c["self"]
        self.latched = False
        self.shot_count = 0
        self.hold_started = None

    def update(self, now_s, estimate_valid, range_m, range_rate_mps,
               axis_error_rad, los_rate_rad_s, authorization_passed,
               engagement_timeout=False):
        if self.latched:
            return False, "LATCHED"
        c = self.c
        base = (authorization_passed and estimate_valid
                and c["effective_range_min_m"] <= range_m <= c["effective_range_max_m"]
                and math.isfinite(axis_error_rad) and axis_error_rad >= 0.0
                and abs(los_rate_rad_s) <= c["maximum_los_rate_rad_s"])
        if not base:
            self.hold_started = None
            return False, "NONE"
        if axis_error_rad <= c["primary_axis_error_rad"]:
            self.latched = True
            self.shot_count += 1
            return True, "PRIMARY_WINDOW"
        if axis_error_rad <= c["window_axis_error_rad"]:
            if self.hold_started is None:
                self.hold_started = now_s
            if now_s - self.hold_started + 1e-9 >= c["window_hold_s"]:
                self.latched = True
                self.shot_count += 1
                return True, "WINDOW_HOLD"
            return False, "NONE"
        self.hold_started = None
        if c["blind_shot_enabled"] and axis_error_rad <= c["blind_shot_max_axis_error_rad"]:
            exit_pred = range_m + range_rate_mps * c["blind_shot_exit_margin_s"]
            exiting = (exit_pred > c["effective_range_max_m"] or
                       exit_pred < c["effective_range_min_m"])
            if exiting or engagement_timeout:
                self.latched = True
                self.shot_count += 1
                return True, ("BLIND_SHOT_TIMEOUT" if engagement_timeout
                              else "BLIND_SHOT_EXIT")
        return False, "NONE"


# ============ 裁判（instant_coverage_mode 同构） ============
def instant_referee(shot_origin_ned, shot_direction_ned, target_abs_ned,
                    coverage_half_extent_m=1.5,
                    effective_range_min_m=6.0, effective_range_max_m=10.0):
    to_target = np.asarray(target_abs_ned) - np.asarray(shot_origin_ned)
    rng = float(np.linalg.norm(to_target))
    along = float(to_target @ shot_direction_ned)
    lateral = to_target - along * shot_direction_ned
    d = shot_direction_ned
    ref = np.array([0.0, 0.0, 1.0]) if abs(d[2]) < 0.9 else np.array([1.0, 0.0, 0.0])
    u = np.cross(d, ref)
    u /= np.linalg.norm(u)
    v = np.cross(d, u)
    lu = abs(float(lateral @ u))
    lv = abs(float(lateral @ v))
    hit = (lu <= coverage_half_extent_m and lv <= coverage_half_extent_m
           and effective_range_min_m <= rng <= effective_range_max_m)
    return hit, math.hypot(lu, lv), rng
