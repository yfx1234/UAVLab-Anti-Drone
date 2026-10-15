"""虚拟刚体动力学（Virtual Vehicle Dynamics）。

为纯虚拟仿真设计的简化刚体模型，包含完整的平移 + 姿态（SE(3)）状态：
    状态量：p (位置), v (速度), q (四元数), omega (机体角速率)
    输入量：SE3Controller 输出的推力加速度向量 a_T 与角速率指令 omega_cmd

积分流程（每步 dt）：
    1. 推力饱和：|a_T| 超过 max_thrust_accel 时按比例截断（推力限制）；
    2. 速度上限硬保护：达到 v_max 后去掉沿速度方向的加速分量（保证 <= 16 m/s）；
    3. 姿态动力学：角速率一阶跟踪 omega_cmd（时间常数 attitude_tau），
       角速率限幅 max_body_rate，再积分四元数 q_dot = 0.5 q (x) omega；
    4. 平动积分：a_applied = a_T + g_vec（重力补偿后真实加速度），
       v += a*dt, p += v*dt。

注意：这里不包含电机/气动模型 —— 推力直接表现为加速度指令，
姿态也只做一阶近似跟随。这是虚拟仿真允许的简化（详见 README 仿真边界）。
"""

import numpy as np

from .utils import GRAVITY, quat_from_yaw, quat_mul, quat_normalize, skew


class VirtualVehicle:
    def __init__(self, name, cfg_pursuer, init_p, init_yaw):
        self.name = name
        self.max_speed = float(cfg_pursuer['max_speed'])
        self.max_thrust_accel = float(cfg_pursuer['max_thrust_accel_mps2'])
        self.max_body_rate = float(cfg_pursuer['max_body_rate_rad_s'])
        self.attitude_tau = float(cfg_pursuer['attitude_tau_s'])

        # SE(3) 状态：位置 / 速度 / 姿态四元数 / 机体角速率
        self.p = np.asarray(init_p, dtype=float)
        self.v = np.zeros(3)
        self.q = quat_from_yaw(init_yaw)
        self.omega = np.zeros(3)

        # 诊断量（每步记录）
        self.a_applied = np.zeros(3)
        self.thrust_saturated = False
        self.speed_limited = False

    @property
    def speed(self):
        return float(np.linalg.norm(self.v))

    def step(self, dt, a_T, omega_cmd):
        """推进一个积分步。

        a_T:      控制器给出的重力补偿后推力加速度向量（NED）
        omega_cmd:控制器给出的机体角速率指令（body 系）
        """
        # ---- 1. 推力饱和：限制推力加速度幅值 ----
        m = np.linalg.norm(a_T)
        if m > self.max_thrust_accel:
            a_T = a_T * (self.max_thrust_accel / m)
            self.thrust_saturated = True
        else:
            self.thrust_saturated = False

        # ---- 2. 速度上限硬保护 ----
        # 若已经达到最大速度且加速度沿速度方向，则去掉该分量，
        # 保证实际速度严格不超过 v_max（速度上限是硬约束）。
        self.speed_limited = False
        v2 = float(self.v @ self.v)
        v_max2 = self.max_speed * self.max_speed
        if v2 >= v_max2:
            av = float(self.v @ a_T)  # 推力沿速度方向的分量
            if av > 0.0:
                a_T = a_T - (av / v2) * self.v
                self.speed_limited = True

        # ---- 3. 姿态动力学（一阶跟踪角速率指令 + 限幅）----
        omega_cmd = np.clip(omega_cmd, -self.max_body_rate, self.max_body_rate)
        # 角速率一阶响应：tau 越小跟随越快（简化转动惯量模型）
        self.omega += (omega_cmd - self.omega) * (dt / self.attitude_tau)
        self.omega = np.clip(self.omega, -self.max_body_rate, self.max_body_rate)
        # 四元数运动学：q_dot = 0.5 * q (x) [0, omega]，再归一化
        q_dot = 0.5 * quat_mul(self.q, np.concatenate([[0.0], self.omega]))
        self.q = quat_normalize(self.q + q_dot * dt)

        # ---- 4. 平动积分（重力补偿后真实加速度 = a_T + g_vec）----
        self.a_applied = a_T + np.array([0.0, 0.0, GRAVITY])
        self.v = self.v + self.a_applied * dt
        # 离散化越界保护：积分后再压回速度上限（若仍有超出则按比例缩放）
        v_now = np.linalg.norm(self.v)
        if v_now > self.max_speed:
            self.v *= self.max_speed / v_now
        self.p = self.p + self.v * dt

    def reset(self):
        """恢复初始状态（暂时不用，保留接口）。"""
        pass
