"""简化 SE(3) 几何控制器（Simplified Geometric SE(3) Controller）。

实现经典的几何跟踪控制思想（Lee, Leok & McClamroch 的简化版），
但刻意保持结构简单、适合虚拟仿真：

输入：
    p, v         当前 3D 位置 / 速度（NED）
    R            当前姿态（body -> NED 旋转矩阵）
    p_d, v_d     参考位置 / 速度
    a_d          参考加速度（前馈）
    yaw_d        期望航向（速度方向投影后的朝向）

处理链路：
    1) 位置环（PD + 前馈）：
           a_cmd = a_d + Kp*(p_d - p) + Kv*(v_d - v)      （幅值限幅）
    2) 重力补偿得到期望推力加速度向量（NED 中重力沿 +z 向下）：
           a_T = a_cmd - [0, 0, g]
    3) 期望姿态：机体 -z 轴对准推力方向，
           z_b_des = -normalize(a_T)
       期望航向 h = [cos(yaw_d), sin(yaw_d), 0] 投影到垂直于 z_b_des 的平面：
           x_b_des = normalize(h - z_b_des*(h . z_b_des))
           y_b_des = cross(z_b_des, x_b_des)
       得期望姿态矩阵 R_des = [x_b_des, y_b_des, z_b_des]
    4) 姿态误差（几何控制 vee 运算）：
           e_R = 0.5 * vee(R_des^T R - R^T R_des)
       机体角速率指令（限幅到 max_body_rate）：
           omega_cmd = -KR * e_R
    5) 输出：a_T（供虚拟动力学积分）、R_des、omega_cmd、
       thrust_norm = |a_T| / max_thrust_accel（0~1 归一化推力）

整个链路：上层协同制导 -> 参考轨迹 -> SE(3) 控制器 -> 虚拟刚体动力学，
在本仿真中真实参与每个控制周期的状态更新。
"""

import numpy as np

from .utils import GRAVITY, clamp, normalize, skew, vee


class SE3Controller:
    def __init__(self, cfg_pursuer):
        self.kp = float(cfg_pursuer['se3_position_gain'])
        self.kv = float(cfg_pursuer['se3_velocity_gain'])
        self.kr = float(cfg_pursuer['se3_attitude_gain'])
        # 指令加速度幅值上限（平滑性限制）与推力上限（机动能力）解耦：
        # a_cmd 截断用 max_command_accel，推力饱和用 max_thrust_accel。
        self.max_accel = float(cfg_pursuer.get('max_command_accel_mps2',
                                               cfg_pursuer['max_thrust_accel_mps2']))
        self.max_thrust = float(cfg_pursuer['max_thrust_accel_mps2'])
        self.max_body_rate = float(cfg_pursuer['max_body_rate_rad_s'])
        self.gravity = GRAVITY

    def compute(self, p, v, R, p_d, v_d, a_d, yaw_d):
        """返回控制指令字典（详见类注释）。"""
        # ---- 1. 位置环：期望加速度（PD + 参考前馈），幅值限幅 ----
        a_cmd = a_d + self.kp * (p_d - p) + self.kv * (v_d - v)
        m = np.linalg.norm(a_cmd)
        if m > self.max_accel:
            a_cmd = a_cmd * (self.max_accel / m)

        # ---- 2. 重力补偿：期望推力加速度向量 ----
        a_T = a_cmd - np.array([0.0, 0.0, self.gravity])

        # ---- 3. 期望姿态 ----
        z_b_des = -normalize(a_T)  # 机体 -z 对准推力方向
        if np.linalg.norm(z_b_des) < 0.5:
            # 数值保护：推力接近零（失重），保持水平姿态
            z_b_des = np.array([0.0, 0.0, 1.0])
        # 期望航向投影到水平面：先投影到垂直于 z_b_des 的平面再归一化
        h = np.array([np.cos(yaw_d), np.sin(yaw_d), 0.0])
        x_b_des = normalize(h - z_b_des * float(h @ z_b_des))
        if np.linalg.norm(x_b_des) < 0.5:
            x_b_des = normalize(np.cross(np.array([0.0, 1.0, 0.0]), z_b_des))
        y_b_des = np.cross(z_b_des, x_b_des)
        R_des = np.column_stack((x_b_des, y_b_des, z_b_des))

        # ---- 4. 姿态误差 -> 机体角速率指令（几何控制）----
        # e_R = 0.5 * (R_des^T R - R^T R_des) 的 vee 运算，
        # 即跟踪姿态误差在机体系中的旋转向量（小角度近似）。
        e_R = 0.5 * vee(R_des.T @ R - R.T @ R_des)
        omega_cmd = -self.kr * e_R
        omega_cmd = np.clip(omega_cmd, -self.max_body_rate, self.max_body_rate)

        thrust_norm = clamp(np.linalg.norm(a_T) / self.max_thrust, 0.0, 1.0)
        return {
            'a_cmd': a_cmd,
            'a_T': a_T,
            'R_des': R_des,
            'omega_cmd': omega_cmd,
            'e_R': e_R,
            'thrust_norm': thrust_norm,
        }
