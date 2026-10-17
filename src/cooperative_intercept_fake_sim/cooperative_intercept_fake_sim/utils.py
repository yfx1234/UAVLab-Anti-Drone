"""通用数学工具：向量/四元数/旋转矩阵（NED 约定）。

坐标系约定（与原项目一致，frame_id = map_ned）：
    NED：x 前、y 右、z 下；重力加速度 +g 沿 z 向下。
    旋转矩阵 R 为 body -> NED 的方向余弦矩阵，其列为机体轴在 NED 中的分量。
    四元数使用 Hamilton 约定 q = [w, x, y, z]，表示 body -> NED 旋转。
"""

import numpy as np

GRAVITY = 9.81  # NED 中重力加速度标量（方向 +z，即向下）


def vec3(x):
    """转成 (3,) float64 数组。"""
    return np.asarray(x, dtype=float).reshape(3)


def norm(v):
    return float(np.linalg.norm(vec3(v)))


def normalize(v, eps=1e-9):
    v = vec3(v)
    n = np.linalg.norm(v)
    if n < eps:
        return np.zeros(3)
    return v / n


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def clamp_f(x):
    """夹到 [0, 1]（包内唯一实现，hybrid_sim/metrics/wingman_offboard 共用）。"""
    return max(0.0, min(1.0, float(x)))


def safe_atan2(y, x, fallback=0.0):
    """atan2，x^2+y^2 过小时返回 fallback，避免方向噪声。"""
    if x * x + y * y < 1e-12:
        return fallback
    return float(np.arctan2(y, x))


def skew(w):
    """反对称矩阵 [w]x。"""
    w = vec3(w)
    return np.array([
        [0.0, -w[2], w[1]],
        [w[2], 0.0, -w[0]],
        [-w[1], w[0], 0.0],
    ])


def vee(M):
    """反对称矩阵的 vee 运算（取回向量）。"""
    return np.array([M[2, 1], M[0, 2], M[1, 0]])


def quat_mul(q1, q2):
    """四元数乘法（Hamilton）。"""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_normalize(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_from_yaw(yaw):
    """由 yaw 构造单位四元数（水平姿态）。"""
    return np.array([np.cos(yaw / 2.0), 0.0, 0.0, np.sin(yaw / 2.0)])


def quat_to_R(q):
    """四元数 -> 旋转矩阵（body -> NED）。"""
    q = quat_normalize(q)
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
