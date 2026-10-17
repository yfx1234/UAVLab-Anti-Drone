#!/usr/bin/env python3
"""zed_track_filter_node:目标三维滤波节点(M2)。

订阅感知节点的 target_json(机体坐标观测,深度/伪测距双模式),用飞控姿态
转到 NED 世界系,喂入六状态匀速卡尔曼滤波(复用 falcon_intercept 的
ConstantVelocityKalmanFilter),以固定频率输出"外推到当前时刻"的目标
位置/速度,供跟踪控制节点使用。

坐标链: 相机光学系 --外参--> 机体FLU --(x,-y,-z)--> 机体FRD --姿态四元数--> NED。

时间源说明:ZED 图像时间戳与 ROS 时钟同为 UTC 纪元,直接喂滤波;
PX4 姿态消息的时间戳是飞控启动时钟,不能跨域配对,因此姿态按
"接收时刻最近"使用(延迟在数十毫秒量级)。
"""

from __future__ import annotations

import json
import math
from collections import deque

import numpy as np

from falcon_intercept.target_kf_node import ConstantVelocityKalmanFilter


# ---------------------------------------------------------------------------
# 纯函数区(单元测试覆盖)
# ---------------------------------------------------------------------------

def quat_wxyz_to_matrix(quaternion_wxyz):
    """wxyz 四元数 -> 3x3 旋转矩阵(主动旋转,矩阵作用于列向量)。"""
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1.0e-12 or not math.isfinite(norm):
        raise ValueError("quaternion must be finite and non-zero")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def body_flu_to_frd(point_flu):
    """机体 前左上 -> 前右下:前不变,左右/上下取反。"""
    x, y, z = (float(value) for value in point_flu)
    return np.array([x, -y, -z], dtype=float)


def transform_body_to_ned(quaternion_wxyz, point_flu):
    """机体FLU 点 -> NED 世界系(姿态四元数为机体FRD相对NED)。"""
    return quat_wxyz_to_matrix(quaternion_wxyz) @ body_flu_to_frd(point_flu)


def observation_absolute_position(quaternion_wxyz, point_flu, own_position_ned):
    """相对观测 -> 目标绝对 NED 位置:自身位置 + 姿态旋转后的相对向量。

    滤波器维护的是目标绝对位置;漏加自身位置时,观测会随机体平移,
    速度估计被机体自身运动污染(2026-10-07 仿真诊断结论)。
    """
    relative = transform_body_to_ned(quaternion_wxyz, point_flu)
    return np.array(
        [relative[axis] + float(own_position_ned[axis]) for axis in range(3)],
        dtype=float,
    )


def observation_sigma(mode, range_m, focal_px, pixel_sigma_px,
                      depth_base_m, depth_scale, pseudo_base_m, pseudo_scale):
    """按观测模式给出 [x,y,z] 各轴 1-sigma(米)。

    depth:横向噪声 = 像素噪声角度投影;深度噪声随距离线性增长。
    pseudo:各向同性,比例 + 基数,远大于深度模式。
    """
    z = max(0.2, float(range_m))
    if mode == "depth":
        lateral = float(pixel_sigma_px) / max(1.0, float(focal_px)) * z
        depth = float(depth_base_m) + float(depth_scale) * z
        return np.array([lateral, lateral, depth], dtype=float)
    scale = float(pseudo_scale)
    base = float(pseudo_base_m)
    sigma = base + scale * z
    return np.array([sigma, sigma, sigma], dtype=float)


def extrapolate_state(state, covariance, dt_s, process_accel_variance):
    """只读外推:返回 (x(t+dt), P(t+dt)),不修改滤波器内部状态。"""
    dt = float(dt_s)
    if dt <= 0.0:
        return np.asarray(state, dtype=float).copy(), np.asarray(
            covariance, dtype=float).copy()
    transition = np.eye(6, dtype=float)
    transition[0:3, 3:6] = np.eye(3, dtype=float) * dt
    variance = float(process_accel_variance)
    noise = np.zeros((6, 6), dtype=float)
    identity = np.eye(3, dtype=float)
    noise[0:3, 0:3] = identity * variance * dt ** 4 / 4.0
    noise[0:3, 3:6] = identity * variance * dt ** 3 / 2.0
    noise[3:6, 0:3] = identity * variance * dt ** 3 / 2.0
    noise[3:6, 3:6] = identity * variance * dt ** 2
    state_now = transition @ np.asarray(state, dtype=float)
    covariance_now = (
        transition @ np.asarray(covariance, dtype=float) @ transition.T + noise
    )
    return state_now, 0.5 * (covariance_now + covariance_now.T)


def parse_target_message(text):
    """解析感知节点 target_json,返回 (stamp_s, mode, body_xyz, range_m);无目标时 mode='none'。"""
    try:
        payload = json.loads(str(text))
    except (ValueError, TypeError):
        return None, "invalid", None, None
    mode = str(payload.get("mode", "none"))
    if mode == "none":
        return float(payload.get("stamp_s", 0.0)), "none", None, None
    body = payload.get("body_xyz")
    if not isinstance(body, (list, tuple)) or len(body) != 3:
        return float(payload.get("stamp_s", 0.0)), "invalid", None, None
    try:
        position = [float(value) for value in body]
    except (TypeError, ValueError):
        return float(payload.get("stamp_s", 0.0)), "invalid", None, None
    if not all(math.isfinite(value) for value in position):
        return float(payload.get("stamp_s", 0.0)), "invalid", None, None
    stamp = float(payload.get("stamp_s", 0.0))
    if not math.isfinite(stamp) or stamp <= 0.0:
        return None, "invalid", None, None
    try:
        range_m = float(payload.get("range_m", 0.0))
    except (TypeError, ValueError):
        range_m = 0.0
    return stamp, mode, position, range_m


# ---------------------------------------------------------------------------
# 节点
# ---------------------------------------------------------------------------

def main(args=None):
    import rclpy
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    class ZedTrackFilterNode(Node):
        def __init__(self):
            super().__init__("zed_track_filter_node")
            self.declare_parameter("rate_hz", 20.0)
            self.declare_parameter("target_topic", "/zed_perception_node/target_json")
            self.declare_parameter("attitude_topic", "/fmu/out/vehicle_attitude")
            self.declare_parameter("local_position_topic", "/fmu/out/vehicle_local_position")
            self.declare_parameter("track_topic", "/zed_tracking/track_json")
            self.declare_parameter("process_accel_variance", 16.0)
            self.declare_parameter("initial_velocity_variance", 25.0)
            self.declare_parameter("innovation_gate_nis", 21.7)
            self.declare_parameter("reset_after_gap_s", 1.0)
            self.declare_parameter("hold_timeout_s", 1.2)
            self.declare_parameter("minimum_hits", 3)
            self.declare_parameter("maximum_covariance_trace", 9.0)
            self.declare_parameter("attitude_max_age_s", 0.3)
            self.declare_parameter("position_max_age_s", 0.3)
            self.declare_parameter("focal_length_px", 1266.5)
            self.declare_parameter("pixel_sigma_px", 2.0)
            self.declare_parameter("depth_sigma_base_m", 0.08)
            self.declare_parameter("depth_sigma_scale", 0.03)
            self.declare_parameter("pseudo_sigma_base_m", 0.5)
            self.declare_parameter("pseudo_sigma_scale", 0.25)
            number = lambda name: float(self.get_parameter(name).value)  # noqa: E731

            self._rate_hz = max(1.0, number("rate_hz"))
            self._attitude_max_age = number("attitude_max_age_s")
            self._position_max_age = number("position_max_age_s")
            self._hold_timeout = number("hold_timeout_s")
            self._minimum_hits = int(self.get_parameter("minimum_hits").value)
            self._maximum_trace = number("maximum_covariance_trace")
            self._focal = number("focal_length_px")
            self._pixel_sigma = number("pixel_sigma_px")
            self._depth_base = number("depth_sigma_base_m")
            self._depth_scale = number("depth_sigma_scale")
            self._pseudo_base = number("pseudo_sigma_base_m")
            self._pseudo_scale = number("pseudo_sigma_scale")

            self._filter = ConstantVelocityKalmanFilter(
                process_accel_variance=number("process_accel_variance"),
                initial_velocity_variance=number("initial_velocity_variance"),
                innovation_gate_nis=number("innovation_gate_nis"),
                reset_after_gap_s=number("reset_after_gap_s"),
            )
            self._attitudes = deque(maxlen=300)
            self._positions = deque(maxlen=300)
            self._last_mode = "none"
            self._last_result = ""
            self._accepted = 0
            self._rejected = 0

            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("target_topic").value),
                self._on_target,
                sensor_qos,
            )
            self.create_subscription(
                VehicleAttitude,
                str(self.get_parameter("attitude_topic").value),
                self._on_attitude,
                sensor_qos,
            )
            self.create_subscription(
                VehicleLocalPosition,
                str(self.get_parameter("local_position_topic").value),
                self._on_local_position,
                sensor_qos,
            )
            self._track_publisher = self.create_publisher(
                String, str(self.get_parameter("track_topic").value), 10
            )
            self.create_timer(1.0 / self._rate_hz, self._publish_track)
            self.get_logger().warning("zed_track_filter ready (cv filter, dual-mode obs)")

        def _now_s(self):
            return self.get_clock().now().nanoseconds / 1.0e9

        def _on_attitude(self, message):
            self._attitudes.append(
                (self._now_s(), tuple(float(value) for value in message.q[:4]))
            )

        def _on_local_position(self, message):
            self._positions.append(
                (self._now_s(), (float(message.x), float(message.y), float(message.z)))
            )

        def _sample_for(self, cache, stamp_s, max_age_s):
            if not cache:
                return None
            best = min(cache, key=lambda item: abs(item[0] - stamp_s))
            if abs(best[0] - stamp_s) > max_age_s + 0.2:
                return None
            return best[1]

        def _attitude_for(self, stamp_s):
            return self._sample_for(self._attitudes, stamp_s, self._attitude_max_age)

        def _position_for(self, stamp_s):
            return self._sample_for(
                self._positions, stamp_s, self._position_max_age)

        def _on_target(self, message):
            stamp_s, mode, body_flu, range_m = parse_target_message(message.data)
            if mode == "none":
                self._last_mode = "none"
                return
            if mode == "invalid" or body_flu is None:
                return
            quaternion = self._attitude_for(stamp_s)
            own_position = self._position_for(stamp_s)
            if quaternion is None or own_position is None:
                self._last_result = "attitude_or_position_unavailable"
                return
            position_ned = observation_absolute_position(
                quaternion, body_flu, own_position)
            sigma = observation_sigma(
                mode, range_m, self._focal, self._pixel_sigma,
                self._depth_base, self._depth_scale,
                self._pseudo_base, self._pseudo_scale,
            )
            covariance = np.diag(sigma * sigma)
            if not self._filter.initialized:
                self._filter.reset(position_ned, stamp_s, covariance)
                self._last_result = "initialized"
                self._last_mode = mode
                return
            result = self._filter.update(position_ned, stamp_s, covariance)
            self._last_mode = mode
            self._last_result = result.reason
            if result.accepted:
                self._accepted += 1
            else:
                self._rejected += 1
                if result.reason == "innovation_gate_rejected":
                    self.get_logger().warning(
                        f"obs rejected (nis gate): mode={mode} "
                        f"nis={result.innovation_nis or -1.0:.1f}"
                    )

        def _publish_track(self):
            now = self._now_s()
            payload = {
                "stamp_s": now,
                "valid": False,
                "reason": "not_initialized",
                "pos_ned": None,
                "vel_ned": None,
                "covariance_trace": None,
                "age_s": None,
                "last_mode": self._last_mode,
                "accepted": self._accepted,
                "rejected": self._rejected,
                "last_update_reason": self._last_result,
            }
            if self._filter.initialized:
                dt = max(0.0, now - (self._filter.timestamp_s or now))
                state, covariance = extrapolate_state(
                    self._filter.state, self._filter.covariance, dt,
                    self._filter.process_accel_variance,
                )
                health = self._filter.health(
                    now, self._hold_timeout, self._minimum_hits,
                    self._maximum_trace,
                )
                payload.update(
                    {
                        "valid": health["valid"],
                        "reason": health["reason"],
                        "pos_ned": [float(value) for value in state[0:3]],
                        "vel_ned": [float(value) for value in state[3:6]],
                        "covariance_trace": health["position_covariance_trace"],
                        "age_s": health["age_s"],
                    }
                )
            text = json.dumps(payload, separators=(",", ":"))
            message = String()
            message.data = text
            self._track_publisher.publish(message)

    rclpy.init(args=args)
    node = None
    try:
        node = ZedTrackFilterNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
