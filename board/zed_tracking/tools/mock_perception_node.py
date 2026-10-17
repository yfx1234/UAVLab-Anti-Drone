#!/usr/bin/env python3
"""mock_perception_node:闭环仿真用的理想感知模拟器。

按脚本目标轨迹(hover/circle/line)生成目标 NED 世界位置,读取模拟机体
的位置与姿态,把目标换算到机体 FLU 坐标,以感知节点相同的话题与格式
发布 target_json(depth/pseudo 双模式 + 噪声 + 可配丢帧率),
驱动滤波与控制链路完成闭环。
"""

from __future__ import annotations

import json
import math
import random


def target_position(motion, center, radius_m, angular_rate, elapsed_s):
    """脚本目标轨迹,返回 NED 位置。"""
    center = [float(v) for v in center]
    if motion == "hover":
        return list(center)
    if motion == "circle":
        angle = float(angular_rate) * float(elapsed_s)
        return [
            center[0] + float(radius_m) * math.cos(angle),
            center[1] + float(radius_m) * math.sin(angle),
            center[2],
        ]
    if motion == "line":
        period = max(1.0, 2.0 * float(radius_m) / max(0.1, abs(float(angular_rate))))
        phase = (float(elapsed_s) % period) / period
        offset = (2.0 * phase - 1.0) * float(radius_m)
        return [center[0] + offset, center[1], center[2]]
    raise ValueError(f"unknown motion: {motion}")


def quat_to_matrix(quaternion_wxyz):
    w, x, y, z = (float(v) for v in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )


def rotate_transpose(matrix, vector):
    """R^T v(世界向量 -> 机体坐标,矩阵为机体->世界)。"""
    return [
        matrix[0][0] * vector[0] + matrix[1][0] * vector[1] + matrix[2][0] * vector[2],
        matrix[0][1] * vector[0] + matrix[1][1] * vector[1] + matrix[2][1] * vector[2],
        matrix[0][2] * vector[0] + matrix[1][2] * vector[1] + matrix[2][2] * vector[2],
    ]


def main(args=None):
    import rclpy
    from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String

    class MockPerceptionNode(Node):
        def __init__(self):
            super().__init__("mock_perception_node")
            self.declare_parameter("motion", "hover")
            self.declare_parameter("center_ned", [0.0, 0.0, -10.0])
            self.declare_parameter("radius_m", 3.0)
            self.declare_parameter("angular_rate", 0.5)
            self.declare_parameter("detect_rate_hz", 15.0)
            self.declare_parameter("depth_max_m", 10.5)
            self.declare_parameter("dropout_rate", 0.05)
            self.declare_parameter("depth_noise_base_m", 0.05)
            self.declare_parameter("depth_noise_scale", 0.03)
            self.declare_parameter("bearing_noise_deg", 0.3)
            self.declare_parameter("target_topic", "/zed_perception_node/target_json")
            number = lambda name: float(self.get_parameter(name).value)  # noqa: E731
            self._motion = str(self.get_parameter("motion").value)
            self._center = [float(v) for v in
                            self.get_parameter("center_ned").value]
            self._radius = number("radius_m")
            self._rate_rad = number("angular_rate")
            self._rate_hz = max(1.0, number("detect_rate_hz"))
            self._depth_max = number("depth_max_m")
            self._dropout = max(0.0, min(0.9, number("dropout_rate")))
            self._noise_base = number("depth_noise_base_m")
            self._noise_scale = number("depth_noise_scale")
            self._bearing_noise = math.radians(number("bearing_noise_deg"))
            self._random = random.Random(20261007)
            self._vehicle_position = [0.0, 0.0, 0.0]
            self._attitude = (1.0, 0.0, 0.0, 0.0)
            self._sequence = 0
            self._start_s = None

            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(
                VehicleLocalPosition, "/fmu/out/vehicle_local_position",
                self._on_local, sensor_qos)
            self.create_subscription(
                VehicleAttitude, "/fmu/out/vehicle_attitude",
                self._on_attitude, sensor_qos)
            self._target_pub = self.create_publisher(
                String, str(self.get_parameter("target_topic").value), 10)
            self._truth_pub = self.create_publisher(
                String, "/sim/target_truth", 10)
            self.create_timer(1.0 / self._rate_hz, self._publish)
            self.get_logger().warning(
                f"mock perception ready motion={self._motion} "
                f"center={self._center} dropout={self._dropout}")

        def _on_local(self, message):
            self._vehicle_position = [float(message.x), float(message.y),
                                      float(message.z)]

        def _on_attitude(self, message):
            self._attitude = tuple(float(v) for v in message.q[:4])

        def _now_s(self):
            return self.get_clock().now().nanoseconds / 1.0e9

        def _publish(self):
            now = self._now_s()
            if self._start_s is None:
                self._start_s = now
            if self._random.random() < self._dropout:
                return
            elapsed = now - self._start_s
            target = target_position(
                self._motion, self._center, self._radius, self._rate_rad, elapsed)
            rel_ned = [target[i] - self._vehicle_position[i] for i in range(3)]
            distance = math.sqrt(sum(v * v for v in rel_ned))
            if distance < 0.5:
                return
            rotation = quat_to_matrix(self._attitude)
            rel_frd = rotate_transpose(rotation, rel_ned)
            body_flu = [rel_frd[0], -rel_frd[1], -rel_frd[2]]
            # 距离/方向加噪。
            if distance <= self._depth_max:
                mode = "depth"
                measured_range = distance + self._random.gauss(
                    0.0, self._noise_base + self._noise_scale * distance)
            else:
                mode = "pseudo"
                measured_range = max(
                    1.0, distance * (1.0 + self._random.gauss(0.0, 0.15)))
            bearing_noise = self._random.gauss(0.0, self._bearing_noise)
            scale = math.cos(bearing_noise)
            body_flu = [
                body_flu[0] * scale - body_flu[1] * math.sin(bearing_noise),
                body_flu[0] * math.sin(bearing_noise) + body_flu[1] * scale,
                body_flu[2],
            ]
            body_flu = [v / distance * measured_range for v in body_flu]
            self._sequence += 1
            payload = {
                "stamp_s": now,
                "sequence": self._sequence,
                "mode": mode,
                "confidence": round(1.0 - self._random.random() * 0.2, 3),
                "body_xyz": [round(v, 4) for v in body_flu],
                "range_m": round(measured_range, 3),
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self._target_pub.publish(message)
            truth = String()
            truth.data = json.dumps(
                {"stamp_s": now, "pos_ned": [round(v, 4) for v in target],
                 "vel_ned": None},
                separators=(",", ":"))
            self._truth_pub.publish(truth)

    rclpy.init(args=args)
    node = None
    try:
        node = MockPerceptionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
