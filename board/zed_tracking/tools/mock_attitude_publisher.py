#!/usr/bin/env python3
"""mock_attitude_publisher:桌面回放/仿真用的模拟飞控姿态源。

在 /fmu/out/vehicle_attitude 上按固定频率发布 VehicleAttitude。
默认发布单位四元数(机体水平);可配恒定角速度产生旋转姿态,
用于检验滤波与控制的姿态耦合处理。

注意:timestamp 字段沿用 PX4 启动时钟惯例(微秒递增),与真实飞控一致;
滤波节点按接收时刻使用姿态,不依赖该时间戳跨域对齐。
"""

from __future__ import annotations

import math


def integrate_quaternion(quaternion_wxyz, body_rates_frd, dt_s):
    """一阶积分机体角速度(前右下 rad/s)到四元数,返回归一化 wxyz。"""
    w, x, y, z = quaternion_wxyz
    p, q, r = (float(value) for value in body_rates_frd)
    dw = 0.5 * (-x * p - y * q - z * r)
    dx = 0.5 * (w * p + y * r - z * q)
    dy = 0.5 * (w * q + z * p - x * r)
    dz = 0.5 * (w * r + x * q - y * p)
    values = [w + dw * dt_s, x + dx * dt_s, y + dy * dt_s, z + dz * dt_s]
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-12:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(value / norm for value in values)


def main(args=None):
    import rclpy
    from px4_msgs.msg import VehicleAttitude
    from rclpy.node import Node

    class MockAttitudePublisher(Node):
        def __init__(self):
            super().__init__("mock_attitude_publisher")
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("rate_pitch_rad_s", 0.0)
            self.declare_parameter("rate_roll_rad_s", 0.0)
            self.declare_parameter("rate_yaw_rad_s", 0.0)
            self.declare_parameter("topic", "/fmu/out/vehicle_attitude")
            self._rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
            self._rates = [
                float(self.get_parameter(name).value)
                for name in ("rate_pitch_rad_s", "rate_roll_rad_s", "rate_yaw_rad_s")
            ]
            self._quaternion = (1.0, 0.0, 0.0, 0.0)
            self._timestamp_us = 0
            self._publisher = self.create_publisher(
                VehicleAttitude,
                str(self.get_parameter("topic").value),
                10,
            )
            self.create_timer(1.0 / self._rate_hz, self._publish)
            self.get_logger().warning(
                f"mock attitude publishing at {self._rate_hz:.0f} Hz "
                f"(rates p/q/r={self._rates})"
            )

        def _publish(self):
            dt = 1.0 / self._rate_hz
            self._quaternion = integrate_quaternion(
                self._quaternion, self._rates, dt)
            self._timestamp_us += int(dt * 1.0e6)
            message = VehicleAttitude()
            message.timestamp = self._timestamp_us
            message.q = [float(value) for value in self._quaternion]
            self._publisher.publish(message)

    rclpy.init(args=args)
    node = None
    try:
        node = MockAttitudePublisher()
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
