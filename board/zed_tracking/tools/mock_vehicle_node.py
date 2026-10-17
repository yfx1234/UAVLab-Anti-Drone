#!/usr/bin/env python3
"""mock_vehicle_node:闭环仿真用的机体/飞控模拟。

消费轨迹设定值(/fmu/in/trajectory_setpoint,与真飞控一致的输入侧),
一阶速度动力学积分,发布 vehicle_local_position / vehicle_attitude /
vehicle_status / vehicle_land_detected,另发轻量 /sim/vehicle_state
供仿真分析。仿真姿态取水平 + yaw(倾斜对观测的影响按小量忽略;
该近似只影响仿真保真度,不影响链路正确性验证)。
"""

from __future__ import annotations

import json
import math


def quaternion_from_yaw(yaw):
    half = 0.5 * float(yaw)
    return (math.cos(half), 0.0, 0.0, math.sin(half))


def wrap_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def main(args=None):
    import rclpy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                              VehicleAttitude, VehicleLandDetected,
                              VehicleLocalPosition, VehicleStatus)
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                           ReliabilityPolicy)
    from std_msgs.msg import String

    class MockVehicleNode(Node):
        def __init__(self):
            super().__init__("mock_vehicle_node")
            self.declare_parameter("sim_rate_hz", 100.0)
            self.declare_parameter("velocity_tau_s", 0.35)
            self.declare_parameter("yaw_tau_s", 0.6)
            self.declare_parameter("initial_ned", [0.0, 0.0, 0.0])
            number = lambda name: float(self.get_parameter(name).value)  # noqa: E731
            self._rate = max(10.0, number("sim_rate_hz"))
            self._velocity_tau = max(0.05, number("velocity_tau_s"))
            self._yaw_tau = max(0.05, number("yaw_tau_s"))
            initial = [float(v) for v in
                       self.get_parameter("initial_ned").value]
            self._position = list(initial)
            self._velocity = [0.0, 0.0, 0.0]
            self._yaw = 0.0
            self._command_velocity = [0.0, 0.0, 0.0]
            self._command_yaw = 0.0
            self._mode_received = False
            self._timestamp_us = 0
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(
                TrajectorySetpoint, "/fmu/in/trajectory_setpoint",
                self._on_setpoint, sensor_qos)
            self.create_subscription(
                OffboardControlMode, "/fmu/in/offboard_control_mode",
                self._on_mode, sensor_qos)
            self._local_pub = self.create_publisher(
                VehicleLocalPosition, "/fmu/out/vehicle_local_position", sensor_qos)
            self._attitude_pub = self.create_publisher(
                VehicleAttitude, "/fmu/out/vehicle_attitude", sensor_qos)
            self._status_pub = self.create_publisher(
                VehicleStatus, "/fmu/out/vehicle_status", sensor_qos)
            self._land_pub = self.create_publisher(
                VehicleLandDetected, "/fmu/out/vehicle_land_detected", sensor_qos)
            self._state_pub = self.create_publisher(
                String, "/sim/vehicle_state", 10)
            period = 1.0 / self._rate
            self._dt = period
            self._tick = 0
            self.create_timer(period, self._step)
            self.get_logger().warning(
                f"mock vehicle ready rate={self._rate} Hz tau={self._velocity_tau}s")

        def _on_setpoint(self, message):
            self._command_velocity = [float(v) for v in message.velocity[:3]]
            self._command_yaw = float(message.yaw)

        def _on_mode(self, _message):
            self._mode_received = True

        def _step(self):
            self._tick += 1
            self._timestamp_us += int(self._dt * 1.0e6)
            alpha = min(1.0, self._dt / self._velocity_tau)
            for axis in range(3):
                self._velocity[axis] += (
                    self._command_velocity[axis] - self._velocity[axis]) * alpha
            for axis in range(3):
                self._position[axis] += self._velocity[axis] * self._dt
            yaw_alpha = min(1.0, self._dt / self._yaw_tau)
            self._yaw += wrap_pi(self._command_yaw - self._yaw) * yaw_alpha
            if self._tick % max(1, int(self._rate / 50)) == 0:
                self._publish_states()
            if self._tick % max(1, int(self._rate / 10)) == 0:
                self._publish_slow()
            if self._tick % max(1, int(self._rate / 20)) == 0:
                self._publish_sim_state()

        def _publish_states(self):
            local = VehicleLocalPosition()
            local.timestamp = self._timestamp_us
            local.x, local.y, local.z = self._position
            local.vx, local.vy, local.vz = self._velocity
            # 裁判与适配器都检查位置有效性标志,必须显式置位。
            local.xy_valid = True
            local.z_valid = True
            local.v_xy_valid = True
            local.v_z_valid = True
            self._local_pub.publish(local)
            attitude = VehicleAttitude()
            attitude.timestamp = self._timestamp_us
            attitude.q = list(quaternion_from_yaw(self._yaw))
            self._attitude_pub.publish(attitude)

        def _publish_slow(self):
            status = VehicleStatus()
            status.timestamp = self._timestamp_us
            status.arming_state = int(VehicleStatus.ARMING_STATE_ARMED)
            status.nav_state = int(VehicleStatus.NAVIGATION_STATE_OFFBOARD)
            self._status_pub.publish(status)
            land = VehicleLandDetected()
            land.timestamp = self._timestamp_us
            land.landed = False
            self._land_pub.publish(land)

        def _publish_sim_state(self):
            payload = {
                "stamp_s": self.get_clock().now().nanoseconds / 1.0e9,
                "pos_ned": list(self._position),
                "vel_ned": list(self._velocity),
                "yaw": self._yaw,
                "mode_received": self._mode_received,
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"))
            self._state_pub.publish(message)

    rclpy.init(args=args)
    node = None
    try:
        node = MockVehicleNode()
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
