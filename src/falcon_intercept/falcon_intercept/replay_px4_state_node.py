"""Deterministic PX4 state source for isolated rosbag shadow tests only."""

from __future__ import annotations


def main(args=None) -> int:
    import rclpy
    from px4_msgs.msg import (
        VehicleAttitude,
        VehicleLandDetected,
        VehicleLocalPosition,
        VehicleStatus,
    )
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    class ReplayPx4State(Node):
        def __init__(self) -> None:
            super().__init__("replay_px4_state_node")
            self.declare_parameter("px4_namespace", "/test/fmu")
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("arm_after_s", 1.0)
            self.declare_parameter("offboard_after_s", 2.5)
            self.declare_parameter("flight_height_m", 5.0)
            prefix = str(self.get_parameter("px4_namespace").value).rstrip("/")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._attitude_pub = self.create_publisher(
                VehicleAttitude, f"{prefix}/out/vehicle_attitude", qos
            )
            self._local_pub = self.create_publisher(
                VehicleLocalPosition, f"{prefix}/out/vehicle_local_position", qos
            )
            self._status_pub = self.create_publisher(
                VehicleStatus, f"{prefix}/out/vehicle_status", qos
            )
            self._land_pub = self.create_publisher(
                VehicleLandDetected, f"{prefix}/out/vehicle_land_detected", qos
            )
            self._start_s = None
            rate_hz = max(20.0, float(self.get_parameter("rate_hz").value))
            self.create_timer(1.0 / rate_hz, self._tick)

        def _tick(self) -> None:
            now = self.get_clock().now()
            now_s = now.nanoseconds / 1.0e9
            if now_s <= 0.0:
                return
            if self._start_s is None:
                self._start_s = now_s
            elapsed = max(0.0, now_s - self._start_s)
            arm_after = float(self.get_parameter("arm_after_s").value)
            offboard_after = float(self.get_parameter("offboard_after_s").value)
            height = float(self.get_parameter("flight_height_m").value)
            armed = elapsed >= arm_after
            offboard = elapsed >= offboard_after
            climb_fraction = min(1.0, max(0.0, (elapsed - arm_after) / max(0.1, offboard_after - arm_after)))
            timestamp = now.nanoseconds // 1000

            attitude = VehicleAttitude()
            attitude.timestamp = timestamp
            attitude.q = [1.0, 0.0, 0.0, 0.0]
            self._attitude_pub.publish(attitude)

            local = VehicleLocalPosition()
            local.timestamp = timestamp
            local.xy_valid = True
            local.z_valid = True
            local.v_xy_valid = True
            local.v_z_valid = True
            local.x = 0.0
            local.y = 0.0
            local.z = -height * climb_fraction
            local.vx = 0.0
            local.vy = 0.0
            local.vz = 0.0
            self._local_pub.publish(local)

            status = VehicleStatus()
            status.timestamp = timestamp
            status.arming_state = (
                VehicleStatus.ARMING_STATE_ARMED
                if armed
                else VehicleStatus.ARMING_STATE_DISARMED
            )
            status.nav_state = (
                VehicleStatus.NAVIGATION_STATE_OFFBOARD
                if offboard
                else VehicleStatus.NAVIGATION_STATE_POSCTL
            )
            status.pre_flight_checks_pass = True
            status.failsafe = False
            status.takeoff_time = timestamp if armed else 0
            self._status_pub.publish(status)

            landed = VehicleLandDetected()
            landed.timestamp = timestamp
            landed.landed = not armed
            self._land_pub.publish(landed)

    rclpy.init(args=args)
    node = None
    try:
        node = ReplayPx4State()
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
