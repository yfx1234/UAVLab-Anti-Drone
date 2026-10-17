"""Isolate upstream PX4 outputs and expose only approved setpoint classes.

VehicleCommand is intentionally neither subscribed nor republished.  The real
PX4 namespace is owned exclusively by the downstream safety adapter.
"""

from __future__ import annotations


def main(args=None) -> int:
    import rclpy
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleAttitudeSetpoint,
    )
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    class UpstreamGate(Node):
        def __init__(self) -> None:
            super().__init__("upstream_gate_node")
            self.declare_parameter("upstream_prefix", "/upstream/fmu/in")
            self.declare_parameter("adapter_prefix", "/intercept/upstream_px4")
            upstream = str(self.get_parameter("upstream_prefix").value).rstrip("/")
            adapter = str(self.get_parameter("adapter_prefix").value).rstrip("/")
            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._mode_pub = self.create_publisher(
                OffboardControlMode, f"{adapter}/offboard_control_mode", qos
            )
            self._trajectory_pub = self.create_publisher(
                TrajectorySetpoint, f"{adapter}/trajectory_setpoint", qos
            )
            self._attitude_pub = self.create_publisher(
                VehicleAttitudeSetpoint, f"{adapter}/vehicle_attitude_setpoint", qos
            )
            self.create_subscription(
                OffboardControlMode,
                f"{upstream}/offboard_control_mode",
                self._mode_pub.publish,
                qos,
            )
            self.create_subscription(
                TrajectorySetpoint,
                f"{upstream}/trajectory_setpoint",
                self._trajectory_pub.publish,
                qos,
            )
            self.create_subscription(
                VehicleAttitudeSetpoint,
                f"{upstream}/vehicle_attitude_setpoint",
                self._attitude_pub.publish,
                qos,
            )
            self.get_logger().warning(
                "upstream PX4 outputs isolated: VehicleCommand and body-rate setpoints are blocked"
            )

    rclpy.init(args=args)
    node = None
    try:
        node = UpstreamGate()
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
