#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class TargetKfSmoke(Node):
    def __init__(self) -> None:
        super().__init__("target_kf_runtime_smoke")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/target/relative_measurement",
            qos_profile_sensor_data,
        )
        self.health: dict | None = None
        self.state: Odometry | None = None
        self.create_subscription(String, "/target/state_health", self._on_health, 10)
        self.create_subscription(
            Odometry, "/target/relative_state", self._on_state, qos_profile_sensor_data
        )

    def _on_health(self, msg: String) -> None:
        try:
            self.health = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_state(self, msg: Odometry) -> None:
        self.state = msg

    def publish_measurement(self, x: float, y: float = 0.0, z: float = 0.0) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "stereo_camera"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.position.z = float(z)
        msg.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14):
            msg.pose.covariance[index] = 0.04
        self.publisher.publish(msg)


def spin_until(node: TargetKfSmoke, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def publish_and_spin(node: TargetKfSmoke, x: float, duration_s: float = 0.25) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        node.publish_measurement(x)
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.02)


def main() -> int:
    rclpy.init()
    node = TargetKfSmoke()
    try:
        # Repeated finite measurements provide a data-path handshake without relying on
        # middleware discovery counters, which can lag on the Jetson Fast DDS setup.
        publish_and_spin(node, 1.0, 1.5)
        if not spin_until(
            node,
            lambda: node.health is not None
            and node.health.get("valid") is True
            and node.state is not None,
            3.0,
        ):
            print(f"KF_ACQUIRE_FAIL health={node.health} state={node.state}")
            return 1
        assert node.state is not None
        values = [
            node.state.pose.covariance[0],
            node.state.twist.covariance[0],
            node.state.twist.twist.linear.x,
        ]
        if not all(math.isfinite(value) for value in values) or values[0] <= 0 or values[1] <= 0:
            print(f"KF_COVARIANCE_FAIL values={values}")
            return 1

        # A gross jump must be rejected by the innovation gate.
        node.publish_measurement(100.0)
        if not spin_until(
            node,
            lambda: node.health is not None
            and node.health.get("last_update_reason") == "innovation_gate_rejected",
            2.0,
        ):
            print(f"KF_GATE_FAIL health={node.health}")
            return 1

        # With no accepted measurement the health topic must close automatically.
        if not spin_until(
            node,
            lambda: node.health is not None
            and node.health.get("valid") is False
            and node.health.get("reason") == "measurement_timeout",
            2.0,
        ):
            print(f"KF_TIMEOUT_FAIL health={node.health}")
            return 1

        # Wait beyond reset_after_gap_sec, then reacquire with two consistent measurements.
        time.sleep(0.7)
        node.publish_measurement(3.0)
        if not spin_until(
            node,
            lambda: node.health is not None
            and node.health.get("last_update_reason") == "reinitialized_after_gap"
            and node.health.get("reason") == "acquiring",
            2.0,
        ):
            print(f"KF_REINITIALIZE_FAIL health={node.health}")
            return 1
        time.sleep(0.1)
        node.publish_measurement(3.1)
        if not spin_until(
            node,
            lambda: node.health is not None
            and node.health.get("valid") is True
            and node.health.get("consecutive_hits", 0) >= 2,
            2.0,
        ):
            print(f"KF_REACQUIRE_FAIL health={node.health}")
            return 1
        print(
            "TARGET_KF_RUNTIME_SMOKE_PASS "
            "acquire=PASS covariance=PASS gate=PASS timeout=PASS reacquire=PASS"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
