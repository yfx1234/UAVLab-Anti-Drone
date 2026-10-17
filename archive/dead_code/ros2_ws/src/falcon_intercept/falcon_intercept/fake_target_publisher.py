from __future__ import annotations

from .detection_utils import build_detection_json


def _stamp_to_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def _target_x_for_elapsed(elapsed_s: float, start_x: float, fire_x: float, approach_duration_s: float) -> float:
    if elapsed_s <= 1.0:
        return start_x
    ratio = min(1.0, max(0.0, (elapsed_s - 1.0) / max(approach_duration_s, 1e-6)))
    return start_x + ratio * (fire_x - start_x)


def main(args=None) -> int:
    import rclpy
    from falcon_intercept_msgs.msg import TargetCameraPosition
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from std_msgs.msg import Bool, String

    class FakeTargetPublisher(Node):
        def __init__(self) -> None:
            super().__init__("fake_target_publisher")
            self.declare_parameter("target_state_topic", "/target/relative_state")
            self.declare_parameter("front_detections_topic", "/stereo/front/detections_json")
            self.declare_parameter("back_detections_topic", "/stereo/back/detections_json")
            self.declare_parameter("search_detections_topic", "/search/detections_json")
            self.declare_parameter(
                "camera_position_topic", "/perception/target/camera_position"
            )
            self.declare_parameter("target_health_topic", "/target/state_health")
            self.declare_parameter("frame_id", "stereo_camera")
            self.declare_parameter("rate_hz", 10.0)
            self.declare_parameter("start_x_m", 5.0)
            self.declare_parameter("fire_x_m", 1.6)
            self.declare_parameter("approach_duration_s", 6.0)
            self.declare_parameter("image_width", 640)
            self.declare_parameter("image_height", 480)
            self.declare_parameter("center_offset_px", 0.0)
            self.declare_parameter("back_center_offset_px", 0.0)
            self.declare_parameter("back_vertical_offset_px", 0.0)
            self.declare_parameter("confidence", 0.9)
            self.declare_parameter("landed_after_s", -1.0)

            self._frame_id = str(self.get_parameter("frame_id").value)
            self._start_time = self.get_clock().now()
            self._state_pub = self.create_publisher(
                Odometry,
                str(self.get_parameter("target_state_topic").value),
                10,
            )
            self._front_pub = self.create_publisher(
                String,
                str(self.get_parameter("front_detections_topic").value),
                10,
            )
            self._back_pub = self.create_publisher(
                String,
                str(self.get_parameter("back_detections_topic").value),
                10,
            )
            self._search_pub = self.create_publisher(
                String,
                str(self.get_parameter("search_detections_topic").value),
                10,
            )
            self._camera_pub = self.create_publisher(
                TargetCameraPosition,
                str(self.get_parameter("camera_position_topic").value),
                10,
            )
            self._health_pub = self.create_publisher(
                String,
                str(self.get_parameter("target_health_topic").value),
                10,
            )
            self._landed_pub = self.create_publisher(Bool, "/vehicle/landed", 10)
            self._sequence = 0
            rate_hz = max(1.0, float(self.get_parameter("rate_hz").value))
            self.create_timer(1.0 / rate_hz, self._on_timer)
            self.get_logger().info("fake_target_publisher ready")

        def _on_timer(self) -> None:
            now = self.get_clock().now()
            elapsed = (now - self._start_time).nanoseconds / 1_000_000_000.0
            start_x = float(self.get_parameter("start_x_m").value)
            fire_x = float(self.get_parameter("fire_x_m").value)
            duration = float(self.get_parameter("approach_duration_s").value)
            x = _target_x_for_elapsed(elapsed, start_x, fire_x, duration)
            vx = 0.0 if elapsed <= 1.0 or elapsed >= duration + 1.0 else (fire_x - start_x) / duration

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = self._frame_id
            odom.child_frame_id = "fake_target"
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.position.z = 0.0
            odom.pose.pose.orientation.w = 1.0
            odom.twist.twist.linear.x = vx
            self._state_pub.publish(odom)

            stamp_s = _stamp_to_seconds(odom.header.stamp)
            image_width = int(self.get_parameter("image_width").value)
            image_height = int(self.get_parameter("image_height").value)
            offset = float(self.get_parameter("center_offset_px").value)
            confidence = float(self.get_parameter("confidence").value)
            center = (float(image_width) / 2.0 + offset, float(image_height) / 2.0)

            front_msg = String()
            front_msg.data = build_detection_json("stereo_front_camera", stamp_s, center, confidence=confidence)
            back_msg = String()
            back_center = (
                center[0] + float(self.get_parameter("back_center_offset_px").value),
                center[1] + float(self.get_parameter("back_vertical_offset_px").value),
            )
            back_msg.data = build_detection_json("stereo_back_camera", stamp_s, back_center, confidence=confidence)
            self._front_pub.publish(front_msg)
            self._back_pub.publish(back_msg)
            search_msg = String()
            search_msg.data = build_detection_json(
                "search_camera", stamp_s, center, confidence=confidence, sequence=self._sequence
            )
            self._search_pub.publish(search_msg)

            camera = TargetCameraPosition()
            camera.header.stamp = odom.header.stamp
            camera.header.frame_id = "pursuer_camera_optical"
            camera.sequence = self._sequence
            # Forward-looking standard optical convention: +Z is forward.
            camera.position_camera.x = 0.0
            camera.position_camera.y = 0.0
            camera.position_camera.z = x
            camera.position_covariance = [
                0.01,
                0.0,
                0.0,
                0.0,
                0.01,
                0.0,
                0.0,
                0.0,
                0.04,
            ]
            camera.confidence = confidence
            camera.valid = True
            self._camera_pub.publish(camera)

            health = String()
            health.data = (
                '{"valid":true,"reason":"fake_target",'
                f'"last_measurement_timestamp_s":{stamp_s:.9f}'
                "}"
            )
            self._health_pub.publish(health)
            landed = Bool()
            landed_after_s = float(self.get_parameter("landed_after_s").value)
            landed.data = landed_after_s >= 0.0 and elapsed >= landed_after_s
            self._landed_pub.publish(landed)
            self._sequence = (self._sequence + 1) & 0xFFFFFFFF

    rclpy.init(args=args)
    node = None
    try:
        node = FakeTargetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
