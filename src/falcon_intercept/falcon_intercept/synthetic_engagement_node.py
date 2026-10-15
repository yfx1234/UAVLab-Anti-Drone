"""Test-only coherent observations for the isolated completion replay."""

from __future__ import annotations


def main(args=None) -> int:
    import rclpy
    from anti_drone_interfaces.msg import DepthObservation, SearchObservation
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node

    class SyntheticEngagement(Node):
        def __init__(self) -> None:
            super().__init__("synthetic_engagement_node")
            self.declare_parameter("rate_hz", 50.0)
            self.declare_parameter("range_m", 17.0)
            self._sequence = 0
            self._search_pub = self.create_publisher(
                SearchObservation, "/perception/search/observation", 10
            )
            self._depth_pub = self.create_publisher(
                DepthObservation, "/perception/depth/observation", 10
            )
            rate_hz = max(20.0, float(self.get_parameter("rate_hz").value))
            self.create_timer(1.0 / rate_hz, self._tick)

        def _tick(self) -> None:
            now = self.get_clock().now().to_msg()
            target_range = float(self.get_parameter("range_m").value)
            self._sequence += 1

            search = SearchObservation()
            search.interface_version = SearchObservation.INTERFACE_VERSION_V1
            search.header.stamp = now
            search.header.frame_id = "search_camera_optical_frame"
            search.sequence = self._sequence
            search.track_id = 1
            search.visible = True
            search.confidence = 1.0
            search.image_width = 2560
            search.image_height = 1440
            search.pixel_u = 1280.0
            search.pixel_v = 720.0
            search.bounding_box_valid = True
            search.bounding_box_width_px = 32.0
            search.bounding_box_height_px = 32.0
            search.angles_valid = True
            search.horizontal_angle_rad = 0.0
            search.vertical_angle_rad = 0.0
            search.angle_covariance = [1.0e-6, 0.0, 0.0, 1.0e-6]
            search.unit_los_valid = True
            search.unit_los_optical.x = 0.0
            search.unit_los_optical.y = 0.0
            search.unit_los_optical.z = 1.0
            self._search_pub.publish(search)

            depth = DepthObservation()
            depth.interface_version = DepthObservation.INTERFACE_VERSION_V1
            depth.header.stamp = now
            depth.header.frame_id = "left_camera_optical_frame"
            depth.sequence = self._sequence
            depth.track_id = 1
            depth.visible = True
            depth.confidence = 1.0
            depth.image_width = 2560
            depth.image_height = 1440
            depth.pixel_u = 1280.0
            depth.pixel_v = 720.0
            depth.bounding_box_valid = True
            depth.bounding_box_width_px = 32.0
            depth.bounding_box_height_px = 32.0
            depth.angles_valid = True
            depth.horizontal_angle_rad = 0.0
            depth.vertical_angle_rad = 0.0
            depth.angle_covariance = [1.0e-6, 0.0, 0.0, 1.0e-6]
            depth.depth_valid = True
            depth.depth_z_m = target_range
            depth.range_m = target_range
            depth.position_optical.x = 0.0
            depth.position_optical.y = 0.0
            depth.position_optical.z = target_range
            depth.position_covariance = [
                0.01, 0.0, 0.0,
                0.0, 0.01, 0.0,
                0.0, 0.0, 0.01,
            ]
            self._depth_pub.publish(depth)

    rclpy.init(args=args)
    node = None
    try:
        node = SyntheticEngagement()
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
