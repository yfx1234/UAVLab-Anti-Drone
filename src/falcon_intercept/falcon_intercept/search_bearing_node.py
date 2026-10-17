"""Publish timestamped 2-D bearing observations for the 25 FPS search camera."""

from __future__ import annotations


def main(args=None) -> int:
    import rclpy
    from falcon_intercept_msgs.msg import CameraBearingObservation
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    from .detection_utils import parse_detections_json

    def set_stamp(stamp, seconds: float) -> None:
        sec = int(seconds)
        nanosec = int(round((float(seconds) - sec) * 1_000_000_000.0))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        stamp.sec = sec
        stamp.nanosec = nanosec

    class SearchBearingNode(Node):
        def __init__(self) -> None:
            super().__init__("search_bearing_node")
            self.declare_parameter("detections_topic", "/search/detections_json")
            self.declare_parameter("bearing_topic", "/perception/search/bearing")
            self.declare_parameter("min_confidence", 0.5)
            self.declare_parameter("timestamp_sigma_s", 0.004)
            self.declare_parameter("pixel_variance_px2", 9.0)
            self.declare_parameter("class_id", -1)
            self.declare_parameter("class_name", "")
            self._last_key = None
            self._pub = self.create_publisher(
                CameraBearingObservation,
                str(self.get_parameter("bearing_topic").value),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("detections_topic").value),
                self._on_detections,
                qos_profile_sensor_data,
            )

        def _on_detections(self, message: String) -> None:
            try:
                frame = parse_detections_json(str(message.data))
            except (TypeError, ValueError, KeyError):
                return
            if frame.stamp_s is None:
                return
            class_id = int(self.get_parameter("class_id").value)
            class_name = str(self.get_parameter("class_name").value).strip()
            min_confidence = float(self.get_parameter("min_confidence").value)
            candidates = [
                item
                for item in frame.detections
                if item.confidence >= min_confidence
                and (class_id < 0 or item.class_id == class_id)
                and (not class_name or item.class_name == class_name)
            ]
            if not candidates:
                return
            detection = max(candidates, key=lambda item: item.confidence)
            key = (frame.sequence, frame.stamp_s, detection.bbox_center)
            if key == self._last_key:
                return
            self._last_key = key
            out = CameraBearingObservation()
            set_stamp(out.header.stamp, frame.stamp_s)
            out.header.frame_id = frame.frame_id or "search_camera"
            out.camera_id = "search"
            out.sequence = int(frame.sequence or 0)
            out.host_receive_ns = int(self.get_clock().now().nanoseconds)
            out.timestamp_sigma_s = float(
                frame.timestamp_sigma_s
                if frame.timestamp_sigma_s is not None
                else self.get_parameter("timestamp_sigma_s").value
            )
            out.exposure_time_s = -1.0
            out.frame_readout_time_s = -1.0
            out.pixel_u = float(detection.bbox_center[0])
            out.pixel_v = float(detection.bbox_center[1])
            variance = float(self.get_parameter("pixel_variance_px2").value)
            out.pixel_covariance = [variance, 0.0, 0.0, variance]
            out.confidence = float(detection.confidence)
            out.valid = True
            self._pub.publish(out)

    rclpy.init(args=args)
    node = None
    try:
        node = SearchBearingNode()
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
