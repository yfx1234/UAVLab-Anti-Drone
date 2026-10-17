from __future__ import annotations

from .detection_utils import ros_stamp_to_seconds as _stamp_to_seconds



def _build_gstreamer_pipeline_scaled(
    device_path: str,
    source_width: int,
    source_height: int,
    source_fps: float,
    output_width: int,
    output_height: int,
    fourcc: str,
    hardware_decode: bool = False,
) -> str:
    """Build a low-latency GStreamer capture pipeline for USB cameras."""
    requested_fourcc = str(fourcc).strip().upper()
    fps_value = max(1, int(round(float(source_fps))))
    device = str(device_path).replace('"', "\\\"")

    if requested_fourcc in ("MJPG", "JPEG"):
        caps = "image/jpeg,width=%d,height=%d,framerate=%d/1" % (
            source_width, source_height, fps_value
        )
        if hardware_decode:
            decode = (
                "nvv4l2decoder mjpeg=1 ! nvvidconv ! "
                "video/x-raw,width=%d,height=%d,format=BGRx ! videoconvert"
                % (output_width, output_height)
            )
        else:
            decode = (
                "jpegparse ! jpegdec ! videoscale ! "
                "video/x-raw,width=%d,height=%d ! videoconvert"
                % (output_width, output_height)
            )
    elif requested_fourcc in ("YUYV", "YUY2"):
        caps = "video/x-raw,format=YUY2,width=%d,height=%d,framerate=%d/1" % (
            source_width,
            source_height,
            fps_value,
        )
        decode = "videoscale ! video/x-raw,width=%d,height=%d ! videoconvert" % (
            output_width, output_height
        )
    else:
        caps = "video/x-raw,width=%d,height=%d,framerate=%d/1" % (
            source_width, source_height, fps_value
        )
        decode = "videoscale ! video/x-raw,width=%d,height=%d ! videoconvert" % (
            output_width, output_height
        )

    return (
        'v4l2src device="%s" ! %s ! %s ! '
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false" % (device, caps, decode)
    )


def _build_gstreamer_pipeline(
    device_path: str,
    width: int,
    height: int,
    fps: float,
    fourcc: str,
    hardware_decode: bool = False,
) -> str:
    """Backward-compatible same-source/output pipeline helper."""
    return _build_gstreamer_pipeline_scaled(
        device_path,
        width,
        height,
        fps,
        width,
        height,
        fourcc,
        hardware_decode,
    )


def _gstreamer_pipeline_candidates(
    device_path: str,
    source_width: int,
    source_height: int,
    source_fps: float,
    output_width: int,
    output_height: int,
    fourcc: str,
) -> list[tuple[str, str]]:
    candidates: list[tuple[str, str]] = []
    if str(fourcc).strip().upper() in ("MJPG", "JPEG"):
        candidates.append(
            (
                "gstreamer-hw",
                _build_gstreamer_pipeline_scaled(
                    device_path,
                    source_width,
                    source_height,
                    source_fps,
                    output_width,
                    output_height,
                    fourcc,
                    True,
                ),
            )
        )
    candidates.append(
        (
            "gstreamer",
            _build_gstreamer_pipeline_scaled(
                device_path,
                source_width,
                source_height,
                source_fps,
                output_width,
                output_height,
                fourcc,
                False,
            ),
        )
    )
    return candidates


def _opencv_has_gstreamer(cv2_module) -> bool:
    try:
        return "GStreamer:                   YES" in cv2_module.getBuildInformation()
    except (AttributeError, TypeError):
        return False


def main(args=None) -> int:
    import cv2
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class V4L2CameraNode(Node):
        def __init__(self) -> None:
            super().__init__("v4l2_camera_node")
            self.declare_parameter("device_path", "/dev/video0")
            self.declare_parameter("image_topic", "/search/image_raw")
            self.declare_parameter("frame_id", "search_camera")
            self.declare_parameter("width", 640)
            self.declare_parameter("height", 480)
            self.declare_parameter("fps", 30.0)
            self.declare_parameter("source_width", 0)
            self.declare_parameter("source_height", 0)
            self.declare_parameter("source_fps", 0.0)
            self.declare_parameter("fourcc", "MJPG")
            self.declare_parameter("capture_backend", "auto")

            self._device_path = str(self.get_parameter("device_path").value)
            self._frame_id = str(self.get_parameter("frame_id").value)
            self._width = int(self.get_parameter("width").value)
            self._height = int(self.get_parameter("height").value)
            self._fps = max(1.0, float(self.get_parameter("fps").value))
            self._source_width = int(self.get_parameter("source_width").value)
            self._source_height = int(self.get_parameter("source_height").value)
            self._source_fps = float(self.get_parameter("source_fps").value)
            if self._source_width <= 0:
                self._source_width = self._width
            if self._source_height <= 0:
                self._source_height = self._height
            if self._source_fps <= 0.0:
                self._source_fps = self._fps
            self._fourcc = str(self.get_parameter("fourcc").value)
            self._capture_backend_requested = str(
                self.get_parameter("capture_backend").value
            ).strip().lower()
            if self._capture_backend_requested not in ("auto", "gstreamer", "v4l2"):
                raise ValueError(
                    "capture_backend must be one of auto, gstreamer, v4l2; got %s"
                    % self._capture_backend_requested
                )
            self._capture_backend_actual = "none"
            self._last_warn_s = 0.0
            self._last_stats_s = 0.0
            self._read_count = 0
            self._publish_count = 0
            self._last_error = "none"

            self._pub = self.create_publisher(
                Image,
                str(self.get_parameter("image_topic").value),
                qos_profile_sensor_data,
            )
            self._cap = None
            self._open_capture(cv2)
            self.create_timer(1.0 / self._fps, self._on_timer)
            self.get_logger().info(
                "v4l2_camera_node ready backend=%s device=%s topic=%s source=%dx%d@%.1f output=%dx%d@%.1f fourcc=%s"
                % (
                    self._capture_backend_actual,
                    self._device_path,
                    str(self.get_parameter("image_topic").value),
                    self._source_width,
                    self._source_height,
                    self._source_fps,
                    self._width,
                    self._height,
                    self._fps,
                    self._fourcc,
                )
            )

        def _open_capture(self, cv2) -> None:
            backend = self._capture_backend_requested
            if backend in ("auto", "gstreamer") and _opencv_has_gstreamer(cv2):
                for candidate_name, pipeline in _gstreamer_pipeline_candidates(
                    self._device_path,
                    self._source_width,
                    self._source_height,
                    self._source_fps,
                    self._width,
                    self._height,
                    self._fourcc,
                ):
                    self.get_logger().info(
                        "trying %s capture pipeline: %s" % (candidate_name, pipeline)
                    )
                    gstreamer_cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
                    if gstreamer_cap.isOpened():
                        self._cap = gstreamer_cap
                        self._capture_backend_actual = candidate_name
                        return
                    gstreamer_cap.release()
                if backend == "gstreamer":
                    self._last_error = "failed to open all GStreamer capture pipelines"
                    self.get_logger().error(self._last_error)
                    return
                self.get_logger().warn("GStreamer capture failed; falling back to V4L2")
            elif backend == "gstreamer":
                self._last_error = "OpenCV was built without GStreamer support"
                self.get_logger().error(self._last_error)
                return

            self._cap = cv2.VideoCapture(self._device_path, cv2.CAP_V4L2)
            self._configure_v4l2_capture()

        def _configure_v4l2_capture(self) -> None:
            if self._cap is None:
                return
            if self._fourcc:
                self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self._fourcc[:4]))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._source_width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._source_height)
            self._cap.set(cv2.CAP_PROP_FPS, self._source_fps)
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            if not self._cap.isOpened():
                self._capture_backend_actual = "v4l2"
                self.get_logger().error("failed to open camera device %s" % self._device_path)
                return
            self._capture_backend_actual = "v4l2"
            actual_width = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = float(self._cap.get(cv2.CAP_PROP_FPS))
            self.get_logger().info(
                "camera opened actual=%dx%d@%.2f" % (actual_width, actual_height, actual_fps)
            )

        def _on_timer(self) -> None:
            try:
                if self._cap is None or not self._cap.isOpened():
                    self._last_error = "camera device is not open"
                    self._warn_throttled(self._last_error)
                    self._log_stats()
                    return
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    self._last_error = "failed to read frame from %s" % self._device_path
                    self._warn_throttled(self._last_error)
                    self._log_stats()
                    return
                self._read_count += 1
                if frame.ndim != 3 or frame.shape[2] != 3:
                    self._last_error = "unexpected frame shape %s" % (frame.shape,)
                    self._warn_throttled(self._last_error)
                    self._log_stats()
                    return

                if frame.shape[1] != self._width or frame.shape[0] != self._height:
                    frame = cv2.resize(
                        frame,
                        (self._width, self._height),
                        interpolation=cv2.INTER_AREA,
                    )

                now = self.get_clock().now().to_msg()
                msg = Image()
                msg.header.stamp = now
                msg.header.frame_id = self._frame_id
                msg.height = int(frame.shape[0])
                msg.width = int(frame.shape[1])
                msg.encoding = "bgr8"
                msg.is_bigendian = 0
                msg.step = int(frame.strides[0])
                msg.data = frame.tobytes()
                self._pub.publish(msg)
                self._publish_count += 1
                self._last_error = "none"
                self._log_stats()
            except Exception as exc:  # noqa: BLE001
                self._last_error = "%s: %s" % (type(exc).__name__, exc)
                self.get_logger().error("camera timer failed: %s" % self._last_error)

        def _warn_throttled(self, message: str) -> None:
            now_s = _stamp_to_seconds(self.get_clock().now().to_msg())
            if now_s - self._last_warn_s >= 2.0:
                self.get_logger().warn(message)
                self._last_warn_s = now_s

        def _log_stats(self) -> None:
            now_s = _stamp_to_seconds(self.get_clock().now().to_msg())
            if now_s - self._last_stats_s >= 2.0:
                self.get_logger().info(
                    "camera stats reads=%d publishes=%d last_error=%s"
                    % (self._read_count, self._publish_count, self._last_error)
                )
                self._last_stats_s = now_s

        def destroy_node(self) -> bool:
            if hasattr(self, "_cap") and self._cap is not None:
                self._cap.release()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = V4L2CameraNode()
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
