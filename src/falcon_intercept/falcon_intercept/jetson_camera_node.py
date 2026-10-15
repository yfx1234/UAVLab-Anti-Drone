from __future__ import annotations


def _build_argus_pipeline(
    sensor_id: int,
    width: int,
    height: int,
    fps: float,
    exposure_compensation: float,
) -> str:
    """Build a low-latency Argus/NVMM pipeline for a CSI camera."""
    fps_value = max(1, int(round(float(fps))))
    if int(width) >= 3840 and int(height) >= 2160:
        source_width, source_height = 3840, 2160
    else:
        source_width, source_height = 1920, 1080
    return (
        "nvarguscamerasrc sensor-id=%d wbmode=1 awblock=false "
        "exposurecompensation=%.2f ! "
        "video/x-raw(memory:NVMM),width=%d,height=%d,framerate=%d/1,format=NV12 ! "
        "nvvidconv interpolation-method=2 ! "
        "video/x-raw,width=%d,height=%d,format=BGRx ! "
        "appsink drop=true max-buffers=1 sync=false"
        % (
            int(sensor_id),
            float(exposure_compensation),
            source_width,
            source_height,
            fps_value,
            int(width),
            int(height),
        )
    )


def main(args=None) -> int:
    import cv2
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    class JetsonCameraNode(Node):
        def __init__(self) -> None:
            super().__init__("jetson_camera_node")
            self.declare_parameter("sensor_id", 0)
            self.declare_parameter("image_topic", "/stereo/front/image_raw")
            self.declare_parameter("frame_id", "stereo_front_camera")
            self.declare_parameter("width", 1920)
            self.declare_parameter("height", 1080)
            self.declare_parameter("fps", 30.0)
            self.declare_parameter("stereo_color_correction", True)
            self.declare_parameter("stereo_exposure_compensation", 0.0)

            self._sensor_id = int(self.get_parameter("sensor_id").value)
            self._image_topic = str(self.get_parameter("image_topic").value)
            self._frame_id = str(self.get_parameter("frame_id").value)
            self._width = int(self.get_parameter("width").value)
            self._height = int(self.get_parameter("height").value)
            self._fps = max(1.0, float(self.get_parameter("fps").value))
            self._color_correction = bool(
                self.get_parameter("stereo_color_correction").value
            )
            self._exposure_compensation = min(
                2.0,
                max(
                    -2.0,
                    float(self.get_parameter("stereo_exposure_compensation").value),
                ),
            )
            self._read_count = 0
            self._publish_count = 0
            self._last_error = "none"
            self._last_stats_s = 0.0

            pipeline = _build_argus_pipeline(
                self._sensor_id,
                self._width,
                self._height,
                self._fps,
                self._exposure_compensation,
            )
            self.get_logger().info("opening Argus pipeline: %s" % pipeline)
            self._cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if not self._cap.isOpened():
                self._cap.release()
                raise RuntimeError(
                    "failed to open Argus sensor %d; check nvarguscamerasrc and camera overlay"
                    % self._sensor_id
                )
            self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            self._pub = self.create_publisher(
                Image,
                self._image_topic,
                qos_profile_sensor_data,
            )
            self.create_timer(1.0 / self._fps, self._on_timer)
            self.get_logger().info(
                "jetson_camera_node ready sensor=%d topic=%s frame_id=%s "
                "requested=%dx%d@%.1f color_correction=%s"
                % (
                    self._sensor_id,
                    self._image_topic,
                    self._frame_id,
                    self._width,
                    self._height,
                    self._fps,
                    self._color_correction,
                )
            )

        def _on_timer(self) -> None:
            try:
                ok, frame = self._cap.read()
                if not ok or frame is None:
                    self._last_error = "failed to read Argus frame"
                    self._log_stats()
                    return
                self._read_count += 1
                if frame.ndim != 3 or frame.shape[2] not in (3, 4):
                    self._last_error = "unexpected frame shape %s" % (frame.shape,)
                    self._log_stats()
                    return
                bgr_frame = frame[:, :, :3]
                if self._color_correction:
                    corrected = bgr_frame.astype(np.float32)
                    corrected *= np.array([0.91, 2.08, 0.92], dtype=np.float32)
                    corrected = np.clip(
                        (corrected - 116.0) * 1.10 + 116.0,
                        0.0,
                        255.0,
                    )
                    magenta = np.maximum(
                        0.0,
                        np.minimum(corrected[:, :, 2], corrected[:, :, 0])
                        - corrected[:, :, 1],
                    )
                    corrected[:, :, 2] -= 0.52 * magenta
                    corrected[:, :, 1] += 0.62 * magenta
                    corrected[:, :, 0] -= 0.20 * magenta
                    corrected = np.clip(corrected, 0.0, 255.0)
                    corrected = 255.0 * np.power(corrected / 255.0, 0.55)
                    gray = (
                        0.114 * corrected[:, :, 0]
                        + 0.587 * corrected[:, :, 1]
                        + 0.299 * corrected[:, :, 2]
                    )
                    corrected = 0.85 * corrected + 0.15 * gray[:, :, None]
                    bgr_frame = np.clip(corrected + 0.5, 0, 255).astype(np.uint8)

                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self._frame_id
                msg.height = int(bgr_frame.shape[0])
                msg.width = int(bgr_frame.shape[1])
                msg.encoding = "bgr8"
                msg.is_bigendian = 0
                msg.step = int(bgr_frame.shape[1] * 3)
                msg.data = bgr_frame.tobytes()
                self._pub.publish(msg)
                self._publish_count += 1
                self._last_error = "none"
                self._log_stats()
            except Exception as exc:  # noqa: BLE001
                self._last_error = "%s: %s" % (type(exc).__name__, exc)
                self.get_logger().error("Argus camera timer failed: %s" % self._last_error)

        def _log_stats(self) -> None:
            now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
            if now_s - self._last_stats_s >= 2.0:
                self.get_logger().info(
                    "camera stats sensor=%d reads=%d publishes=%d last_error=%s"
                    % (
                        self._sensor_id,
                        self._read_count,
                        self._publish_count,
                        self._last_error,
                    )
                )
                self._last_stats_s = now_s

        def destroy_node(self) -> bool:
            if hasattr(self, "_cap") and self._cap is not None:
                self._cap.release()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = JetsonCameraNode()
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
