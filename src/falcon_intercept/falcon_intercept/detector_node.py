from __future__ import annotations

import json
import time

import cv2
import numpy as np

from .detection_utils import ros_stamp_to_seconds as _stamp_to_seconds
from .detection_utils import stamp_to_dict
from .yolo_detections import draw_detections, extract_yolo_detections, parse_class_filter



def _detection_json(frame_id: str, stamp_s: float, detections: list[dict]) -> str:
    payload = {
        "header": {"stamp": stamp_to_dict(stamp_s), "frame_id": frame_id},
        "detections": detections,
    }
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))


class DetectorBackend:
    def detect(self, frame_bgr: np.ndarray) -> list[dict]:
        raise NotImplementedError


class EmptyDetectorBackend(DetectorBackend):
    def detect(self, frame_bgr: np.ndarray) -> list[dict]:
        return []


class UltralyticsDetectorBackend(DetectorBackend):
    """The migrated colleague implementation, configured for this ROS2 node."""

    def __init__(
        self,
        model_path: str,
        confidence: float,
        iou: float,
        imgsz: int,
        device: str,
        half: bool,
        max_det: int,
        classes: list[int] | None,
        class_name_override: str,
    ) -> None:
        try:
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "ultralytics is not installed. Install the Jetson-compatible PyTorch first, "
                "then install ultralytics in the ROS2 Python environment."
            ) from exc

        self._model = YOLO(model_path)
        self._confidence = float(confidence)
        self._iou = float(iou)
        self._imgsz = int(imgsz)
        self._device = str(device).strip()
        self._half = bool(half)
        self._max_det = int(max_det)
        self._classes = classes
        self._class_name_override = class_name_override

    def detect(self, frame_bgr: np.ndarray) -> list[dict]:
        kwargs = {
            "source": frame_bgr,
            "conf": self._confidence,
            "iou": self._iou,
            "imgsz": self._imgsz,
            "half": self._half,
            "max_det": self._max_det,
            "classes": self._classes,
            "verbose": False,
        }
        if self._device:
            kwargs["device"] = self._device
        results = self._model.predict(**kwargs)
        result = results[0] if results else None
        return extract_yolo_detections(result, self._model.names, self._class_name_override)


def _make_backend(
    backend_name: str,
    model_path: str,
    confidence: float,
    iou: float,
    imgsz: int,
    device: str,
    half: bool,
    max_det: int,
    classes: list[int] | None,
    class_name_override: str,
) -> DetectorBackend:
    normalized = backend_name.strip().lower()
    if normalized in ("", "none", "empty"):
        return EmptyDetectorBackend()
    if normalized == "ultralytics":
        if not model_path:
            raise ValueError("model_path is required when detector_backend=ultralytics")
        return UltralyticsDetectorBackend(
            model_path,
            confidence,
            iou,
            imgsz,
            device,
            half,
            max_det,
            classes,
            class_name_override,
        )
    raise ValueError("unsupported detector_backend=%s" % backend_name)


def _build_vision_detection_array(header, detections):
    from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

    msg = Detection2DArray()
    msg.header = header
    for item in detections:
        x1, y1, x2, y2 = [float(value) for value in item["bbox_xyxy"]]
        detection = Detection2D()
        detection.header = header
        detection.id = str(item["class_name"])
        detection.bbox.center.position.x = (x1 + x2) / 2.0
        detection.bbox.center.position.y = (y1 + y2) / 2.0
        detection.bbox.center.theta = 0.0
        detection.bbox.size_x = x2 - x1
        detection.bbox.size_y = y2 - y1
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = str(item["class_name"])
        hypothesis.hypothesis.score = float(item["confidence"])
        detection.results.append(hypothesis)
        msg.detections.append(detection)
    return msg


def main(args=None) -> int:
    import rclpy
    from cv_bridge import CvBridge, CvBridgeError
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image
    from std_msgs.msg import Header, String

    class DetectorNode(Node):
        def __init__(self) -> None:
            super().__init__("detector_node")
            self.declare_parameter("input_mode", "ros_image")
            self.declare_parameter("image_topic", "/search/image_raw")
            self.declare_parameter("detections_topic", "/search/detections_json")
            self.declare_parameter("detections_json_topic", "")
            self.declare_parameter("vision_detections_topic", "/search/detections")
            self.declare_parameter("debug_image_topic", "/search/image_yolo")
            self.declare_parameter("publish_debug_image", False)
            self.declare_parameter("publish_vision_msgs", False)
            self.declare_parameter("device_path", "/dev/video0")
            self.declare_parameter("frame_id", "search_camera")
            self.declare_parameter("width", 640)
            self.declare_parameter("height", 480)
            self.declare_parameter("fps", 10.0)
            self.declare_parameter("fourcc", "MJPG")
            self.declare_parameter("detector_backend", "empty")
            self.declare_parameter("model_path", "")
            self.declare_parameter("model", "")
            self.declare_parameter("confidence", 0.50)
            self.declare_parameter("conf", -1.0)
            self.declare_parameter("iou", 0.45)
            self.declare_parameter("imgsz", 640)
            self.declare_parameter("device", "")
            self.declare_parameter("half", False)
            self.declare_parameter("max_det", 100)
            self.declare_parameter("classes", "")
            self.declare_parameter("frame_stride", 1)
            self.declare_parameter("class_name", "")

            self._input_mode = str(self.get_parameter("input_mode").value)
            self._frame_id = str(self.get_parameter("frame_id").value)
            self._fps = max(1.0, float(self.get_parameter("fps").value))
            self._publish_count = 0
            self._processed_count = 0
            self._last_stats_s = time.monotonic()
            self._last_inference_s = 0.0
            self._cap = None
            self._bridge = CvBridge()
            model_path = str(self.get_parameter("model_path").value).strip()
            if not model_path:
                model_path = str(self.get_parameter("model").value).strip()
            confidence = float(self.get_parameter("confidence").value)
            legacy_conf = float(self.get_parameter("conf").value)
            if legacy_conf >= 0.0:
                confidence = legacy_conf
            self._frame_stride = max(1, int(self.get_parameter("frame_stride").value))
            self._frame_count = 0
            self._class_name = str(self.get_parameter("class_name").value)
            self._backend_name = str(self.get_parameter("detector_backend").value)
            self._backend = _make_backend(
                self._backend_name,
                model_path,
                confidence,
                float(self.get_parameter("iou").value),
                int(self.get_parameter("imgsz").value),
                str(self.get_parameter("device").value),
                bool(self.get_parameter("half").value),
                int(self.get_parameter("max_det").value),
                parse_class_filter(str(self.get_parameter("classes").value)),
                self._class_name,
            )

            json_topic = str(self.get_parameter("detections_json_topic").value).strip()
            if not json_topic:
                json_topic = str(self.get_parameter("detections_topic").value)
            self._json_pub = self.create_publisher(String, json_topic, qos_profile_sensor_data)
            self._debug_pub = None
            if bool(self.get_parameter("publish_debug_image").value):
                self._debug_pub = self.create_publisher(
                    Image,
                    str(self.get_parameter("debug_image_topic").value),
                    qos_profile_sensor_data,
                )

            self._vision_pub = None
            if bool(self.get_parameter("publish_vision_msgs").value):
                try:
                    from vision_msgs.msg import Detection2DArray

                    self._vision_pub = self.create_publisher(
                        Detection2DArray,
                        str(self.get_parameter("vision_detections_topic").value),
                        qos_profile_sensor_data,
                    )
                except ImportError:
                    self.get_logger().warning(
                        "vision_msgs is not installed; JSON detections remain available on %s" % json_topic
                    )

            if self._input_mode == "ros_image":
                self.create_subscription(
                    Image,
                    str(self.get_parameter("image_topic").value),
                    self._on_image,
                    qos_profile_sensor_data,
                )
            elif self._input_mode == "direct_camera":
                self._open_direct_camera()
                self.create_timer(1.0 / self._fps, self._on_timer)
            else:
                raise ValueError("unsupported input_mode=%s" % self._input_mode)

            self.get_logger().info(
                "detector_node ready input_mode=%s backend=%s model=%s image=%s json=%s"
                % (self._input_mode, self._backend_name, model_path or "none", self.get_parameter("image_topic").value, json_topic)
            )

        def _open_direct_camera(self) -> None:
            device_path = str(self.get_parameter("device_path").value)
            width = int(self.get_parameter("width").value)
            height = int(self.get_parameter("height").value)
            fourcc = str(self.get_parameter("fourcc").value)
            self._cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
            if fourcc:
                self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            self._cap.set(cv2.CAP_PROP_FPS, self._fps)
            if not self._cap.isOpened():
                self.get_logger().error("failed to open direct camera %s" % device_path)
            else:
                self.get_logger().info("direct camera opened device=%s" % device_path)

        def _on_image(self, msg: Image) -> None:
            self._frame_count += 1
            if self._frame_count % self._frame_stride != 0:
                return
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except CvBridgeError as exc:
                self.get_logger().warning("image conversion failed: %s" % exc)
                return
            self._publish_detections(frame, msg.header)

        def _on_timer(self) -> None:
            if self._cap is None or not self._cap.isOpened():
                return
            ok, frame = self._cap.read()
            if not ok or frame is None:
                self.get_logger().warning("failed to read direct camera frame")
                return
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self._frame_id
            self._frame_count += 1
            if self._frame_count % self._frame_stride != 0:
                return
            self._publish_detections(frame, header)

        def _publish_detections(self, frame: np.ndarray, header) -> None:
            inference_start = time.perf_counter()
            try:
                detections = self._backend.detect(frame)
            except Exception as exc:
                self.get_logger().error("YOLO inference failed: %s" % exc)
                return
            self._last_inference_s = time.perf_counter() - inference_start
            frame_id = header.frame_id or self._frame_id
            stamp_s = _stamp_to_seconds(header.stamp)
            json_msg = String()
            json_msg.data = _detection_json(frame_id, stamp_s, detections)
            self._json_pub.publish(json_msg)

            if self._vision_pub is not None:
                self._vision_pub.publish(_build_vision_detection_array(header, detections))

            if self._debug_pub is not None:
                debug_msg = self._bridge.cv2_to_imgmsg(draw_detections(frame, detections), encoding="bgr8")
                debug_msg.header = header
                self._debug_pub.publish(debug_msg)

            self._publish_count += 1
            self._processed_count += 1
            now = time.monotonic()
            # Log the first completed inference once, then report a real
            # windowed throughput every two seconds.  Using
            # ``_processed_count == 1`` here would be true after every
            # reset, causing one log line per frame and an always-zero FPS.
            if self._publish_count == 1 or now - self._last_stats_s >= 2.0:
                elapsed = max(1e-6, now - self._last_stats_s)
                fps = self._processed_count / elapsed
                self.get_logger().info(
                    "detector stats publishes=%d detections=%d inference_ms=%.1f avg_fps=%.1f"
                    % (self._publish_count, len(detections), self._last_inference_s * 1000.0, fps)
                )
                self._last_stats_s = now
                self._processed_count = 0

        def destroy_node(self) -> bool:
            if self._cap is not None:
                self._cap.release()
            return super().destroy_node()

    rclpy.init(args=args)
    node = None
    try:
        node = DetectorNode()
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
