"""Project search-camera detections into a stereo ROI JSON contract.

This node is the ROS2-side bridge for the next DeepStream stage. It publishes
only ROI metadata; the current detector and camera nodes remain unchanged until
measured calibration and a real NVMM crop/inference consumer are available.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

from .detection_utils import DetectionBox, parse_detections_json, stamp_to_dict
from .perception_geometry import CameraGeometry, project_search_bbox_to_stereo_rois


@dataclass(frozen=True)
class RoiProjectionConfig:
    min_confidence: float = 0.5
    class_id: int | None = None
    class_name: str = ""
    depth_min_m: float = 5.0
    depth_max_m: float = 100.0
    margin_px: float = 32.0
    max_source_age_s: float = 0.20
    max_future_skew_s: float = 0.05


def _select_search_detection(
    detections: tuple[DetectionBox, ...], config: RoiProjectionConfig
) -> DetectionBox | None:
    candidates = [
        detection
        for detection in detections
        if detection.confidence >= float(config.min_confidence)
        and (config.class_id is None or detection.class_id == int(config.class_id))
        and (not config.class_name or detection.class_name == str(config.class_name))
    ]
    if not candidates:
        return None
    return max(candidates, key=lambda detection: float(detection.confidence))


def _roi_payload(roi) -> dict[str, Any]:
    return {
        "x1": int(roi.x1),
        "y1": int(roi.y1),
        "x2": int(roi.x2),
        "y2": int(roi.y2),
        "width": int(roi.width),
        "height": int(roi.height),
        "image_width": int(roi.image_width),
        "image_height": int(roi.image_height),
    }


def _base_payload(parsed) -> dict[str, Any]:
    return {
        "header": {
            "stamp": None if parsed.stamp_s is None else stamp_to_dict(parsed.stamp_s),
            "frame_id": parsed.frame_id,
        },
        "valid": False,
        "reason": "unknown",
        "search_bbox_xyxy": None,
        "source_class_id": None,
        "source_class_name": None,
        "source_confidence": None,
        "depth_min_m": None,
        "depth_max_m": None,
        "margin_px": None,
        "rois": {"front": None, "back": None},
    }


def build_roi_projection_payload(
    search_json: str,
    geometry: CameraGeometry | None,
    config: RoiProjectionConfig,
    *,
    now_s: float | None = None,
) -> dict[str, Any] | None:
    """Build one JSON payload from the newest search detection frame."""

    try:
        parsed = parse_detections_json(search_json)
    except (TypeError, ValueError, KeyError, json.JSONDecodeError):
        return None

    payload = _base_payload(parsed)
    if parsed.stamp_s is None:
        payload["reason"] = "missing_timestamp"
        return payload

    # The ROI is consumed by the stereo pipeline asynchronously.  A ROS
    # receive-time watchdog alone is insufficient: a stalled search source can
    # keep an old ROI alive while the ROS process is still responsive.  Gate on
    # the source timestamp before projecting or publishing it.
    if now_s is not None:
        source_age_s = float(now_s) - float(parsed.stamp_s)
        payload["source_age_ms"] = source_age_s * 1000.0
        if source_age_s > float(config.max_source_age_s):
            payload["reason"] = "stale_source_timestamp"
            return payload
        if source_age_s < -float(config.max_future_skew_s):
            payload["reason"] = "future_source_timestamp"
            return payload

    detection = _select_search_detection(parsed.detections, config)
    if detection is None:
        payload["reason"] = "no_detection"
        return payload

    payload.update(
        {
            "search_bbox_xyxy": list(detection.bbox_xyxy),
            "source_class_id": int(detection.class_id),
            "source_class_name": detection.class_name,
            "source_confidence": float(detection.confidence),
            "depth_min_m": float(config.depth_min_m),
            "depth_max_m": float(config.depth_max_m),
            "margin_px": float(config.margin_px),
        }
    )
    if geometry is None:
        payload["reason"] = "calibration_not_ready"
        return payload

    try:
        rois = project_search_bbox_to_stereo_rois(
            detection.bbox_xyxy,
            geometry,
            depth_min_m=float(config.depth_min_m),
            depth_max_m=float(config.depth_max_m),
            margin_px=float(config.margin_px),
        )
    except (TypeError, ValueError):
        payload["reason"] = "projection_failed"
        return payload

    if rois["front"] is None or rois["back"] is None:
        payload["reason"] = "projection_failed"
        return payload

    payload["valid"] = True
    payload["reason"] = "ok"
    payload["rois"] = {
"front": _roi_payload(rois["front"]),
"back": _roi_payload(rois["back"]),
    }
    return payload


def main(args=None) -> int:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    from .perception_geometry import load_camera_geometry

    class StereoRoiNode(Node):
        def __init__(self) -> None:
            super().__init__("stereo_roi_node")
            self.declare_parameter("search_detections_topic", "/search/detections_json")
            self.declare_parameter("rois_topic", "/stereo/rois_json")
            self.declare_parameter("calibration_path", "")
            self.declare_parameter("extrinsics_path", "")
            self.declare_parameter("strict_calibration", False)
            self.declare_parameter("publish_invalid", True)
            self.declare_parameter("min_confidence", 0.5)
            self.declare_parameter("class_id", -1)
            self.declare_parameter("class_name", "")
            self.declare_parameter("depth_min_m", 5.0)
            self.declare_parameter("depth_max_m", 100.0)
            self.declare_parameter("margin_px", 32.0)

            class_id = int(self.get_parameter("class_id").value)
            self._config = RoiProjectionConfig(
                min_confidence=float(self.get_parameter("min_confidence").value),
                class_id=None if class_id < 0 else class_id,
                class_name=str(self.get_parameter("class_name").value),
                depth_min_m=float(self.get_parameter("depth_min_m").value),
                depth_max_m=float(self.get_parameter("depth_max_m").value),
                margin_px=float(self.get_parameter("margin_px").value),
            )
            calibration_path = str(self.get_parameter("calibration_path").value).strip()
            extrinsics_path = str(self.get_parameter("extrinsics_path").value).strip()
            self._geometry: CameraGeometry | None = None
            try:
                if not calibration_path or not extrinsics_path:
                    raise ValueError("calibration_path and extrinsics_path are required")
                self._geometry = load_camera_geometry(calibration_path, extrinsics_path)
            except (ImportError, OSError, RuntimeError, TypeError, ValueError) as exc:
                if bool(self.get_parameter("strict_calibration").value):
                    raise
                self.get_logger().error(
                    "stereo ROI disabled until measured calibration is ready: %s" % exc
                )

            rois_topic = str(self.get_parameter("rois_topic").value)
            self._publish_invalid = bool(self.get_parameter("publish_invalid").value)
            self._pub = self.create_publisher(String, rois_topic, qos_profile_sensor_data)
            self._max_source_age_s = float(
                self.declare_parameter("max_source_age_s", 0.20).value
            )
            self._max_future_skew_s = float(
                self.declare_parameter("max_future_skew_s", 0.05).value
            )
            self.create_subscription(
                String,
                str(self.get_parameter("search_detections_topic").value),
                self._on_search_detections,
                qos_profile_sensor_data,
            )
            self.get_logger().info(
                "stereo_roi_node ready input=%s output=%s calibration=%s"
                % (
                    self.get_parameter("search_detections_topic").value,
                    rois_topic,
                    "loaded" if self._geometry is not None else "not-ready",
                )
            )

        def _on_search_detections(self, msg: String) -> None:
            now_s = self.get_clock().now().nanoseconds / 1_000_000_000.0
            config = RoiProjectionConfig(
                **{
                    **self._config.__dict__,
                    "max_source_age_s": self._max_source_age_s,
                    "max_future_skew_s": self._max_future_skew_s,
                }
            )
            payload = build_roi_projection_payload(
                msg.data, self._geometry, config, now_s=now_s
            )
            if payload is None:
                self.get_logger().warning("ignored malformed search detections JSON")
                return
            if not payload["valid"] and not self._publish_invalid:
                return
            output = String()
            output.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
            self._pub.publish(output)

    rclpy.init(args=args)
    node = None
    try:
        node = StereoRoiNode()
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
