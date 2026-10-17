from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class DetectionBox:
    bbox_xyxy: tuple[float, float, float, float]
    bbox_center: tuple[float, float]
    confidence: float
    class_id: int = 0
    class_name: str = "target"


@dataclass(frozen=True)
class TimestampedDetections:
    stamp_s: float | None
    frame_id: str
    sequence: int | None
    detections: tuple[DetectionBox, ...]
    timestamp_source: str = "unknown"
    host_receive_time_s: float | None = None
    timestamp_sigma_s: float | None = None


def stamp_dict_to_seconds(stamp: Any) -> float | None:
    if not isinstance(stamp, dict):
        return None
    try:
        return float(stamp["sec"]) + float(stamp.get("nanosec", 0)) / 1_000_000_000.0
    except (KeyError, TypeError, ValueError):
        return None


def stamp_to_dict(stamp_s: float) -> dict[str, int]:
    sec = int(stamp_s)
    nanosec = int(round((float(stamp_s) - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    return {"sec": sec, "nanosec": nanosec}


def ros_stamp_to_seconds(stamp: Any) -> float:
    """ROS 消息 stamp 对象（sec/nanosec 属性，builtin_interfaces.Time 鸭子类型）转秒。"""
    return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0


def parse_detections_json(text: str) -> TimestampedDetections:
    payload = json.loads(text)
    header = payload.get("header", {})
    detections: list[DetectionBox] = []
    for item in payload.get("detections", []):
        bbox_xyxy = tuple(float(value) for value in item["bbox_xyxy"])
        if len(bbox_xyxy) != 4:
            continue
        center_value = item.get("bbox_center")
        if center_value is None:
            x1, y1, x2, y2 = bbox_xyxy
            center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)
        else:
            center = (float(center_value[0]), float(center_value[1]))
        detections.append(
            DetectionBox(
                bbox_xyxy=bbox_xyxy,
                bbox_center=center,
                confidence=float(item.get("confidence", 0.0)),
                class_id=int(item.get("class_id", 0)),
                class_name=str(item.get("class_name", "target")),
            )
        )
    return TimestampedDetections(
        stamp_s=stamp_dict_to_seconds(header.get("stamp")),
        frame_id=str(header.get("frame_id", "")),
        sequence=None if payload.get("sequence") is None else int(payload["sequence"]),
        detections=tuple(detections),
        timestamp_source=str(payload.get("timestamp_source", header.get("timestamp_source", "unknown"))),
        host_receive_time_s=stamp_dict_to_seconds(
            payload.get("host_receive_time", header.get("host_receive_time"))
        ),
        timestamp_sigma_s=(
            None
            if payload.get("timestamp_sigma_s", header.get("timestamp_sigma_s")) is None
            else float(payload.get("timestamp_sigma_s", header.get("timestamp_sigma_s")))
        ),
    )


def best_detection(text: str, min_confidence: float) -> DetectionBox | None:
    parsed = parse_detections_json(text)
    candidates = [box for box in parsed.detections if box.confidence >= float(min_confidence)]
    if not candidates:
        return None
    return max(candidates, key=lambda box: box.confidence)


def center_error_px(
    json_text: str,
    image_width: int,
    image_height: int,
    min_confidence: float,
) -> tuple[float, float] | None:
    try:
        detection = best_detection(json_text, min_confidence)
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if detection is None:
        return None
    center_x, center_y = detection.bbox_center
    return center_x - float(image_width) / 2.0, center_y - float(image_height) / 2.0


def fresh_center_error_px(
    json_text: str,
    image_width: int,
    image_height: int,
    min_confidence: float,
    reference_stamp_s: float,
    timeout_s: float,
) -> tuple[float, float] | None:
    try:
        parsed = parse_detections_json(json_text)
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if parsed.stamp_s is None:
        return None
    age_s = float(reference_stamp_s) - float(parsed.stamp_s)
    if age_s < 0.0 or age_s > float(timeout_s):
        return None
    candidates = [box for box in parsed.detections if box.confidence >= float(min_confidence)]
    if not candidates:
        return None
    center_x, center_y = max(candidates, key=lambda box: box.confidence).bbox_center
    return center_x - float(image_width) / 2.0, center_y - float(image_height) / 2.0


def build_detection_json(
    frame_id: str,
    stamp_s: float,
    center: tuple[float, float],
    box_size: tuple[float, float] = (48.0, 32.0),
    confidence: float = 0.9,
    class_id: int = 0,
    class_name: str = "target",
    sequence: int | None = None,
) -> str:
    center_x, center_y = center
    width, height = box_size
    x1 = float(center_x) - float(width) / 2.0
    y1 = float(center_y) - float(height) / 2.0
    x2 = float(center_x) + float(width) / 2.0
    y2 = float(center_y) + float(height) / 2.0
    payload = {
        "header": {"stamp": stamp_to_dict(stamp_s), "frame_id": frame_id},
        "detections": [
            {
                "class_id": int(class_id),
                "class_name": str(class_name),
                "confidence": float(confidence),
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_xywh": [x1, y1, float(width), float(height)],
                "bbox_center": [float(center_x), float(center_y)],
            }
        ],
    }
    if sequence is not None:
        payload["sequence"] = int(sequence)
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
