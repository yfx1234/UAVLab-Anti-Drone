from __future__ import annotations

from typing import Any

import cv2
import numpy as np


def _to_numpy(value: Any) -> np.ndarray:
    """Convert a torch/numpy-like Ultralytics value to a numpy array."""

    if hasattr(value, "detach"):
        value = value.detach()
    if hasattr(value, "cpu"):
        value = value.cpu()
    if hasattr(value, "numpy"):
        value = value.numpy()
    return np.asarray(value)


def resolve_class_name(names: dict[int, str] | list[str] | None, class_id: int) -> str:
    if isinstance(names, dict):
        return str(names.get(class_id, class_id))
    if isinstance(names, list) and 0 <= class_id < len(names):
        return str(names[class_id])
    return str(class_id)


def extract_yolo_detections(
    result: Any,
    names: dict[int, str] | list[str] | None,
    class_name_override: str = "",
) -> list[dict[str, Any]]:
    """Convert one Ultralytics result to the project's JSON detection schema."""

    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return []

    xyxy_values = _to_numpy(getattr(boxes, "xyxy", []))
    confidence_values = _to_numpy(getattr(boxes, "conf", []))
    class_values = _to_numpy(getattr(boxes, "cls", []))
    if xyxy_values.size == 0 or confidence_values.size == 0:
        return []

    xyxy_values = np.asarray(xyxy_values, dtype=float).reshape((-1, 4))
    confidence_values = np.asarray(confidence_values, dtype=float).reshape(-1)
    if class_values.size:
        class_values = np.asarray(class_values, dtype=int).reshape(-1)
    else:
        class_values = np.zeros(len(xyxy_values), dtype=int)

    detections: list[dict[str, Any]] = []
    for xyxy, confidence, class_id in zip(xyxy_values, confidence_values, class_values):
        x1, y1, x2, y2 = [float(value) for value in xyxy]
        name = class_name_override.strip() or resolve_class_name(names, int(class_id))
        detections.append(
            {
                "class_id": int(class_id),
                "class_name": name,
                "confidence": float(confidence),
                "bbox_xyxy": [x1, y1, x2, y2],
                "bbox_xywh": [x1, y1, x2 - x1, y2 - y1],
                "bbox_center": [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
            }
        )
    return detections


def parse_class_filter(value: str) -> list[int] | None:
    value = str(value).strip()
    if not value:
        return None
    return [int(part.strip()) for part in value.split(",") if part.strip()]


def draw_detections(frame: np.ndarray, detections: list[dict[str, Any]]) -> np.ndarray:
    annotated = frame.copy()
    for detection in detections:
        x1, y1, x2, y2 = [int(round(value)) for value in detection["bbox_xyxy"]]
        label = "%s %.2f" % (detection["class_name"], float(detection["confidence"]))
        cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(
            annotated,
            label,
            (x1, max(15, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
    return annotated
