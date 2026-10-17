"""Map detections from a cropped/resized ROI back to the source image."""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any, Mapping, Sequence

from .detection_utils import parse_detections_json, stamp_to_dict


@dataclass(frozen=True)
class RoiRect:
    x1: float
    y1: float
    x2: float
    y2: float
    image_width: int
    image_height: int

    @classmethod
    def from_mapping(cls, value: Mapping[str, Any]) -> "RoiRect":
        try:
            roi = cls(
                x1=float(value["x1"]),
                y1=float(value["y1"]),
                x2=float(value["x2"]),
                y2=float(value["y2"]),
                image_width=int(value["image_width"]),
                image_height=int(value["image_height"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("ROI requires x1/y1/x2/y2 and image dimensions") from exc
        if (
            roi.image_width <= 0
            or roi.image_height <= 0
            or roi.x2 <= roi.x1
            or roi.y2 <= roi.y1
        ):
            raise ValueError("ROI must have positive extent and image dimensions")
        if (
            roi.x1 < 0.0
            or roi.y1 < 0.0
            or roi.x2 > roi.image_width
            or roi.y2 > roi.image_height
        ):
            raise ValueError("ROI lies outside its source image")
        return roi


def backmap_bbox_xyxy(
    bbox_xyxy: Sequence[float],
    roi: RoiRect | Mapping[str, Any],
    crop_width: int,
    crop_height: int,
) -> tuple[float, float, float, float]:
    """Map one detector bbox from crop pixels to source-image pixels."""

    rect = roi if isinstance(roi, RoiRect) else RoiRect.from_mapping(roi)
    if len(bbox_xyxy) != 4 or crop_width <= 0 or crop_height <= 0:
        raise ValueError("bbox must contain four values and crop dimensions must be positive")
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    scale_x = (rect.x2 - rect.x1) / float(crop_width)
    scale_y = (rect.y2 - rect.y1) / float(crop_height)
    mapped = (
        rect.x1 + x1 * scale_x,
        rect.y1 + y1 * scale_y,
        rect.x1 + x2 * scale_x,
        rect.y1 + y2 * scale_y,
    )
    return (
        max(rect.x1, min(rect.x2, mapped[0])),
        max(rect.y1, min(rect.y2, mapped[1])),
        max(rect.x1, min(rect.x2, mapped[2])),
        max(rect.y1, min(rect.y2, mapped[3])),
    )


def _detection_payload_item(item, bbox_xyxy: Sequence[float]) -> dict[str, Any]:
    x1, y1, x2, y2 = [float(value) for value in bbox_xyxy]
    return {
        "class_id": int(item.class_id),
        "class_name": item.class_name,
        "confidence": float(item.confidence),
        "bbox_xyxy": [x1, y1, x2, y2],
        "bbox_xywh": [x1, y1, x2 - x1, y2 - y1],
        "bbox_center": [(x1 + x2) / 2.0, (y1 + y2) / 2.0],
    }


def backmap_detection_payload(
    cropped_json: str,
    roi_payload: Mapping[str, Any],
    stream: str,
    crop_width: int | None = None,
    crop_height: int | None = None,
) -> dict[str, Any] | None:
    """Backmap detections from one ROI stream into its source-image contract."""

    if not bool(roi_payload.get("valid")):
        return None
    rois = roi_payload.get("rois")
    if not isinstance(rois, Mapping):
        return None
    stream_key = str(stream).strip().lower()
    roi_value = rois.get(stream_key)
    if not isinstance(roi_value, Mapping):
        return None

    try:
        parsed = parse_detections_json(cropped_json)
        rect = RoiRect.from_mapping(roi_value)
        actual_crop_width = rect.x2 - rect.x1 if crop_width is None else int(crop_width)
        actual_crop_height = rect.y2 - rect.y1 if crop_height is None else int(crop_height)
        detections = [
            _detection_payload_item(
                item,
                backmap_bbox_xyxy(
                    item.bbox_xyxy,
                    rect,
                    int(actual_crop_width),
                    int(actual_crop_height),
                ),
            )
            for item in parsed.detections
        ]
    except (TypeError, ValueError, KeyError, json.JSONDecodeError):
        return None

    return {
        "header": {
            "stamp": None if parsed.stamp_s is None else stamp_to_dict(parsed.stamp_s),
            "frame_id": {
                "front": "stereo_front_camera",
                "back": "stereo_back_camera",
            }.get(stream_key, stream_key),
        },
        "image_width": rect.image_width,
        "image_height": rect.image_height,
        "source": "roi_backmap",
        "roi": {
            "x1": rect.x1,
            "y1": rect.y1,
            "x2": rect.x2,
            "y2": rect.y2,
        },
        "detections": detections,
    }
