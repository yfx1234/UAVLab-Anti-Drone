import json

from falcon_intercept.detection_utils import build_detection_json
from falcon_intercept.roi_mapping import (
    RoiRect,
    backmap_bbox_xyxy,
    backmap_detection_payload,
)


def _roi_payload():
    return {
        "valid": True,
        "rois": {
            "front": {
                "x1": 100,
                "y1": 200,
                "x2": 300,
                "y2": 400,
                "image_width": 3840,
                "image_height": 2160,
            },
            "back": {
                "x1": 110,
                "y1": 210,
                "x2": 310,
                "y2": 410,
                "image_width": 3840,
                "image_height": 2160,
            },
        },
    }


def test_backmap_bbox_accounts_for_crop_resize():
    roi = RoiRect(100.0, 200.0, 300.0, 400.0, 3840, 2160)
    assert backmap_bbox_xyxy((0.0, 0.0, 640.0, 480.0), roi, 640, 480) == (
        100.0,
        200.0,
        300.0,
        400.0,
    )
    assert backmap_bbox_xyxy((160.0, 120.0, 480.0, 360.0), roi, 640, 480) == (
        150.0,
        250.0,
        250.0,
        350.0,
    )


def test_backmap_detection_payload_restores_full_image_coordinates():
    cropped = build_detection_json(
        "roi_front",
        8.25,
        (320.0, 240.0),
        box_size=(320.0, 240.0),
        class_id=4,
        class_name="airplane",
    )
    payload = backmap_detection_payload(cropped, _roi_payload(), "front", 640, 480)
    assert payload is not None
    assert payload["header"] == {
        "stamp": {"sec": 8, "nanosec": 250000000},
        "frame_id": "stereo_front_camera",
    }
    assert payload["detections"][0]["bbox_xyxy"] == [150.0, 250.0, 250.0, 350.0]
    json.dumps(payload)


def test_backmap_rejects_invalid_roi_contract():
    assert backmap_detection_payload(
        build_detection_json("roi_front", 1.0, (10.0, 10.0)),
        {"valid": False, "rois": {"front": None}},
        "front",
    ) is None
