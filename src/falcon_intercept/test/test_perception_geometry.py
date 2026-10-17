import json

import numpy as np
import pytest

from falcon_intercept.detection_utils import build_detection_json
from falcon_intercept.perception_geometry import (
    CameraGeometry,
    CameraIntrinsics,
    RigidTransform,
    project_search_bbox_to_roi,
    project_search_bbox_to_stereo_rois,
    rpy_deg_to_rotation,
)
from falcon_intercept.stereo_roi_node import RoiProjectionConfig, build_roi_projection_payload


def _test_intrinsics() -> CameraIntrinsics:
    return CameraIntrinsics(
        width=640,
        height=480,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
    )


def test_identity_geometry_preserves_bbox_without_margin():
    intrinsics = _test_intrinsics()
    roi = project_search_bbox_to_roi(
        (300.0, 220.0, 340.0, 260.0),
        intrinsics,
        intrinsics,
        RigidTransform.identity(),
        depth_min_m=10.0,
        depth_max_m=20.0,
        margin_px=0.0,
    )
    assert roi is not None
    assert (roi.x1, roi.y1, roi.x2, roi.y2) == (300, 220, 340, 260)


def test_stereo_projection_returns_two_clipped_rois():
    intrinsics = _test_intrinsics()
    geometry = CameraGeometry(
        search=intrinsics,
        stereo_front=intrinsics,
        stereo_back=intrinsics,
        search_to_front=RigidTransform.identity(),
        search_to_back=RigidTransform.identity(),
    )
    rois = project_search_bbox_to_stereo_rois(
        (-20.0, -20.0, 700.0, 500.0),
        geometry,
        depth_min_m=5.0,
        depth_max_m=10.0,
        margin_px=0.0,
    )
    assert rois["front"] is not None
    assert rois["back"] is not None
    assert (rois["front"].x1, rois["front"].y1, rois["front"].x2, rois["front"].y2) == (0, 0, 640, 480)


def test_rigid_transform_inverse_roundtrip():
    transform = RigidTransform(
        rpy_deg_to_rotation(2.0, -3.0, 15.0),
        np.array([0.4, -0.2, 0.1]),
    )
    point = np.array([8.0, 1.2, -0.7])
    assert np.allclose(transform.inverse().apply(transform.apply(point)), point)


def test_roi_projection_rejects_invalid_depth_interval():
    intrinsics = _test_intrinsics()
    with pytest.raises(ValueError):
        project_search_bbox_to_roi(
            (300.0, 220.0, 340.0, 260.0),
            intrinsics,
            intrinsics,
            RigidTransform.identity(),
            depth_min_m=0.0,
            depth_max_m=10.0,
        )


def test_stereo_roi_payload_projects_search_detection_with_contract_fields():
    intrinsics = _test_intrinsics()
    geometry = CameraGeometry(
        search=intrinsics,
        stereo_front=intrinsics,
        stereo_back=intrinsics,
        search_to_front=RigidTransform.identity(),
        search_to_back=RigidTransform.identity(),
    )
    payload = build_roi_projection_payload(
        build_detection_json(
            "search_camera",
            12.5,
            (320.0, 240.0),
            box_size=(40.0, 30.0),
            class_id=4,
            class_name="airplane",
        ),
        geometry,
        RoiProjectionConfig(
            min_confidence=0.5,
            class_id=4,
            class_name="airplane",
            depth_min_m=5.0,
            depth_max_m=20.0,
            margin_px=0.0,
        ),
    )
    assert payload is not None
    assert payload["valid"] is True
    assert payload["reason"] == "ok"
    assert payload["header"]["stamp"] == {"sec": 12, "nanosec": 500000000}
    assert payload["rois"]["front"] == {
        "x1": 300,
        "y1": 225,
        "x2": 340,
        "y2": 255,
        "width": 40,
        "height": 30,
        "image_width": 640,
        "image_height": 480,
    }
    json.dumps(payload)


def test_stereo_roi_payload_blocks_until_calibration_is_ready():
    payload = build_roi_projection_payload(
        build_detection_json("search_camera", 1.0, (320.0, 240.0)),
        None,
        RoiProjectionConfig(),
    )
    assert payload is not None
    assert payload["valid"] is False
    assert payload["reason"] == "calibration_not_ready"
    assert payload["rois"] == {"front": None, "back": None}
