"""Synthetic-rig tests for the point-level rectified stereo geometry."""

from pathlib import Path

import numpy as np
import pytest
import yaml

from falcon_intercept.rectified_stereo import (
    CV_TO_FRU,
    RectifiedStereoGeometry,
    geometry_from_dict,
    load_rectified_stereo_geometry,
    undistort_normalized,
)


def _synthetic_rig():
    rng = np.random.default_rng(7)
    k_front = np.array([[9940.0, 0.0, 1919.6], [0.0, 9933.0, 730.3], [0.0, 0.0, 1.0]])
    k_back = np.array([[9959.0, 0.0, 1983.9], [0.0, 9956.3, 758.5], [0.0, 0.0, 1.0]])
    d_front = np.array([0.0601, -0.7221, -0.0063, -0.0019, 0.0])
    d_back = np.array([0.0531, -0.6365, -0.0072, 0.0017, 0.0])
    angle = np.deg2rad(1.0)
    rotation = np.array(
        [
            [np.cos(angle), -np.sin(angle), 0.0],
            [np.sin(angle), np.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    translation = np.array([-0.0991, -0.0011, -0.0003])
    return k_front, d_front, k_back, d_back, rotation, translation


def _stereo_rectify(k1, d1, k2, d2, size, rotation, translation):
    """numpy re-implementation of cv2.stereoRectify(alpha=0) essentials.

    Common-orientation construction: R2 = R1 R^T makes both virtual cameras
    share orientation R1 in the front frame; the R1 x axis is chosen along
    -(R^T t) so the back rectified camera sits at +x and disparity stays
    positive.
    """

    a = rotation.T @ translation
    x_axis = -a / np.linalg.norm(a)
    z0 = np.array([0.0, 0.0, 1.0])
    z_axis = z0 - x_axis * (x_axis @ z0)
    z_axis /= np.linalg.norm(z_axis)
    y_axis = np.cross(z_axis, x_axis)
    r_rect = np.stack([x_axis, y_axis, z_axis], axis=0)
    r1 = r_rect
    r2 = r_rect @ rotation.T
    fx = 0.5 * (k1[0, 0] + k2[0, 0])
    fy = 0.5 * (k1[1, 1] + k2[1, 1])
    cx = 0.5 * (k1[0, 2] + k2[0, 2])
    cy = 0.5 * (k1[1, 2] + k2[1, 2])
    baseline = float(np.linalg.norm(translation))
    p1 = np.array([[fx, 0.0, cx, 0.0], [0.0, fy, cy, 0.0], [0.0, 0.0, 1.0, 0.0]])
    p2 = np.array([[fx, 0.0, cx, -fx * baseline], [0.0, fy, cy, 0.0], [0.0, 0.0, 1.0, 0.0]])
    return r1, r2, p1, p2


def _project_point(point_cv, camera_matrix, distortion):
    x = point_cv[0] / point_cv[2]
    y = point_cv[1] / point_cv[2]
    k1, k2, p1, p2 = distortion[:4]
    k3 = distortion[4] if len(distortion) > 4 else 0.0
    r2 = x * x + y * y
    radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
    x_d = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
    y_d = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
    return (
        camera_matrix[0, 0] * x_d + camera_matrix[0, 2],
        camera_matrix[1, 1] * y_d + camera_matrix[1, 2],
    )


def _geometry():
    k1, d1, k2, d2, rotation, translation = _synthetic_rig()
    r1, r2, p1, p2 = _stereo_rectify(k1, d1, k2, d2, (3840, 2160), rotation, translation)
    return RectifiedStereoGeometry(
        k_front=k1,
        d_front=d1,
        k_back=k2,
        d_back=d2,
        rotation_left_to_right=rotation,
        translation_left_to_right_m=translation,
        r1=r1,
        r2=r2,
        p1=p1,
        p2=p2,
        q=np.eye(4),
        baseline_m=float(np.linalg.norm(translation)),
    )


def test_undistort_normalized_inverts_forward_distortion():
    k1, d1, _, _, _, _ = _synthetic_rig()
    rng = np.random.default_rng(3)
    us = rng.uniform(100, 3700, 64)
    vs = rng.uniform(100, 2000, 64)
    xn, yn = undistort_normalized(us, vs, k1, d1)
    for u, v, x, y in zip(us, vs, xn, yn):
        u2, v2 = _project_point(np.array([x, y, 1.0]), k1, d1)
        assert abs(u2 - u) < 1e-4
        assert abs(v2 - v) < 1e-4


def test_rectified_pair_is_horizontal_and_triangulates():
    geometry = _geometry()
    rng = np.random.default_rng(11)
    for _ in range(32):
        point_front = np.array(
            [rng.uniform(-2.0, 2.0), rng.uniform(-1.0, 1.0), rng.uniform(2.0, 30.0)]
        )
        point_back = geometry.rotation_left_to_right @ point_front + geometry.translation_left_to_right_m
        ul, vl = _project_point(point_front, geometry.k_front, geometry.d_front)
        ur, vr = _project_point(point_back, geometry.k_back, geometry.d_back)
        rul, rvl = geometry.rectify_front(ul, vl)
        rur, rvr = geometry.rectify_back(ur, vr)
        assert abs(rvl - rvr) < 1e-6
        disparity = rul - rur
        assert disparity > 0.0
        depth = geometry.fx_rect * geometry.baseline_rect_m / disparity
        assert depth == pytest.approx(
            (geometry.r1 @ point_front)[2], rel=1e-9
        )
        point_rect = geometry.triangulate_rectified(rul, rvl, rur)
        np.testing.assert_allclose(point_rect, geometry.r1 @ point_front, atol=1e-9)
        point_fru = geometry.triangulate_fru(ul, vl, ur, vr)
        np.testing.assert_allclose(point_fru, CV_TO_FRU @ point_front, atol=1e-9)


def test_pair_metrics_zero_vertical_error_for_true_match():
    geometry = _geometry()
    point_front = np.array([0.3, -0.2, 8.0])
    point_back = geometry.rotation_left_to_right @ point_front + geometry.translation_left_to_right_m
    ul, vl = _project_point(point_front, geometry.k_front, geometry.d_front)
    ur, vr = _project_point(point_back, geometry.k_back, geometry.d_back)
    disparity, vertical = geometry.pair_metrics(ul, vl, ur, vr)
    assert vertical < 1e-6
    assert disparity == pytest.approx(
        geometry.fx_rect * geometry.baseline_rect_m / (geometry.r1 @ point_front)[2],
        rel=1e-9,
    )


def test_rotation_fru_rectified_to_front_roundtrip():
    geometry = _geometry()
    rotation = geometry.rotation_fru_rectified_to_front()
    point_front = np.array([5.0, 1.0, -0.5])
    point_rect_fru = CV_TO_FRU @ geometry.r1 @ (CV_TO_FRU.T @ point_front)
    np.testing.assert_allclose(rotation @ point_rect_fru, point_front, atol=1e-12)


def test_geometry_from_dict_and_legacy_detection(tmp_path: Path):
    geometry = _geometry()
    payload = {
        "stereo_front": {
            "fx": float(geometry.k_front[0, 0]),
            "fy": float(geometry.k_front[1, 1]),
            "cx": float(geometry.k_front[0, 2]),
            "cy": float(geometry.k_front[1, 2]),
            "distortion": geometry.d_front.tolist(),
        },
        "stereo_back": {
            "fx": float(geometry.k_back[0, 0]),
            "fy": float(geometry.k_back[1, 1]),
            "cx": float(geometry.k_back[0, 2]),
            "cy": float(geometry.k_back[1, 2]),
            "distortion": geometry.d_back.tolist(),
        },
        "stereo_pair": {
            "baseline_m": geometry.baseline_m,
            "rotation_left_to_right_opencv": geometry.rotation_left_to_right.tolist(),
            "translation_left_to_right_m": geometry.translation_left_to_right_m.tolist(),
            "rectification": {
                "R1": geometry.r1.tolist(),
                "R2": geometry.r2.tolist(),
                "P1": geometry.p1.tolist(),
                "P2": geometry.p2.tolist(),
                "Q": np.eye(4).tolist(),
            },
        },
    }
    path = tmp_path / "rectified.yaml"
    path.write_text(yaml.safe_dump(payload))
    loaded = load_rectified_stereo_geometry(path)
    assert loaded is not None
    assert loaded.fx_rect == pytest.approx(geometry.fx_rect)
    assert loaded.baseline_rect_m == pytest.approx(geometry.baseline_rect_m)

    legacy = tmp_path / "legacy.yaml"
    legacy.write_text(
        yaml.safe_dump(
            {
                "stereo_front": {"fx": 1.0, "fy": 1.0, "cx": 0.0, "cy": 0.0},
                "stereo_pair": {"baseline_m": 0.1},
            }
        )
    )
    assert load_rectified_stereo_geometry(legacy) is None


from falcon_intercept.detection_utils import build_detection_json
from falcon_intercept.stereo_target_solver import (
    PinholeStereoConfig,
    solve_pinhole_stereo_measurement,
    stereo_pair_metrics,
    triangulate_stereo_pair,
)
from falcon_intercept.async_state_estimator import (
    AsyncStereoConfig,
    AsyncStereoStateEstimator,
    PixelObservation,
)
from falcon_intercept.detection_utils import parse_detections_json


def _solver_config(rectified=True):
    return PinholeStereoConfig(
        fx=9940.0,
        fy=9933.0,
        cx=1919.6,
        cy=730.3,
        baseline_m=0.0991,
        rectified=_geometry() if rectified else None,
    )


def test_solver_rectified_solve_recovers_target():
    geometry = _geometry()
    config = _solver_config(rectified=True)
    point_front = np.array([0.5, 0.3, 10.0])  # CV frame of the raw front camera
    point_back = geometry.rotation_left_to_right @ point_front + geometry.translation_left_to_right_m
    ul, vl = _project_point(point_front, geometry.k_front, geometry.d_front)
    ur, vr = _project_point(point_back, geometry.k_back, geometry.d_back)
    front_json = build_detection_json("front", 1.0, (ul, vl))
    back_json = build_detection_json("back", 1.0, (ur, vr))
    solved = solve_pinhole_stereo_measurement(front_json, back_json, config)
    assert solved is not None
    expected_fru = CV_TO_FRU @ point_front
    np.testing.assert_allclose(solved, tuple(expected_fru), atol=1e-6)


def test_solver_rectified_rejects_vertical_mismatch():
    geometry = _geometry()
    config = _solver_config(rectified=True)
    point_front = np.array([0.5, 0.3, 10.0])
    point_back = geometry.rotation_left_to_right @ point_front + geometry.translation_left_to_right_m
    ul, vl = _project_point(point_front, geometry.k_front, geometry.d_front)
    ur, vr = _project_point(point_back, geometry.k_back, geometry.d_back)
    front_json = build_detection_json("front", 1.0, (ul, vl))
    back_json = build_detection_json("back", 1.0, (ur, vr + 30.0))
    assert solve_pinhole_stereo_measurement(front_json, back_json, config) is None


def test_solver_legacy_mode_unchanged_when_no_rectified():
    config = _solver_config(rectified=False)
    front_json = build_detection_json("front", 1.0, (340.0, 240.0))
    back_json = build_detection_json("back", 1.0, (300.0, 240.0))
    position = solve_pinhole_stereo_measurement(front_json, back_json, config)
    assert position is not None
    assert position[0] > 0.0


def test_async_estimator_runs_on_rectified_pixels():
    geometry = _geometry()
    config = AsyncStereoConfig(
        fx=geometry.fx_rect,
        fy=geometry.fy_rect,
        cx=geometry.cx_rect,
        cy=geometry.cy_rect,
        baseline_m=geometry.baseline_rect_m,
        vertical_offset_front_minus_back_px=0.0,
        vertical_slope_front_minus_back=0.0,
        vertical_reference_u_px=0.0,
        horizontal_zero_disparity_offset_px=0.0,
        horizontal_disparity_sign=1.0,
    )
    estimator = AsyncStereoStateEstimator(config)
    # Target moves forward in the rectified-front FRU frame.
    def target(t):
        return np.array([10.0 + 1.0 * t, 0.4, -0.2])

    def observe(t, camera_id):
        point = target(t)
        half = -0.5 * config.baseline_m if camera_id == "front" else 0.5 * config.baseline_m
        y_rel = point[1] - half
        u = config.fx * y_rel / point[0] + config.cx
        v = config.cy - config.fy * point[2] / point[0]
        return PixelObservation(camera_id=camera_id, timestamp_s=t, u=u, v=v)

    accepted = 0
    for index in range(6):
        t = 0.02 * index
        result = estimator.update(observe(t, "front"), observe(t + 0.01, "back"))
        accepted += int(result.accepted)
    assert estimator.initialized
    assert accepted >= 3
    expected_range = float(np.linalg.norm(target(0.1)))
    assert result.range_m == pytest.approx(expected_range, rel=0.05)
