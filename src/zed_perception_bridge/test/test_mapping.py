"""mapping.py 纯逻辑单测（无 ROS 依赖，PC 上直接 pytest）。

JSON 样例字段名逐字对齐 zed_perception_standalone 的 observations_json 输出
（不适用字段为 null），见 mapping.py 模块注释。
"""

import math

import pytest

from zed_perception_bridge import mapping


def make_frame(**overrides):
    frame = {
        "schema_version": 2,
        "run_id": "run-uuid",
        "boot_id": "boot-uuid",
        "frame_id": "zed_left_camera_optical_frame",
        "sequence": 42,
        "stamp_ns": 1_760_000_000_000_000_000,
        "image_width": 1920,
        "image_height": 1080,
        "detector_enabled": True,
        "observations_fresh": True,
        "targets": [],
    }
    frame.update(overrides)
    return frame


def make_target(**overrides):
    target = {
        "track_id": 7,
        "class_id": 0,
        "confidence": 0.83,
        "bbox_xyxy": [800.0, 400.0, 900.0, 460.0],
        "detector_source": "yolo",
        "score_kind": "class_confidence",
        "class_confirmed": True,
        "pixel_u": 850.0,
        "pixel_v": 430.0,
        "horizontal_angle_rad": -0.05,
        "vertical_angle_rad": 0.02,
        "mode": "depth",
        "position_valid": True,
        "depth_measured_valid": True,
        "depth_z_m": 6.2,
        "estimated_depth_z_m": None,
        "distance_m": 6.21,
        "position_camera": [-0.31, 0.124, 6.2],
        "reason": "accepted",
    }
    target.update(overrides)
    return target


class TestFrameGate:
    def test_valid_frame_passes(self):
        assert mapping.frame_reject_reason(make_frame()) is None

    def test_wrong_schema_version_rejected(self):
        assert mapping.frame_reject_reason(make_frame(schema_version=3)) is not None

    def test_missing_stamp_rejected(self):
        frame = make_frame()
        del frame["stamp_ns"]
        assert mapping.frame_reject_reason(frame).startswith("missing:")

    def test_non_positive_stamp_rejected(self):
        assert mapping.frame_reject_reason(make_frame(stamp_ns=0)) is not None

    def test_fresh_gate_requires_both_flags(self):
        assert mapping.frame_is_fresh(make_frame()) is True
        assert mapping.frame_is_fresh(make_frame(observations_fresh=False)) is False
        assert mapping.frame_is_fresh(make_frame(detector_enabled=False)) is False


class TestSelectTarget:
    def test_default_picks_highest_confidence(self):
        targets = [
            make_target(track_id=1, confidence=0.6),
            make_target(track_id=2, confidence=0.9),
        ]
        target, reason = mapping.select_target(targets)
        assert reason is None and target["track_id"] == 2

    def test_confidence_tie_breaks_by_bbox_area(self):
        targets = [
            make_target(track_id=1, confidence=0.9, bbox_xyxy=[0, 0, 50, 50]),
            make_target(track_id=2, confidence=0.9, bbox_xyxy=[0, 0, 200, 200]),
        ]
        target, _ = mapping.select_target(targets)
        assert target["track_id"] == 2

    def test_generic_excluded_by_default(self):
        targets = [make_target(class_confirmed=False, detector_source="generic")]
        target, reason = mapping.select_target(targets)
        assert target is None and reason == "no_class_confirmed_target"

    def test_generic_allowed_when_requested(self):
        targets = [make_target(class_confirmed=False, detector_source="generic")]
        target, _ = mapping.select_target(targets, allow_generic=True)
        assert target is not None

    def test_nearest_policy_uses_distance(self):
        targets = [
            make_target(track_id=1, distance_m=9.0),
            make_target(track_id=2, distance_m=5.0),
        ]
        target, _ = mapping.select_target(targets, policy="nearest")
        assert target["track_id"] == 2

    def test_nearest_policy_falls_back_without_distance(self):
        targets = [
            make_target(track_id=1, distance_m=None, confidence=0.3),
            make_target(track_id=2, distance_m=None, confidence=0.8),
        ]
        target, _ = mapping.select_target(targets, policy="nearest")
        assert target["track_id"] == 2

    def test_invalid_pixel_target_skipped(self):
        targets = [
            make_target(track_id=1, pixel_u=None),
            make_target(track_id=2, pixel_v=float("nan")),
            make_target(track_id=3),
        ]
        target, _ = mapping.select_target(targets)
        assert target["track_id"] == 3

    def test_empty_targets(self):
        target, reason = mapping.select_target([])
        assert target is None and reason == "no_valid_target"


class TestBuildObservation:
    def test_full_depth_mapping(self):
        fields = mapping.build_observation(make_frame(), make_target())
        assert fields["interface_version"] == 1
        assert fields["stamp_ns"] == 1_760_000_000_000_000_000
        assert fields["frame_id"] == "zed_left_camera_optical_frame"
        assert fields["sequence"] == 42
        assert fields["track_id"] == 7
        assert fields["visible"] is True
        assert fields["confidence"] == pytest.approx(0.83)
        assert fields["pixel_u"] == 850.0 and fields["pixel_v"] == 430.0
        assert fields["bounding_box_valid"] is True
        assert fields["bounding_box_width_px"] == pytest.approx(100.0)
        assert fields["bounding_box_height_px"] == pytest.approx(60.0)
        assert fields["angles_valid"] is True
        assert fields["horizontal_angle_rad"] == pytest.approx(-0.05)
        assert fields["depth_valid"] is True
        assert fields["depth_z_m"] == pytest.approx(6.2)
        assert fields["range_m"] == pytest.approx(6.21)
        assert fields["position_optical"] == pytest.approx([-0.31, 0.124, 6.2])
        assert len(fields["angle_covariance"]) == 4
        assert len(fields["position_covariance"]) == 9

    def test_generic_track_flag_masked(self):
        fields = mapping.build_observation(
            make_frame(), make_target(track_id=0x80000005, class_confirmed=False)
        )
        assert fields["track_id"] == 5

    def test_size_estimate_degrades_to_bearing_only(self):
        target = make_target(
            mode="size_estimate",
            depth_measured_valid=False,
            depth_z_m=None,
            estimated_depth_z_m=7.4,
            distance_m=7.5,
            position_camera=[-0.3, 0.1, 7.4],
        )
        fields = mapping.build_observation(make_frame(), target)
        assert fields["depth_valid"] is False
        assert fields["depth_z_m"] == 0.0
        assert fields["range_m"] == 0.0
        assert fields["position_optical"] == [0.0, 0.0, 0.0]

    def test_bearing_only_target(self):
        target = make_target(
            mode="bearing_only",
            position_valid=False,
            depth_measured_valid=False,
            depth_z_m=None,
            distance_m=None,
            position_camera=None,
        )
        fields = mapping.build_observation(make_frame(), target)
        assert fields["depth_valid"] is False
        assert fields["angles_valid"] is True

    def test_depth_flag_without_value_degrades(self):
        target = make_target(depth_measured_valid=True, depth_z_m=None)
        fields = mapping.build_observation(make_frame(), target)
        assert fields["depth_valid"] is False

    def test_missing_bbox_raises(self):
        with pytest.raises(mapping.BridgeMappingError):
            mapping.build_observation(make_frame(), make_target(bbox_xyxy=[1, 2, 3]))

    def test_missing_pixel_raises(self):
        with pytest.raises(mapping.BridgeMappingError):
            mapping.build_observation(make_frame(), make_target(pixel_u=None))

    def test_covariance_defaults_are_finite_and_positive(self):
        fields = mapping.build_observation(make_frame(), make_target())
        assert all(math.isfinite(v) and v >= 0.0 for v in fields["angle_covariance"])
        assert all(math.isfinite(v) and v >= 0.0 for v in fields["position_covariance"])
