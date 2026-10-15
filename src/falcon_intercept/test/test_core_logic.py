import math
import json
from types import SimpleNamespace

import numpy as np

from falcon_intercept.detection_utils import build_detection_json, center_error_px, fresh_center_error_px
from falcon_intercept.guidance import GuidanceConfig, compute_approach_velocity, compute_search_yaw_rate
from falcon_intercept.intercept_fsm import FsmConfig, InterceptFsm, InterceptState, TargetObservation
from falcon_intercept.intercept_fsm_node import target_health_allows_state
from falcon_intercept.px4_adapter_node import (
    evaluate_adapter_gate,
    fallback_landed_from_vehicle_status,
    flu_to_frd,
    normalize_mode,
    quaternion_rotate_body_frd_to_ned,
)
from falcon_intercept.stereo_target_solver import (
    CAMERA_POSITION_QOS_DEPTH,
    PinholeStereoConfig,
    forward_right_up_covariance_to_optical,
    forward_right_up_to_optical,
    load_stereo_alignment_values,
    load_stereo_geometry_values,
    select_stereo_detection_pair,
    solve_pinhole_stereo_measurement,
)
from falcon_intercept.jetson_camera_node import _build_argus_pipeline
from falcon_intercept.v4l2_camera_node import _build_gstreamer_pipeline
from falcon_intercept.yolo_detections import extract_yolo_detections, parse_class_filter


def test_vehicle_status_landed_fallback_requires_takeoff_and_arm():
    assert fallback_landed_from_vehicle_status(armed=False, takeoff_time_us=0)
    assert fallback_landed_from_vehicle_status(armed=True, takeoff_time_us=0)
    assert not fallback_landed_from_vehicle_status(armed=True, takeoff_time_us=123456)
    assert fallback_landed_from_vehicle_status(armed=False, takeoff_time_us=123456)


def test_migrated_yolo_result_maps_to_project_detection_contract():
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            xyxy=np.array([[10.0, 20.0, 110.0, 220.0]]),
            conf=np.array([0.875]),
            cls=np.array([4.0]),
        )
    )
    detections = extract_yolo_detections(result, {4: "airplane"})
    assert detections == [
        {
            "class_id": 4,
            "class_name": "airplane",
            "confidence": 0.875,
            "bbox_xyxy": [10.0, 20.0, 110.0, 220.0],
            "bbox_xywh": [10.0, 20.0, 100.0, 200.0],
            "bbox_center": [60.0, 120.0],
        }
    ]


def test_yolo_class_filter_accepts_empty_and_csv_values():
    assert parse_class_filter("") is None
    assert parse_class_filter("4, 5") == [4, 5]


def test_detection_json_center_error_is_zero_at_image_center():
    payload = build_detection_json("front", 1.0, (320.0, 240.0))
    assert center_error_px(payload, 640, 480, 0.5) == (0.0, 0.0)


def test_detection_json_preserves_source_frame_sequence():
    from falcon_intercept.detection_utils import parse_detections_json

    payload = build_detection_json("front", 1.0, (320.0, 240.0), sequence=1258)
    assert parse_detections_json(payload).sequence == 1258


def test_internal_camera_coordinates_and_covariance_map_to_optical_frame():
    assert CAMERA_POSITION_QOS_DEPTH == 1
    assert forward_right_up_to_optical((30.25, 0.18, 0.07)) == (0.18, -0.07, 30.25)
    internal = (1.0, 0.1, 0.2, 0.1, 2.0, 0.3, 0.2, 0.3, 3.0)
    assert forward_right_up_covariance_to_optical(internal) == (
        2.0,
        -0.3,
        0.1,
        -0.3,
        3.0,
        -0.2,
        0.1,
        -0.2,
        1.0,
    )


def test_stale_detection_cannot_satisfy_center_contract():
    payload = build_detection_json("front", 1.0, (320.0, 240.0))
    assert fresh_center_error_px(payload, 640, 480, 0.5, 2.0, 0.5) is None


def test_guidance_limits_forward_velocity():
    command = compute_approach_velocity((10.0, 10.0, 10.0), GuidanceConfig())
    assert command == (0.8, 0.8, 0.5)


def test_search_yaw_rate_scans_then_tracks_pixel_error_with_limit():
    config = GuidanceConfig()
    assert compute_search_yaw_rate(None, config) == 0.2
    assert compute_search_yaw_rate(200.0, config) == 0.35
    assert compute_search_yaw_rate(-200.0, config) == -0.35


def test_stereo_solver_computes_forward_distance_from_disparity():
    front = build_detection_json("front", 1.0, (340.0, 240.0))
    back = build_detection_json("back", 1.0, (300.0, 240.0))
    position = solve_pinhole_stereo_measurement(
        front,
        back,
        PinholeStereoConfig(fx=320.0, fy=320.0, cx=320.0, cy=240.0, baseline_m=0.2),
    )
    assert position == (1.6, 0.1, -0.0)


def test_stereo_solver_applies_measured_horizontal_offset_and_negative_sign():
    config = PinholeStereoConfig(
        fx=10238.013493164,
        fy=10322.580645161,
        cx=1919.5,
        cy=1079.5,
        baseline_m=0.1,
        vertical_offset_front_minus_back_px=157.525168574,
        vertical_slope_front_minus_back=0.028807764872,
        vertical_reference_u_px=1919.5,
        horizontal_zero_disparity_offset_px=344.231858804,
        horizontal_disparity_sign=-1.0,
    )
    far_front = build_detection_json("front", 1.0, (2586.0, 628.0))
    far_back = build_detection_json("back", 1.0, (2440.0, 463.0))
    near_front = build_detection_json("front", 1.0, (849.0, 191.0))
    near_back = build_detection_json("back", 1.0, (915.0, 67.0))

    far_position = solve_pinhole_stereo_measurement(
        far_front, far_back, config
    )
    near_position = solve_pinhole_stereo_measurement(
        near_front, near_back, config
    )

    assert far_position is not None
    assert near_position is not None
    assert abs(far_position[0] - 4.74) <= 0.01
    assert abs(near_position[0] - 2.33) <= 0.01


def test_stereo_solver_rejects_unaligned_detection_timestamps():
    front = build_detection_json("front", 1.0, (340.0, 240.0))
    back = build_detection_json("back", 1.2, (300.0, 240.0))
    config = PinholeStereoConfig(
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        baseline_m=0.2,
        max_time_delta_s=0.05,
    )
    assert select_stereo_detection_pair(front, back, config) is None


def test_stereo_solver_pairs_same_class_and_rectified_centers():
    front = build_detection_json(
        "front", 1.0, (340.0, 240.0), confidence=0.8, class_id=4, class_name="airplane"
    )
    back = build_detection_json(
        "back", 1.02, (300.0, 244.0), confidence=0.9, class_id=4, class_name="airplane"
    )
    config = PinholeStereoConfig(
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        baseline_m=0.2,
        max_time_delta_s=0.05,
        max_vertical_error_px=8.0,
        class_id=4,
        class_name="airplane",
    )
    pair = select_stereo_detection_pair(front, back, config)
    assert pair is not None
    assert pair.timestamp_s == 1.01
    assert pair.front.class_id == pair.back.class_id == 4


def test_stereo_solver_compensates_locked_vertical_offset_and_roll():
    front = build_detection_json(
        "front", 1.0, (440.0, 393.0), class_id=4, class_name="airplane"
    )
    back = build_detection_json(
        "back", 1.0, (400.0, 240.0), class_id=4, class_name="airplane"
    )
    config = PinholeStereoConfig(
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        baseline_m=0.2,
        max_vertical_error_px=8.0,
        vertical_offset_front_minus_back_px=150.0,
        vertical_slope_front_minus_back=0.03,
        vertical_reference_u_px=300.0,
    )
    assert select_stereo_detection_pair(front, back, config) is not None


def test_stereo_solver_rejects_residual_after_field_alignment():
    front = build_detection_json("front", 1.0, (440.0, 410.0))
    back = build_detection_json("back", 1.0, (400.0, 240.0))
    config = PinholeStereoConfig(
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
        baseline_m=0.2,
        max_vertical_error_px=8.0,
        vertical_offset_front_minus_back_px=150.0,
        vertical_slope_front_minus_back=0.03,
        vertical_reference_u_px=300.0,
    )
    assert select_stereo_detection_pair(front, back, config) is None


def test_gstreamer_pipeline_uses_low_latency_mjpeg_decode():
    pipeline = _build_gstreamer_pipeline("/dev/video2", 640, 480, 30.0, "MJPG")
    assert 'v4l2src device="/dev/video2"' in pipeline
    assert "image/jpeg,width=640,height=480,framerate=30/1" in pipeline
    assert "jpegparse ! jpegdec" in pipeline
    assert "appsink drop=true max-buffers=1 sync=false" in pipeline
    hardware_pipeline = _build_gstreamer_pipeline("/dev/video2", 640, 480, 30.0, "MJPG", True)
    assert "nvv4l2decoder mjpeg=1" in hardware_pipeline


def test_argus_pipeline_uses_nvmm_and_bgr_appsink():
    pipeline = _build_argus_pipeline(1, 1920, 1080, 30.0, 0.0)
    assert "nvarguscamerasrc sensor-id=1" in pipeline
    assert "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12" in pipeline
    assert (
        "nvvidconv interpolation-method=2 ! "
        "video/x-raw,width=1920,height=1080,format=BGRx"
    ) in pipeline
    assert "appsink drop=true max-buffers=1 sync=false" in pipeline
    scaled_pipeline = _build_argus_pipeline(0, 640, 480, 30.0, 0.0)
    assert "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1,format=NV12" in scaled_pipeline
    assert (
        "nvvidconv interpolation-method=2 ! "
        "video/x-raw,width=640,height=480,format=BGRx"
    ) in scaled_pipeline


def test_load_stereo_geometry_values_uses_vendor_and_solidworks_yaml(tmp_path):
    calibration = tmp_path / "stereo_calib.yaml"
    calibration.write_text(
        """
stereo_front:
  fx: 10322.580645161
  fy: 10322.580645161
  cx: 1919.5
  cy: 1079.5
stereo_pair:
  baseline_m: 0.1
  field_alignment:
    vertical_offset_front_minus_back_px: 157.5
    vertical_slope_front_minus_back: 0.0288
    vertical_reference_u_px: 1919.5
    horizontal_zero_disparity_offset_px: 344.2
    horizontal_disparity_sign: -1
""",
        encoding="utf-8",
    )
    assert load_stereo_geometry_values(calibration) == (
        10322.580645161,
        10322.580645161,
        1919.5,
        1079.5,
        0.1,
    )
    assert load_stereo_alignment_values(calibration) == (
        157.5,
        0.0288,
        1919.5,
        344.2,
        -1.0,
    )


def test_fsm_reaches_fire_then_land_once_window_holds():
    fsm = InterceptFsm(
        FsmConfig(
            target_timeout_sec=1.0,
            fire_distance_m=2.0,
            center_threshold_px=80.0,
            fire_hold_frames=2,
            max_search_rounds=100,
            land_hold_sec=0.1,
        )
    )
    obs = TargetObservation(
        stamp_s=0.1,
        position_m=(1.5, 0.0, 0.0),
        front_center_error_px=(0.0, 0.0),
        back_center_error_px=(0.0, 0.0),
    )
    fsm.update_target(obs)
    assert fsm.step(0.1).state == InterceptState.TRACK
    assert fsm.step(0.2).state == InterceptState.APPROACH
    assert fsm.step(0.3).state == InterceptState.APPROACH
    fire = fsm.step(0.4)
    assert fire.state == InterceptState.FIRE
    assert fire.fire_event_json is not None
    land = fsm.step(0.5)
    assert land.state == InterceptState.LAND
    assert land.fire_event_json is None


def test_no_target_stays_search_then_lands_after_search_rounds():
    fsm = InterceptFsm(FsmConfig(max_search_rounds=3, search_round_period_sec=1.0))
    assert fsm.step(0.0).state == InterceptState.SEARCH
    assert fsm.step(0.5).state == InterceptState.SEARCH
    assert fsm.step(1.1).state == InterceptState.SEARCH
    assert fsm.step(2.2).state == InterceptState.SEARCH
    assert fsm.step(3.3).state == InterceptState.LAND


def test_far_centered_target_does_not_fire():
    fsm = InterceptFsm(FsmConfig(target_timeout_sec=1.0, fire_distance_m=2.0, fire_hold_frames=1))
    fsm.update_target(
        TargetObservation(
            stamp_s=0.1,
            position_m=(5.0, 0.0, 0.0),
            front_center_error_px=(0.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    assert fsm.step(0.1).state == InterceptState.TRACK
    assert fsm.step(0.2).state == InterceptState.APPROACH
    out = fsm.step(0.3)
    assert out.state == InterceptState.APPROACH
    assert out.fire_event_json is None


def test_close_offcenter_target_does_not_fire():
    fsm = InterceptFsm(
        FsmConfig(
            target_timeout_sec=1.0,
            fire_distance_m=2.0,
            center_threshold_px=80.0,
            fire_hold_frames=1,
        )
    )
    fsm.update_target(
        TargetObservation(
            stamp_s=0.1,
            position_m=(1.5, 0.0, 0.0),
            front_center_error_px=(120.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    assert fsm.step(0.1).state == InterceptState.TRACK
    assert fsm.step(0.2).state == InterceptState.APPROACH
    out = fsm.step(0.3)
    assert out.state == InterceptState.APPROACH
    assert out.fire_event_json is None


def test_centered_but_far_target_does_not_fire():
    fsm = InterceptFsm(FsmConfig(target_timeout_sec=1.0, fire_distance_m=2.0, fire_hold_frames=1))
    fsm.update_target(
        TargetObservation(
            stamp_s=0.1,
            position_m=(2.5, 0.0, 0.0),
            front_center_error_px=(0.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    fsm.step(0.1)
    fsm.step(0.2)
    out = fsm.step(0.3)
    assert out.state == InterceptState.APPROACH
    assert out.fire_event_json is None


def test_abort_in_approach_enters_abort_and_stops_control():
    fsm = InterceptFsm(FsmConfig(target_timeout_sec=1.0, max_search_rounds=100))
    fsm.update_target(
        TargetObservation(
            stamp_s=0.1,
            position_m=(5.0, 0.0, 0.0),
            front_center_error_px=(0.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    fsm.step(0.1)
    assert fsm.step(0.2).state == InterceptState.APPROACH
    out = fsm.abort(0.3)
    assert out.state == InterceptState.ABORT
    assert out.cmd_velocity == (0.0, 0.0, 0.0)


def test_init_waits_for_manual_offboard_mission_enable():
    fsm = InterceptFsm(FsmConfig(mission_start_enabled=False))
    assert fsm.step(0.0).state == InterceptState.INIT
    assert fsm.step(0.0).cmd_velocity == (0.0, 0.0, 0.0)
    fsm.set_mission_enabled(True)
    assert fsm.step(0.1).state == InterceptState.SEARCH


def test_land_waits_for_vehicle_landed_confirmation():
    fsm = InterceptFsm(
        FsmConfig(
            target_timeout_sec=1.0,
            fire_hold_frames=1,
            land_hold_sec=0.1,
            require_land_confirmation=True,
        )
    )
    fsm.update_target(
        TargetObservation(
            stamp_s=0.0,
            position_m=(1.0, 0.0, 0.0),
            front_center_error_px=(0.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    fsm.step(0.0)
    fsm.step(0.1)
    fsm.step(0.2)
    assert fsm.step(0.3).state == InterceptState.LAND
    assert fsm.step(1.0).state == InterceptState.LAND
    fsm.set_landed(True)
    assert fsm.step(1.1).state == InterceptState.DONE


def test_invalid_detection_json_returns_no_center_error():
    assert center_error_px("{bad json", 640, 480, 0.5) is None


def test_target_health_requires_valid_matching_measurement_timestamp():
    payload = json.dumps(
        {"valid": True, "last_measurement_timestamp_s": 10.0}, separators=(",", ":")
    )
    assert target_health_allows_state(payload, 10.02, 0.05)
    assert not target_health_allows_state(payload, 10.2, 0.05)
    assert not target_health_allows_state('{"valid":false}', 10.0)
    assert not target_health_allows_state("{bad", 10.0)


def test_nan_target_position_does_not_output_nonfinite_velocity():
    command = compute_approach_velocity((math.nan, 10.0, math.inf), GuidanceConfig())
    assert all(math.isfinite(value) for value in command)
    assert command == (0.0, 0.8, 0.0)


def test_short_target_loss_does_not_leave_approach_before_timeout():
    fsm = InterceptFsm(FsmConfig(target_timeout_sec=1.0, max_search_rounds=100))
    fsm.update_target(
        TargetObservation(
            stamp_s=0.1,
            position_m=(5.0, 0.0, 0.0),
            front_center_error_px=(0.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    fsm.step(0.1)
    assert fsm.step(0.2).state == InterceptState.APPROACH
    assert fsm.step(0.9).state == InterceptState.APPROACH


def test_target_loss_after_timeout_returns_search():
    fsm = InterceptFsm(FsmConfig(target_timeout_sec=0.5, max_search_rounds=100))
    fsm.update_target(
        TargetObservation(
            stamp_s=0.1,
            position_m=(5.0, 0.0, 0.0),
            front_center_error_px=(0.0, 0.0),
            back_center_error_px=(0.0, 0.0),
        )
    )
    fsm.step(0.1)
    assert fsm.step(0.2).state == InterceptState.APPROACH
    assert fsm.step(0.7).state == InterceptState.SEARCH


def test_px4_adapter_rotates_body_frd_velocity_to_ned():
    assert quaternion_rotate_body_frd_to_ned((1.0, 0.0, 0.0, 0.0), (1.0, 2.0, 3.0)) == (
        1.0,
        2.0,
        3.0,
    )
    rotated = quaternion_rotate_body_frd_to_ned(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)), (1.0, 0.0, 0.0)
    )
    assert np.allclose(rotated, (0.0, 1.0, 0.0), atol=1e-9)


def test_px4_adapter_converts_body_flu_to_frd_signs():
    assert flu_to_frd((1.0, 2.0, 3.0)) == (1.0, -2.0, -3.0)


def test_shadow_adapter_never_authorizes_px4_input_publication():
    gate = evaluate_adapter_gate(
        mode="shadow",
        command_present=True,
        command_valid=True,
        command_age_s=0.01,
        command_timeout_s=0.25,
        invalid_elapsed_s=0.0,
        zero_hold_s=0.5,
        attitude_fresh=True,
        local_position_fresh=True,
        status_fresh=True,
        failsafe=False,
        takeover_latched=False,
        armed=True,
        landed=False,
        offboard=True,
    )
    assert not gate.publish_heartbeat
    assert not gate.apply_command


def test_live_adapter_requires_manual_takeoff_then_manual_px4_offboard():
    on_ground = evaluate_adapter_gate(
        mode="live",
        command_present=True,
        command_valid=True,
        command_age_s=0.01,
        command_timeout_s=0.25,
        invalid_elapsed_s=0.0,
        zero_hold_s=0.5,
        attitude_fresh=True,
        local_position_fresh=True,
        status_fresh=True,
        failsafe=False,
        takeover_latched=False,
        armed=True,
        landed=True,
        offboard=False,
    )
    assert not on_ground.publish_heartbeat
    assert not on_ground.apply_command
    assert on_ground.reason == "manual arm and takeoff required"

    preheat = evaluate_adapter_gate(
        mode="live",
        command_present=True,
        command_valid=True,
        command_age_s=0.01,
        command_timeout_s=0.25,
        invalid_elapsed_s=0.0,
        zero_hold_s=0.5,
        attitude_fresh=True,
        local_position_fresh=True,
        status_fresh=True,
        failsafe=False,
        takeover_latched=False,
        armed=True,
        landed=False,
        offboard=False,
    )
    assert preheat.publish_heartbeat
    assert not preheat.apply_command
    allowed = evaluate_adapter_gate(
        mode="live",
        command_present=True,
        command_valid=True,
        command_age_s=0.01,
        command_timeout_s=0.25,
        invalid_elapsed_s=0.0,
        zero_hold_s=0.5,
        attitude_fresh=True,
        local_position_fresh=True,
        status_fresh=True,
        failsafe=False,
        takeover_latched=False,
        armed=True,
        landed=False,
        offboard=True,
    )
    assert allowed.publish_heartbeat
    assert allowed.apply_command


def test_live_adapter_stops_output_for_px4_failsafe_or_invalid_local_velocity():
    common = dict(
        mode="live",
        command_present=True,
        command_valid=True,
        command_age_s=0.01,
        command_timeout_s=0.25,
        invalid_elapsed_s=0.0,
        zero_hold_s=0.5,
        attitude_fresh=True,
        local_position_fresh=True,
        status_fresh=True,
        failsafe=False,
        takeover_latched=False,
        armed=True,
        landed=False,
        offboard=True,
    )
    failsafe = evaluate_adapter_gate(**{**common, "failsafe": True})
    assert not failsafe.publish_heartbeat
    assert not failsafe.apply_command
    invalid_velocity = evaluate_adapter_gate(
        **{**common, "local_position_fresh": False}
    )
    assert not invalid_velocity.publish_heartbeat
    assert not invalid_velocity.apply_command
    takeover = evaluate_adapter_gate(**{**common, "takeover_latched": True})
    assert not takeover.publish_heartbeat
    assert not takeover.apply_command


def test_invalid_adapter_mode_is_rejected():
    assert normalize_mode("dry_run") == "dry-run"
    with np.testing.assert_raises(ValueError):
        normalize_mode("attitude")
