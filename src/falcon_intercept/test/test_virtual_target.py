import math

import pytest

from falcon_intercept.stereo_target_solver import (
    PinholeStereoConfig,
    solve_pinhole_stereo_measurement,
    forward_right_up_covariance_to_optical,
    forward_right_up_to_optical,
    world_position_estimate_to_rig,
)
from falcon_intercept.virtual_target import (
    body_flu_to_ned,
    build_logical_detection_json,
    project_field_aligned_stereo,
    relative_ned_to_camera_fru,
    sample_trajectory_offset,
)


def test_body_camera_round_trip_at_identity_attitude():
    q = (1.0, 0.0, 0.0, 0.0)
    ned = body_flu_to_ned(q, (25.0, 3.0, 2.0))
    camera = relative_ned_to_camera_fru(ned, q, (0.0, 0.0, 0.0))
    assert camera == (25.0, -3.0, 2.0)


def test_field_aligned_projection_inverts_through_runtime_solver():
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
    expected = (25.0, 0.8, -0.4)
    projected = project_field_aligned_stereo(
        expected,
        fx=config.fx,
        fy=config.fy,
        cx=config.cx,
        cy=config.cy,
        baseline_m=config.baseline_m,
        vertical_offset_px=config.vertical_offset_front_minus_back_px,
        vertical_slope=config.vertical_slope_front_minus_back,
        vertical_reference_u_px=config.vertical_reference_u_px,
        horizontal_zero_px=config.horizontal_zero_disparity_offset_px,
        horizontal_sign=config.horizontal_disparity_sign,
    )
    assert projected is not None
    front = build_logical_detection_json(
        frame_id="front",
        stamp_s=10.0,
        image_width=3840,
        image_height=2160,
        center=projected.front,
        sequence=1,
    )
    back = build_logical_detection_json(
        frame_id="back",
        stamp_s=10.012,
        image_width=3840,
        image_height=2160,
        center=projected.back,
        sequence=1,
    )
    solved = solve_pinhole_stereo_measurement(front, back, config)
    assert solved is not None
    assert solved == pytest.approx(expected, abs=1e-8)


def test_trajectory_profiles_are_finite_and_stationary_is_fixed():
    fixed, velocity = sample_trajectory_offset("stationary", 12.0, 2.0, 20.0, 8.0)
    assert fixed == (0.0, 0.0, 0.0)
    assert velocity == (0.0, 0.0, 0.0)
    for scenario in ("straight", "circle", "s_curve", "figure_eight", "rectangle"):
        position, velocity = sample_trajectory_offset(
            scenario, 3.0, 2.0, 20.0, 8.0, 40.0
        )
        assert all(math.isfinite(value) for value in position + velocity)


def test_rectangle_starts_at_origin_and_turns_at_each_edge():
    origin, velocity = sample_trajectory_offset(
        "rectangle", 0.0, 2.0, 20.0, 8.0, 40.0
    )
    assert origin == (0.0, 0.0, 0.0)
    assert velocity == (2.0, 0.0, 0.0)
    corner, velocity = sample_trajectory_offset(
        "rectangle", 20.5, 2.0, 20.0, 8.0, 40.0
    )
    assert corner == pytest.approx((40.0, 1.0, 0.0))
    assert velocity == (0.0, 2.0, 0.0)


def test_world_estimate_is_returned_to_camera_relative_frame():
    position, covariance = world_position_estimate_to_rig(
        position_world=(25.0, 0.0, 5.0),
        covariance_world=((1.0, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 3.0)),
        rotation_world_from_rig=((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)),
        translation_world_rig=(0.0, 0.0, 5.0),
    )
    assert position == pytest.approx((25.0, 0.0, 0.0))
    assert covariance.diagonal() == pytest.approx((1.0, 2.0, 3.0))
    assert forward_right_up_to_optical(tuple(position)) == pytest.approx(
        (0.0, 0.0, 25.0)
    )
    assert forward_right_up_covariance_to_optical(
        covariance.reshape(-1)
    ) == pytest.approx((2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 1.0))
