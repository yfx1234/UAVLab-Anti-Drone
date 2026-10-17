"""Pure-logic tests for the search camera servo mode."""

import math

import pytest

from falcon_intercept.search_servo_node import (
    FireGate,
    RotationTracker,
    apply_lens_tilt,
    ServoCommands,
    body_minus_z_axis_ned,
    bounded_prediction_horizon,
    compute_servo_commands,
    los_azimuth_ned,
    los_body_frd_from_unit,
    low_pass,
    predict_bearing,
    quaternion_to_rotation_matrix,
    range_from_bbox_ratio,
    rotate_vector,
    unit_fru_from_bearing_angles,
    wrap_pi,
)


def _commands(**overrides) -> ServoCommands:
    kwargs = dict(
        horizontal_rad=0.0,
        vertical_rad=0.1745,
        range_m=7.5,
        desired_range_m=7.5,
        pitch_center_rad=0.1745,
        gain_forward=1.2,
        gain_right=1.2,
        gain_range=0.8,
        max_forward_mps=3.0,
        max_right_mps=2.0,
        max_vertical_mps=1.5,
        height_m=12.0,
        minimum_height_m=8.0,
    )
    kwargs.update(overrides)
    return compute_servo_commands(**kwargs)


def test_wrap_pi():
    assert wrap_pi(math.pi + 0.1) < 0.0
    assert abs(wrap_pi(3.0 * math.pi) - math.pi) < 1e-9
    assert wrap_pi(-3.0 * math.pi + 0.2) - (-math.pi + 0.2) < 1e-9
    assert wrap_pi(float("nan")) == 0.0


def test_range_from_bbox_ratio_scales_inversely():
    # Field calibration 2026-09-06: 55 px bbox at 10 m in a 2560-wide image.
    assert range_from_bbox_ratio(55.0, 2560.0, 10.0, 55.0, 2560.0) == 10.0
    assert range_from_bbox_ratio(110.0, 2560.0, 10.0, 55.0, 2560.0) == 5.0
    assert range_from_bbox_ratio(27.5, 2560.0, 10.0, 55.0, 2560.0) == 20.0


def test_range_from_bbox_ratio_is_resolution_independent():
    # The same target fraction of the image width must give the same range
    # regardless of the detection resolution.
    at_2560 = range_from_bbox_ratio(55.0, 2560.0, 10.0, 55.0, 2560.0)
    at_1280 = range_from_bbox_ratio(27.5, 1280.0, 10.0, 55.0, 2560.0)
    at_640 = range_from_bbox_ratio(13.75, 640.0, 10.0, 55.0, 2560.0)
    assert at_1280 == at_2560 == at_640 == 10.0


def test_range_from_bbox_ratio_gates_bad_input():
    assert range_from_bbox_ratio(4.0, 2560.0, 10.0, 55.0, 2560.0,
                                 minimum_bbox_px=8.0) is None
    assert range_from_bbox_ratio(100.0, 2560.0, 10.0, 0.0, 2560.0) is None
    assert range_from_bbox_ratio(100.0, 0.0, 10.0, 55.0, 2560.0) is None


def test_low_pass_seeds_and_converges():
    assert low_pass(None, 5.0, 0.3, 0.1) == 5.0
    value = low_pass(0.0, 10.0, 0.3, 0.1)
    assert 0.0 < value < 10.0
    for _ in range(200):
        value = low_pass(value, 10.0, 0.3, 0.1)
    assert abs(value - 10.0) < 0.1
    assert low_pass(3.0, 10.0, 0.3, 0.0) == 3.0


def test_prediction_horizon_accounts_for_frame_age_and_is_capped():
    requested, horizon, limited = bounded_prediction_horizon(0.12, 0.08, 0.35)
    assert requested == pytest.approx(0.20)
    assert horizon == pytest.approx(0.20)
    assert not limited

    requested, horizon, limited = bounded_prediction_horizon(0.50, 0.08, 0.35)
    assert requested == pytest.approx(0.58)
    assert horizon == pytest.approx(0.35)
    assert limited


def test_prediction_moves_with_filtered_angular_rate():
    prediction = predict_bearing(
        horizontal_rad=0.10,
        vertical_rad=-0.05,
        horizontal_rate_rad_s=0.20,
        vertical_rate_rad_s=-0.10,
        measurement_age_s=0.10,
        response_delay_s=0.05,
        maximum_horizon_s=0.35,
        maximum_rate_rad_s=0.70,
        maximum_angular_offset_rad=0.12,
    )
    assert prediction.horizon_s == pytest.approx(0.15)
    assert prediction.horizontal_rad == pytest.approx(0.13)
    assert prediction.vertical_rad == pytest.approx(-0.065)
    assert prediction.angular_offset_rad == pytest.approx(math.hypot(0.03, 0.015))
    assert not prediction.limited


def test_prediction_rate_and_offset_are_bounded():
    prediction = predict_bearing(
        horizontal_rad=0.0,
        vertical_rad=0.0,
        horizontal_rate_rad_s=10.0,
        vertical_rate_rad_s=0.0,
        measurement_age_s=1.0,
        response_delay_s=0.2,
        maximum_horizon_s=0.35,
        maximum_rate_rad_s=0.70,
        maximum_angular_offset_rad=0.12,
    )
    assert prediction.requested_horizon_s == pytest.approx(1.2)
    assert prediction.horizon_s == pytest.approx(0.35)
    assert prediction.horizontal_rad == pytest.approx(0.12)
    assert prediction.angular_offset_rad == pytest.approx(0.12)
    assert prediction.limited


def test_predicted_bearing_maps_back_to_the_same_angles():
    horizontal = 0.22
    vertical = -0.11
    x, y, z = unit_fru_from_bearing_angles(horizontal, vertical)
    assert math.atan2(x, z) == pytest.approx(horizontal)
    assert math.atan2(y, z) == pytest.approx(vertical)
    assert math.sqrt(x * x + y * y + z * z) == pytest.approx(1.0)


def test_rotation_identity_and_yaw():
    identity = quaternion_to_rotation_matrix((1.0, 0.0, 0.0, 0.0))
    assert rotate_vector(identity, (1.0, 2.0, 3.0)) == (1.0, 2.0, 3.0)
    # +90 deg yaw: body nose (x) points east.
    yaw90 = quaternion_to_rotation_matrix(
        (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
    )
    assert rotate_vector(yaw90, (1.0, 0.0, 0.0)) == pytest.approx((0.0, 1.0, 0.0))


def test_body_minus_z_axis_matches_referee_convention():
    assert body_minus_z_axis_ned((1.0, 0.0, 0.0, 0.0)) == (0.0, 0.0, -1.0)
    # 45 deg pitch up: the up-pointing lens tips toward the tail (-north).
    pitch45 = quaternion_to_rotation_matrix(
        (math.cos(math.pi / 8.0), 0.0, math.sin(math.pi / 8.0), 0.0)
    )
    axis = rotate_vector(pitch45, (0.0, 0.0, -1.0))
    assert axis == pytest.approx((-math.sqrt(0.5), 0.0, -math.sqrt(0.5)))
    assert body_minus_z_axis_ned(
        (math.cos(math.pi / 8.0), 0.0, math.sin(math.pi / 8.0), 0.0)
    ) == pytest.approx((-math.sqrt(0.5), 0.0, -math.sqrt(0.5)))


def test_los_frame_conversion():
    # Canonical forward maps to body x, canonical up maps to body -z.
    assert los_body_frd_from_unit((0.0, 1.0, 0.0)) == (1.0, 0.0, 0.0)
    assert los_body_frd_from_unit((0.0, 0.0, 1.0)) == (0.0, 0.0, -1.0)


def test_los_azimuth():
    assert los_azimuth_ned((1.0, 0.0, 0.0)) == 0.0
    assert abs(los_azimuth_ned((0.0, 1.0, 0.0)) - math.pi / 2) < 1e-9
    assert los_azimuth_ned((0.0, 0.0, 1.0)) is None


def test_pitch_error_drives_forward_command():
    # Target farther toward the nose than the station -> move forward.
    assert _commands(vertical_rad=0.35).forward_mps > 0.0
    # Target overhead (interceptor too far forward) -> move back.
    assert _commands(vertical_rad=0.0).forward_mps < 0.0
    # At the station -> zero.
    assert _commands(vertical_rad=0.1745).forward_mps == 0.0


def test_azimuth_error_drives_right_command():
    assert _commands(horizontal_rad=0.2).right_mps > 0.0
    assert _commands(horizontal_rad=-0.2).right_mps < 0.0


def test_range_error_drives_vertical_command():
    # Too far -> climb (negative down).
    assert _commands(range_m=12.0).down_mps < 0.0
    # Too close -> descend (positive down).
    assert _commands(range_m=5.0).down_mps > 0.0


def test_minimum_height_floor_blocks_descent():
    above_floor = _commands(range_m=5.0, height_m=12.0)
    below_floor = _commands(range_m=5.0, height_m=5.0, minimum_height_m=8.0)
    assert above_floor.down_mps > 0.0
    assert below_floor.down_mps == 0.0


def test_command_limits_are_respected():
    commands = _commands(
        horizontal_rad=5.0, vertical_rad=5.0, range_m=100.0,
        max_forward_mps=3.0, max_right_mps=2.0, max_vertical_mps=1.5,
    )
    assert commands.forward_mps == 3.0
    assert commands.right_mps == 2.0
    assert commands.down_mps == -1.5


def test_fire_gate_requires_hold_then_cooldown():
    gate = FireGate(hold_s=0.6, cooldown_s=3.0)
    assert not gate.update(0.0, True)      # starts holding
    assert not gate.update(0.3, True)      # still holding
    assert gate.update(0.7, True)          # fires
    assert gate.shot_id == 1
    # Leaving the window resets the hold; the cooldown still blocks.
    assert not gate.update(1.0, False)
    assert not gate.update(1.5, True)
    assert not gate.update(3.5, True)      # past cooldown, hold restarted at 1.5
    assert gate.update(4.2, True)          # 4.2 - 1.5 >= 0.6, cooldown over
    assert gate.shot_id == 2


def test_fire_gate_resets_hold_on_window_exit():
    gate = FireGate(hold_s=0.6, cooldown_s=0.0)
    assert not gate.update(0.0, True)
    assert not gate.update(0.5, False)     # exit resets
    assert not gate.update(0.7, True)      # hold restarts from 0.7
    assert not gate.update(1.1, True)
    assert gate.update(1.4, True)
    assert gate.shot_id == 1


def test_rotation_tracker_accumulates_wrapped_yaw():
    tracker = RotationTracker()
    tracker.update(3.1)
    tracker.update(-3.1)                   # wrap: +0.183 rad of motion
    assert abs(tracker.accumulated_rad - abs(wrap_pi(-3.1 - 3.1))) < 1e-9
    tracker.update(0.0)                    # +3.1 more
    assert tracker.progress == pytest.approx(
        min(1.0, (abs(wrap_pi(-3.1 - 3.1)) + 3.1) / (2 * math.pi))
    )


def test_rotation_tracker_completes_after_full_circle():
    tracker = RotationTracker()
    step = 0.1
    yaw = 0.0
    for _ in range(70):                    # 70 * 0.1 = 7 rad > 2 pi
        tracker.update(yaw)
        yaw += step
    assert tracker.completed
    assert tracker.progress == 1.0


def test_rotation_tracker_reset():
    tracker = RotationTracker()
    tracker.update(1.0)
    tracker.update(2.0)
    tracker.reset()
    assert tracker.accumulated_rad == 0.0
    assert not tracker.completed
    tracker.update(2.05)                   # seeds the previous yaw
    tracker.update(2.15)
    assert abs(tracker.accumulated_rad - 0.1) < 1e-9

def test_apply_lens_tilt_maps_lens_axis_to_nose():
    # Target at image centre (lens axis) is 10 deg nose-ward of body-up.
    tilted = apply_lens_tilt((0.0, 0.0, 1.0), math.radians(10.0))
    assert tilted[1] == pytest.approx(math.sin(math.radians(10.0)), abs=1e-9)
    assert tilted[2] == pytest.approx(math.cos(math.radians(10.0)), abs=1e-9)
    assert tilted[0] == 0.0


def test_apply_lens_tilt_maps_zenith_station_to_true_up():
    # The body-axis station (target 10 deg tail-ward in the published frame)
    # must map to straight body-up.
    unit = (0.0, -math.sin(math.radians(10.0)), math.cos(math.radians(10.0)))
    tilted = apply_lens_tilt(unit, math.radians(10.0))
    assert tilted == pytest.approx((0.0, 0.0, 1.0), abs=1e-9)

