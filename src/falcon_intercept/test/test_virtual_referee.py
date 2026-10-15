import pytest

from falcon_intercept.virtual_referee import evaluate_virtual_shot


def test_referee_hits_stationary_target_on_forward_axis():
    result = evaluate_virtual_shot(
        shooter_position_ned=(0.0, 0.0, 0.0),
        shooter_q_body_to_ned=(1.0, 0.0, 0.0, 0.0),
        target_position_ned=(25.0, 0.0, 0.0),
        target_velocity_ned=(0.0, 0.0, 0.0),
        gravity_mps2=0.0,
        hit_radius_m=0.2,
    )
    assert result.hit
    assert result.closest_time_s == pytest.approx(25.0 / 400.0, abs=0.001)


def test_referee_rejects_large_lateral_miss():
    result = evaluate_virtual_shot(
        shooter_position_ned=(0.0, 0.0, 0.0),
        shooter_q_body_to_ned=(1.0, 0.0, 0.0, 0.0),
        target_position_ned=(25.0, 5.0, 0.0),
        target_velocity_ned=(0.0, 0.0, 0.0),
        gravity_mps2=0.0,
        hit_radius_m=1.0,
    )
    assert not result.hit
    assert result.miss_distance_m == pytest.approx(5.0, abs=0.01)
