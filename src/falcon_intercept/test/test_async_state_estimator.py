import numpy as np

from falcon_intercept.async_state_estimator import (
    AsyncStereoConfig,
    AsyncStereoStateEstimator,
    PX4PoseHistory,
    simulate_async_pair,
)


def _run_motion(
    delta_s: float,
    *,
    timestamp_sigma_s: float = 0.0015,
    velocity_y_mps: float = 5.0,
):
    config = AsyncStereoConfig(timestamp_sigma_s=timestamp_sigma_s)
    estimator = AsyncStereoStateEstimator(config)
    position = np.array([20.0, 0.0, 0.0])
    velocity = np.array([0.0, velocity_y_mps, 0.0])
    results = []
    for index in range(8):
        t_left = index / 30.0
        t_right = t_left + delta_s
        midpoint_position = position + velocity * (0.5 * (t_left + t_right))
        results.append(
            estimator.update(
                *simulate_async_pair(
                    midpoint_position,
                    velocity,
                    t_left,
                    t_right,
                    config,
                )
            )
        )
    return estimator, results


def test_batch_initialization_recovers_fast_baseline_motion():
    estimator, results = _run_motion(0.0121)
    assert results[0].reason == "initializing_batch"
    assert results[1].reason == "initializing_batch"
    assert results[2].accepted
    assert np.isclose(results[-1].position[0], 20.0, atol=1e-3)
    assert np.isclose(results[-1].velocity[1], 5.0, atol=1e-3)
    assert results[-1].reprojection_error_px < 1e-6
    assert estimator.accepted_count >= 8


def test_static_target_does_not_gain_false_velocity_for_board_deltas():
    for delta_s in (0.0, 0.0055, 0.0121, 0.0141):
        _, results = _run_motion(delta_s, velocity_y_mps=0.0)
        assert results[-1].accepted
        assert np.linalg.norm(results[-1].velocity) < 1e-3


def test_oosm_is_replayed_and_old_observation_is_dropped():
    estimator, _ = _run_motion(0.0121)
    position = np.array([20.0, 0.0, 0.0])
    velocity = np.array([0.0, 5.0, 0.0])
    current = estimator.update(
        *simulate_async_pair(position + velocity * 0.30605, velocity, 0.30, 0.3121)
    )
    assert current.accepted
    late = estimator.update(
        *simulate_async_pair(position + velocity * 0.27605, velocity, 0.27, 0.2821)
    )
    assert late.accepted
    assert late.reason == "accepted_oosm_replay"
    assert late.oosm_count == 1
    dropped = estimator.update(
        *simulate_async_pair(position + velocity * -0.2, velocity, -0.2, -0.1879)
    )
    assert not dropped.accepted
    assert dropped.reason == "OOSM_DROP"
    assert dropped.oosm_drop_count == 1


def test_timestamp_uncertainty_inflates_pixel_measurement_covariance():
    config = AsyncStereoConfig(timestamp_sigma_s=0.01)
    estimator = AsyncStereoStateEstimator(config)
    estimator.state = np.array([20.0, 0.0, 0.0, 0.0, 5.0, 0.0])
    estimator.reference_time_s = 0.0
    front, _ = simulate_async_pair((20.0, 0.0, 0.0), (0.0, 5.0, 0.0), 0.0, 0.0121, config)
    covariance = estimator._measurement_covariance(front, estimator.state)
    assert covariance[0, 0] > config.pixel_variance_px2


def test_px4_history_interpolates_ned_position_and_frd_attitude_without_enu_mix():
    history = PX4PoseHistory()
    history.add_position(10.0, (3.0, 4.0, 5.0), receive_time_s=100.0)
    history.add_position(11.0, (5.0, 6.0, 7.0))
    history.add_attitude(10.0, (1.0, 0.0, 0.0, 0.0))
    rotation, translation = history.pose_at(100.5)
    assert np.allclose(rotation, np.eye(3))
    assert np.allclose(translation, [4.0, 5.0, -6.0])


def test_batch_initialization_trims_outlier_pair():
    """A mixed batch (good pairs at 20 m + one false match at 60 m) must
    fail the trimmed attempt, then initialize from the inlier majority once
    the buffer grows past the outlier."""
    config = AsyncStereoConfig(minimum_initialization_pairs=4)
    estimator = AsyncStereoStateEstimator(config)
    for index in range(3):
        t_left = index / 30.0
        result = estimator.update(
            *simulate_async_pair((20.0, 0.0, 0.0), (0.0, 0.0, 0.0), t_left, t_left + 0.005, config)
        )
        assert not result.accepted
    result = estimator.update(
        *simulate_async_pair((60.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.15, 0.155, config)
    )
    assert not result.accepted
    assert result.reason == "initialization_geometry_invalid"
    # Buffer grows with another good pair: trimming keeps 4 good ones.
    result = estimator.update(
        *simulate_async_pair((20.0, 0.0, 0.0), (0.0, 0.0, 0.0), 0.2, 0.205, config)
    )
    assert result.accepted
    assert np.isclose(result.position[0], 20.0, atol=0.5), result.position
    assert np.linalg.norm(result.position) < 30.0


def test_innovation_deadlock_recovers_and_reinitializes():
    """After N consecutive innovation-gate rejections the filter must reset
    to batch initialization instead of deadlocking against all good data."""
    config = AsyncStereoConfig(innovation_gate_max_consecutive=3, innovation_gate_nis=1.0)
    estimator = AsyncStereoStateEstimator(config)
    # Normal initialization at 20 m.
    for index in range(4):
        t = index / 30.0
        result = estimator.update(
            *simulate_async_pair((20.0, 0.0, 0.0), (0.0, 0.0, 0.0), t, t + 0.005, config)
        )
    assert result.accepted
    assert estimator.initialized
    # Feed measurements from a wildly different range: innovation gate must
    # reject them, and after the configured streak the filter resets.
    reset_reason = None
    for index in range(6):
        t = 1.0 + index / 30.0
        result = estimator.update(
            *simulate_async_pair((60.0, 0.0, 0.0), (0.0, 0.0, 0.0), t, t + 0.005, config)
        )
        if result.reason == "reinit_after_innovation_rejections":
            reset_reason = result.reason
            break
    assert reset_reason == "reinit_after_innovation_rejections"
    assert not estimator.initialized
    # Fresh good data re-initializes the filter normally.
    for index in range(3):
        t = 2.0 + index / 30.0
        result = estimator.update(
            *simulate_async_pair((20.0, 0.0, 0.0), (0.0, 0.0, 0.0), t, t + 0.005, config)
        )
    assert result.accepted
    assert np.isclose(result.position[0], 20.0, atol=0.5), result.position
