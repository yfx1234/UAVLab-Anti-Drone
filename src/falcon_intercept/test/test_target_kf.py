import numpy as np

from falcon_intercept.target_kf_node import (
    ConstantVelocityKalmanFilter,
    optical_position_covariance_to_fru,
)


def measurement_covariance(variance=0.04):
    return np.eye(3, dtype=float) * variance


def test_optical_measurement_covariance_maps_to_internal_fru_axes():
    optical = np.array(
        [
            [2.0, -0.3, 0.1],
            [-0.3, 3.0, -0.2],
            [0.1, -0.2, 1.0],
        ],
        dtype=float,
    )

    internal = optical_position_covariance_to_fru(optical)

    assert np.allclose(
        internal,
        [
            [1.0, 0.1, 0.2],
            [0.1, 2.0, 0.3],
            [0.2, 0.3, 3.0],
        ],
    )


def test_constant_velocity_kf_estimates_velocity_and_covariance():
    kf = ConstantVelocityKalmanFilter(
        process_accel_variance=0.1,
        innovation_gate_nis=100.0,
    )
    assert kf.update((0.0, 0.0, 0.0), 1.0, measurement_covariance()).accepted
    assert kf.update((1.0, 0.0, 0.0), 2.0, measurement_covariance()).accepted
    assert 0.8 < kf.state[3] < 1.2
    assert np.all(np.linalg.eigvalsh(kf.covariance) >= -1e-10)
    health = kf.health(2.0, 0.5, 2, 100.0)
    assert health["valid"] is True
    assert health["reason"] == "ok"


def test_kf_rejects_large_innovation_without_accepting_timestamp():
    kf = ConstantVelocityKalmanFilter(
        process_accel_variance=0.1,
        innovation_gate_nis=16.27,
    )
    kf.update((0.0, 0.0, 0.0), 1.0, measurement_covariance())
    result = kf.update((100.0, 0.0, 0.0), 1.1, measurement_covariance())
    assert result.accepted is False
    assert result.reason == "innovation_gate_rejected"
    assert kf.last_measurement_timestamp_s == 1.0
    assert kf.rejected_measurements == 1


def test_kf_health_closes_on_timeout_and_covariance_grows():
    kf = ConstantVelocityKalmanFilter(process_accel_variance=2.0)
    kf.update((1.0, 2.0, 3.0), 5.0, measurement_covariance())
    kf.update((1.1, 2.0, 3.0), 5.1, measurement_covariance())
    fresh = kf.health(5.2, 0.5, 2, 1000.0)
    stale = kf.health(6.0, 0.5, 2, 1000.0)
    assert fresh["valid"] is True
    assert stale["valid"] is False
    assert stale["reason"] == "measurement_timeout"
    assert stale["position_covariance_trace"] > fresh["position_covariance_trace"]


def test_kf_requires_confirmed_hits_and_reinitializes_after_gap():
    kf = ConstantVelocityKalmanFilter(reset_after_gap_s=0.5)
    first = kf.update((0.0, 0.0, 0.0), 1.0, measurement_covariance())
    assert first.accepted
    assert kf.health(1.0, 1.0, 2, 100.0)["reason"] == "acquiring"
    reacquired = kf.update((5.0, 0.0, 0.0), 2.0, measurement_covariance())
    assert reacquired.reinitialized is True
    assert np.allclose(kf.state[0:3], [5.0, 0.0, 0.0])
    assert np.allclose(kf.state[3:6], [0.0, 0.0, 0.0])
    assert kf.health(2.0, 1.0, 2, 100.0)["valid"] is False


def test_kf_rejects_out_of_order_and_nonfinite_measurements():
    kf = ConstantVelocityKalmanFilter()
    kf.update((0.0, 0.0, 0.0), 1.0, measurement_covariance())
    assert kf.update((0.1, 0.0, 0.0), 1.0, measurement_covariance()).reason == (
        "out_of_order_timestamp"
    )
    assert kf.update((float("nan"), 0.0, 0.0), 1.1).reason == "nonfinite_measurement"
