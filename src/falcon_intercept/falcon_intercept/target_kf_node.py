from __future__ import annotations

import copy
import json
import math
from dataclasses import dataclass

import numpy as np


def optical_position_covariance_to_fru(
    covariance: np.ndarray | list[float],
) -> np.ndarray:
    """Map optical (right, down, forward) covariance into FRU."""

    optical_to_fru = np.array(
        [
            [0.0, 0.0, 1.0],
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
        ],
        dtype=float,
    )
    optical_covariance = np.asarray(covariance, dtype=float).reshape(3, 3)
    return optical_to_fru @ optical_covariance @ optical_to_fru.T


@dataclass(frozen=True)
class KalmanUpdateResult:
    accepted: bool
    reason: str
    innovation_nis: float | None
    reinitialized: bool = False


class ConstantVelocityKalmanFilter:
    """Six-state linear KF for relative position and velocity in one Cartesian frame."""

    def __init__(
        self,
        process_accel_variance: float = 4.0,
        initial_velocity_variance: float = 25.0,
        minimum_measurement_variance: float = 1e-4,
        innovation_gate_nis: float = 16.27,
        reset_after_gap_s: float = 1.0,
    ) -> None:
        self.process_accel_variance = max(0.0, float(process_accel_variance))
        self.initial_velocity_variance = max(1e-6, float(initial_velocity_variance))
        self.minimum_measurement_variance = max(
            1e-9, float(minimum_measurement_variance)
        )
        self.innovation_gate_nis = max(0.0, float(innovation_gate_nis))
        self.reset_after_gap_s = max(0.0, float(reset_after_gap_s))
        self.state = np.zeros(6, dtype=float)
        self.covariance = np.eye(6, dtype=float)
        self.timestamp_s: float | None = None
        self.last_measurement_timestamp_s: float | None = None
        self.consecutive_hits = 0
        self.accepted_measurements = 0
        self.rejected_measurements = 0
        self.last_update_reason = "not_initialized"
        self.last_innovation_nis: float | None = None

    @property
    def initialized(self) -> bool:
        return self.timestamp_s is not None

    @staticmethod
    def _transition(dt: float) -> np.ndarray:
        matrix = np.eye(6, dtype=float)
        matrix[0:3, 3:6] = np.eye(3, dtype=float) * dt
        return matrix

    def _process_noise(self, dt: float) -> np.ndarray:
        identity = np.eye(3, dtype=float)
        variance = self.process_accel_variance
        noise = np.zeros((6, 6), dtype=float)
        noise[0:3, 0:3] = identity * variance * (dt**4) / 4.0
        noise[0:3, 3:6] = identity * variance * (dt**3) / 2.0
        noise[3:6, 0:3] = identity * variance * (dt**3) / 2.0
        noise[3:6, 3:6] = identity * variance * (dt**2)
        return noise

    def _sanitize_measurement_covariance(
        self, covariance: np.ndarray | list[list[float]] | None
    ) -> np.ndarray:
        if covariance is None:
            return np.eye(3, dtype=float) * 0.25
        matrix = np.asarray(covariance, dtype=float)
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            return np.eye(3, dtype=float) * 0.25
        matrix = 0.5 * (matrix + matrix.T)
        try:
            eigenvalues, eigenvectors = np.linalg.eigh(matrix)
            eigenvalues = np.maximum(eigenvalues, self.minimum_measurement_variance)
            return eigenvectors @ np.diag(eigenvalues) @ eigenvectors.T
        except np.linalg.LinAlgError:
            return np.eye(3, dtype=float) * 0.25

    def reset(
        self,
        position_m: tuple[float, float, float],
        timestamp_s: float,
        measurement_covariance: np.ndarray | list[list[float]] | None = None,
        *,
        reason: str = "initialized",
    ) -> None:
        position = np.asarray(position_m, dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("position must contain three finite values")
        timestamp = float(timestamp_s)
        if not math.isfinite(timestamp):
            raise ValueError("timestamp must be finite")
        measurement_cov = self._sanitize_measurement_covariance(measurement_covariance)
        self.state = np.concatenate((position, np.zeros(3, dtype=float)))
        self.covariance = np.zeros((6, 6), dtype=float)
        self.covariance[0:3, 0:3] = measurement_cov
        self.covariance[3:6, 3:6] = (
            np.eye(3, dtype=float) * self.initial_velocity_variance
        )
        self.timestamp_s = timestamp
        self.last_measurement_timestamp_s = timestamp
        self.consecutive_hits = 1
        self.accepted_measurements += 1
        self.last_update_reason = reason
        self.last_innovation_nis = 0.0

    def _predict_in_place(self, timestamp_s: float) -> None:
        if self.timestamp_s is None:
            raise RuntimeError("filter is not initialized")
        dt = float(timestamp_s) - self.timestamp_s
        if dt < 0.0:
            raise ValueError("prediction timestamp moved backwards")
        if dt == 0.0:
            return
        transition = self._transition(dt)
        self.state = transition @ self.state
        self.covariance = (
            transition @ self.covariance @ transition.T + self._process_noise(dt)
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.timestamp_s = float(timestamp_s)

    def update(
        self,
        position_m: tuple[float, float, float],
        timestamp_s: float,
        measurement_covariance: np.ndarray | list[list[float]] | None = None,
    ) -> KalmanUpdateResult:
        position = np.asarray(position_m, dtype=float)
        timestamp = float(timestamp_s)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            self.rejected_measurements += 1
            self.last_update_reason = "nonfinite_measurement"
            return KalmanUpdateResult(False, self.last_update_reason, None)
        if not math.isfinite(timestamp):
            self.rejected_measurements += 1
            self.last_update_reason = "nonfinite_timestamp"
            return KalmanUpdateResult(False, self.last_update_reason, None)
        measurement_cov = self._sanitize_measurement_covariance(measurement_covariance)

        if not self.initialized:
            self.reset(position_m, timestamp, measurement_cov, reason="initialized")
            return KalmanUpdateResult(True, "initialized", 0.0)
        assert self.timestamp_s is not None
        assert self.last_measurement_timestamp_s is not None
        if timestamp <= self.timestamp_s:
            self.rejected_measurements += 1
            self.last_update_reason = "out_of_order_timestamp"
            return KalmanUpdateResult(False, self.last_update_reason, None)
        if (
            self.reset_after_gap_s > 0.0
            and timestamp - self.last_measurement_timestamp_s > self.reset_after_gap_s
        ):
            self.reset(position_m, timestamp, measurement_cov, reason="reinitialized_after_gap")
            return KalmanUpdateResult(True, "reinitialized_after_gap", 0.0, True)

        self._predict_in_place(timestamp)
        observation = np.zeros((3, 6), dtype=float)
        observation[:, 0:3] = np.eye(3, dtype=float)
        innovation = position - observation @ self.state
        innovation_cov = observation @ self.covariance @ observation.T + measurement_cov
        try:
            solved = np.linalg.solve(innovation_cov, innovation)
            innovation_nis = float(innovation.T @ solved)
        except np.linalg.LinAlgError:
            self.rejected_measurements += 1
            self.consecutive_hits = 0
            self.last_update_reason = "singular_innovation_covariance"
            self.last_innovation_nis = None
            return KalmanUpdateResult(False, self.last_update_reason, None)
        self.last_innovation_nis = innovation_nis
        if self.innovation_gate_nis > 0.0 and innovation_nis > self.innovation_gate_nis:
            self.rejected_measurements += 1
            self.consecutive_hits = 0
            self.last_update_reason = "innovation_gate_rejected"
            return KalmanUpdateResult(False, self.last_update_reason, innovation_nis)

        kalman_gain = np.linalg.solve(
            innovation_cov.T, (self.covariance @ observation.T).T
        ).T
        self.state = self.state + kalman_gain @ innovation
        identity = np.eye(6, dtype=float)
        residual = identity - kalman_gain @ observation
        # Joseph form preserves symmetry and positive semidefiniteness under roundoff.
        self.covariance = (
            residual @ self.covariance @ residual.T
            + kalman_gain @ measurement_cov @ kalman_gain.T
        )
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.last_measurement_timestamp_s = timestamp
        self.consecutive_hits += 1
        self.accepted_measurements += 1
        self.last_update_reason = "accepted"
        return KalmanUpdateResult(True, "accepted", innovation_nis)

    def predicted_covariance(self, timestamp_s: float) -> np.ndarray:
        if not self.initialized or self.timestamp_s is None:
            return self.covariance.copy()
        dt = max(0.0, float(timestamp_s) - self.timestamp_s)
        transition = self._transition(dt)
        return transition @ self.covariance @ transition.T + self._process_noise(dt)

    def health(
        self,
        now_s: float,
        timeout_s: float,
        minimum_confirmed_hits: int,
        maximum_position_covariance_trace: float,
    ) -> dict:
        now = float(now_s)
        age_s = None
        covariance_trace = None
        valid = False
        reason = "not_initialized"
        if self.initialized and self.last_measurement_timestamp_s is not None:
            age_s = max(0.0, now - self.last_measurement_timestamp_s)
            covariance_trace = float(np.trace(self.predicted_covariance(now)[0:3, 0:3]))
            if age_s > float(timeout_s):
                reason = "measurement_timeout"
            elif self.consecutive_hits < int(minimum_confirmed_hits):
                reason = "acquiring"
            elif covariance_trace > float(maximum_position_covariance_trace):
                reason = "position_covariance_too_large"
            else:
                valid = True
                reason = "ok"
        return {
            "valid": valid,
            "reason": reason,
            "initialized": self.initialized,
            "age_s": age_s,
            "consecutive_hits": int(self.consecutive_hits),
            "accepted_measurements": int(self.accepted_measurements),
            "rejected_measurements": int(self.rejected_measurements),
            "position_covariance_trace": covariance_trace,
            "last_update_reason": self.last_update_reason,
            "innovation_nis": self.last_innovation_nis,
            "last_measurement_timestamp_s": self.last_measurement_timestamp_s,
        }


class FixedLagPositionKalmanFilter:
    """Fixed-lag wrapper for position observations with capture timestamps.

    The legacy filter intentionally rejects out-of-order timestamps.  The
    production asynchronous path uses this wrapper instead: observations in
    the last ``fixed_lag_s`` are inserted by capture time and the linear KF
    is replayed.  Keeping the old class unchanged preserves its public
    fallback behaviour and its existing tests.
    """

    def __init__(self, *, fixed_lag_s: float = 0.3, **filter_kwargs) -> None:
        self.fixed_lag_s = max(0.0, float(fixed_lag_s))
        self._filter_kwargs = dict(filter_kwargs)
        self._kf = ConstantVelocityKalmanFilter(**self._filter_kwargs)
        self._records: list[tuple[float, np.ndarray, np.ndarray]] = []
        self._record_post_filters: list[ConstantVelocityKalmanFilter] = []
        self._history_base_filter: ConstantVelocityKalmanFilter | None = None
        self.latest_seen_timestamp_s: float | None = None
        self.oosm_count = 0
        self.oosm_drop_count = 0

    @property
    def state(self) -> np.ndarray:
        return self._kf.state

    @property
    def covariance(self) -> np.ndarray:
        return self._kf.covariance

    @property
    def timestamp_s(self) -> float | None:
        return self._kf.timestamp_s

    @property
    def last_measurement_timestamp_s(self) -> float | None:
        return self._kf.last_measurement_timestamp_s

    @property
    def consecutive_hits(self) -> int:
        return self._kf.consecutive_hits

    @property
    def accepted_measurements(self) -> int:
        return self._kf.accepted_measurements

    @property
    def rejected_measurements(self) -> int:
        return self._kf.rejected_measurements

    @property
    def last_update_reason(self) -> str:
        return self._kf.last_update_reason

    @property
    def last_innovation_nis(self) -> float | None:
        return self._kf.last_innovation_nis

    @property
    def initialized(self) -> bool:
        return self._kf.initialized

    @property
    def innovation_gate_nis(self) -> float:
        return self._kf.innovation_gate_nis

    def _rebuild(
        self,
        records: list[tuple[float, np.ndarray, np.ndarray]],
        base_filter: ConstantVelocityKalmanFilter | None = None,
    ) -> KalmanUpdateResult:
        rebuilt = (
            copy.deepcopy(base_filter)
            if base_filter is not None
            else ConstantVelocityKalmanFilter(**self._filter_kwargs)
        )
        post_filters: list[ConstantVelocityKalmanFilter] = []
        if not records:
            self._kf = rebuilt
            self._record_post_filters = []
            return KalmanUpdateResult(True, "accepted", rebuilt.last_innovation_nis)
        last_result = rebuilt.update(records[0][1], records[0][0], records[0][2])
        if not last_result.accepted:
            return last_result
        post_filters.append(copy.deepcopy(rebuilt))
        for timestamp, position, covariance in records[1:]:
            result = rebuilt.update(position, timestamp, covariance)
            if not result.accepted:
                return result
            last_result = result
            post_filters.append(copy.deepcopy(rebuilt))
        self._kf = rebuilt
        self._record_post_filters = post_filters
        return last_result

    def _prune_records_to_fixed_lag(self) -> None:
        if self.latest_seen_timestamp_s is None or not self._records:
            return
        window_start = self.latest_seen_timestamp_s - self.fixed_lag_s
        remove_count = 0
        while remove_count < len(self._records) and self._records[remove_count][0] < window_start:
            remove_count += 1
        if remove_count <= 0:
            return
        if remove_count <= len(self._record_post_filters):
            self._history_base_filter = copy.deepcopy(self._record_post_filters[remove_count - 1])
        self._records = self._records[remove_count:]
        self._record_post_filters = self._record_post_filters[remove_count:]

    def update(
        self,
        position_m: tuple[float, float, float],
        timestamp_s: float,
        measurement_covariance: np.ndarray | list[list[float]] | None = None,
    ) -> KalmanUpdateResult:
        position = np.asarray(position_m, dtype=float)
        timestamp = float(timestamp_s)
        if position.shape != (3,) or not np.all(np.isfinite(position)) or not math.isfinite(timestamp):
            self._kf.rejected_measurements += 1
            self._kf.last_update_reason = "nonfinite_measurement"
            return KalmanUpdateResult(False, "nonfinite_measurement", None)
        covariance = self._kf._sanitize_measurement_covariance(measurement_covariance)
        if self.latest_seen_timestamp_s is not None and timestamp < self.latest_seen_timestamp_s - self.fixed_lag_s:
            self.oosm_drop_count += 1
            self._kf.rejected_measurements += 1
            self._kf.last_update_reason = "OOSM_DROP"
            return KalmanUpdateResult(False, "OOSM_DROP", None)
        if self.latest_seen_timestamp_s is not None and timestamp < self.latest_seen_timestamp_s:
            self.oosm_count += 1
        candidate = (timestamp, position, covariance)
        # Fast path: a normal capture-time ordered measurement is handled by
        # the existing linear KF exactly once.  Full fixed-lag replay is
        # reserved for genuinely out-of-sequence observations.
        if self._kf.timestamp_s is None or timestamp > self._kf.timestamp_s:
            result = self._kf.update(position, timestamp, covariance)
            if not result.accepted:
                return result
            self._records.append(candidate)
            self._record_post_filters.append(copy.deepcopy(self._kf))
            self.latest_seen_timestamp_s = timestamp if self.latest_seen_timestamp_s is None else max(
                self.latest_seen_timestamp_s, timestamp
            )
            self._prune_records_to_fixed_lag()
            return result

        # OOSM path: insert into the retained lag window and replay from the
        # last fixed-lag base snapshot.
        old_records = list(self._records)
        old_post_filters = [copy.deepcopy(item) for item in self._record_post_filters]
        old_base_filter = copy.deepcopy(self._history_base_filter)
        old_filter = self._kf
        records = old_records + [candidate]
        records.sort(key=lambda item: item[0])
        result = self._rebuild(records, self._history_base_filter)
        if not result.accepted:
            self._kf = old_filter
            self._records = old_records
            self._record_post_filters = old_post_filters
            self._history_base_filter = old_base_filter
            self._kf.rejected_measurements += 1
            self._kf.consecutive_hits = 0
            self._kf.last_update_reason = result.reason
            return result
        self._records = records
        self.latest_seen_timestamp_s = max(self.latest_seen_timestamp_s or timestamp, timestamp)
        self._prune_records_to_fixed_lag()
        return KalmanUpdateResult(
            True,
            "accepted_oosm_replay",
            self._kf.last_innovation_nis,
            result.reinitialized,
        )

    def predicted_covariance(self, timestamp_s: float) -> np.ndarray:
        return self._kf.predicted_covariance(timestamp_s)

    def health(self, *args, **kwargs) -> dict:
        payload = self._kf.health(*args, **kwargs)
        payload.update(
            {
                "oosm_count": int(self.oosm_count),
                "oosm_drop_count": int(self.oosm_drop_count),
            }
        )
        return payload


def main(args=None) -> int:
    import rclpy
    from falcon_intercept_msgs.msg import AsyncStereoMeasurement, TargetCameraPosition, TargetTrack
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from std_msgs.msg import String

    def stamp_to_seconds(stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) / 1_000_000_000.0

    def set_stamp_from_seconds(stamp, seconds: float) -> None:
        sec = int(seconds)
        nanosec = int(round((float(seconds) - sec) * 1_000_000_000))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        stamp.sec = sec
        stamp.nanosec = nanosec

    class TargetKfNode(Node):
        def __init__(self) -> None:
            super().__init__("target_kf_node")
            self.declare_parameter("measurement_topic", "/perception/target/camera_position")
            self.declare_parameter(
                "async_measurement_topic", "/perception/async_stereo_measurement"
            )
            self.declare_parameter("use_async_primary", True)
            self.declare_parameter("state_topic", "/target/relative_state")
            self.declare_parameter("health_topic", "/target/state_health")
            self.declare_parameter("track_topic", "/target/track")
            self.declare_parameter("frame_id", "stereo_camera")
            self.declare_parameter("child_frame_id", "target_estimate")
            self.declare_parameter("process_accel_variance", 4.0)
            self.declare_parameter("initial_velocity_variance", 25.0)
            self.declare_parameter("minimum_measurement_variance", 1e-4)
            self.declare_parameter("innovation_gate_nis", 16.27)
            self.declare_parameter("reset_after_gap_sec", 1.0)
            self.declare_parameter("target_timeout_sec", 0.5)
            self.declare_parameter("minimum_confirmed_hits", 5)
            self.declare_parameter("maximum_position_covariance_trace", 100.0)
            self.declare_parameter("health_rate_hz", 10.0)
            filter_kwargs = dict(
                process_accel_variance=float(
                    self.get_parameter("process_accel_variance").value
                ),
                initial_velocity_variance=float(
                    self.get_parameter("initial_velocity_variance").value
                ),
                minimum_measurement_variance=float(
                    self.get_parameter("minimum_measurement_variance").value
                ),
                innovation_gate_nis=float(
                    self.get_parameter("innovation_gate_nis").value
                ),
                reset_after_gap_s=float(self.get_parameter("reset_after_gap_sec").value),
            )
            self._kf = FixedLagPositionKalmanFilter(
                fixed_lag_s=0.30,
                **filter_kwargs,
            )
            self._use_async_primary = bool(self.get_parameter("use_async_primary").value)
            self._frame_id = str(self.get_parameter("frame_id").value)
            self._child_frame_id = str(self.get_parameter("child_frame_id").value)
            self._timeout_s = float(self.get_parameter("target_timeout_sec").value)
            self._minimum_hits = int(self.get_parameter("minimum_confirmed_hits").value)
            self._maximum_covariance_trace = float(
                self.get_parameter("maximum_position_covariance_trace").value
            )
            self._last_health_valid: bool | None = None
            self._last_source_sequence = 0
            self._last_confidence = 0.0
            self._async_seen = False
            self._last_async_quality = {
                "timing_delta_s": math.inf,
                "timing_sigma_s": math.inf,
                "reprojection_error_px": math.inf,
                "range_observability": 0.0,
                "measurement_geometry_score": 0.0,
                "timing_valid": False,
                "range_observable": False,
            }
            self._state_pub = self.create_publisher(
                Odometry,
                str(self.get_parameter("state_topic").value),
                qos_profile_sensor_data,
            )
            self._health_pub = self.create_publisher(
                String, str(self.get_parameter("health_topic").value), 10
            )
            self._track_pub = self.create_publisher(
                TargetTrack, str(self.get_parameter("track_topic").value), qos_profile_sensor_data
            )
            self.create_subscription(
                AsyncStereoMeasurement,
                str(self.get_parameter("async_measurement_topic").value),
                self._on_async_measurement,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                TargetCameraPosition,
                str(self.get_parameter("measurement_topic").value),
                self._on_measurement,
                qos_profile_sensor_data,
            )
            health_rate_hz = max(1.0, float(self.get_parameter("health_rate_hz").value))
            self.create_timer(1.0 / health_rate_hz, self._on_health_timer)
            self.get_logger().info(
                "constant-velocity KF ready timeout=%.3fs min_hits=%d gate_nis=%.3f"
                % (
                    self._timeout_s,
                    self._minimum_hits,
                    self._kf.innovation_gate_nis,
                )
            )

        def _now_s(self) -> float:
            return stamp_to_seconds(self.get_clock().now().to_msg())

        def _health(self, now_s: float) -> dict:
            return self._kf.health(
                now_s,
                self._timeout_s,
                self._minimum_hits,
                self._maximum_covariance_trace,
            )

        def _publish_health(self, now_s: float, health: dict | None = None) -> dict:
            payload = self._health(now_s) if health is None else health
            payload = {"header": {"stamp_s": now_s, "frame_id": self._frame_id}, **payload}
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
            self._health_pub.publish(message)
            valid = bool(payload["valid"])
            if valid != self._last_health_valid:
                self.get_logger().info(
                    "TARGET_HEALTH valid=%s reason=%s"
                    % (str(valid).lower(), payload["reason"])
                )
                self._last_health_valid = valid
            return payload

        def _on_health_timer(self) -> None:
            now_s = self._now_s()
            health = self._publish_health(now_s)
            self._publish_track(now_s, health, predicted=True)

        def _publish_track(self, now_s: float, health: dict, *, predicted: bool) -> None:
            out = TargetTrack()
            set_stamp_from_seconds(out.header.stamp, now_s)
            out.header.frame_id = self._frame_id
            out.sequence = int(self._last_source_sequence)
            if self._kf.initialized:
                # TargetTrack keeps the legacy REP-103 optical contract for
                # downstream guidance: optical=(right, down, forward), while
                # the async estimator/KF state is internal FRU=(forward,
                # right, up).
                out.position_camera.x = float(self._kf.state[1])
                out.position_camera.y = float(-self._kf.state[2])
                out.position_camera.z = float(self._kf.state[0])
                out.velocity_camera.x = float(self._kf.state[4])
                out.velocity_camera.y = float(-self._kf.state[5])
                out.velocity_camera.z = float(self._kf.state[3])
                internal_covariance = self._kf.predicted_covariance(now_s)
                optical_transform = np.array(
                    [
                        [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, -1.0, 0.0, 0.0, 0.0],
                        [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
                        [0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
                        [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
                    ],
                    dtype=float,
                )
                out.covariance = [
                    float(value)
                    for value in (optical_transform @ internal_covariance @ optical_transform.T).reshape(-1)
                ]
            out.innovation_nis = float(self._kf.last_innovation_nis or 0.0)
            out.confidence = float(self._last_confidence)
            out.age_s = float(health["age_s"] if health["age_s"] is not None else math.inf)
            out.consecutive_hits = int(self._kf.consecutive_hits)
            out.predicted = bool(predicted)
            out.valid = bool(health["valid"])
            out.reason = str(health["reason"])
            if self._kf.initialized:
                position = self._kf.state[:3]
                out.range_m = float(np.linalg.norm(position))
                direction = position / max(out.range_m, 1e-9)
                out.range_sigma_m = float(
                    math.sqrt(max(0.0, direction @ self._kf.predicted_covariance(now_s)[:3, :3] @ direction))
                )
            out.timing_delta_s = float(self._last_async_quality["timing_delta_s"])
            out.timing_sigma_s = float(self._last_async_quality["timing_sigma_s"])
            out.reprojection_error_px = float(self._last_async_quality["reprojection_error_px"])
            out.range_observability = float(self._last_async_quality["range_observability"])
            out.measurement_geometry_score = float(
                self._last_async_quality["measurement_geometry_score"]
            )
            out.oosm_count = int(health.get("oosm_count", 0))
            out.oosm_drop_count = int(health.get("oosm_drop_count", 0))
            out.timing_valid = bool(self._last_async_quality["timing_valid"])
            out.range_observable = bool(self._last_async_quality["range_observable"])
            self._track_pub.publish(out)

        def _process_measurement(
            self,
            *,
            position: tuple[float, float, float],
            timestamp_s: float,
            covariance: np.ndarray,
            sequence: int,
            confidence: float,
            quality: dict | None = None,
        ) -> None:
            if timestamp_s <= 0.0:
                timestamp_s = self._now_s()
            self._last_source_sequence = int(sequence)
            self._last_confidence = float(confidence)
            if quality is not None:
                self._last_async_quality = quality
            result = self._kf.update(
                position_m=position,
                timestamp_s=timestamp_s,
                measurement_covariance=covariance,
            )
            now_s = self._now_s()
            health = self._publish_health(now_s)
            if not result.accepted:
                self.get_logger().warning(
                    "target measurement rejected reason=%s nis=%s"
                    % (result.reason, result.innovation_nis)
                )
                return

            self._publish_track(now_s, health, predicted=False)

            out = Odometry()
            out.header.frame_id = self._frame_id
            set_stamp_from_seconds(out.header.stamp, timestamp_s)
            out.child_frame_id = self._child_frame_id
            out.pose.pose.position.x = float(self._kf.state[0])
            out.pose.pose.position.y = float(self._kf.state[1])
            out.pose.pose.position.z = float(self._kf.state[2])
            out.pose.pose.orientation.w = 1.0
            out.twist.twist.linear.x = float(self._kf.state[3])
            out.twist.twist.linear.y = float(self._kf.state[4])
            out.twist.twist.linear.z = float(self._kf.state[5])

            pose_covariance = [0.0] * 36
            twist_covariance = [0.0] * 36
            for row in range(3):
                for column in range(3):
                    pose_covariance[row * 6 + column] = float(
                        self._kf.covariance[row, column]
                    )
                    twist_covariance[row * 6 + column] = float(
                        self._kf.covariance[row + 3, column + 3]
                    )
            for index in (3, 4, 5):
                pose_covariance[index * 6 + index] = 1e6
                twist_covariance[index * 6 + index] = 1e6
            out.pose.covariance = pose_covariance
            out.twist.covariance = twist_covariance
            # Health is published first so downstream safety gating observes the same update.
            if health["valid"]:
                self._state_pub.publish(out)

        def _on_async_measurement(self, msg: AsyncStereoMeasurement) -> None:
            self._async_seen = True
            quality = {
                "timing_delta_s": float(msg.timing_delta),
                "timing_sigma_s": float(msg.timing_sigma),
                "reprojection_error_px": float(msg.reprojection_error_px),
                "range_observability": float(msg.range_observability),
                "measurement_geometry_score": float(msg.measurement_geometry_score),
                "timing_valid": bool(msg.valid and msg.timing_sigma > 0.0),
                "range_observable": bool(msg.valid and msg.range_observability >= 0.0),
            }
            if not msg.valid:
                self._last_async_quality = quality
                return
            self._process_measurement(
                position=(
                    float(msg.position_camera.z),
                    float(msg.position_camera.x),
                    float(-msg.position_camera.y),
                ),
                timestamp_s=stamp_to_seconds(msg.header.stamp),
                covariance=optical_position_covariance_to_fru(
                    msg.position_covariance
                ),
                sequence=int(msg.sequence),
                confidence=float(msg.confidence),
                quality=quality,
            )

        def _on_measurement(self, msg: TargetCameraPosition) -> None:
            if self._use_async_primary and self._async_seen:
                return
            if not msg.valid:
                return
            self._process_measurement(
                position=(
                    float(msg.position_camera.x),
                    float(msg.position_camera.y),
                    float(msg.position_camera.z),
                ),
                timestamp_s=stamp_to_seconds(msg.header.stamp),
                covariance=np.asarray(msg.position_covariance, dtype=float).reshape(3, 3),
                sequence=int(msg.sequence),
                confidence=float(msg.confidence),
                quality={
                    "timing_delta_s": 0.0,
                    "timing_sigma_s": 0.0,
                    "reprojection_error_px": 0.0,
                    "range_observability": 1.0,
                    "measurement_geometry_score": 1.0,
                    "timing_valid": True,
                    "range_observable": True,
                },
            )

    rclpy.init(args=args)
    node = None
    try:
        node = TargetKfNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
