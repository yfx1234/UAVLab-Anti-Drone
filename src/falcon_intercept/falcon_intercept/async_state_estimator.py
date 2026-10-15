"""Asynchronous stereo state estimation primitives.

The module deliberately has no ROS dependency so that the timing, motion
compensation and fixed-lag behaviour can be tested on a workstation.  Camera
coordinates use the project convention x-forward, y-right, z-up.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Protocol

import numpy as np


@dataclass(frozen=True)
class AsyncStereoConfig:
    fx: float = 10238.013493164
    fy: float = 10322.580645161
    cx: float = 1919.5
    cy: float = 1079.5
    baseline_m: float = 0.1
    vertical_offset_front_minus_back_px: float = 0.0
    vertical_slope_front_minus_back: float = 0.0
    vertical_reference_u_px: float = 0.0
    horizontal_zero_disparity_offset_px: float = 0.0
    horizontal_disparity_sign: float = 1.0
    pixel_variance_px2: float = 64.0
    timestamp_sigma_s: float = 0.0015
    process_accel_variance: float = 4.0
    fixed_lag_s: float = 0.3
    minimum_initialization_pairs: int = 3
    initialization_max_reprojection_error_px: float = 8.0
    initialization_max_range_spread_fraction: float = 0.15
    minimum_range_m: float = 0.5
    maximum_range_m: float = 250.0
    innovation_gate_nis: float = 25.0
    innovation_gate_max_consecutive: int = 10


@dataclass(frozen=True)
class PixelObservation:
    camera_id: str
    timestamp_s: float
    u: float
    v: float
    sequence: int = 0
    timestamp_sigma_s: float | None = None
    pixel_variance_px2: float | None = None
    confidence: float = 1.0


class PoseProvider(Protocol):
    """Return world-from-rig pose as (rotation, translation) at a timestamp."""

    def pose_at(self, timestamp_s: float) -> tuple[np.ndarray, np.ndarray]: ...


def _quaternion_to_matrix(quaternion: Iterable[float]) -> np.ndarray:
    q = np.asarray(tuple(quaternion), dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)):
        return np.eye(3, dtype=float)
    norm = np.linalg.norm(q)
    if norm <= 1e-9:
        return np.eye(3, dtype=float)
    w, x, y, z = q / norm
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=float,
    )


class PX4PoseHistory:
    """Interpolation cache converting PX4 NED/FRD telemetry to FRU."""

    def __init__(
        self,
        *,
        body_to_rig_translation_fru: Iterable[float] = (0.0, 0.0, 0.0),
        history_s: float = 2.0,
    ) -> None:
        self.body_to_rig_translation = np.asarray(tuple(body_to_rig_translation_fru), dtype=float)
        if self.body_to_rig_translation.shape != (3,):
            raise ValueError("body_to_rig_translation_fru must contain three values")
        self.history_s = max(0.1, float(history_s))
        self.time_offset_s: float | None = None
        self.time_mapping_sigma_s = 0.005
        self._attitudes: list[tuple[float, np.ndarray]] = []
        self._positions: list[tuple[float, np.ndarray]] = []
        self._angular_velocities: list[tuple[float, np.ndarray]] = []

    def set_time_offset(self, jetson_minus_px4_s: float, sigma_s: float = 0.005) -> None:
        self.time_offset_s = float(jetson_minus_px4_s)
        self.time_mapping_sigma_s = max(1e-6, float(sigma_s))

    def _jetson_time(self, px4_timestamp_s: float, receive_time_s: float | None) -> float:
        if self.time_offset_s is None and receive_time_s is not None:
            self.set_time_offset(float(receive_time_s) - float(px4_timestamp_s))
        return float(px4_timestamp_s) + float(self.time_offset_s or 0.0)

    @staticmethod
    def _ned_to_fru(vector_ned: Iterable[float]) -> np.ndarray:
        north, east, down = (float(value) for value in vector_ned)
        return np.array([north, east, -down], dtype=float)

    def add_position(self, px4_timestamp_s: float, position_ned: Iterable[float], receive_time_s: float | None = None) -> None:
        timestamp = self._jetson_time(px4_timestamp_s, receive_time_s)
        self._positions.append((timestamp, self._ned_to_fru(position_ned)))
        self._trim(timestamp)

    def add_attitude(self, px4_timestamp_s: float, quaternion_wxyz: Iterable[float], receive_time_s: float | None = None) -> None:
        timestamp = self._jetson_time(px4_timestamp_s, receive_time_s)
        rotation_ned_from_frd = _quaternion_to_matrix(quaternion_wxyz)
        axis_flip = np.diag([1.0, 1.0, -1.0])
        self._attitudes.append((timestamp, axis_flip @ rotation_ned_from_frd @ axis_flip))
        self._trim(timestamp)

    def add_angular_velocity(self, px4_timestamp_s: float, angular_velocity_frd: Iterable[float], receive_time_s: float | None = None) -> None:
        timestamp = self._jetson_time(px4_timestamp_s, receive_time_s)
        wx, wy, wz = (float(value) for value in angular_velocity_frd)
        self._angular_velocities.append((timestamp, np.array([wx, wy, -wz], dtype=float)))
        self._trim(timestamp)

    def _trim(self, newest_time_s: float) -> None:
        cutoff = float(newest_time_s) - self.history_s
        self._positions = [item for item in self._positions if item[0] >= cutoff]
        self._attitudes = [item for item in self._attitudes if item[0] >= cutoff]
        self._angular_velocities = [item for item in self._angular_velocities if item[0] >= cutoff]

    @staticmethod
    def _interpolate_vector(samples: list[tuple[float, np.ndarray]], timestamp_s: float) -> np.ndarray:
        if not samples:
            return np.zeros(3, dtype=float)
        samples = sorted(samples, key=lambda item: item[0])
        if timestamp_s <= samples[0][0]:
            return samples[0][1].copy()
        if timestamp_s >= samples[-1][0]:
            return samples[-1][1].copy()
        for before, after in zip(samples, samples[1:]):
            if before[0] <= timestamp_s <= after[0]:
                ratio = (timestamp_s - before[0]) / max(after[0] - before[0], 1e-9)
                return before[1] * (1.0 - ratio) + after[1] * ratio
        return samples[-1][1].copy()

    def pose_at(self, timestamp_s: float) -> tuple[np.ndarray, np.ndarray]:
        rotation = np.eye(3, dtype=float)
        if self._attitudes:
            samples = sorted(self._attitudes, key=lambda item: item[0])
            if timestamp_s <= samples[0][0]:
                rotation = samples[0][1].copy()
            elif timestamp_s >= samples[-1][0]:
                rotation = samples[-1][1].copy()
            else:
                for before, after in zip(samples, samples[1:]):
                    if before[0] <= timestamp_s <= after[0]:
                        ratio = (timestamp_s - before[0]) / max(after[0] - before[0], 1e-9)
                        rotation = before[1] * (1.0 - ratio) + after[1] * ratio
                        u, _, vh = np.linalg.svd(rotation)
                        rotation = u @ vh
                        break
        translation = self._interpolate_vector(self._positions, timestamp_s)
        translation = translation + rotation @ self.body_to_rig_translation
        return rotation, translation


@dataclass(frozen=True)
class AsyncStereoResult:
    accepted: bool
    reason: str
    reference_time_s: float
    state: np.ndarray
    covariance: np.ndarray
    timing_delta_s: float
    timing_sigma_s: float
    reprojection_error_px: float
    range_observability: float
    measurement_geometry_score: float
    innovation_nis: float | None
    oosm_count: int
    oosm_drop_count: int

    @property
    def position(self) -> np.ndarray:
        return self.state[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.state[3:6].copy()

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.state[:3]))

    @property
    def range_sigma_m(self) -> float:
        position_cov = self.covariance[:3, :3]
        norm = np.linalg.norm(self.state[:3])
        if norm <= 1e-9:
            return math.inf
        direction = self.state[:3] / norm
        return float(math.sqrt(max(0.0, direction @ position_cov @ direction)))


def _transition(dt: float) -> np.ndarray:
    matrix = np.eye(6, dtype=float)
    matrix[:3, 3:] = np.eye(3, dtype=float) * float(dt)
    return matrix


def _process_noise(dt: float, variance: float) -> np.ndarray:
    dt = max(0.0, float(dt))
    q = np.zeros((6, 6), dtype=float)
    identity = np.eye(3, dtype=float)
    q[:3, :3] = identity * variance * dt**4 / 4.0
    q[:3, 3:] = identity * variance * dt**3 / 2.0
    q[3:, :3] = identity * variance * dt**3 / 2.0
    q[3:, 3:] = identity * variance * dt**2
    return q


class AsyncStereoStateEstimator:
    """Six-state EKF using two pixels acquired at different times.

    The filter state is expressed at ``reference_time_s`` in the front-camera
    rig frame.  The two camera centres are separated along the y axis.  The
    nonlinear pixel model is evaluated at each camera's own acquisition time;
    this is the essential difference from a conventional instantaneous
    disparity calculation.
    """

    def __init__(
        self,
        config: AsyncStereoConfig | None = None,
        pose_provider: PoseProvider | None = None,
    ) -> None:
        self.config = config or AsyncStereoConfig()
        self.pose_provider = pose_provider
        self.state = np.zeros(6, dtype=float)
        self.covariance = np.eye(6, dtype=float) * 1e6
        self.reference_time_s: float | None = None
        self.last_measurement_time_s: float | None = None
        self.consecutive_hits = 0
        self.accepted_count = 0
        self.rejected_count = 0
        self.oosm_count = 0
        self.oosm_drop_count = 0
        self.last_nis: float | None = None
        self.last_reason = "not_initialized"
        # A fixed-lag record set is replayed in capture-time order.  Keeping
        # the records here (rather than only the latest state) is what makes
        # an out-of-order observation a real OOSM update instead of a late
        # measurement silently applied at the wrong time.
        self._records: list[tuple[PixelObservation, PixelObservation]] = []
        self._record_post_states: list[np.ndarray] = []
        self._record_post_covariances: list[np.ndarray] = []
        self._record_keys: set[tuple[float, float, float, float, float, float]] = set()
        self._latest_seen_time_s: float | None = None
        self._initialization_buffer: list[tuple[PixelObservation, PixelObservation]] = []
        self._history_base_state: np.ndarray | None = None
        self._history_base_covariance: np.ndarray | None = None
        self._history_base_time_s: float | None = None
        self._consecutive_innovation_rejections = 0

    def _reset_filter(self, reason: str) -> None:
        """Drop the current track and return to batch initialization.

        A filter initialized on a bad seed (e.g. a self-consistent false
        match) must not be allowed to deadlock against all future good
        measurements via the innovation gate.
        """
        self.state = np.zeros(6, dtype=float)
        self.covariance = np.eye(6, dtype=float) * 1e6
        self.reference_time_s = None
        self.last_measurement_time_s = None
        self.consecutive_hits = 0
        self._records = []
        self._record_post_states = []
        self._record_post_covariances = []
        self._record_keys = set()
        self._initialization_buffer = []
        self._history_base_state = None
        self._history_base_covariance = None
        self._history_base_time_s = None
        self._consecutive_innovation_rejections = 0
        self.last_reason = reason

    @property
    def initialized(self) -> bool:
        return self.reference_time_s is not None

    def set_pose_provider(self, pose_provider: PoseProvider | None) -> None:
        self.pose_provider = pose_provider

    def _pose_at(self, timestamp_s: float) -> tuple[np.ndarray, np.ndarray]:
        if self.pose_provider is None:
            return np.eye(3, dtype=float), np.zeros(3, dtype=float)
        rotation, translation = self.pose_provider.pose_at(float(timestamp_s))
        rotation = np.asarray(rotation, dtype=float)
        translation = np.asarray(translation, dtype=float)
        if rotation.shape != (3, 3) or translation.shape != (3,) or not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            return np.eye(3, dtype=float), np.zeros(3, dtype=float)
        return rotation, translation

    def _camera_center_y(self, camera_id: str) -> float:
        if camera_id == "front":
            return -0.5 * self.config.baseline_m
        if camera_id == "back":
            return 0.5 * self.config.baseline_m
        return 0.0

    def _canonical_pair(
        self,
        front: PixelObservation,
        back: PixelObservation,
    ) -> tuple[PixelObservation, PixelObservation]:
        """Map the measured pair to the rectified pinhole convention.

        The field calibration already used by the legacy solver contains a
        vertical shear, a zero-disparity offset and, on the current board,
        the camera-channel disparity sign.  The EKF must see the same
        corrected pixels, otherwise it would re-introduce a static range
        bias while solving the temporal problem.
        """

        back_u = float(back.u)
        back_v = float(back.v)
        corrected_back_u = back_u - self.config.vertical_slope_front_minus_back * (
            back_v - self.config.cy
        )
        expected_vertical_delta = self.config.vertical_offset_front_minus_back_px + (
            self.config.vertical_slope_front_minus_back
            * (back_u - self.config.vertical_reference_u_px)
        )
        corrected_back_v = back_v + expected_vertical_delta
        calibrated_disparity = self.config.horizontal_disparity_sign * (
            float(front.u) - corrected_back_u - self.config.horizontal_zero_disparity_offset_px
        )
        if not math.isfinite(calibrated_disparity):
            calibrated_disparity = float(front.u) - corrected_back_u
        # The internal camera model has positive front-minus-back disparity.
        # Reconstruct the canonical back pixel from the calibrated disparity
        # so both the initial triangulation and the EKF use one convention.
        canonical_back_u = float(front.u) - calibrated_disparity
        canonical_back = PixelObservation(
            camera_id="back",
            timestamp_s=back.timestamp_s,
            u=canonical_back_u,
            v=corrected_back_v,
            sequence=back.sequence,
            timestamp_sigma_s=back.timestamp_sigma_s,
            pixel_variance_px2=back.pixel_variance_px2,
            confidence=back.confidence,
        )
        return front, canonical_back

    def _project(self, state: np.ndarray, observation: PixelObservation) -> np.ndarray:
        assert self.reference_time_s is not None
        dt = float(observation.timestamp_s) - float(self.reference_time_s)
        point_world = state[:3] + state[3:] * dt
        rotation_world_from_rig, translation_world_rig = self._pose_at(observation.timestamp_s)
        camera_center_rig = np.array([0.0, self._camera_center_y(observation.camera_id), 0.0])
        camera_center_world = translation_world_rig + rotation_world_from_rig @ camera_center_rig
        point = rotation_world_from_rig.T @ (point_world - camera_center_world)
        if point[0] <= 1e-6:
            return np.array([math.nan, math.nan], dtype=float)
        return np.array(
            [
                self.config.fx * point[1] / point[0] + self.config.cx,
                self.config.cy - self.config.fy * point[2] / point[0],
            ],
            dtype=float,
        )

    def _jacobian(self, state: np.ndarray, observation: PixelObservation) -> np.ndarray:
        # Analytic derivative of the pinhole projection with respect to p/v.
        assert self.reference_time_s is not None
        dt = float(observation.timestamp_s) - float(self.reference_time_s)
        point_world = state[:3] + state[3:] * dt
        rotation_world_from_rig, translation_world_rig = self._pose_at(observation.timestamp_s)
        camera_center_rig = np.array([0.0, self._camera_center_y(observation.camera_id), 0.0])
        camera_center_world = translation_world_rig + rotation_world_from_rig @ camera_center_rig
        point = rotation_world_from_rig.T @ (point_world - camera_center_world)
        x, y, z = point
        if x <= 1e-6:
            return np.zeros((2, 6), dtype=float)
        base = np.array(
            [
                [-self.config.fx * y / (x * x), self.config.fx / x, 0.0],
                [self.config.fy * z / (x * x), 0.0, -self.config.fy / x],
            ],
            dtype=float,
        )
        rotation_rig_from_world = rotation_world_from_rig.T
        position_jacobian = base @ rotation_rig_from_world
        return np.hstack((position_jacobian, position_jacobian * dt))

    def _time_jacobian(self, state: np.ndarray, observation: PixelObservation) -> np.ndarray:
        eps = 1e-5
        before = PixelObservation(
            observation.camera_id, observation.timestamp_s - eps, observation.u, observation.v
        )
        after = PixelObservation(
            observation.camera_id, observation.timestamp_s + eps, observation.u, observation.v
        )
        return (self._project(state, after) - self._project(state, before)) / (2.0 * eps)

    def _triangulate_initial(self, front: PixelObservation, back: PixelObservation) -> np.ndarray | None:
        disparity = float(front.u - back.u)
        if abs(disparity) < 1e-6:
            return None
        x = self.config.fx * self.config.baseline_m / disparity
        if not math.isfinite(x) or x <= self.config.minimum_range_m:
            return None
        y_front = (front.u - self.config.cx) * x / self.config.fx + self._camera_center_y("front")
        z = -(front.v - self.config.cy) * x / self.config.fy
        point_rig = np.array([x, y_front, z], dtype=float)
        reference_time = 0.5 * (front.timestamp_s + back.timestamp_s)
        rotation_world_from_rig, translation_world_rig = self._pose_at(reference_time)
        point_world = translation_world_rig + rotation_world_from_rig @ point_rig
        return np.concatenate((point_world, np.zeros(3, dtype=float)))

    @staticmethod
    def _record_key(
        front: PixelObservation, back: PixelObservation
    ) -> tuple[float, float, float, float, float, float]:
        return (
            float(front.timestamp_s),
            float(back.timestamp_s),
            float(front.u),
            float(front.v),
            float(back.u),
            float(back.v),
        )

    @staticmethod
    def _record_time(record: tuple[PixelObservation, PixelObservation]) -> float:
        front, back = record
        return 0.5 * (float(front.timestamp_s) + float(back.timestamp_s))

    def _batch_initialize(
        self, records: list[tuple[PixelObservation, PixelObservation]]
    ) -> tuple[bool, float]:
        """Fit position and velocity jointly before enabling 3-D output."""
        minimum_pairs = max(2, int(self.config.minimum_initialization_pairs))
        # Trim pairs whose triangulated range disagrees with the batch
        # median before fitting.  A mixed batch (one false match among good
        # pairs) must be fitted from its inlier majority instead of letting
        # the outlier drag the LSQ solution to a bad seed.
        if len(records) >= minimum_pairs:
            ranges = []
            for front, back in records:
                seed = self._triangulate_initial(front, back)
                ranges.append(None if seed is None else float(np.linalg.norm(seed[:3])))
            valid_ranges = [value for value in ranges if value is not None]
            if valid_ranges:
                center = float(np.median(valid_ranges))
                deviations = [abs(value - center) for value in valid_ranges]
                mad = float(np.median(deviations)) if deviations else 0.0
                tolerance = max(
                    3.0 * 1.4826 * mad,
                    float(self.config.initialization_max_range_spread_fraction) * center,
                )
                kept = [
                    record
                    for record, value in zip(records, ranges)
                    if value is not None and abs(value - center) <= tolerance
                ]
                if len(kept) >= minimum_pairs:
                    records = kept
                else:
                    # Trimming killed the batch; fail and retry with a larger
                    # buffer on the next update rather than fitting garbage.
                    return False, math.inf

        reference_time = max(self._record_time(item) for item in records)
        self.reference_time_s = reference_time
        # With a stationary rig the pinhole equations become linear when the
        # unknowns are position/velocity at the reference time. This removes
        # the local-minimum problem that occurs if a fast target's first
        # asynchronous disparity is used as a depth seed.
        identity_pose = True
        for front, back in records:
            for observation in (front, back):
                rotation, translation = self._pose_at(observation.timestamp_s)
                if not np.allclose(rotation, np.eye(3), atol=1e-6) or not np.allclose(translation, 0.0, atol=1e-6):
                    identity_pose = False
                    break
        if identity_pose:
            rows = []
            values = []
            for front, back in records:
                for observation in (front, back):
                    dt = float(observation.timestamp_s) - reference_time
                    normalized_u = (float(observation.u) - self.config.cx) / self.config.fx
                    normalized_z = (self.config.cy - float(observation.v)) / self.config.fy
                    if observation.camera_id == "front":
                        baseline_half = 0.5 * self.config.baseline_m
                    else:
                        baseline_half = -0.5 * self.config.baseline_m
                    rows.append([normalized_u, normalized_u * dt, -1.0, -dt, 0.0, 0.0])
                    values.append(baseline_half)
                    rows.append([normalized_z, normalized_z * dt, 0.0, 0.0, -1.0, -dt])
                    values.append(0.0)
            try:
                design = np.asarray(rows, dtype=float)
                solution, _, rank, _ = np.linalg.lstsq(design, np.asarray(values, dtype=float), rcond=None)
            except np.linalg.LinAlgError:
                rank = 0
                solution = np.zeros(6, dtype=float)
            if rank >= 4 and np.all(np.isfinite(solution)) and solution[0] > self.config.minimum_range_m:
                self.state = np.array(
                    [solution[0], solution[2], solution[4], solution[1], solution[3], solution[5]],
                    dtype=float,
                )
                self.covariance = np.eye(6, dtype=float) * 10.0
                predicted = []
                measured = []
                for front, back in records:
                    for observation in (front, back):
                        predicted.extend(self._project(self.state, observation))
                        measured.extend([observation.u, observation.v])
                reprojection = float(
                    np.sqrt(np.mean(np.square(np.asarray(measured) - np.asarray(predicted))))
                )
                if reprojection <= self.config.initialization_max_reprojection_error_px:
                    self._history_base_state = self.state.copy()
                    self._history_base_covariance = self.covariance.copy()
                    self._history_base_time_s = reference_time
                    return True, reprojection

        seed = self._triangulate_initial(*records[-1])
        if seed is None:
            return False, math.inf
        self.state = seed
        self.covariance = np.eye(6, dtype=float) * 100.0
        for _ in range(12):
            predicted = []
            measured = []
            jacobians = []
            covariance_blocks = []
            for front, back in records:
                for observation in (front, back):
                    prediction = self._project(self.state, observation)
                    if not np.all(np.isfinite(prediction)):
                        return False, math.inf
                    predicted.extend(prediction)
                    measured.extend([observation.u, observation.v])
                    jacobians.append(self._jacobian(self.state, observation))
                    covariance_blocks.append(self._measurement_covariance(observation, self.state))
            H = np.vstack(jacobians)
            residual = np.asarray(measured, dtype=float) - np.asarray(predicted, dtype=float)
            R = np.zeros((len(covariance_blocks) * 2, len(covariance_blocks) * 2), dtype=float)
            for index, block in enumerate(covariance_blocks):
                R[2 * index:2 * index + 2, 2 * index:2 * index + 2] = block
            try:
                information = H.T @ np.linalg.solve(R, H) + np.eye(6) * 1e-8
                step = np.linalg.solve(information, H.T @ np.linalg.solve(R, residual))
            except np.linalg.LinAlgError:
                return False, math.inf
            self.state = self.state + step
            if np.linalg.norm(step) < 1e-5:
                break
        reprojection = float(np.sqrt(np.mean(np.square(residual))))
        position_range = float(np.linalg.norm(self.state[:3]))
        if (
            not np.all(np.isfinite(self.state))
            or position_range < self.config.minimum_range_m
            or position_range > self.config.maximum_range_m
            or reprojection > self.config.initialization_max_reprojection_error_px
        ):
            return False, reprojection
        try:
            self.covariance = np.linalg.inv(information)
        except np.linalg.LinAlgError:
            self.covariance = np.eye(6, dtype=float) * 100.0
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self._history_base_state = self.state.copy()
        self._history_base_covariance = self.covariance.copy()
        self._history_base_time_s = reference_time
        return True, reprojection

    def _predict_to(self, timestamp_s: float) -> None:
        if self.reference_time_s is None:
            return
        dt = float(timestamp_s) - float(self.reference_time_s)
        if dt < 0.0:
            return
        transition = _transition(dt)
        self.state = transition @ self.state
        self.covariance = transition @ self.covariance @ transition.T
        self.covariance += _process_noise(dt, self.config.process_accel_variance)
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        self.reference_time_s = float(timestamp_s)

    def _prune_records_to_fixed_lag(self) -> None:
        """Advance the replay base while retaining only the fixed-lag window."""

        if self._latest_seen_time_s is None or not self._records:
            return
        window_start = self._latest_seen_time_s - self.config.fixed_lag_s
        keep_index = 0
        while (
            keep_index < len(self._records)
            and self._record_time(self._records[keep_index]) < window_start
        ):
            keep_index += 1
        if keep_index <= 0:
            return
        # The post-update snapshot of the last removed record is the exact
        # state from which an in-window OOSM replay must start.
        if keep_index <= len(self._record_post_states):
            base_index = keep_index - 1
            self._history_base_state = self._record_post_states[base_index].copy()
            self._history_base_covariance = self._record_post_covariances[base_index].copy()
            self._history_base_time_s = self._record_time(self._records[base_index])
        self._records = self._records[keep_index:]
        self._record_post_states = self._record_post_states[keep_index:]
        self._record_post_covariances = self._record_post_covariances[keep_index:]
        self._record_keys = {self._record_key(*item) for item in self._records}

    def _measurement_covariance(self, observation: PixelObservation, state: np.ndarray) -> np.ndarray:
        pixel_variance = observation.pixel_variance_px2
        if pixel_variance is None:
            pixel_variance = self.config.pixel_variance_px2
        sigma_t = observation.timestamp_sigma_s
        if sigma_t is None:
            sigma_t = self.config.timestamp_sigma_s
        time_jacobian = self._time_jacobian(state, observation)
        variance = float(pixel_variance) + np.square(time_jacobian * float(sigma_t))
        return np.diag(np.maximum(variance, 1e-6))

    def _update_pixels(self, observations: Iterable[PixelObservation]) -> tuple[bool, float | None, float]:
        observations = tuple(observations)
        predicted = np.concatenate([self._project(self.state, item) for item in observations])
        measured = np.concatenate([[item.u, item.v] for item in observations])
        if not np.all(np.isfinite(predicted)):
            return False, None, math.inf
        h_blocks = [self._jacobian(self.state, item) for item in observations]
        H = np.vstack(h_blocks)
        R = np.zeros((2 * len(observations), 2 * len(observations)), dtype=float)
        for index, item in enumerate(observations):
            R[2 * index:2 * index + 2, 2 * index:2 * index + 2] = self._measurement_covariance(
                item, self.state
            )
        innovation = measured - predicted
        innovation_cov = H @ self.covariance @ H.T + R
        try:
            solved = np.linalg.solve(innovation_cov, innovation)
            nis = float(innovation.T @ solved)
            if self.config.innovation_gate_nis > 0.0 and nis > self.config.innovation_gate_nis:
                return False, nis, float(np.sqrt(np.mean(np.square(innovation))))
            gain = np.linalg.solve(innovation_cov.T, (self.covariance @ H.T).T).T
        except np.linalg.LinAlgError:
            return False, None, math.inf
        self.state = self.state + gain @ innovation
        identity = np.eye(6, dtype=float)
        residual = identity - gain @ H
        self.covariance = residual @ self.covariance @ residual.T + gain @ R @ gain.T
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        return True, nis, float(np.sqrt(np.mean(np.square(innovation))))

    def update(self, front: PixelObservation, back: PixelObservation) -> AsyncStereoResult:
        if front.camera_id != "front" or back.camera_id != "back":
            return self._reject("invalid_camera_id", front, back)
        if not all(math.isfinite(value) for value in (front.timestamp_s, back.timestamp_s, front.u, front.v, back.u, back.v)):
            return self._reject("nonfinite_observation", front, back)
        front, back = self._canonical_pair(front, back)
        reference_time = 0.5 * (float(front.timestamp_s) + float(back.timestamp_s))
        timing_delta = abs(float(front.timestamp_s) - float(back.timestamp_s))
        timing_sigma = max(
            float(front.timestamp_sigma_s or self.config.timestamp_sigma_s),
            float(back.timestamp_sigma_s or self.config.timestamp_sigma_s),
        )
        if self._latest_seen_time_s is not None and reference_time < self._latest_seen_time_s - self.config.fixed_lag_s:
            self.oosm_drop_count += 1
            return self._reject("OOSM_DROP", front, back, timing_delta, timing_sigma)
        if self.reference_time_s is not None and reference_time < self.reference_time_s:
            self.oosm_count += 1

        if not self.initialized:
            key = self._record_key(front, back)
            if key in self._record_keys:
                return self._reject("duplicate_observation", front, back, timing_delta, timing_sigma)
            self._initialization_buffer.append((front, back))
            self._record_keys.add(key)
            self._latest_seen_time_s = max(
                reference_time, self._latest_seen_time_s or reference_time
            )
            minimum_pairs = max(2, int(self.config.minimum_initialization_pairs))
            if len(self._initialization_buffer) < minimum_pairs:
                self.last_reason = "initializing_batch"
                return self._result(
                    False,
                    self.last_reason,
                    front,
                    back,
                    timing_delta,
                    timing_sigma,
                    None,
                )
            initialized, reprojection = self._batch_initialize(self._initialization_buffer)
            if not initialized:
                self.last_reason = "initialization_geometry_invalid"
                return self._result(
                    False,
                    self.last_reason,
                    front,
                    back,
                    timing_delta,
                    timing_sigma,
                    None,
                )
            self._records = []
            self._record_post_states = []
            self._record_post_covariances = []
            self._record_keys = {self._record_key(*item) for item in self._initialization_buffer}
            self.last_measurement_time_s = reference_time
            self.consecutive_hits = len(self._initialization_buffer)
            self.accepted_count += len(self._initialization_buffer)
            self.last_reason = "initialized_batch"
            nis = 0.0
        else:
            key = self._record_key(front, back)
            if key in self._record_keys:
                return self._reject("duplicate_observation", front, back, timing_delta, timing_sigma)
            record = (front, back)
            is_oosm = self.reference_time_s is not None and reference_time < self.reference_time_s
            if not is_oosm:
                # The common path is an in-order measurement.  Predict and
                # update once; do not replay the whole lag window on every
                # frame.  This keeps the Python estimator above the 25 Hz
                # production target on the Jetson.
                previous_state = self.state.copy()
                previous_covariance = self.covariance.copy()
                self._predict_to(reference_time)
                accepted, nis, _ = self._update_pixels((front, back))
                if not accepted:
                    self.state = previous_state
                    self.covariance = previous_covariance
                    self.rejected_count += 1
                    self.consecutive_hits = 0
                    self.last_nis = nis
                    self._consecutive_innovation_rejections += 1
                    if self._consecutive_innovation_rejections >= max(
                        1, int(self.config.innovation_gate_max_consecutive)
                    ):
                        self._reset_filter("reinit_after_innovation_rejections")
                        return self._result(
                            False, self.last_reason, front, back, timing_delta, timing_sigma, nis
                        )
                    self.last_reason = "innovation_gate_rejected" if nis is not None else "pixel_update_failed"
                    return self._result(
                        False,
                        self.last_reason,
                        front,
                        back,
                        timing_delta,
                        timing_sigma,
                        nis,
                    )
                self._consecutive_innovation_rejections = 0
                self._records.append(record)
                self._record_post_states.append(self.state.copy())
                self._record_post_covariances.append(self.covariance.copy())
                self._latest_seen_time_s = max(
                    reference_time, self._latest_seen_time_s or reference_time
                )
                self._record_keys.add(key)
                self._prune_records_to_fixed_lag()
                accepted = True
                reason = "accepted"
            else:
                # An out-of-sequence measurement is inserted and only this
                # exceptional path replays the retained fixed-lag records.
                old_records = list(self._records)
                old_post_states = [item.copy() for item in self._record_post_states]
                old_post_covariances = [item.copy() for item in self._record_post_covariances]
                self._records.append(record)
                self._record_post_states.append(np.zeros(6, dtype=float))
                self._record_post_covariances.append(np.eye(6, dtype=float))
                order = sorted(range(len(self._records)), key=lambda index: self._record_time(self._records[index]))
                self._records = [self._records[index] for index in order]
                self._record_post_states = [self._record_post_states[index] for index in order]
                self._record_post_covariances = [self._record_post_covariances[index] for index in order]
                accepted, nis, reason = self._replay_records()
                if not accepted:
                    # The candidate failed its innovation gate.  Restore the
                    # last valid record set and state; a bad late pixel must
                    # not poison a healthy track.
                    self._records = old_records
                    self._record_post_states = old_post_states
                    self._record_post_covariances = old_post_covariances
                    self._record_keys = {self._record_key(*item) for item in old_records}
                    self._replay_records()
                    self.rejected_count += 1
                    self.consecutive_hits = 0
                    self.last_nis = nis
                    self._consecutive_innovation_rejections += 1
                    if self._consecutive_innovation_rejections >= max(
                        1, int(self.config.innovation_gate_max_consecutive)
                    ):
                        self._reset_filter("reinit_after_innovation_rejections")
                    else:
                        self.last_reason = reason
                    return self._result(False, self.last_reason, front, back, timing_delta, timing_sigma, nis)
                self._consecutive_innovation_rejections = 0
                self._record_keys = {self._record_key(*item) for item in self._records}
            self.last_measurement_time_s = reference_time
            self.consecutive_hits += 1
            self.accepted_count += 1
            self.last_reason = "accepted_oosm_replay" if reference_time < float(self.reference_time_s) else "accepted"

        self.last_nis = nis
        self.last_measurement_time_s = reference_time
        position = self.state[:3]
        speed_along_baseline = abs(float(self.state[4]))
        range_m = float(np.linalg.norm(position))
        rt = speed_along_baseline * timing_delta / max(self.config.baseline_m, 1e-6)
        observability = float(max(0.0, min(1.0, 1.0 - rt)))
        disparity = abs(float(front.u) - float(back.u))
        disparity_score = min(1.0, disparity / 20.0)
        range_score = min(1.0, max(0.0, self.config.maximum_range_m / max(range_m, 1e-6)))
        geometry_score = float(max(0.0, min(1.0, disparity_score * observability * range_score)))
        return self._result(True, self.last_reason, front, back, timing_delta, timing_sigma, nis, observability, geometry_score)

    def _replay_records(self) -> tuple[bool, float | None, str]:
        """Replay all retained observations in capture-time order."""

        if not self._records:
            if self._history_base_state is None:
                return False, None, "no_records"
            self.state = self._history_base_state.copy()
            self.covariance = self._history_base_covariance.copy()
            self.reference_time_s = self._history_base_time_s
            return True, 0.0, "accepted"
        records = sorted(self._records, key=self._record_time)
        if self._history_base_state is None or self._history_base_time_s is None:
            return False, None, "no_history_base"
        self.state = self._history_base_state.copy()
        self.covariance = self._history_base_covariance.copy()
        self.reference_time_s = self._history_base_time_s
        accepted_records = []
        post_states = []
        post_covariances = []
        last_nis: float | None = 0.0
        for record in records:
            record_time = self._record_time(record)
            if record_time < float(self.reference_time_s):
                return False, None, "OOSM_BEFORE_INITIALIZATION_BASE"
            self._predict_to(record_time)
            previous_state = self.state.copy()
            previous_covariance = self.covariance.copy()
            accepted, nis, _ = self._update_pixels(record)
            if not accepted:
                self.state = previous_state
                self.covariance = previous_covariance
                return False, nis, "innovation_gate_rejected" if nis is not None else "pixel_update_failed"
            accepted_records.append(record)
            post_states.append(self.state.copy())
            post_covariances.append(self.covariance.copy())
            last_nis = nis
        self._records = accepted_records
        self._record_post_states = post_states
        self._record_post_covariances = post_covariances
        self._record_keys = {self._record_key(*item) for item in self._records}
        self.last_nis = last_nis
        self.last_measurement_time_s = self.reference_time_s
        return True, last_nis, "accepted"

    def _reject(
        self,
        reason: str,
        front: PixelObservation,
        back: PixelObservation,
        timing_delta: float | None = None,
        timing_sigma: float | None = None,
    ) -> AsyncStereoResult:
        self.rejected_count += 1
        self.consecutive_hits = 0
        self.last_reason = reason
        return self._result(
            False,
            reason,
            front,
            back,
            abs(front.timestamp_s - back.timestamp_s) if timing_delta is None else timing_delta,
            self.config.timestamp_sigma_s if timing_sigma is None else timing_sigma,
            None,
        )

    def _result(
        self,
        accepted: bool,
        reason: str,
        front: PixelObservation,
        back: PixelObservation,
        timing_delta: float,
        timing_sigma: float,
        nis: float | None,
        observability: float = 0.0,
        geometry_score: float = 0.0,
    ) -> AsyncStereoResult:
        reprojection = math.inf
        if self.initialized:
            residuals = []
            for observation in (front, back):
                prediction = self._project(self.state, observation)
                if np.all(np.isfinite(prediction)):
                    residuals.extend([observation.u - prediction[0], observation.v - prediction[1]])
            if residuals:
                reprojection = float(np.sqrt(np.mean(np.square(residuals))))
        return AsyncStereoResult(
            accepted=accepted,
            reason=reason,
            reference_time_s=float(0.5 * (front.timestamp_s + back.timestamp_s)),
            state=self.state.copy(),
            covariance=self.covariance.copy(),
            timing_delta_s=float(timing_delta),
            timing_sigma_s=float(timing_sigma),
            reprojection_error_px=reprojection,
            range_observability=float(observability),
            measurement_geometry_score=float(geometry_score),
            innovation_nis=nis,
            oosm_count=self.oosm_count,
            oosm_drop_count=self.oosm_drop_count,
        )


def simulate_async_pair(
    position: Iterable[float],
    velocity: Iterable[float],
    front_timestamp_s: float,
    back_timestamp_s: float,
    config: AsyncStereoConfig | None = None,
    pixel_noise_px: float = 0.0,
) -> tuple[PixelObservation, PixelObservation]:
    """Create deterministic synthetic front/back pixels for tests and replay."""

    cfg = config or AsyncStereoConfig()
    position = np.asarray(tuple(position), dtype=float)
    velocity = np.asarray(tuple(velocity), dtype=float)
    observations = []
    for camera_id, timestamp in (("front", front_timestamp_s), ("back", back_timestamp_s)):
        point = position + velocity * (float(timestamp) - 0.5 * (front_timestamp_s + back_timestamp_s))
        point = point.copy()
        point[1] -= -0.5 * cfg.baseline_m if camera_id == "front" else 0.5 * cfg.baseline_m
        u = cfg.fx * point[1] / point[0] + cfg.cx
        v = cfg.cy - cfg.fy * point[2] / point[0]
        observations.append(
            PixelObservation(
                camera_id=camera_id,
                timestamp_s=float(timestamp),
                u=float(u + pixel_noise_px),
                v=float(v),
            )
        )
    return observations[0], observations[1]
