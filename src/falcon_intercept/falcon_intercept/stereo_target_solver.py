from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

from .detection_utils import DetectionBox, parse_detections_json
from .rectified_stereo import RectifiedStereoGeometry, load_rectified_stereo_geometry
from .async_state_estimator import (
    AsyncStereoConfig,
    AsyncStereoStateEstimator,
    PX4PoseHistory,
    PixelObservation,
)


CAMERA_POSITION_QOS_DEPTH = 1


@dataclass(frozen=True)
class PinholeStereoConfig:
    fx: float
    fy: float
    cx: float
    cy: float
    baseline_m: float
    min_confidence: float = 0.5
    max_time_delta_s: float = 0.05
    max_vertical_error_px: float = 8.0
    min_disparity_px: float = 1.0
    vertical_offset_front_minus_back_px: float = 0.0
    vertical_slope_front_minus_back: float = 0.0
    vertical_reference_u_px: float = 0.0
    horizontal_zero_disparity_offset_px: float = 0.0
    horizontal_disparity_sign: float = 1.0
    class_id: int | None = None
    class_name: str = ""
    # When set (checkerboard-measured calibration), point-level undistortion
    # and rectification replace the legacy empirical alignment corrections.
    rectified: RectifiedStereoGeometry | None = None


@dataclass(frozen=True)
class StereoDetectionPair:
    front: DetectionBox
    back: DetectionBox
    timestamp_s: float
    front_timestamp_s: float
    back_timestamp_s: float
    front_sequence: int | None
    back_sequence: int | None


def corrected_stereo_disparity(
    front: DetectionBox,
    back: DetectionBox,
    config: PinholeStereoConfig,
) -> float:
    """Return positive metric disparity after measured field corrections."""

    back_u, back_v = (float(value) for value in back.bbox_center)
    corrected_u_back = back_u - float(
        config.vertical_slope_front_minus_back
    ) * (back_v - float(config.cy))
    observed_disparity = float(front.bbox_center[0]) - corrected_u_back
    return float(config.horizontal_disparity_sign) * (
        observed_disparity
        - float(config.horizontal_zero_disparity_offset_px)
    )


def legacy_vertical_error(
    front: DetectionBox,
    back: DetectionBox,
    config: PinholeStereoConfig,
) -> float:
    """Vertical mismatch after the legacy empirical alignment correction."""

    back_u, back_v = (float(value) for value in back.bbox_center)
    expected_vertical_delta = (
        float(config.vertical_offset_front_minus_back_px)
        + float(config.vertical_slope_front_minus_back)
        * (back_u - float(config.vertical_reference_u_px))
    )
    return abs(float(front.bbox_center[1]) - back_v - expected_vertical_delta)


def stereo_pair_metrics(
    front: DetectionBox,
    back: DetectionBox,
    config: PinholeStereoConfig,
) -> tuple[float, float]:
    """Return (disparity_px, vertical_error_px) with the active geometry.

    Rectified mode undistorts/rectifies both detection centres point-by-point
    and measures the disparity there; legacy mode keeps the empirical field
    corrections.
    """

    if config.rectified is not None:
        return config.rectified.pair_metrics(
            float(front.bbox_center[0]),
            float(front.bbox_center[1]),
            float(back.bbox_center[0]),
            float(back.bbox_center[1]),
        )
    return (
        corrected_stereo_disparity(front, back, config),
        legacy_vertical_error(front, back, config),
    )


def triangulate_stereo_pair(
    front: DetectionBox,
    back: DetectionBox,
    config: PinholeStereoConfig,
) -> tuple[float, float, float] | None:
    """Triangulate a matched pair into x-forward/y-right/z-up metres."""

    if config.rectified is not None:
        disparity, _ = config.rectified.pair_metrics(
            float(front.bbox_center[0]),
            float(front.bbox_center[1]),
            float(back.bbox_center[0]),
            float(back.bbox_center[1]),
        )
        if disparity <= float(config.min_disparity_px):
            return None
        point = config.rectified.triangulate_fru(
            float(front.bbox_center[0]),
            float(front.bbox_center[1]),
            float(back.bbox_center[0]),
            float(back.bbox_center[1]),
        )
        return (float(point[0]), float(point[1]), float(point[2]))
    disparity = corrected_stereo_disparity(front, back, config)
    if disparity <= float(config.min_disparity_px):
        return None
    u_front, v_front = front.bbox_center
    x_forward = float(config.fx) * float(config.baseline_m) / disparity
    y_back = (float(u_front) - float(config.cx)) * x_forward / float(config.fx)
    z_up = -(float(v_front) - float(config.cy)) * x_forward / float(config.fy)
    return x_forward, y_back, z_up


def load_stereo_geometry_values(
    calibration_path: str | Path,
) -> tuple[float, float, float, float, float]:
    """Load front-camera pinhole values and the assembled stereo baseline."""

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load stereo calibration") from exc

    with Path(calibration_path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    front = data.get("stereo_front", data.get("stereo_left"))
    pair = data.get("stereo_pair")
    if not isinstance(front, dict) or not isinstance(pair, dict):
        raise ValueError(
            "stereo calibration requires stereo_front (or legacy stereo_left) and stereo_pair sections"
        )
    required = ("fx", "fy", "cx", "cy")
    if any(name not in front for name in required) or "baseline_m" not in pair:
        raise ValueError("stereo calibration is missing fx/fy/cx/cy or baseline_m")
    values = tuple(float(front[name]) for name in required) + (float(pair["baseline_m"]),)
    if values[0] <= 0.0 or values[1] <= 0.0 or values[4] <= 0.0:
        raise ValueError("stereo focal lengths and baseline must be positive")
    return values


def load_stereo_alignment_values(
    calibration_path: str | Path,
) -> tuple[float, float, float, float, float]:
    """Load the measured physical-back to physical-front alignment correction."""

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load stereo calibration") from exc

    with Path(calibration_path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    pair = data.get("stereo_pair")
    if not isinstance(pair, dict):
        raise ValueError("stereo calibration requires a stereo_pair section")
    alignment = pair.get("field_alignment", {})
    if not isinstance(alignment, dict):
        raise ValueError("stereo_pair.field_alignment must be a mapping")
    values = (
        float(alignment.get("vertical_offset_front_minus_back_px", alignment.get("vertical_offset_left_minus_right_px", 0.0))),
        float(alignment.get("vertical_slope_front_minus_back", alignment.get("vertical_slope_left_minus_right", 0.0))),
        float(alignment.get("vertical_reference_u_px", 0.0)),
        float(alignment.get("horizontal_zero_disparity_offset_px", 0.0)),
        float(alignment.get("horizontal_disparity_sign", 1.0)),
    )
    if not all(value == value and abs(value) != float("inf") for value in values):
        raise ValueError("stereo field-alignment values must be finite")
    if values[4] not in (-1.0, 1.0):
        raise ValueError("horizontal_disparity_sign must be -1 or 1")
    return values


def select_stereo_detection_pair(
    front_json: str,
    back_json: str,
    config: PinholeStereoConfig,
    *,
    enforce_time_delta: bool = True,
) -> StereoDetectionPair | None:
    """Select one time-aligned, class-consistent, rectified stereo pair."""

    try:
        front_frame = parse_detections_json(front_json)
        back_frame = parse_detections_json(back_json)
    except (TypeError, ValueError, KeyError):
        return None

    if front_frame.stamp_s is None or back_frame.stamp_s is None:
        return None
    if enforce_time_delta and abs(float(front_frame.stamp_s) - float(back_frame.stamp_s)) > float(config.max_time_delta_s):
        return None

    def eligible(frame) -> list[DetectionBox]:
        result: list[DetectionBox] = []
        for detection in frame.detections:
            if detection.confidence < float(config.min_confidence):
                continue
            if config.class_id is not None and detection.class_id != int(config.class_id):
                continue
            if config.class_name and detection.class_name != str(config.class_name):
                continue
            result.append(detection)
        return result

    pairs: list[tuple[float, float, DetectionBox, DetectionBox]] = []
    for front in eligible(front_frame):
        for back in eligible(back_frame):
            if front.class_id != back.class_id or front.class_name != back.class_name:
                continue
            disparity, vertical_error = stereo_pair_metrics(front, back, config)
            if disparity <= float(config.min_disparity_px):
                continue
            if vertical_error > float(config.max_vertical_error_px):
                continue
            pairs.append(
                (float(front.confidence) * float(back.confidence), vertical_error, front, back)
            )

    if not pairs:
        return None
    _, _, front, back = max(pairs, key=lambda item: (item[0], -item[1]))
    return StereoDetectionPair(
        front=front,
        back=back,
        timestamp_s=(float(front_frame.stamp_s) + float(back_frame.stamp_s)) / 2.0,
        front_timestamp_s=float(front_frame.stamp_s),
        back_timestamp_s=float(back_frame.stamp_s),
        front_sequence=front_frame.sequence,
        back_sequence=back_frame.sequence,
    )


def solve_pinhole_stereo_measurement(
    front_json: str,
    back_json: str,
    config: PinholeStereoConfig,
) -> tuple[float, float, float] | None:
    pair = select_stereo_detection_pair(front_json, back_json, config)
    if pair is None:
        return None
    return triangulate_stereo_pair(pair.front, pair.back, config)


def forward_right_up_to_optical(
    position: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Map internal x-forward/y-right/z-up into REP-103 optical X-back/Y-down/Z-forward."""

    x_forward, y_back, z_up = (float(value) for value in position)
    return y_back, -z_up, x_forward


def forward_right_up_covariance_to_optical(
    covariance: tuple[float, ...] | list[float],
) -> tuple[float, ...]:
    """Apply C_optical = T C_internal T^T to a row-major 3x3 covariance."""

    if len(covariance) != 9:
        raise ValueError("covariance must contain 9 row-major values")
    source = [[float(covariance[row * 3 + column]) for column in range(3)] for row in range(3)]
    transform = (
        (0.0, 1.0, 0.0),
        (0.0, 0.0, -1.0),
        (1.0, 0.0, 0.0),
    )
    product = [
        [sum(transform[row][axis] * source[axis][column] for axis in range(3)) for column in range(3)]
        for row in range(3)
    ]
    result = [
        sum(product[row][axis] * transform[column][axis] for axis in range(3))
        for row in range(3)
        for column in range(3)
    ]
    return tuple(result)


def world_position_estimate_to_rig(
    position_world,
    covariance_world,
    rotation_world_from_rig,
    translation_world_rig,
):
    """Convert an EKF world-frame position/covariance to current rig FRU."""

    import numpy as np

    position = np.asarray(position_world, dtype=float).reshape(3)
    covariance = np.asarray(covariance_world, dtype=float).reshape(3, 3)
    rotation = np.asarray(rotation_world_from_rig, dtype=float).reshape(3, 3)
    translation = np.asarray(translation_world_rig, dtype=float).reshape(3)
    rotation_rig_from_world = rotation.T
    return (
        rotation_rig_from_world @ (position - translation),
        rotation_rig_from_world @ covariance @ rotation,
    )


def _set_stamp_from_seconds(stamp, seconds: float) -> None:
    sec = int(seconds)
    nanosec = int(round((float(seconds) - sec) * 1_000_000_000))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec -= 1_000_000_000
    stamp.sec = sec
    stamp.nanosec = nanosec


def main(args=None) -> int:
    import rclpy
    from falcon_intercept_msgs.msg import (
        AsyncStereoMeasurement,
        CameraBearingObservation,
        TargetCameraPosition,
    )
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from px4_msgs.msg import VehicleAngularVelocity, VehicleAttitude, VehicleLocalPosition
    from rclpy.node import Node
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import String

    class StereoTargetSolverNode(Node):
        def __init__(self) -> None:
            super().__init__("stereo_target_solver_node")
            self.declare_parameter("front_detections_topic", "/stereo/front/detections_json")
            self.declare_parameter("back_detections_topic", "/stereo/back/detections_json")
            self.declare_parameter("measurement_topic", "/target/relative_measurement")
            self.declare_parameter("frame_id", "stereo_camera")
            self.declare_parameter(
                "camera_position_topic", "/perception/target/camera_position"
            )
            self.declare_parameter("camera_optical_frame_id", "pursuer_camera_optical")
            self.declare_parameter("publish_invalid_camera_position", True)
            self.declare_parameter("calibration_path", "")
            # Safe direct-run fallback mirrors the packaged field-refined estimate.
            # Production launch still loads the YAML through calibration_path.
            self.declare_parameter("fx", 10238.013493164)
            self.declare_parameter("fy", 10322.580645161)
            self.declare_parameter("cx", 1919.5)
            self.declare_parameter("cy", 1079.5)
            self.declare_parameter("baseline_m", 0.10)
            self.declare_parameter("min_confidence", 0.5)
            self.declare_parameter("measurement_variance_m2", 0.25)
            self.declare_parameter("invalid_variance_m2", 1e6)
            self.declare_parameter("max_time_delta_s", 0.05)
            self.declare_parameter("max_vertical_error_px", 8.0)
            self.declare_parameter("min_disparity_px", 1.0)
            self.declare_parameter(
                "vertical_offset_front_minus_back_px", -50.421808192
            )
            self.declare_parameter(
                "vertical_slope_front_minus_back", -0.025377902666
            )
            self.declare_parameter("vertical_reference_u_px", 1919.5)
            self.declare_parameter(
                "horizontal_zero_disparity_offset_px", -365.544120787
            )
            self.declare_parameter("horizontal_disparity_sign", 1.0)
            self.declare_parameter("class_id", -1)
            self.declare_parameter("class_name", "")
            self.declare_parameter("async_enabled", True)
            self.declare_parameter(
                "async_measurement_topic", "/perception/async_stereo_measurement"
            )
            self.declare_parameter(
                "front_bearing_topic", "/perception/stereo/front/bearing"
            )
            self.declare_parameter(
                "back_bearing_topic", "/perception/stereo/back/bearing"
            )
            self.declare_parameter("async_max_observation_delta_s", 0.10)
            self.declare_parameter("async_pair_buffer_s", 0.25)
            self.declare_parameter("timestamp_sigma_s", 0.0015)
            self.declare_parameter("async_fixed_lag_s", 0.30)
            self.declare_parameter("async_pixel_variance_px2", 64.0)
            # 2026-08-06 RTK-anchored field fit: constant back-rectified-u
            # offset (px) removing the +3-4% range bias in the 10-20 m working
            # range; see ledger sigma calibration entry.
            self.declare_parameter("rectified_back_u_offset_px", -1.65)
            self.declare_parameter("async_process_accel_variance", 4.0)
            # 2026-08-14: range plausibility gate on published measurements.
            # Mis-paired detections occasionally produce garbage ranges
            # (observed 14.5 m -> 0.77 m jumps) which trip the upstream
            # safety_minimum_depth_m abort.  A physical relative rate limit
            # rejects them; anything after a >reacquisition_gap_s silence is
            # accepted unconditionally (fresh acquisition).
            self.declare_parameter("max_range_rate_mps", 40.0)
            self.declare_parameter("range_rate_margin_m", 0.5)
            self.declare_parameter("reacquisition_gap_s", 1.0)
            # Absolute plausibility ceiling: at 0.1 m baseline a range beyond
            # ~80 m is a sub-2 px disparity artefact, never a real target.
            self.declare_parameter("max_range_m", 80.0)
            self.declare_parameter("publish_legacy_fallback", False)
            self.declare_parameter("use_px4_pose_history", True)
            self.declare_parameter(
                "px4_local_position_topic", "/fmu/out/vehicle_local_position"
            )
            self.declare_parameter("px4_attitude_topic", "/fmu/out/vehicle_attitude")
            self.declare_parameter(
                "px4_angular_velocity_topic", "/fmu/out/vehicle_angular_velocity"
            )

            # Keep short capture-time buffers.  Pairing the latest front JSON
            # with the latest back JSON can silently create a 1-3 frame
            # mismatch when the two callbacks are serviced at different
            # times, even if the source timestamps are available.
            self._front_buffer: list[tuple[float, str]] = []
            self._back_buffer: list[tuple[float, str]] = []
            # DDS/GStreamer integration can occasionally deliver the same
            # source JSON to this process twice even though independent topic
            # observers see a clean stream.  Without an input-side guard, the
            # duplicate can be paired with the next frame from the other
            # camera and produce two KF measurements at one capture time.
            self._last_input_frame_key: dict[str, tuple[int | None, float] | None] = {
                "front": None,
                "back": None,
            }
            self._last_pair_key: tuple[float, float, float, float, float] | None = None
            self._last_camera_frame_key: (
                tuple[int | None, float, int | None, float] | None
            ) = None
            self._last_camera_sequence = -1
            self._frame_id = str(self.get_parameter("frame_id").value)
            self._camera_frame_id = str(
                self.get_parameter("camera_optical_frame_id").value
            )
            self._publish_invalid = bool(
                self.get_parameter("publish_invalid_camera_position").value
            )
            self._async_enabled = bool(self.get_parameter("async_enabled").value)
            self._async_max_delta_s = float(
                self.get_parameter("async_max_observation_delta_s").value
            )
            self._pair_buffer_s = max(
                self._async_max_delta_s,
                float(self.get_parameter("async_pair_buffer_s").value),
            )
            self._timestamp_sigma_s = float(self.get_parameter("timestamp_sigma_s").value)
            self._publish_legacy_fallback = bool(
                self.get_parameter("publish_legacy_fallback").value
            )
            self._max_range_rate_mps = float(
                self.get_parameter("max_range_rate_mps").value
            )
            self._range_rate_margin_m = float(
                self.get_parameter("range_rate_margin_m").value
            )
            self._reacquisition_gap_s = float(
                self.get_parameter("reacquisition_gap_s").value
            )
            self._max_range_m = float(self.get_parameter("max_range_m").value)
            self._last_valid_range_m = None
            self._last_valid_pub_s = None
            self._variance = float(self.get_parameter("measurement_variance_m2").value)
            self._invalid_variance = float(self.get_parameter("invalid_variance_m2").value)
            class_id = int(self.get_parameter("class_id").value)
            geometry_values = (
                float(self.get_parameter("fx").value),
                float(self.get_parameter("fy").value),
                float(self.get_parameter("cx").value),
                float(self.get_parameter("cy").value),
                float(self.get_parameter("baseline_m").value),
            )
            calibration_path = str(self.get_parameter("calibration_path").value).strip()
            alignment_values = (
                float(
                    self.get_parameter(
                        "vertical_offset_front_minus_back_px"
                    ).value
                ),
                float(
                    self.get_parameter(
                        "vertical_slope_front_minus_back"
                    ).value
                ),
                float(self.get_parameter("vertical_reference_u_px").value),
                float(
                    self.get_parameter(
                        "horizontal_zero_disparity_offset_px"
                    ).value
                ),
                float(self.get_parameter("horizontal_disparity_sign").value),
            )
            if calibration_path:
                geometry_values = load_stereo_geometry_values(calibration_path)
                alignment_values = load_stereo_alignment_values(calibration_path)
                self.get_logger().info(
                    "loaded stereo geometry path=%s fx=%.6f baseline_m=%.6f "
                    "vertical_offset=%.3f vertical_slope=%.8f "
                    "horizontal_zero=%.3f horizontal_sign=%.0f"
                    % (
                        calibration_path,
                        geometry_values[0],
                        geometry_values[4],
                        alignment_values[0],
                        alignment_values[1],
                        alignment_values[3],
                        alignment_values[4],
                    )
                )
            self._rectified_geometry = (
                load_rectified_stereo_geometry(calibration_path)
                if calibration_path
                else None
            )
            if self._rectified_geometry is not None:
                from dataclasses import replace as _dataclass_replace

                self._rectified_geometry = _dataclass_replace(
                    self._rectified_geometry,
                    back_u_offset_px=float(
                        self.get_parameter("rectified_back_u_offset_px").value
                    ),
                )
            self._rectified_rotation_fru = (
                self._rectified_geometry.rotation_fru_rectified_to_front()
                if self._rectified_geometry is not None
                else None
            )
            if self._rectified_geometry is not None:
                self.get_logger().info(
                    "rectified stereo mode: point-level undistort/rectify, "
                    "fx_rect=%.3f baseline_rect_m=%.6f; legacy alignment "
                    "corrections disabled"
                    % (
                        self._rectified_geometry.fx_rect,
                        self._rectified_geometry.baseline_rect_m,
                    )
                )
            self._config = PinholeStereoConfig(
                fx=geometry_values[0],
                fy=geometry_values[1],
                cx=geometry_values[2],
                cy=geometry_values[3],
                baseline_m=geometry_values[4],
                min_confidence=float(self.get_parameter("min_confidence").value),
                max_time_delta_s=float(self.get_parameter("max_time_delta_s").value),
                max_vertical_error_px=float(self.get_parameter("max_vertical_error_px").value),
                min_disparity_px=float(self.get_parameter("min_disparity_px").value),
                vertical_offset_front_minus_back_px=alignment_values[0],
                vertical_slope_front_minus_back=alignment_values[1],
                vertical_reference_u_px=alignment_values[2],
                horizontal_zero_disparity_offset_px=alignment_values[3],
                horizontal_disparity_sign=alignment_values[4],
                class_id=None if class_id < 0 else class_id,
                class_name=str(self.get_parameter("class_name").value).strip(),
                rectified=self._rectified_geometry,
            )
            if self._rectified_geometry is not None:
                # The asynchronous EKF assumes one shared pinhole model with
                # the cameras translated along the baseline: exactly the
                # rectified virtual pair.  Feed it rectified pixels and zero
                # the empirical corrections.
                async_geometry_values = (
                    self._rectified_geometry.fx_rect,
                    self._rectified_geometry.fy_rect,
                    self._rectified_geometry.cx_rect,
                    self._rectified_geometry.cy_rect,
                    self._rectified_geometry.baseline_rect_m,
                )
                async_alignment_values = (0.0, 0.0, 0.0, 0.0, 1.0)
            else:
                async_geometry_values = geometry_values
                async_alignment_values = alignment_values
            self._async_config = AsyncStereoConfig(
                fx=async_geometry_values[0],
                fy=async_geometry_values[1],
                cx=async_geometry_values[2],
                cy=async_geometry_values[3],
                baseline_m=async_geometry_values[4],
                vertical_offset_front_minus_back_px=async_alignment_values[0],
                vertical_slope_front_minus_back=async_alignment_values[1],
                vertical_reference_u_px=async_alignment_values[2],
                horizontal_zero_disparity_offset_px=async_alignment_values[3],
                horizontal_disparity_sign=async_alignment_values[4],
                pixel_variance_px2=float(self.get_parameter("async_pixel_variance_px2").value),
                timestamp_sigma_s=self._timestamp_sigma_s,
                process_accel_variance=float(
                    self.get_parameter("async_process_accel_variance").value
                ),
                fixed_lag_s=float(self.get_parameter("async_fixed_lag_s").value),
            )
            self._async_estimator = AsyncStereoStateEstimator(self._async_config)
            self._pose_history = PX4PoseHistory(history_s=2.0)
            self._use_px4_pose_history = bool(
                self.get_parameter("use_px4_pose_history").value
            )
            if self._use_px4_pose_history:
                self._async_estimator.set_pose_provider(self._pose_history)
            self._pub = self.create_publisher(
                PoseWithCovarianceStamped,
                str(self.get_parameter("measurement_topic").value),
                qos_profile_sensor_data,
            )
            camera_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=CAMERA_POSITION_QOS_DEPTH,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self._camera_pub = self.create_publisher(
                TargetCameraPosition,
                str(self.get_parameter("camera_position_topic").value),
                camera_qos,
            )
            self._async_pub = self.create_publisher(
                AsyncStereoMeasurement,
                str(self.get_parameter("async_measurement_topic").value),
                camera_qos,
            )
            self._front_bearing_pub = self.create_publisher(
                CameraBearingObservation,
                str(self.get_parameter("front_bearing_topic").value),
                camera_qos,
            )
            self._back_bearing_pub = self.create_publisher(
                CameraBearingObservation,
                str(self.get_parameter("back_bearing_topic").value),
                camera_qos,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("front_detections_topic").value),
                self._on_front,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("back_detections_topic").value),
                self._on_back,
                qos_profile_sensor_data,
            )
            if self._use_px4_pose_history:
                self.create_subscription(
                    VehicleLocalPosition,
                    str(self.get_parameter("px4_local_position_topic").value),
                    self._on_px4_local_position,
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    VehicleAttitude,
                    str(self.get_parameter("px4_attitude_topic").value),
                    self._on_px4_attitude,
                    qos_profile_sensor_data,
                )
                self.create_subscription(
                    VehicleAngularVelocity,
                    str(self.get_parameter("px4_angular_velocity_topic").value),
                    self._on_px4_angular_velocity,
                    qos_profile_sensor_data,
                )

        def _on_front(self, msg: String) -> None:
            if self._push_detection("front", self._front_buffer, str(msg.data)):
                self._try_publish()

        def _on_back(self, msg: String) -> None:
            if self._push_detection("back", self._back_buffer, str(msg.data)):
                self._try_publish()

        def _push_detection(
            self, stream: str, buffer: list[tuple[float, str]], payload: str
        ) -> bool:
            try:
                frame = parse_detections_json(payload)
            except (TypeError, ValueError, KeyError):
                return False
            if frame.stamp_s is None:
                return False
            frame_key = (frame.sequence, float(frame.stamp_s))
            if self._last_input_frame_key[stream] == frame_key:
                return False
            self._last_input_frame_key[stream] = frame_key
            buffer.append((float(frame.stamp_s), payload))
            cutoff = float(frame.stamp_s) - self._pair_buffer_s
            while buffer and buffer[0][0] < cutoff:
                buffer.pop(0)
            # A malformed or stalled source must not grow the queue without
            # bound.  At 30 Hz this retains more than the configured lag.
            del buffer[:-32]
            return True

        def _px4_timestamp_s(self, message) -> float:
            return float(getattr(message, "timestamp", 0)) / 1_000_000.0

        def _on_px4_local_position(self, message) -> None:
            self._pose_history.add_position(
                self._px4_timestamp_s(message),
                (float(message.x), float(message.y), float(message.z)),
                receive_time_s=float(self.get_clock().now().nanoseconds) / 1e9,
            )

        def _on_px4_attitude(self, message) -> None:
            self._pose_history.add_attitude(
                self._px4_timestamp_s(message),
                tuple(float(value) for value in message.q),
                receive_time_s=float(self.get_clock().now().nanoseconds) / 1e9,
            )

        def _on_px4_angular_velocity(self, message) -> None:
            values = getattr(message, "angular_velocity", (0.0, 0.0, 0.0))
            self._pose_history.add_angular_velocity(
                self._px4_timestamp_s(message),
                tuple(float(value) for value in values),
                receive_time_s=float(self.get_clock().now().nanoseconds) / 1e9,
            )

        def _publish_bearing(self, publisher, camera_id: str, frame, detection, sequence: int) -> None:
            message = CameraBearingObservation()
            _set_stamp_from_seconds(message.header.stamp, float(frame.stamp_s))
            message.header.frame_id = self._camera_frame_id
            message.camera_id = str(camera_id)
            message.sequence = int(sequence)
            message.host_receive_ns = int(self.get_clock().now().nanoseconds)
            message.timestamp_sigma_s = float(
                frame.timestamp_sigma_s
                if frame.timestamp_sigma_s is not None
                else self._timestamp_sigma_s
            )
            message.exposure_time_s = -1.0
            message.frame_readout_time_s = -1.0
            message.pixel_u = float(detection.bbox_center[0])
            message.pixel_v = float(detection.bbox_center[1])
            pixel_variance = float(self._async_config.pixel_variance_px2)
            message.pixel_covariance = [pixel_variance, 0.0, 0.0, pixel_variance]
            message.confidence = float(detection.confidence)
            message.valid = True
            publisher.publish(message)

        def _publish_async_measurement(self, pair, front_frame, back_frame, sequence: int):
            if self._rectified_geometry is not None:
                front_u, front_v = self._rectified_geometry.rectify_front(
                    float(pair.front.bbox_center[0]), float(pair.front.bbox_center[1])
                )
                back_u, back_v = self._rectified_geometry.rectify_back(
                    float(pair.back.bbox_center[0]), float(pair.back.bbox_center[1])
                )
            else:
                front_u, front_v = (float(value) for value in pair.front.bbox_center)
                back_u, back_v = (float(value) for value in pair.back.bbox_center)
            front_observation = PixelObservation(
                camera_id="front",
                timestamp_s=float(pair.front_timestamp_s),
                u=float(front_u),
                v=float(front_v),
                sequence=int(pair.front_sequence or sequence),
                timestamp_sigma_s=(
                    float(front_frame.timestamp_sigma_s)
                    if front_frame.timestamp_sigma_s is not None
                    else self._timestamp_sigma_s
                ),
                pixel_variance_px2=float(self._async_config.pixel_variance_px2),
                confidence=float(pair.front.confidence),
            )
            back_observation = PixelObservation(
                camera_id="back",
                timestamp_s=float(pair.back_timestamp_s),
                u=float(back_u),
                v=float(back_v),
                sequence=int(pair.back_sequence or sequence),
                timestamp_sigma_s=(
                    float(back_frame.timestamp_sigma_s)
                    if back_frame.timestamp_sigma_s is not None
                    else self._timestamp_sigma_s
                ),
                pixel_variance_px2=float(self._async_config.pixel_variance_px2),
                confidence=float(pair.back.confidence),
            )
            result = self._async_estimator.update(front_observation, back_observation)
            position = result.position
            position_covariance = result.covariance[:3, :3]
            if self._use_px4_pose_history:
                # The asynchronous EKF uses a world-frame state so each pixel
                # can be evaluated at its own PX4 pose.  The public message,
                # however, has always promised a camera-relative FRU vector.
                # Convert at the estimator reference time before crossing the
                # ROS interface boundary; otherwise local altitude/position
                # leak into target range and lateral guidance.
                rotation_world_from_rig, translation_world_rig = (
                    self._pose_history.pose_at(result.reference_time_s)
                )
                position, position_covariance = world_position_estimate_to_rig(
                    result.position,
                    position_covariance,
                    rotation_world_from_rig,
                    translation_world_rig,
                )
            if self._rectified_rotation_fru is not None:
                # The EKF ran in the rectified-front frame; rotate its output
                # back into the raw front-camera FRU frame so downstream
                # consumers keep the established convention.
                import numpy as np

                position = self._rectified_rotation_fru @ np.asarray(position, dtype=float)
                position_covariance = (
                    self._rectified_rotation_fru
                    @ np.asarray(position_covariance, dtype=float).reshape(3, 3)
                    @ self._rectified_rotation_fru.T
                )
            # AsyncStereoMeasurement.position_camera follows the same REP-103
            # optical contract as TargetCameraPosition and TargetTrack:
            # +X back, +Y down, +Z forward.  The estimator itself remains in
            # internal forward/back/up coordinates.
            optical_position = forward_right_up_to_optical(tuple(position))
            optical_covariance = forward_right_up_covariance_to_optical(
                position_covariance.reshape(-1)
            )
            message = AsyncStereoMeasurement()
            _set_stamp_from_seconds(message.header.stamp, result.reference_time_s)
            message.header.frame_id = self._frame_id
            message.sequence = int(sequence)
            message.front_capture_ns = int(round(pair.front_timestamp_s * 1_000_000_000.0))
            message.back_capture_ns = int(round(pair.back_timestamp_s * 1_000_000_000.0))
            message.reference_time = float(result.reference_time_s)
            message.timing_delta = float(result.timing_delta_s)
            message.timing_sigma = float(result.timing_sigma_s)
            message.front_pixel_u = float(pair.front.bbox_center[0])
            message.front_pixel_v = float(pair.front.bbox_center[1])
            message.back_pixel_u = float(pair.back.bbox_center[0])
            message.back_pixel_v = float(pair.back.bbox_center[1])
            message.position_camera.x = float(optical_position[0])
            message.position_camera.y = float(optical_position[1])
            message.position_camera.z = float(optical_position[2])
            message.position_covariance = [
                float(value) for value in optical_covariance
            ]
            message.reprojection_error_px = float(result.reprojection_error_px)
            message.range_observability = float(result.range_observability)
            message.measurement_geometry_score = float(result.measurement_geometry_score)
            message.confidence = float(
                min(pair.front.confidence, pair.back.confidence)
                * max(0.0, min(1.0, result.range_observability))
            )
            message.valid = bool(result.accepted)
            message.reason = str(result.reason)
            if message.valid:
                range_m = math.sqrt(
                    sum(value * value for value in optical_position)
                )
                now_s = self.get_clock().now().nanoseconds / 1.0e9
                if range_m > self._max_range_m:
                    message.valid = False
                    message.reason = f"range plausibility: {range_m:.2f}m > {self._max_range_m:.0f}m"
                if message.valid and self._last_valid_range_m is not None and self._last_valid_pub_s is not None:
                    gap_s = now_s - self._last_valid_pub_s
                    allowed = (
                        self._max_range_rate_mps * max(gap_s, 0.0)
                        + self._range_rate_margin_m
                    )
                    if (
                        gap_s <= self._reacquisition_gap_s
                        and abs(range_m - self._last_valid_range_m) > allowed
                    ):
                        message.valid = False
                        message.reason = (
                            f"range rate gate: {self._last_valid_range_m:.2f}m -> "
                            f"{range_m:.2f}m in {gap_s:.3f}s"
                        )
                if message.valid:
                    self._last_valid_range_m = range_m
                    self._last_valid_pub_s = now_s
            self._async_pub.publish(message)
            return result

        def _try_publish(self) -> None:
            if not self._front_buffer or not self._back_buffer:
                return
            maximum_delta = (
                self._async_max_delta_s if self._async_enabled else self._config.max_time_delta_s
            )
            candidates = []
            for front_index, (front_timestamp, _) in enumerate(self._front_buffer):
                for back_index, (back_timestamp, _) in enumerate(self._back_buffer):
                    delta = abs(front_timestamp - back_timestamp)
                    if delta <= maximum_delta:
                        candidates.append((delta, -max(front_timestamp, back_timestamp), front_index, back_index))
            if not candidates:
                return
            _, _, front_index, back_index = min(candidates)
            _, front_json = self._front_buffer.pop(front_index)
            _, back_json = self._back_buffer.pop(back_index)
            try:
                front_frame = parse_detections_json(front_json)
                back_frame = parse_detections_json(back_json)
            except (TypeError, ValueError, KeyError):
                return
            if front_frame.stamp_s is None or back_frame.stamp_s is None:
                return
            camera_frame_key = (
                front_frame.sequence,
                float(front_frame.stamp_s),
                back_frame.sequence,
                float(back_frame.stamp_s),
            )
            if self._last_camera_frame_key == camera_frame_key:
                return
            self._last_camera_frame_key = camera_frame_key

            pair = select_stereo_detection_pair(
                front_json,
                back_json,
                self._config,
                enforce_time_delta=not self._async_enabled,
            )
            camera_sequence_candidate = front_frame.sequence
            if (
                camera_sequence_candidate is None
                or int(camera_sequence_candidate) <= self._last_camera_sequence
            ):
                camera_sequence_candidate = self._last_camera_sequence + 1
            camera_sequence = int(camera_sequence_candidate) & 0xFFFFFFFF
            self._last_camera_sequence = int(camera_sequence_candidate)

            camera_msg = TargetCameraPosition()
            _set_stamp_from_seconds(camera_msg.header.stamp, float(front_frame.stamp_s))
            camera_msg.header.frame_id = self._camera_frame_id
            camera_msg.sequence = camera_sequence
            camera_msg.valid = pair is not None
            camera_msg.confidence = 0.0
            if pair is None:
                invalid_covariance = [0.0] * 9
                invalid_covariance[0] = self._invalid_variance
                invalid_covariance[4] = self._invalid_variance
                invalid_covariance[8] = self._invalid_variance
                camera_msg.position_covariance = invalid_covariance
                if self._publish_invalid:
                    self._camera_pub.publish(camera_msg)
                return

            pair_key = (
                float(pair.timestamp_s),
                float(pair.front.bbox_center[0]),
                float(pair.front.bbox_center[1]),
                float(pair.back.bbox_center[0]),
                float(pair.back.bbox_center[1]),
            )
            if self._last_pair_key == pair_key:
                return
            self._last_pair_key = pair_key
            async_result = None
            if self._async_enabled:
                self._publish_bearing(
                    self._front_bearing_pub,
                    "front",
                    front_frame,
                    pair.front,
                    camera_sequence,
                )
                self._publish_bearing(
                    self._back_bearing_pub,
                    "back",
                    back_frame,
                    pair.back,
                    camera_sequence,
                )
                async_result = self._publish_async_measurement(
                    pair, front_frame, back_frame, camera_sequence
                )
            solved = triangulate_stereo_pair(pair.front, pair.back, self._config)
            if solved is None:
                camera_msg.valid = False
                invalid_covariance = [0.0] * 9
                invalid_covariance[0] = self._invalid_variance
                invalid_covariance[4] = self._invalid_variance
                invalid_covariance[8] = self._invalid_variance
                camera_msg.position_covariance = invalid_covariance
                if self._publish_invalid:
                    self._camera_pub.publish(camera_msg)
                return
            x_forward, y_back, z_up = solved
            optical_x, optical_y, optical_z = forward_right_up_to_optical(
                (x_forward, y_back, z_up)
            )
            internal_covariance = (
                self._variance,
                0.0,
                0.0,
                0.0,
                self._variance,
                0.0,
                0.0,
                0.0,
                self._variance,
            )
            camera_msg.position_camera.x = optical_x
            camera_msg.position_camera.y = optical_y
            camera_msg.position_camera.z = optical_z
            camera_msg.position_covariance = list(
                forward_right_up_covariance_to_optical(internal_covariance)
            )
            camera_msg.confidence = float(
                min(pair.front.confidence, pair.back.confidence)
            )
            camera_msg.valid = optical_z > 0.0
            if (
                not self._async_enabled
                or async_result is None
                or not async_result.accepted
                or self._publish_legacy_fallback
            ):
                self._camera_pub.publish(camera_msg)

            msg = PoseWithCovarianceStamped()
            _set_stamp_from_seconds(msg.header.stamp, pair.timestamp_s)
            msg.header.frame_id = self._frame_id
            pose_position = (
                async_result.position
                if async_result is not None and async_result.accepted
                else (x_forward, y_back, z_up)
            )
            msg.pose.pose.position.x = float(pose_position[0])
            msg.pose.pose.position.y = float(pose_position[1])
            msg.pose.pose.position.z = float(pose_position[2])
            msg.pose.pose.orientation.w = 1.0
            covariance = [0.0] * 36
            if async_result is not None and async_result.accepted:
                for row in range(3):
                    for column in range(3):
                        covariance[row * 6 + column] = float(
                            async_result.covariance[row, column]
                        )
            else:
                covariance[0] = self._variance
                covariance[7] = self._variance
                covariance[14] = self._variance
            msg.pose.covariance = covariance
            self._pub.publish(msg)

    rclpy.init(args=args)
    node = None
    try:
        node = StereoTargetSolverNode()
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
