"""Search-camera-only visual servo mode (search_servo_v0).

Full-chain fallback mode that needs no stereo depth and no heavy calibration:
the interceptor holds a 5-10 m standoff under the target by servoing the
target's position in the search-camera image (azimuth/pitch) and its apparent
bbox size (range), then publishes VirtualShotEvent for the existing virtual
projectile referee.

Control interface: this node publishes OffboardControlMode + TrajectorySetpoint
on /upstream/fmu/in/* so the production isolation chain (upstream_gate_node ->
px4_upstream_adapter_node) keeps every safety gate: manual arm and takeoff,
offboard preheat heartbeat, human-takeover latch, failsafe cutout.

Geometry conventions (verified against perception_bridge_node and the
2026-09-06 indoor calibration, where aiming the AIRFRAME at the target put
it at ~0.68 of the image width):
  SearchObservation angles are computed in the canonical body frame
  (x=right, y=forward, z=body-up AS IF the lens pointed straight up - the
  physical 10 deg nose tilt is NOT in these angles):
    horizontal = atan2(right, up)   0 = target at image centre, + = right
    vertical   = atan2(forward, up) 0 = image centre, + = toward nose side
  The virtual shot fires along the BODY -Z axis, which sits 10 deg from the
  lens axis: in published angles the body axis is at vertical = -10 deg
  (target then appears ~10 deg tail-ward of the image centre, i.e. right of
  centre in the raw image). Holding pitch_center = -10 deg therefore keeps
  the shot axis on the target with the interceptor directly beneath it.

Servo law (body-frame decoupled; each channel monotone near the station):
  forward  = +k_forward * (vertical - pitch_center)   re-centre pitch
  right    = +k_right  * horizontal                   re-centre azimuth
  down     = -k_range  * (range - desired_range)      hold the standoff
  yaw set  = azimuth of the LOS in NED                nose faces the target
Range comes from the target bbox: range = reference_range_m *
(reference_bbox_px/reference_image_width) / (bbox_px/image_width), i.e. a
width-fraction calibration that survives detection resolution changes.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field

# 双链共享数学（low_pass/rotate_vector），实现与原本地版本一致；本模块
# 名字空间继续再导出，既有 `from search_servo_node import low_pass, ...`
# 的导入方（含测试）不受影响。
from .geometry_math import low_pass, rotate_vector  # noqa: F401  (re-export)


def wrap_pi(angle: float) -> float:
    """Wrap an angle to [-pi, pi].

    契约注释：falcon_contact.contact_logic.wrap_pi 是取模实现，在 3π 边界
    返回 -π；本实现（循环）返回 +π，且被 test_search_servo.py 钉住。
    两链不能互换，合并前必须先统一两侧测试期望。
    """
    angle = float(angle)
    if not math.isfinite(angle):
        return 0.0
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def range_from_bbox_ratio(
    bbox_px: float,
    image_width: float,
    reference_range_m: float,
    reference_bbox_px: float,
    reference_image_width: float,
    minimum_bbox_px: float = 8.0,
) -> float | None:
    """Resolution-independent apparent-size ranging.

    The meaningful quantity is the bbox fraction of the image width, so the
    calibration survives detection-resolution changes:
      range = reference_range * (reference_bbox_px/reference_image_width)
              / (bbox_px/image_width)
    """
    bbox_px = abs(float(bbox_px))
    if bbox_px < float(minimum_bbox_px):
        return None
    if (float(reference_bbox_px) <= 0.0 or float(reference_range_m) <= 0.0
            or float(image_width) <= 0.0 or float(reference_image_width) <= 0.0):
        return None
    reference_ratio = float(reference_bbox_px) / float(reference_image_width)
    observed_ratio = bbox_px / float(image_width)
    if observed_ratio <= 0.0:
        return None
    return float(reference_range_m) * reference_ratio / observed_ratio


def quaternion_to_rotation_matrix(
    quaternion_wxyz: tuple[float, float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    """Rotation body-FRD -> NED from the PX4 VehicleAttitude quaternion.

    契约注释：退化/非有限四元数时本实现返回单位阵继续运行（外场容错）；
    falcon_contact.contact_logic 的同名函数在此情形抛 ValueError。
    故障策略相反，不能互换。
    """
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm < 1e-9 or not math.isfinite(norm):
        return ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return (
        (
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ),
        (
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ),
        (
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ),
    )


def body_minus_z_axis_ned(
    quaternion_wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Physical body -Z (lens) axis in NED; matches the referee convention."""
    rotation = quaternion_to_rotation_matrix(quaternion_wxyz)
    return rotate_vector(rotation, (0.0, 0.0, -1.0))


@dataclass
class ServoCommands:
    """Body-frame velocity commands (FRD)."""

    forward_mps: float = 0.0
    right_mps: float = 0.0
    down_mps: float = 0.0


@dataclass(frozen=True)
class BearingPrediction:
    """Bounded constant-angular-rate prediction in the canonical camera frame."""

    horizontal_rad: float
    vertical_rad: float
    horizon_s: float
    requested_horizon_s: float
    angular_offset_rad: float
    limited: bool


def unit_fru_from_bearing_angles(
    horizontal_rad: float, vertical_rad: float
) -> tuple[float, float, float]:
    """Canonical (right, forward, up) unit LOS from the two bearing angles."""

    right = math.tan(float(horizontal_rad))
    forward = math.tan(float(vertical_rad))
    norm = math.sqrt(right * right + forward * forward + 1.0)
    return right / norm, forward / norm, 1.0 / norm


def bounded_prediction_horizon(
    measurement_age_s: float,
    response_delay_s: float,
    maximum_horizon_s: float,
) -> tuple[float, float, bool]:
    """Return requested and bounded horizon; negative/invalid ages become zero."""

    age = float(measurement_age_s)
    delay = float(response_delay_s)
    maximum = max(0.0, float(maximum_horizon_s))
    if not math.isfinite(age) or age < 0.0:
        age = 0.0
    if not math.isfinite(delay) or delay < 0.0:
        delay = 0.0
    requested = age + delay
    horizon = min(requested, maximum)
    return requested, horizon, requested > maximum + 1.0e-9


def predict_bearing(
    *,
    horizontal_rad: float,
    vertical_rad: float,
    horizontal_rate_rad_s: float,
    vertical_rate_rad_s: float,
    measurement_age_s: float,
    response_delay_s: float,
    maximum_horizon_s: float,
    maximum_rate_rad_s: float,
    maximum_angular_offset_rad: float,
) -> BearingPrediction:
    """Predict a bearing while independently bounding time, rate, and offset."""

    requested, horizon, horizon_limited = bounded_prediction_horizon(
        measurement_age_s, response_delay_s, maximum_horizon_s
    )
    rate_h = float(horizontal_rate_rad_s)
    rate_v = float(vertical_rate_rad_s)
    if not math.isfinite(rate_h):
        rate_h = 0.0
    if not math.isfinite(rate_v):
        rate_v = 0.0
    rate_limit = max(0.0, float(maximum_rate_rad_s))
    rate_norm = math.hypot(rate_h, rate_v)
    rate_limited = rate_norm > rate_limit and rate_norm > 1.0e-12
    if rate_limited:
        scale = rate_limit / rate_norm
        rate_h *= scale
        rate_v *= scale

    delta_h = rate_h * horizon
    delta_v = rate_v * horizon
    offset_limit = max(0.0, float(maximum_angular_offset_rad))
    offset = math.hypot(delta_h, delta_v)
    offset_limited = offset > offset_limit and offset > 1.0e-12
    if offset_limited:
        scale = offset_limit / offset
        delta_h *= scale
        delta_v *= scale
        offset = offset_limit

    # The physical camera never observes a bearing at +/-90 degrees. Keeping a
    # small mathematical margin also prevents tan() from exploding downstream.
    vertical_limit = math.pi / 2.0 - 0.02
    predicted_vertical = max(
        -vertical_limit,
        min(vertical_limit, float(vertical_rad) + delta_v),
    )
    return BearingPrediction(
        horizontal_rad=wrap_pi(float(horizontal_rad) + delta_h),
        vertical_rad=predicted_vertical,
        horizon_s=horizon,
        requested_horizon_s=requested,
        angular_offset_rad=offset,
        limited=horizon_limited or rate_limited or offset_limited,
    )


def compute_servo_commands(
    *,
    horizontal_rad: float,
    vertical_rad: float,
    range_m: float,
    desired_range_m: float,
    pitch_center_rad: float,
    gain_forward: float,
    gain_right: float,
    gain_range: float,
    max_forward_mps: float,
    max_right_mps: float,
    max_vertical_mps: float,
    height_m: float | None,
    minimum_height_m: float,
) -> ServoCommands:
    """Decoupled body-frame servo; each channel is monotone near the station."""
    forward = float(gain_forward) * (float(vertical_rad) - float(pitch_center_rad))
    right = float(gain_right) * float(horizontal_rad)
    down = -float(gain_range) * (float(range_m) - float(desired_range_m))
    if height_m is not None and float(height_m) < float(minimum_height_m):
        # Below the floor: descending (positive down in NED) is forbidden.
        down = min(down, 0.0)
    return ServoCommands(
        forward_mps=max(-float(max_forward_mps), min(float(max_forward_mps), forward)),
        right_mps=max(-float(max_right_mps), min(float(max_right_mps), right)),
        down_mps=max(-float(max_vertical_mps), min(float(max_vertical_mps), down)),
    )


def apply_lens_tilt(
    unit_fru: tuple[float, float, float], tilt_rad: float
) -> tuple[float, float, float]:
    """Convert the published-frame LOS into the TRUE body frame.

    The bridge publishes angles in a frame that treats the lens axis as
    body-up; the physical lens is tilted ``tilt_rad`` toward the nose.  A
    target on the lens axis (image centre) is therefore 10 deg NOSE-ward of
    body-up in the true frame, and true zenith sits 10 deg tail-ward.
    Rotating about the body right axis by +tilt performs the conversion.

    契约注释：本实现不归一化结果（假定输入为单位向量，旋转保范数）；
    falcon_contact.contact_logic.apply_lens_tilt 会对结果做 normalize3。
    """
    right, forward, up = (float(value) for value in unit_fru)
    t = float(tilt_rad)
    sin_t, cos_t = math.sin(t), math.cos(t)
    return (right, forward * cos_t + up * sin_t, -forward * sin_t + up * cos_t)


def los_body_frd_from_unit(unit_fru: tuple[float, float, float]) -> tuple[float, float, float]:
    """Canonical unit LOS (right, forward, up) -> body FRD."""
    right, forward, up = (float(value) for value in unit_fru)
    return (forward, right, -up)


def los_azimuth_ned(los_ned: tuple[float, float, float]) -> float | None:
    """Yaw that points the nose along a NED LOS; None for a vertical LOS."""
    north, east, _ = (float(value) for value in los_ned)
    if abs(north) < 1e-9 and abs(east) < 1e-9:
        return None
    return wrap_pi(math.atan2(east, north))


@dataclass
class FireGate:
    """Fire after a continuous in-window hold, once, then require a fresh hold."""

    hold_s: float = 0.6
    cooldown_s: float = 3.0
    shot_id: int = 0
    _hold_started_s: float | None = field(default=None, repr=False)
    _last_fire_s: float | None = field(default=None, repr=False)

    def update(self, now_s: float, in_window: bool) -> bool:
        if not in_window:
            self._hold_started_s = None
            return False
        if self._hold_started_s is None:
            self._hold_started_s = float(now_s)
            return False
        if float(now_s) - self._hold_started_s < float(self.hold_s):
            return False
        if (
            self._last_fire_s is not None
            and float(now_s) - self._last_fire_s < float(self.cooldown_s)
        ):
            return False
        self._last_fire_s = float(now_s)
        self.shot_id += 1
        self._hold_started_s = None
        return True


class RotationTracker:
    """Accumulate wrapped yaw change until one full revolution is done."""

    def __init__(self, revolution_rad: float = 2.0 * math.pi) -> None:
        self.revolution_rad = abs(float(revolution_rad))
        self.accumulated_rad = 0.0
        self._previous_rad: float | None = None

    def reset(self) -> None:
        self.accumulated_rad = 0.0
        self._previous_rad = None

    def update(self, yaw_rad: float) -> None:
        yaw = wrap_pi(yaw_rad)
        if self._previous_rad is not None:
            self.accumulated_rad += abs(wrap_pi(yaw - self._previous_rad))
        self._previous_rad = yaw

    @property
    def progress(self) -> float:
        if self.revolution_rad <= 0.0:
            return 1.0
        return min(1.0, self.accumulated_rad / self.revolution_rad)

    @property
    def completed(self) -> bool:
        return self.accumulated_rad >= self.revolution_rad


def main(args=None) -> int:
    import rclpy
    from geometry_msgs.msg import PointStamped
    from nav_msgs.msg import Odometry
    from px4_msgs.msg import (
        OffboardControlMode,
        TrajectorySetpoint,
        VehicleAttitude,
        VehicleLocalPosition,
    )
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
        qos_profile_sensor_data,
    )
    from std_msgs.msg import Bool, String

    from anti_drone_interfaces.msg import SearchObservation
    from virtual_interception_interfaces.msg import VirtualHitResult, VirtualShotEvent

    class SearchServoNode(Node):
        def __init__(self) -> None:
            super().__init__("search_servo_node")
            self.declare_parameter("rate_hz", 30.0)
            self.declare_parameter("desired_range_m", 7.5)
            # Indoor two-point calibration (2026-09-06, search camera 2560x1440):
            # the target's larger bbox side measured 105 px at ~5 m and 55 px at
            # ~10 m; size ratios matched the distance ratio, so apparent-size
            # ranging is valid. Stored as a width fraction so a future detection
            # resolution change does not invalidate the calibration.
            self.declare_parameter("reference_range_m", 10.0)
            self.declare_parameter("reference_bbox_px", 55.0)
            self.declare_parameter("reference_image_width", 2560.0)
            self.declare_parameter("minimum_bbox_px", 8.0)
            self.declare_parameter("range_ema_tau_s", 0.3)
            # The virtual shot fires along the BODY -Z axis (straight up from
            # the airframe), and the search lens is tilted 10 deg toward the
            # nose relative to that axis. Published vertical = 0 at the image
            # centre (lens axis), so the body axis sits at vertical = -10 deg:
            # holding pitch_center = -10 deg keeps the target aligned with the
            # shot direction (interceptor directly under the target).
            self.declare_parameter("pitch_center_rad", -0.1745)
            # Physical lens tilt from the body -Z (shot) axis; used to convert
            # the published-frame LOS into the true body frame.
            self.declare_parameter("lens_tilt_rad", 0.1745)
            self.declare_parameter("gain_forward", 1.2)
            self.declare_parameter("gain_right", 1.2)
            self.declare_parameter("gain_range", 0.8)
            self.declare_parameter("max_forward_mps", 3.0)
            self.declare_parameter("max_right_mps", 2.0)
            self.declare_parameter("max_vertical_mps", 1.5)
            self.declare_parameter("minimum_height_m", 3.0)
            self.declare_parameter("observation_timeout_s", 0.5)
            self.declare_parameter("loss_brake_hold_s", 1.5)
            self.declare_parameter("attitude_timeout_s", 0.5)
            self.declare_parameter("min_confidence", 0.5)
            # The observation header retains the original GStreamer capture PTS.
            # Field data on 2026-09-08 showed capture-to-servo ages of roughly
            # 0.11-0.36 s (with longer tails), so predict by the measured age plus
            # a small response allowance, but never extrapolate without bounds.
            self.declare_parameter("prediction_enabled", True)
            self.declare_parameter("prediction_control_delay_s", 0.08)
            self.declare_parameter("prediction_trigger_delay_s", 0.02)
            self.declare_parameter("prediction_maximum_horizon_s", 0.35)
            self.declare_parameter("prediction_rate_filter_tau_s", 0.20)
            self.declare_parameter("prediction_rate_reset_gap_s", 0.50)
            self.declare_parameter("prediction_maximum_rate_rad_s", 0.70)
            self.declare_parameter("prediction_maximum_offset_rad", 0.12)
            self.declare_parameter("search_observation_topic", "/perception/search/observation")
            self.declare_parameter("search_detections_topic", "/search/detections_json")
            self.declare_parameter("px4_attitude_topic", "/fmu/out/vehicle_attitude")
            self.declare_parameter("px4_local_position_topic", "/fmu/out/vehicle_local_position")
            self.declare_parameter("upstream_prefix", "/upstream/fmu/in")
            self.declare_parameter("status_topic", "/tracking/servo/status")
            self.declare_parameter("status_rate_hz", 10.0)
            self.declare_parameter("truth_topic", "/evaluation/relative_truth")
            self.declare_parameter("task_origin_topic", "/tracking/task_origin_ned")
            self.declare_parameter("shot_topic", "/tracking/virtual_shot_event")
            self.declare_parameter("fire_enable", True)
            self.declare_parameter("fire_range_min_m", 5.0)
            self.declare_parameter("fire_range_max_m", 10.0)
            self.declare_parameter("fire_max_align_error_rad", 0.07)   # ~4 deg
            self.declare_parameter("fire_max_los_rate_rad_s", 0.087)   # ~5 deg/s
            self.declare_parameter("fire_hold_s", 0.6)
            self.declare_parameter("fire_cooldown_s", 3.0)
            self.declare_parameter("projectile_speed_mps", 400.0)
            # Completion action: after M referee-confirmed hits OR N shots
            # (whichever comes first), rotate one full revolution in place,
            # then hover.
            self.declare_parameter("enable_completion_action", True)
            self.declare_parameter("shots_for_completion", 3)
            self.declare_parameter("hits_for_completion", 1)
            self.declare_parameter("rotate_yaw_rate_rad_s", 1.5)
            self.declare_parameter("hit_result_topic", "/evaluation/virtual_hit_result")
            self.declare_parameter("mission_complete_topic", "/tracking/mission/complete")

            def param(name: str) -> float:
                return float(self.get_parameter(name).value)

            self._rate_hz = max(5.0, param("rate_hz"))
            self._desired_range_m = param("desired_range_m")
            self._reference_range_m = param("reference_range_m")
            self._reference_bbox_px = param("reference_bbox_px")
            self._reference_image_width = param("reference_image_width")
            self._minimum_bbox_px = param("minimum_bbox_px")
            self._range_ema_tau_s = param("range_ema_tau_s")
            self._pitch_center_rad = param("pitch_center_rad")
            self._lens_tilt_rad = param("lens_tilt_rad")
            self._gain_forward = param("gain_forward")
            self._gain_right = param("gain_right")
            self._gain_range = param("gain_range")
            self._max_forward_mps = param("max_forward_mps")
            self._max_right_mps = param("max_right_mps")
            self._max_vertical_mps = param("max_vertical_mps")
            self._minimum_height_m = param("minimum_height_m")
            self._observation_timeout_s = param("observation_timeout_s")
            self._loss_brake_hold_s = param("loss_brake_hold_s")
            self._attitude_timeout_s = param("attitude_timeout_s")
            self._min_confidence = param("min_confidence")
            self._prediction_enabled = bool(
                self.get_parameter("prediction_enabled").value
            )
            self._prediction_control_delay_s = max(
                0.0, param("prediction_control_delay_s")
            )
            self._prediction_trigger_delay_s = max(
                0.0, param("prediction_trigger_delay_s")
            )
            self._prediction_maximum_horizon_s = max(
                0.0, param("prediction_maximum_horizon_s")
            )
            self._prediction_rate_filter_tau_s = max(
                0.0, param("prediction_rate_filter_tau_s")
            )
            self._prediction_rate_reset_gap_s = max(
                0.01, param("prediction_rate_reset_gap_s")
            )
            self._prediction_maximum_rate_rad_s = max(
                0.0, param("prediction_maximum_rate_rad_s")
            )
            self._prediction_maximum_offset_rad = max(
                0.0, param("prediction_maximum_offset_rad")
            )
            self._fire_enable = bool(self.get_parameter("fire_enable").value)
            self._fire_range_min_m = param("fire_range_min_m")
            self._fire_range_max_m = param("fire_range_max_m")
            self._fire_max_align_error_rad = param("fire_max_align_error_rad")
            self._fire_max_los_rate_rad_s = param("fire_max_los_rate_rad_s")
            self._fire_hold_s = param("fire_hold_s")
            self._fire_cooldown_s = param("fire_cooldown_s")
            self._projectile_speed_mps = param("projectile_speed_mps")
            self._enable_completion_action = bool(
                self.get_parameter("enable_completion_action").value
            )
            self._shots_for_completion = int(
                self.get_parameter("shots_for_completion").value
            )
            self._hits_for_completion = int(
                self.get_parameter("hits_for_completion").value
            )
            self._rotate_yaw_rate_rad_s = param("rotate_yaw_rate_rad_s")
            self._upstream = str(self.get_parameter("upstream_prefix").value).rstrip("/")

            qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                SearchObservation,
                str(self.get_parameter("search_observation_topic").value),
                self._on_search_observation,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                String,
                str(self.get_parameter("search_detections_topic").value),
                self._on_detections,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                VehicleAttitude,
                str(self.get_parameter("px4_attitude_topic").value),
                self._on_attitude,
                qos,
            )
            self.create_subscription(
                VehicleLocalPosition,
                str(self.get_parameter("px4_local_position_topic").value),
                self._on_local_position,
                qos,
            )
            self._mode_pub = self.create_publisher(
                OffboardControlMode, f"{self._upstream}/offboard_control_mode", qos
            )
            self._setpoint_pub = self.create_publisher(
                TrajectorySetpoint, f"{self._upstream}/trajectory_setpoint", qos
            )
            self._status_pub = self.create_publisher(
                String, str(self.get_parameter("status_topic").value), 10
            )
            self._truth_pub = self.create_publisher(
                Odometry, str(self.get_parameter("truth_topic").value), 20
            )
            origin_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self._origin_pub = self.create_publisher(
                PointStamped,
                str(self.get_parameter("task_origin_topic").value),
                origin_qos,
            )
            self._shot_pub = self.create_publisher(
                VirtualShotEvent, str(self.get_parameter("shot_topic").value), 10
            )
            self._hit_result_sub = self.create_subscription(
                VirtualHitResult,
                str(self.get_parameter("hit_result_topic").value),
                self._on_hit_result,
                10,
            )
            self._mission_complete_pub = self.create_publisher(
                Bool,
                str(self.get_parameter("mission_complete_topic").value),
                10,
            )

            self._fire_gate = FireGate(
                hold_s=self._fire_hold_s, cooldown_s=self._fire_cooldown_s
            )
            self._phase = "SERVO"           # SERVO -> ROTATE -> HOVER
            self._hits = 0
            self._rotator = RotationTracker()
            self._rotate_started_s: float | None = None
            self._mission_complete_sent = False
            self._obs_time_s: float | None = None
            self._obs_capture_stamp_s: float | None = None
            self._obs_horizontal = 0.0
            self._obs_vertical = 0.0
            self._obs_los_fru = (0.0, 0.0, 1.0)
            self._det_time_s: float | None = None
            self._det_bbox = (0.0, 0.0, 0.0, 0.0)
            self._det_image_width = self._reference_image_width
            self._att_time_s: float | None = None
            self._attitude_q = (1.0, 0.0, 0.0, 0.0)
            self._local_time_s: float | None = None
            self._local_position = (0.0, 0.0, 0.0)
            self._local_velocity = (0.0, 0.0, 0.0)
            self._range_ema: float | None = None
            self._last_rel: tuple[float, float, float] | None = None
            self._last_truth_s: float | None = None
            self._target_velocity_rel = (0.0, 0.0, 0.0)
            self._last_los_rate = 0.0
            self._horizontal_rate = 0.0
            self._vertical_rate = 0.0
            self._prev_pitch: float | None = None
            self._prev_azimuth: float | None = None
            self._prev_obs_stamp: float | None = None
            self._last_prediction_measurement_age_s = 0.0
            self._last_control_prediction = BearingPrediction(
                0.0, 0.0, 0.0, 0.0, 0.0, False
            )
            self._last_shot_prediction = BearingPrediction(
                0.0, 0.0, 0.0, 0.0, 0.0, False
            )
            self._origin_sent = False
            self._braking = False

            self.create_timer(1.0 / self._rate_hz, self._on_timer)
            self.create_timer(
                1.0 / max(1.0, param("status_rate_hz")), self._on_status_timer
            )
            self.get_logger().warning(
                "search servo ready: desired_range=%.1fm calib=(%.1fm @ %.0fpx) "
                "max_speed=%.1f/%.1f/%.1f m/s min_height=%.1fm fire=%s "
                "prediction=%s max_horizon=%.2fs"
                % (
                    self._desired_range_m,
                    self._reference_range_m,
                    self._reference_bbox_px,
                    self._max_forward_mps,
                    self._max_right_mps,
                    self._max_vertical_mps,
                    self._minimum_height_m,
                    str(self._fire_enable).lower(),
                    str(self._prediction_enabled).lower(),
                    self._prediction_maximum_horizon_s,
                )
            )

        # --- input callbacks -------------------------------------------------
        def _on_search_observation(self, message: SearchObservation) -> None:
            if not message.visible or not message.angles_valid:
                return
            if float(message.confidence) < self._min_confidence:
                return
            received_s = self.get_clock().now().nanoseconds / 1e9
            self._obs_time_s = received_s
            self._obs_horizontal = float(message.horizontal_angle_rad)
            self._obs_vertical = float(message.vertical_angle_rad)
            unit = message.unit_los_optical
            los = (float(unit.x), float(unit.y), float(unit.z))
            norm = math.sqrt(sum(value * value for value in los))
            if norm > 1e-6 and all(math.isfinite(value) for value in los):
                self._obs_los_fru = (los[0] / norm, los[1] / norm, los[2] / norm)
            stamp = float(message.header.stamp.sec) + float(
                message.header.stamp.nanosec
            ) / 1e9
            self._obs_capture_stamp_s = (
                stamp if math.isfinite(stamp) and stamp > 0.0 else None
            )
            if (
                self._prev_obs_stamp is not None
                and stamp > self._prev_obs_stamp
                and self._prev_pitch is not None
                and self._prev_azimuth is not None
            ):
                dt = stamp - self._prev_obs_stamp
                if dt <= self._prediction_rate_reset_gap_s:
                    d_pitch = wrap_pi(self._obs_vertical - self._prev_pitch)
                    d_azimuth = wrap_pi(
                        self._obs_horizontal - self._prev_azimuth
                    )
                    measured_vertical_rate = d_pitch / max(dt, 1e-3)
                    measured_horizontal_rate = d_azimuth / max(dt, 1e-3)
                    self._vertical_rate = low_pass(
                        self._vertical_rate,
                        measured_vertical_rate,
                        self._prediction_rate_filter_tau_s,
                        dt,
                    )
                    self._horizontal_rate = low_pass(
                        self._horizontal_rate,
                        measured_horizontal_rate,
                        self._prediction_rate_filter_tau_s,
                        dt,
                    )
                    rate = math.hypot(
                        measured_vertical_rate, measured_horizontal_rate
                    )
                    self._last_los_rate = low_pass(
                        self._last_los_rate,
                        rate,
                        self._prediction_rate_filter_tau_s,
                        dt,
                    )
                else:
                    # Do not carry a pre-loss angular velocity into a newly
                    # reacquired target.
                    self._vertical_rate = 0.0
                    self._horizontal_rate = 0.0
                    self._last_los_rate = 0.0
            if self._prev_obs_stamp is None or stamp > self._prev_obs_stamp:
                self._prev_obs_stamp = stamp
                self._prev_pitch = self._obs_vertical
                self._prev_azimuth = self._obs_horizontal

        def _on_detections(self, message: String) -> None:
            try:
                payload = json.loads(str(message.data))
            except (TypeError, ValueError):
                return
            best = None
            for item in payload.get("detections", []):
                try:
                    confidence = float(item.get("confidence", 0.0))
                except (TypeError, ValueError):
                    continue
                if confidence < self._min_confidence:
                    continue
                if best is None or confidence > float(best["confidence"]):
                    best = item
            if best is None:
                return
            try:
                x1, y1, x2, y2 = (float(value) for value in best["bbox_xyxy"][:4])
            except (KeyError, TypeError, ValueError):
                return
            self._det_bbox = (x1, y1, x2, y2)
            # Trust the JSON's own image size when present; fall back to the
            # reference width so the ranging ratio stays resolution-safe.
            image_width = payload.get("image_width")
            try:
                self._det_image_width = float(image_width)
            except (TypeError, ValueError):
                self._det_image_width = self._reference_image_width
            if self._det_image_width <= 0.0:
                self._det_image_width = self._reference_image_width
            self._det_time_s = self.get_clock().now().nanoseconds / 1e9

        def _on_attitude(self, message: VehicleAttitude) -> None:
            q = tuple(float(value) for value in message.q[:4])
            if not all(math.isfinite(value) for value in q):
                return
            self._attitude_q = q
            self._att_time_s = self.get_clock().now().nanoseconds / 1e9

        def _on_local_position(self, message: VehicleLocalPosition) -> None:
            if not bool(message.xy_valid) or not bool(message.z_valid):
                return
            self._local_position = (
                float(message.x),
                float(message.y),
                float(message.z),
            )
            self._local_velocity = (
                float(message.vx),
                float(message.vy),
                float(message.vz),
            )
            self._local_time_s = self.get_clock().now().nanoseconds / 1e9

        # --- completion action: rotate one revolution, then hover ------------
        def _enter_rotation(self, reason: str) -> None:
            if self._phase != "SERVO":
                return
            self._phase = "ROTATE"
            self._rotator.reset()
            self._rotate_started_s = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().warning(
                "MISSION COMPLETE condition met (%s): rotating one revolution"
                % reason
            )

        def _handle_completion_phase(self, now_s: float) -> None:
            # Keep the referee's truth feed alive so the shot that triggered
            # completion still receives post-flight target samples and gets a
            # real HIT/MISS verdict instead of a watchdog invalidation.
            obs_age = self._observation_age()
            if obs_age is not None and obs_age <= self._observation_timeout_s:
                range_m = self._current_range_m()
                if range_m is not None:
                    rotation = quaternion_to_rotation_matrix(self._attitude_q)
                    los_ned = rotate_vector(
                        rotation, los_body_frd_from_unit(apply_lens_tilt(self._obs_los_fru, self._lens_tilt_rad))
                    )
                    self._publish_truth(now_s, los_ned, range_m)
            rotation = quaternion_to_rotation_matrix(self._attitude_q)
            if self._phase == "ROTATE":
                yaw = math.atan2(rotation[1][0], rotation[0][0])
                self._rotator.update(yaw)
                # Time-based fallback in case attitude stops updating.
                timed_out = (
                    self._rotate_started_s is not None
                    and now_s - self._rotate_started_s
                    > (2.0 * math.pi / max(self._rotate_yaw_rate_rad_s, 0.1)) * 1.5 + 2.0
                )
                if self._rotator.completed or timed_out:
                    self._phase = "HOVER"
                    self.get_logger().warning(
                        "rotation done (%d%% of a circle tracked): hovering"
                        % round(self._rotator.progress * 100)
                    )
                else:
                    self._publish_mode()
                    setpoint = TrajectorySetpoint()
                    setpoint.timestamp = self._px4_timestamp_us()
                    setpoint.velocity = [0.0, 0.0, 0.0]
                    setpoint.position = [float("nan")] * 3
                    setpoint.acceleration = [float("nan")] * 3
                    setpoint.jerk = [float("nan")] * 3
                    setpoint.yaw = float("nan")
                    setpoint.yawspeed = float(self._rotate_yaw_rate_rad_s)
                    self._setpoint_pub.publish(setpoint)
                    return
            if not self._mission_complete_sent:
                complete = Bool()
                complete.data = True
                self._mission_complete_pub.publish(complete)
                self._mission_complete_sent = True
                self.get_logger().warning("mission complete: hovering in place")
            self._publish_zero_setpoint(None)

        def _on_hit_result(self, message: VirtualHitResult) -> None:
            if not message.hit:
                return
            self._hits += 1
            self.get_logger().warning(
                "referee HIT confirmed: %d hit(s) from %d shot(s)"
                % (self._hits, int(self._fire_gate.shot_id))
            )
            if (
                self._phase == "SERVO"
                and self._enable_completion_action
                and self._hits >= self._hits_for_completion
            ):
                self._enter_rotation("%d referee hits" % self._hits)

        # --- helpers ---------------------------------------------------------
        def _observation_age(self) -> float | None:
            if self._obs_time_s is None:
                return None
            return self.get_clock().now().nanoseconds / 1e9 - self._obs_time_s

        def _prediction_measurement_age(
            self, now_s: float, receive_age_s: float
        ) -> float:
            """Capture-to-control age, falling back to local receive age."""

            if self._obs_capture_stamp_s is not None:
                capture_age = now_s - self._obs_capture_stamp_s
                # A grossly different clock domain must not create a huge lead.
                # Valid field PTS ages have been below one second.
                if math.isfinite(capture_age) and -0.02 <= capture_age <= 2.0:
                    return max(0.0, capture_age)
            return max(0.0, float(receive_age_s))

        def _predict_bearing(
            self, measurement_age_s: float, response_delay_s: float
        ) -> BearingPrediction:
            if not self._prediction_enabled:
                return BearingPrediction(
                    self._obs_horizontal,
                    self._obs_vertical,
                    0.0,
                    0.0,
                    0.0,
                    False,
                )
            return predict_bearing(
                horizontal_rad=self._obs_horizontal,
                vertical_rad=self._obs_vertical,
                horizontal_rate_rad_s=self._horizontal_rate,
                vertical_rate_rad_s=self._vertical_rate,
                measurement_age_s=measurement_age_s,
                response_delay_s=response_delay_s,
                maximum_horizon_s=self._prediction_maximum_horizon_s,
                maximum_rate_rad_s=self._prediction_maximum_rate_rad_s,
                maximum_angular_offset_rad=self._prediction_maximum_offset_rad,
            )

        def _prediction_los_ned(
            self,
            prediction: BearingPrediction,
            rotation: tuple[tuple[float, float, float], ...],
        ) -> tuple[float, float, float]:
            canonical_los = unit_fru_from_bearing_angles(
                prediction.horizontal_rad, prediction.vertical_rad
            )
            body_los = los_body_frd_from_unit(
                apply_lens_tilt(canonical_los, self._lens_tilt_rad)
            )
            return rotate_vector(rotation, body_los)

        def _current_range_m(self) -> float | None:
            """EMA range from the newest bbox; hold the last value otherwise."""
            now_s = self.get_clock().now().nanoseconds / 1e9
            if (
                self._det_time_s is None
                or now_s - self._det_time_s > self._observation_timeout_s
            ):
                return self._range_ema
            x1, y1, x2, y2 = self._det_bbox
            bbox_px = max(abs(x2 - x1), abs(y2 - y1))
            raw = range_from_bbox_ratio(
                bbox_px,
                self._det_image_width,
                self._reference_range_m,
                self._reference_bbox_px,
                self._reference_image_width,
                self._minimum_bbox_px,
            )
            if raw is None:
                return self._range_ema
            self._range_ema = low_pass(
                self._range_ema, raw, self._range_ema_tau_s, 1.0 / self._rate_hz
            )
            return self._range_ema

        def _publish_zero_setpoint(self, yaw_rad: float | None) -> None:
            self._publish_mode()
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = self._px4_timestamp_us()
            setpoint.velocity = [0.0, 0.0, 0.0]
            setpoint.position = [float("nan")] * 3
            setpoint.acceleration = [float("nan")] * 3
            setpoint.jerk = [float("nan")] * 3
            setpoint.yaw = float("nan") if yaw_rad is None else float(yaw_rad)
            setpoint.yawspeed = float("nan")
            self._setpoint_pub.publish(setpoint)

        def _publish_mode(self) -> None:
            mode = OffboardControlMode()
            mode.timestamp = self._px4_timestamp_us()
            mode.velocity = True
            self._mode_pub.publish(mode)

        def _px4_timestamp_us(self) -> int:
            return self.get_clock().now().nanoseconds // 1000

        # --- main control tick ------------------------------------------------
        def _on_timer(self) -> None:
            now_s = self.get_clock().now().nanoseconds / 1e9
            if not self._origin_sent:
                origin = PointStamped()
                origin.header.stamp = self.get_clock().now().to_msg()
                origin.header.frame_id = "map_ned"
                self._origin_pub.publish(origin)
                self._origin_sent = True

            if self._phase != "SERVO":
                self._handle_completion_phase(now_s)
                return

            attitude_age = None if self._att_time_s is None else now_s - self._att_time_s
            if attitude_age is None or attitude_age > self._attitude_timeout_s:
                self._publish_zero_setpoint(None)
                return

            obs_age = self._observation_age()
            if obs_age is None:
                self._publish_zero_setpoint(None)
                return
            if obs_age > self._observation_timeout_s:
                self._fire_gate.update(now_s, False)
                measurement_fresh = False
                if obs_age > self._loss_brake_hold_s or self._range_ema is None:
                    if not self._braking:
                        self.get_logger().warning("target lost: braking to hover")
                        self._braking = True
                    self._publish_zero_setpoint(None)
                    return
                # Within the loss hold: coast on the last cached geometry.
                range_m = self._range_ema
            else:
                measurement_fresh = True
                if self._braking:
                    self._braking = False
                range_m = self._current_range_m()
                if range_m is None:
                    # Bearing-only: hold position, keep centring with yaw.
                    self._publish_zero_setpoint(None)
                    return

            rotation = quaternion_to_rotation_matrix(self._attitude_q)
            measured_los_body_frd = los_body_frd_from_unit(
                apply_lens_tilt(self._obs_los_fru, self._lens_tilt_rad)
            )
            measured_los_ned = rotate_vector(rotation, measured_los_body_frd)
            measurement_age_s = self._prediction_measurement_age(now_s, obs_age)
            control_prediction = self._predict_bearing(
                measurement_age_s, self._prediction_control_delay_s
            )
            projectile_tof_s = float(range_m) / max(
                self._projectile_speed_mps, 1.0
            )
            shot_prediction = self._predict_bearing(
                measurement_age_s,
                self._prediction_trigger_delay_s + projectile_tof_s,
            )
            control_los_ned = self._prediction_los_ned(
                control_prediction, rotation
            )
            shot_los_ned = self._prediction_los_ned(shot_prediction, rotation)
            self._last_prediction_measurement_age_s = measurement_age_s
            self._last_control_prediction = control_prediction
            self._last_shot_prediction = shot_prediction
            yaw_sp = los_azimuth_ned(control_los_ned)

            commands = compute_servo_commands(
                horizontal_rad=control_prediction.horizontal_rad,
                vertical_rad=control_prediction.vertical_rad,
                range_m=range_m,
                desired_range_m=self._desired_range_m,
                pitch_center_rad=self._pitch_center_rad,
                gain_forward=self._gain_forward,
                gain_right=self._gain_right,
                gain_range=self._gain_range,
                max_forward_mps=self._max_forward_mps,
                max_right_mps=self._max_right_mps,
                max_vertical_mps=self._max_vertical_mps,
                height_m=-float(self._local_position[2]),
                minimum_height_m=self._minimum_height_m,
            )
            velocity_ned = rotate_vector(
                rotation,
                (commands.forward_mps, commands.right_mps, commands.down_mps),
            )
            self._publish_mode()
            setpoint = TrajectorySetpoint()
            setpoint.timestamp = self._px4_timestamp_us()
            setpoint.velocity = [float(value) for value in velocity_ned]
            setpoint.position = [float("nan")] * 3
            setpoint.acceleration = [float("nan")] * 3
            setpoint.jerk = [float("nan")] * 3
            setpoint.yaw = float("nan") if yaw_sp is None else float(yaw_sp)
            setpoint.yawspeed = float("nan")
            self._setpoint_pub.publish(setpoint)

            self._publish_truth_and_fire(
                now_s,
                measured_los_ned,
                shot_los_ned,
                range_m,
                measurement_fresh,
                shot_prediction,
            )

        def _target_position_ned(
            self, los_ned: tuple[float, float, float], range_m: float
        ) -> tuple[float, float, float]:
            norm = math.sqrt(sum(value * value for value in los_ned))
            if norm < 1e-6:
                return self._local_position
            unit = tuple(value / norm for value in los_ned)
            return tuple(
                self._local_position[axis] + unit[axis] * range_m for axis in range(3)
            )  # type: ignore[return-value]

        def _publish_truth(
            self,
            now_s: float,
            los_ned: tuple[float, float, float],
            range_m: float,
        ) -> None:
            # Referee contract (matches the simulator's truth bridge): the
            # truth pose is the target position RELATIVE to the pursuer in
            # NED, and the twist is the relative (target - pursuer) velocity.
            norm = math.sqrt(sum(value * value for value in los_ned))
            if norm < 1e-6:
                return
            unit = tuple(value / norm for value in los_ned)
            rel = tuple(unit[axis] * range_m for axis in range(3))
            if self._last_rel is not None and self._last_truth_s is not None:
                dt = now_s - self._last_truth_s
                if 0.0 < dt < 0.5:
                    measured = tuple(
                        (rel[axis] - self._last_rel[axis]) / dt
                        for axis in range(3)
                    )
                    self._target_velocity_rel = tuple(
                        low_pass(self._target_velocity_rel[axis], measured[axis], 0.5, dt)
                        for axis in range(3)
                    )  # type: ignore[assignment]
            self._last_rel = rel
            self._last_truth_s = now_s

            truth = Odometry()
            truth.header.stamp = self.get_clock().now().to_msg()
            truth.header.frame_id = "map_ned"
            truth.child_frame_id = "target_relative_to_pursuer_truth"
            truth.pose.pose.position.x = float(rel[0])
            truth.pose.pose.position.y = float(rel[1])
            truth.pose.pose.position.z = float(rel[2])
            truth.twist.twist.linear.x = float(self._target_velocity_rel[0])
            truth.twist.twist.linear.y = float(self._target_velocity_rel[1])
            truth.twist.twist.linear.z = float(self._target_velocity_rel[2])
            self._truth_pub.publish(truth)

        def _publish_truth_and_fire(
            self,
            now_s: float,
            measured_los_ned: tuple[float, float, float],
            shot_los_ned: tuple[float, float, float],
            range_m: float,
            measurement_fresh: bool,
            shot_prediction: BearingPrediction,
        ) -> None:
            # Keep the referee feed on the measured LOS. Prediction is a control
            # output and must not be recycled as synthetic measurement truth.
            self._publish_truth(now_s, measured_los_ned, range_m)

            if not self._fire_enable:
                self._fire_gate.update(now_s, False)
                return
            norm = math.sqrt(sum(value * value for value in shot_los_ned))
            shot_unit = tuple(
                value / max(norm, 1.0e-9) for value in shot_los_ned
            )
            attack_axis = body_minus_z_axis_ned(self._attitude_q)
            axis_error = math.acos(
                max(
                    -1.0,
                    min(
                        1.0,
                        sum(
                            attack_axis[index] * shot_unit[index]
                            for index in range(3)
                        ),
                    ),
                )
            )
            in_window = (
                measurement_fresh
                and self._fire_range_min_m <= range_m <= self._fire_range_max_m
                and axis_error <= self._fire_max_align_error_rad
                and self._last_los_rate <= self._fire_max_los_rate_rad_s
            )
            if self._fire_gate.update(now_s, in_window):
                self._publish_shot(
                    measured_los_ned,
                    shot_unit,
                    range_m,
                    axis_error,
                    shot_prediction,
                )
                if (
                    self._enable_completion_action
                    and self._fire_gate.shot_id >= self._shots_for_completion
                ):
                    self._enter_rotation(
                        "%d shots fired" % self._fire_gate.shot_id
                    )

        def _publish_shot(
            self,
            measured_los_ned: tuple[float, float, float],
            shot_los_ned: tuple[float, float, float],
            range_m: float,
            axis_error_rad: float,
            shot_prediction: BearingPrediction,
        ) -> None:
            norm = math.sqrt(sum(value * value for value in shot_los_ned))
            unit = tuple(value / max(norm, 1.0e-9) for value in shot_los_ned)
            target_ned = self._target_position_ned(measured_los_ned, range_m)
            predicted_target_ned = self._target_position_ned(unit, range_m)
            tof = float(range_m) / max(self._projectile_speed_mps, 1.0)
            event = VirtualShotEvent()
            event.interface_version = VirtualShotEvent.INTERFACE_VERSION_V1
            event.header.stamp = self.get_clock().now().to_msg()
            event.header.frame_id = "pursuer_local_ned"
            event.shot_id = int(self._fire_gate.shot_id)
            event.projectile_speed_mps = float(self._projectile_speed_mps)
            event.predicted_time_of_flight_s = tof
            event.shot_direction_ned.x = float(unit[0])
            event.shot_direction_ned.y = float(unit[1])
            event.shot_direction_ned.z = float(unit[2])
            event.pursuer_position_ned.x = float(self._local_position[0])
            event.pursuer_position_ned.y = float(self._local_position[1])
            event.pursuer_position_ned.z = float(self._local_position[2])
            event.pursuer_velocity_ned.x = float(self._local_velocity[0])
            event.pursuer_velocity_ned.y = float(self._local_velocity[1])
            event.pursuer_velocity_ned.z = float(self._local_velocity[2])
            event.estimated_target_position_ned.x = float(target_ned[0])
            event.estimated_target_position_ned.y = float(target_ned[1])
            event.estimated_target_position_ned.z = float(target_ned[2])
            absolute_target_velocity = tuple(
                self._target_velocity_rel[axis] + self._local_velocity[axis]
                for axis in range(3)
            )
            event.estimated_target_velocity_ned.x = float(absolute_target_velocity[0])
            event.estimated_target_velocity_ned.y = float(absolute_target_velocity[1])
            event.estimated_target_velocity_ned.z = float(absolute_target_velocity[2])
            event.predicted_intercept_point_ned.x = float(predicted_target_ned[0])
            event.predicted_intercept_point_ned.y = float(predicted_target_ned[1])
            event.predicted_intercept_point_ned.z = float(predicted_target_ned[2])
            event.estimated_range_m = float(range_m)
            event.actual_attack_axis_error_rad = float(axis_error_rad)
            event.predicted_lateral_sigma_m = float(range_m * axis_error_rad)
            event.maximum_position_variance_m2 = float((0.15 * range_m) ** 2)
            event.solution_valid = True
            self._shot_pub.publish(event)
            self.get_logger().warning(
                "VIRTUAL SHOT #%d range=%.1fm axis_error=%.1fdeg "
                "los_rate=%.1fdeg/s prediction=%.3fs%s"
                % (
                    event.shot_id,
                    range_m,
                    math.degrees(axis_error_rad),
                    math.degrees(self._last_los_rate),
                    shot_prediction.horizon_s,
                    " [LIMITED]" if shot_prediction.limited else "",
                )
            )

        # --- status -----------------------------------------------------------
        def _on_status_timer(self) -> None:
            obs_age = self._observation_age()
            payload = {
                "mode": "search_servo",
                "phase": self._phase,
                "stamp_s": self.get_clock().now().nanoseconds / 1e9,
                "has_target": obs_age is not None
                and obs_age <= self._observation_timeout_s,
                "braking": self._braking,
                "observation_age_s": obs_age,
                "range_ema_m": self._range_ema,
                "horizontal_deg": math.degrees(self._obs_horizontal),
                "vertical_deg": math.degrees(self._obs_vertical),
                "pitch_center_deg": math.degrees(self._pitch_center_rad),
                "los_rate_deg_s": math.degrees(self._last_los_rate),
                "prediction": {
                    "enabled": self._prediction_enabled,
                    "measurement_age_s": self._last_prediction_measurement_age_s,
                    "horizontal_rate_deg_s": math.degrees(self._horizontal_rate),
                    "vertical_rate_deg_s": math.degrees(self._vertical_rate),
                    "control_horizon_s": self._last_control_prediction.horizon_s,
                    "control_requested_horizon_s": (
                        self._last_control_prediction.requested_horizon_s
                    ),
                    "control_offset_deg": math.degrees(
                        self._last_control_prediction.angular_offset_rad
                    ),
                    "control_limited": self._last_control_prediction.limited,
                    "predicted_horizontal_deg": math.degrees(
                        self._last_control_prediction.horizontal_rad
                    ),
                    "predicted_vertical_deg": math.degrees(
                        self._last_control_prediction.vertical_rad
                    ),
                    "shot_horizon_s": self._last_shot_prediction.horizon_s,
                    "shot_requested_horizon_s": (
                        self._last_shot_prediction.requested_horizon_s
                    ),
                    "shot_limited": self._last_shot_prediction.limited,
                    "maximum_horizon_s": self._prediction_maximum_horizon_s,
                },
                "height_m": -float(self._local_position[2]),
                "desired_range_m": self._desired_range_m,
                "calibration": {
                    "reference_range_m": self._reference_range_m,
                    "reference_bbox_px": self._reference_bbox_px,
                    "reference_image_width": self._reference_image_width,
                    "detection_image_width": self._det_image_width,
                },
                "fire": {
                    "enabled": self._fire_enable,
                    "shots": int(self._fire_gate.shot_id),
                    "hits": int(self._hits),
                    "hold_s": self._fire_hold_s,
                    "cooldown_s": self._fire_cooldown_s,
                },
                "rotation_progress_deg": math.degrees(self._rotator.accumulated_rad),
            }
            message = String()
            message.data = json.dumps(payload, separators=(",", ":"), allow_nan=False)
            self._status_pub.publish(message)

    rclpy.init(args=args)
    node = None
    try:
        node = SearchServoNode()
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
