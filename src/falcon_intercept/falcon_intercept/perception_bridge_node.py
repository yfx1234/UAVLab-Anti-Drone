"""Bridge the production Falcon observations into anti-drone V1 interfaces."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class PinholeIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image dimensions must be positive")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")


def pixel_to_optical_los(
    pixel_u: float,
    pixel_v: float,
    intrinsics: PinholeIntrinsics,
) -> tuple[tuple[float, float, float], float, float]:
    """Return REP-103 optical LOS and horizontal/vertical bearing angles."""

    intrinsics.validate()
    u = float(pixel_u)
    v = float(pixel_v)
    if not math.isfinite(u) or not math.isfinite(v):
        raise ValueError("pixel coordinates must be finite")
    x = (u - intrinsics.cx) / intrinsics.fx
    y = (v - intrinsics.cy) / intrinsics.fy
    length = math.sqrt(x * x + y * y + 1.0)
    return (x / length, y / length, 1.0 / length), math.atan(x), math.atan(y)


# --- Camera mount compensation (2026-08-13) ---------------------------------
#
# The upstream controller hardcodes the camera mounting (see
# px4_offboard_tracking pure_interceptor_controller.cpp search_los_ned /
# depth_los_ned and the colleague's coordinate spec 相机参数.md):
#
#   canonical optical +X = body +Y (right)
#   canonical optical +Y = body +X (forward/nose; the upstream search-camera
#                          extrinsic already tilts its optical axis 10 deg
#                          toward the nose)
#   canonical optical +Z = body -Z (lens points up)
#
# The 2026-08-13 field ground test (preview annotated with the physical nose
# direction) showed the actual rig is rolled -90 deg about the lens axis:
# image right = tail, image down = body right.  All published observations
# must therefore be roll-corrected into the canonical frame here.  The physical
# search camera already has the same 10 deg nose-up boresight encoded by the
# upstream controller, so this bridge must not apply that pitch a second time.
# search_canonical_tilt_deg is an additional residual correction and is zero
# for the production mount.  If the rig is physically re-mounted to match the
# canonical image axes, set the roll parameters back to 0.


def search_optical_to_canonical(
    direction: tuple[float, float, float],
    roll_deg: float = -90.0,
    tilt_deg: float = 0.0,
) -> tuple[float, float, float]:
    """Map a physical search-camera optical direction into the canonical frame.

    direction is REP-103 optical (x right, y down, z along lens) of the
    physically mounted camera.  roll_deg is the angle of the image-right axis
    from body-right toward body-forward (the current rig: -90).  tilt_deg is
    only an additional residual pitch correction.  It must remain zero when
    the physical camera's 10 degree noseward boresight matches the upstream
    R_body_from_search_optical contract.
    """

    roll = math.radians(float(roll_deg))
    tilt = math.radians(float(tilt_deg))
    sr, cr = math.sin(roll), math.cos(roll)
    st, ct = math.sin(tilt), math.cos(tilt)
    ox, oy, oz = (float(value) for value in direction)
    body_right = cr * ox - sr * oy
    body_forward = sr * ox + cr * oy
    body_up = oz
    return (
        body_right,
        ct * body_forward - st * body_up,
        st * body_forward + ct * body_up,
    )


def depth_optical_to_canonical(
    position_optical: tuple[float, float, float],
    roll_deg: float = -90.0,
) -> tuple[float, float, float]:
    """Map the solver's REP-103 optical position into the canonical frame.

    AsyncStereoMeasurement.position_camera is REP-103 optical of the physical
    left (front) camera: +X image-right, +Y image-down, +Z along the lens.
    The depth chain's canonical frame has no nose tilt, so this is the roll
    part of search_optical_to_canonical with tilt_deg = 0.
    """

    return search_optical_to_canonical(position_optical, roll_deg, 0.0)


def depth_mount_rotation(roll_deg: float) -> tuple[tuple[float, ...], ...]:
    """Return the 3x3 matrix equivalent of depth_optical_to_canonical."""

    roll = math.radians(float(roll_deg))
    sr, cr = math.sin(roll), math.cos(roll)
    return ((cr, -sr, 0.0), (sr, cr, 0.0), (0.0, 0.0, 1.0))


def rotate_covariance_3x3(covariance, rotation) -> list[float]:
    """Return R * covariance(3x3) * R^T flattened row-major."""

    rotated = []
    for i in range(3):
        for j in range(3):
            value = 0.0
            for k in range(3):
                for l in range(3):
                    value += rotation[i][k] * float(covariance[3 * k + l]) * rotation[j][l]
            rotated.append(value)
    return rotated


def rotate_angle_covariance(
    variance_u: float,
    variance_v: float,
    fx: float,
    fy: float,
    roll_deg: float,
) -> list[float]:
    """Rotate the pixel covariance into the canonical frame, then to angles.

    Locally the bearing angles behave like the canonical normalized image
    coordinates, so J = R_roll * diag(1/fx, 1/fy).
    """

    roll = math.radians(float(roll_deg))
    sr, cr = math.sin(roll), math.cos(roll)
    base = (
        (cr * cr * float(variance_u) / (fx * fx))
        + (sr * sr * float(variance_v) / (fy * fy))
    )
    cross = cr * sr * (
        float(variance_u) / (fx * fx) - float(variance_v) / (fy * fy)
    )
    other = (
        (sr * sr * float(variance_u) / (fx * fx))
        + (cr * cr * float(variance_v) / (fy * fy))
    )
    return [base, cross, cross, other]


def optical_range(position: tuple[float, float, float]) -> float:
    values = tuple(float(value) for value in position)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError("optical position must contain three finite values")
    return math.sqrt(sum(value * value for value in values))


def main(args=None) -> int:
    import rclpy
    from anti_drone_interfaces.msg import DepthObservation, SearchObservation
    from falcon_intercept_msgs.msg import AsyncStereoMeasurement, CameraBearingObservation
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    class PerceptionBridge(Node):
        def __init__(self) -> None:
            super().__init__("perception_bridge_node")
            self.declare_parameter("search_bearing_topic", "/perception/search/bearing")
            self.declare_parameter(
                "async_measurement_topic", "/perception/async_stereo_measurement"
            )
            self.declare_parameter(
                "search_observation_topic", "/perception/search/observation"
            )
            self.declare_parameter(
                "depth_observation_topic", "/perception/depth/observation"
            )
            self.declare_parameter("search_image_width", 2560)
            self.declare_parameter("search_image_height", 1440)
            # 2026-08-14: vendor-nominal FOV 60x35 (H×V) for the 2560x1440 mode,
            # matching the search section of stereo_calib.yaml.  The previous
            # defaults (4059.6/1247.1) traced back to an upstream calibration
            # whose H/V FOV were swapped (35x60), which scaled horizontal
            # angles ~1.8x small and vertical ~1.8x large.  Centering was
            # unaffected (zero-seeking), but off-axis guidance gain was wrong.
            self.declare_parameter("search_fx", 2217.025033689)
            self.declare_parameter("search_fy", 2283.984079750)
            self.declare_parameter("search_cx", 1279.5)
            self.declare_parameter("search_cy", 719.5)
            self.declare_parameter("depth_image_width", 3840)
            self.declare_parameter("depth_image_height", 2160)
            # Mount compensation into the canonical logical optical frame (see
            # the module comment above).  Defaults match the 2026-08-13 field
            # measurement: the rig is rolled -90 deg about the lens axis.
            self.declare_parameter("search_mount_roll_deg", -90.0)
            # The upstream controller already owns the measured 10 degree
            # search-camera boresight.  Applying 10 degrees here as well makes
            # the two rotations cancel and incorrectly models the search lens
            # as parallel to the vertical stereo lenses.
            self.declare_parameter("search_canonical_tilt_deg", 0.0)
            self.declare_parameter("depth_mount_roll_deg", -90.0)
            # Depth intrinsics of the canonical (production) left camera, used
            # only to re-express the reported pixel in the canonical image.
            self.declare_parameter("depth_fx", 9941.889883095939)
            self.declare_parameter("depth_fy", 9932.893400360232)
            self.declare_parameter("depth_cx", 1919.633589049889)
            self.declare_parameter("depth_cy", 730.3230363083479)
            self.declare_parameter("track_id", 0)

            self._search_intrinsics = PinholeIntrinsics(
                int(self.get_parameter("search_image_width").value),
                int(self.get_parameter("search_image_height").value),
                float(self.get_parameter("search_fx").value),
                float(self.get_parameter("search_fy").value),
                float(self.get_parameter("search_cx").value),
                float(self.get_parameter("search_cy").value),
            )
            self._search_intrinsics.validate()
            self._depth_width = int(self.get_parameter("depth_image_width").value)
            self._depth_height = int(self.get_parameter("depth_image_height").value)
            self._search_roll_deg = float(
                self.get_parameter("search_mount_roll_deg").value
            )
            self._search_tilt_deg = float(
                self.get_parameter("search_canonical_tilt_deg").value
            )
            self._depth_roll_deg = float(
                self.get_parameter("depth_mount_roll_deg").value
            )
            self._depth_intrinsics = PinholeIntrinsics(
                self._depth_width,
                self._depth_height,
                float(self.get_parameter("depth_fx").value),
                float(self.get_parameter("depth_fy").value),
                float(self.get_parameter("depth_cx").value),
                float(self.get_parameter("depth_cy").value),
            )
            self._depth_intrinsics.validate()
            self._depth_rotation = depth_mount_rotation(self._depth_roll_deg)
            self._track_id = int(self.get_parameter("track_id").value)

            self._search_pub = self.create_publisher(
                SearchObservation,
                str(self.get_parameter("search_observation_topic").value),
                qos_profile_sensor_data,
            )
            self._depth_pub = self.create_publisher(
                DepthObservation,
                str(self.get_parameter("depth_observation_topic").value),
                qos_profile_sensor_data,
            )
            self.create_subscription(
                CameraBearingObservation,
                str(self.get_parameter("search_bearing_topic").value),
                self._on_search,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                AsyncStereoMeasurement,
                str(self.get_parameter("async_measurement_topic").value),
                self._on_depth,
                qos_profile_sensor_data,
            )

        @staticmethod
        def _copy_header(target, source, fallback_frame: str) -> None:
            target.stamp = source.stamp
            target.frame_id = str(source.frame_id) or fallback_frame

        def _on_search(self, source: CameraBearingObservation) -> None:
            output = SearchObservation()
            output.interface_version = SearchObservation.INTERFACE_VERSION_V1
            self._copy_header(output.header, source.header, "search_camera_optical")
            output.sequence = int(source.sequence)
            output.track_id = self._track_id
            output.visible = bool(source.valid)
            output.confidence = float(source.confidence)
            output.image_width = self._search_intrinsics.width
            output.image_height = self._search_intrinsics.height
            output.pixel_u = float(source.pixel_u)
            output.pixel_v = float(source.pixel_v)
            output.bounding_box_valid = False
            output.bounding_box_width_px = 0.0
            output.bounding_box_height_px = 0.0
            try:
                raw = (
                    (float(source.pixel_u) - self._search_intrinsics.cx)
                    / self._search_intrinsics.fx,
                    (float(source.pixel_v) - self._search_intrinsics.cy)
                    / self._search_intrinsics.fy,
                    1.0,
                )
                canonical = search_optical_to_canonical(
                    raw, self._search_roll_deg, self._search_tilt_deg
                )
                if canonical[2] <= 1e-6:
                    raise ValueError("target behind the canonical lens")
                length = math.sqrt(sum(value * value for value in canonical))
                los = tuple(value / length for value in canonical)
                horizontal = math.atan2(canonical[0], canonical[2])
                vertical = math.atan2(canonical[1], canonical[2])
                output.pixel_u = self._search_intrinsics.cx + (
                    self._search_intrinsics.fx * canonical[0] / canonical[2]
                )
                output.pixel_v = self._search_intrinsics.cy + (
                    self._search_intrinsics.fy * canonical[1] / canonical[2]
                )
                output.angles_valid = bool(source.valid)
                output.horizontal_angle_rad = horizontal
                output.vertical_angle_rad = vertical
                variance_u = max(0.0, float(source.pixel_covariance[0]))
                variance_v = max(0.0, float(source.pixel_covariance[3]))
                output.angle_covariance = rotate_angle_covariance(
                    variance_u,
                    variance_v,
                    self._search_intrinsics.fx,
                    self._search_intrinsics.fy,
                    self._search_roll_deg,
                )
                output.unit_los_valid = bool(source.valid)
                output.unit_los_optical.x = los[0]
                output.unit_los_optical.y = los[1]
                output.unit_los_optical.z = los[2]
            except (ValueError, IndexError):
                output.visible = False
                output.angles_valid = False
                output.unit_los_valid = False
            self._search_pub.publish(output)

        def _on_depth(self, source: AsyncStereoMeasurement) -> None:
            output = DepthObservation()
            output.interface_version = DepthObservation.INTERFACE_VERSION_V1
            self._copy_header(output.header, source.header, "pursuer_camera_optical")
            output.sequence = int(source.sequence)
            output.track_id = self._track_id
            output.visible = bool(source.valid)
            output.confidence = float(source.confidence)
            output.image_width = self._depth_width
            output.image_height = self._depth_height
            output.pixel_u = float(source.front_pixel_u)
            output.pixel_v = float(source.front_pixel_v)
            output.bounding_box_valid = False
            output.bounding_box_width_px = 0.0
            output.bounding_box_height_px = 0.0
            position_optical = (
                float(source.position_camera.x),
                float(source.position_camera.y),
                float(source.position_camera.z),
            )
            try:
                canonical = depth_optical_to_canonical(
                    position_optical, self._depth_roll_deg
                )
                range_m = optical_range(canonical)
                geometry_valid = bool(source.valid) and canonical[2] > 0.0 and range_m > 0.0
            except ValueError:
                canonical = (0.0, 0.0, 0.0)
                range_m = 0.0
                geometry_valid = False
            output.visible = geometry_valid
            output.angles_valid = geometry_valid
            output.depth_valid = geometry_valid
            if geometry_valid:
                output.horizontal_angle_rad = math.atan2(canonical[0], canonical[2])
                output.vertical_angle_rad = math.atan2(canonical[1], canonical[2])
                output.depth_z_m = canonical[2]
                output.range_m = range_m
                output.position_optical.x = canonical[0]
                output.position_optical.y = canonical[1]
                output.position_optical.z = canonical[2]
                output.pixel_u = self._depth_intrinsics.cx + (
                    self._depth_intrinsics.fx * canonical[0] / canonical[2]
                )
                output.pixel_v = self._depth_intrinsics.cy + (
                    self._depth_intrinsics.fy * canonical[1] / canonical[2]
                )
                output.position_covariance = rotate_covariance_3x3(
                    source.position_covariance, self._depth_rotation
                )
            else:
                output.horizontal_angle_rad = 0.0
                output.vertical_angle_rad = 0.0
                output.depth_z_m = 0.0
                output.range_m = 0.0
                output.position_covariance = [0.0] * 9
            output.angle_covariance = [0.0] * 4
            self._depth_pub.publish(output)

    rclpy.init(args=args)
    node = None
    try:
        node = PerceptionBridge()
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
