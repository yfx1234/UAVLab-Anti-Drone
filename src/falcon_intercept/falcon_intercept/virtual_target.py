from __future__ import annotations

import json
import math
from dataclasses import dataclass

from .detection_utils import stamp_to_dict


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]


@dataclass(frozen=True)
class StereoProjection:
    front: tuple[float, float]
    back: tuple[float, float]
    metric_disparity_px: float


def rotate_body_frd_to_ned(q_wxyz: Quaternion, value: Vector3) -> Vector3:
    w, x, y, z = (float(item) for item in q_wxyz)
    vx, vy, vz = (float(item) for item in value)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError("invalid body-to-NED quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return (
        (1.0 - 2.0 * (y * y + z * z)) * vx
        + 2.0 * (x * y - z * w) * vy
        + 2.0 * (x * z + y * w) * vz,
        2.0 * (x * y + z * w) * vx
        + (1.0 - 2.0 * (x * x + z * z)) * vy
        + 2.0 * (y * z - x * w) * vz,
        2.0 * (x * z - y * w) * vx
        + 2.0 * (y * z + x * w) * vy
        + (1.0 - 2.0 * (x * x + y * y)) * vz,
    )


def rotate_ned_to_body_frd(q_wxyz: Quaternion, value: Vector3) -> Vector3:
    """Apply the inverse of PX4 VehicleAttitude.q to a local-NED vector."""

    w, x, y, z = q_wxyz
    return rotate_body_frd_to_ned((w, -x, -y, -z), value)


def body_flu_to_ned(q_wxyz: Quaternion, value: Vector3) -> Vector3:
    return rotate_body_frd_to_ned(
        q_wxyz, (float(value[0]), -float(value[1]), -float(value[2]))
    )


def relative_ned_to_camera_fru(
    relative_ned_m: Vector3,
    q_body_frd_to_ned: Quaternion,
    camera_translation_body_flu_m: Vector3,
) -> Vector3:
    """Return camera-relative x-forward/y-right/z-up coordinates."""

    frd = rotate_ned_to_body_frd(q_body_frd_to_ned, relative_ned_m)
    body_flu = (frd[0], -frd[1], -frd[2])
    camera_flu = tuple(
        body_flu[index] - float(camera_translation_body_flu_m[index])
        for index in range(3)
    )
    return camera_flu[0], -camera_flu[1], camera_flu[2]


def sample_trajectory_offset(
    scenario: str,
    elapsed_s: float,
    speed_mps: float,
    radius_m: float,
    path_width_m: float,
    path_length_m: float | None = None,
) -> tuple[Vector3, Vector3]:
    """Sample a target path in its frozen spawn frame (forward, right, down)."""

    kind = str(scenario).strip().lower().replace("-", "_")
    t = max(0.0, float(elapsed_s))
    speed = max(0.0, float(speed_mps))
    radius = max(0.1, float(radius_m))
    width = max(0.1, float(path_width_m))
    length = max(0.1, float(path_length_m if path_length_m is not None else 2.0 * radius))
    if kind in ("stationary", "fixed"):
        return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)
    if kind == "straight":
        return (speed * t, 0.0, 0.0), (speed, 0.0, 0.0)
    if kind == "circle":
        omega = speed / radius
        return (
            radius * math.sin(omega * t),
            radius * (1.0 - math.cos(omega * t)),
            0.0,
        ), (
            speed * math.cos(omega * t),
            speed * math.sin(omega * t),
            0.0,
        )
    if kind == "s_curve":
        omega = speed / max(4.0 * width, 0.1)
        return (
            speed * t,
            width * math.sin(omega * t),
            0.0,
        ), (
            speed,
            width * omega * math.cos(omega * t),
            0.0,
        )
    if kind in ("figure_eight", "figure8"):
        omega = speed / radius
        phase = omega * t
        return (
            radius * math.sin(phase),
            0.5 * radius * math.sin(2.0 * phase),
            0.0,
        ), (
            radius * omega * math.cos(phase),
            radius * omega * math.cos(2.0 * phase),
            0.0,
        )
    if kind == "rectangle":
        # A continuous periodic rectangle.  Corners are intentionally not
        # rounded here: this profile is used to exercise abrupt target
        # velocity changes and estimator recovery.
        perimeter = 2.0 * (length + width)
        distance = (speed * t) % perimeter if speed > 0.0 else 0.0
        if distance < length:
            return (distance, 0.0, 0.0), (speed, 0.0, 0.0)
        distance -= length
        if distance < width:
            return (length, distance, 0.0), (0.0, speed, 0.0)
        distance -= width
        if distance < length:
            return (length - distance, width, 0.0), (-speed, 0.0, 0.0)
        distance -= length
        return (0.0, width - distance, 0.0), (0.0, -speed, 0.0)
    raise ValueError(
        "target_scenario must be stationary, straight, circle, s_curve, "
        "figure_eight, or rectangle"
    )


def project_field_aligned_stereo(
    point_camera_fru_m: Vector3,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    baseline_m: float,
    vertical_offset_px: float,
    vertical_slope: float,
    vertical_reference_u_px: float,
    horizontal_zero_px: float,
    horizontal_sign: float,
) -> StereoProjection | None:
    """Create pixels that invert through the assembled field-calibrated solver."""

    forward, right, up = (float(item) for item in point_camera_fru_m)
    if not all(math.isfinite(item) for item in (forward, right, up)) or forward <= 0.1:
        return None
    disparity = float(fx) * float(baseline_m) / forward
    u_front = float(cx) + float(fx) * right / forward
    v_front = float(cy) - float(fy) * up / forward
    observed_disparity = float(horizontal_zero_px) + float(horizontal_sign) * disparity
    slope = float(vertical_slope)
    u_back = (
        u_front
        + slope * (v_front - float(vertical_offset_px) - float(cy))
        + slope * slope * float(vertical_reference_u_px)
        - observed_disparity
    ) / (1.0 + slope * slope)
    v_back = (
        v_front
        - float(vertical_offset_px)
        - slope * (u_back - float(vertical_reference_u_px))
    )
    return StereoProjection(
        front=(u_front, v_front),
        back=(u_back, v_back),
        metric_disparity_px=disparity,
    )


def project_pinhole(
    point_camera_fru_m: Vector3,
    *,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> tuple[float, float] | None:
    forward, right, up = (float(item) for item in point_camera_fru_m)
    if not all(math.isfinite(item) for item in (forward, right, up)) or forward <= 0.1:
        return None
    return cx + fx * right / forward, cy - fy * up / forward


def build_logical_detection_json(
    *,
    frame_id: str,
    stamp_s: float,
    image_width: int,
    image_height: int,
    center: tuple[float, float] | None,
    sequence: int,
    box_size: tuple[float, float] = (48.0, 32.0),
    confidence: float = 0.95,
) -> str:
    detections = []
    if center is not None:
        u, v = center
        width, height = box_size
        if (
            math.isfinite(u)
            and math.isfinite(v)
            and -width / 2.0 <= u <= image_width + width / 2.0
            and -height / 2.0 <= v <= image_height + height / 2.0
        ):
            x1, y1 = u - width / 2.0, v - height / 2.0
            detections.append(
                {
                    "class_id": 0,
                    "class_name": "target",
                    "confidence": float(confidence),
                    "bbox_xyxy": [x1, y1, x1 + width, y1 + height],
                    "bbox_xywh": [x1, y1, width, height],
                    "bbox_center": [u, v],
                }
            )
    payload = {
        "header": {"stamp": stamp_to_dict(stamp_s), "frame_id": frame_id},
        "image_width": int(image_width),
        "image_height": int(image_height),
        "sequence": int(sequence),
        "timestamp_source": "virtual_capture_clock",
        "host_receive_time": stamp_to_dict(stamp_s),
        "timestamp_sigma_s": 0.0005,
        "source": "real_aircraft_virtual_target",
        "detections": detections,
    }
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
