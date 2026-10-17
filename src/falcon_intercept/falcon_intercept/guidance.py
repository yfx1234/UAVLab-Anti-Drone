from __future__ import annotations

from dataclasses import dataclass

from .safety import VelocityLimit, limit_body_velocity


@dataclass(frozen=True)
class GuidanceConfig:
    kp_xy: float = 0.4
    kp_z: float = 0.3
    max_speed_xy: float = 0.8
    max_speed_z: float = 0.5
    desired_distance_m: float = 2.0
    search_yaw_kp: float = 0.004
    max_search_yaw_rate_rps: float = 0.35
    scan_yaw_rate_rps: float = 0.20


def compute_approach_velocity(
    target_position_m: tuple[float, float, float],
    config: GuidanceConfig,
) -> tuple[float, float, float]:
    """Compute conservative body-frame approach velocity.

    Coordinate convention for V0: x forward, y right, z up. The forward
    command slows to zero at the desired fire/stand-off distance.
    """
    x, y, z = (float(value) for value in target_position_m)
    forward_error = max(0.0, x - float(config.desired_distance_m))
    desired = (
        float(config.kp_xy) * forward_error,
        float(config.kp_xy) * y,
        float(config.kp_z) * z,
    )
    return limit_body_velocity(
        desired,
        VelocityLimit(max_speed_xy=config.max_speed_xy, max_speed_z=config.max_speed_z),
    )


def compute_search_yaw_rate(
    horizontal_error_px: float | None,
    config: GuidanceConfig,
) -> float:
    """Return a conservative body yaw-rate cue for search-camera handoff."""
    if horizontal_error_px is None:
        return float(config.scan_yaw_rate_rps)
    desired = float(config.search_yaw_kp) * float(horizontal_error_px)
    limit = abs(float(config.max_search_yaw_rate_rps))
    return max(-limit, min(limit, desired))
