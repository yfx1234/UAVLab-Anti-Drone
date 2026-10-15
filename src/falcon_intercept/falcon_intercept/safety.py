from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class VelocityLimit:
    max_speed_xy: float = 0.8
    max_speed_z: float = 0.5


def finite_or_zero(value: float) -> float:
    value = float(value)
    return value if math.isfinite(value) else 0.0


def clamp(value: float, lower: float, upper: float) -> float:
    value = finite_or_zero(value)
    return min(max(value, float(lower)), float(upper))


def limit_body_velocity(
    vector: tuple[float, float, float],
    limits: VelocityLimit,
) -> tuple[float, float, float]:
    x, y, z = (finite_or_zero(value) for value in vector)
    return (
        clamp(x, -limits.max_speed_xy, limits.max_speed_xy),
        clamp(y, -limits.max_speed_xy, limits.max_speed_xy),
        clamp(z, -limits.max_speed_z, limits.max_speed_z),
    )
