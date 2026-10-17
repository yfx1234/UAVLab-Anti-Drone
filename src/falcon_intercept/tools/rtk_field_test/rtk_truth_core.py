#!/usr/bin/env python3
"""Core math for the RTK-truth vs stereo-ranging field test.

Pure standard library so the same code runs on the dev laptop (no ROS)
and on the Jetson. Used by compare_rtk_stereo_truth.py and by the unit
tests in src/falcon_intercept/test/test_rtk_truth_core.py.
"""

from __future__ import annotations

import math
import statistics
from typing import Iterable, Sequence

# WGS84 ellipsoid constants.
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    x = (n + alt_m) * cos_lat * math.cos(lon)
    y = (n + alt_m) * cos_lat * math.sin(lon)
    z = (n * (1.0 - WGS84_E2) + alt_m) * sin_lat
    return (x, y, z)


def geodetic_to_enu(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    ref_lat_deg: float,
    ref_lon_deg: float,
    ref_alt_m: float,
) -> tuple[float, float, float]:
    """Convert geodetic to local ENU (east, north, up) around a reference."""
    x, y, z = geodetic_to_ecef(lat_deg, lon_deg, alt_m)
    x0, y0, z0 = geodetic_to_ecef(ref_lat_deg, ref_lon_deg, ref_alt_m)
    dx, dy, dz = x - x0, y - y0, z - z0
    lat0 = math.radians(ref_lat_deg)
    lon0 = math.radians(ref_lon_deg)
    sin_lat, cos_lat = math.sin(lat0), math.cos(lat0)
    sin_lon, cos_lon = math.sin(lon0), math.cos(lon0)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return (east, north, up)


def percentile(values: Sequence[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def fit_clock_offset(pairs: Sequence[tuple[float, float]]) -> dict:
    """Robust constant offset between two clocks.

    pairs: (local_unix, reference_unix) samples. Returns offset such that
    reference_unix + offset ~= local_unix, with MAD-based outlier rejection
    and a drift estimate (linear slope of residuals, ms/s).
    """
    if not pairs:
        return {"offset_s": 0.0, "residual_std_ms": None, "drift_ms_per_s": None, "n_used": 0, "n_total": 0}
    raw = [local - ref for local, ref in pairs]
    med = statistics.median(raw)
    abs_dev = [abs(value - med) for value in raw]
    mad = statistics.median(abs_dev)
    tol = max(5.0 * 1.4826 * mad, 0.050)  # at least 50 ms tolerance
    kept = [(local, ref) for (local, ref), value in zip(pairs, raw) if abs(value - med) <= tol]
    offset = statistics.median([local - ref for local, ref in kept])
    residuals = [(local - ref) - offset for local, ref in kept]
    drift = None
    if len(kept) >= 3:
        t0 = kept[0][0]
        xs = [local - t0 for local, _ in kept]
        mean_x = statistics.fmean(xs)
        mean_y = statistics.fmean(residuals)
        denom = sum((x - mean_x) ** 2 for x in xs)
        if denom > 0:
            drift = sum((x - mean_x) * (y - mean_y) for x, y in zip(xs, residuals)) / denom * 1000.0
    return {
        "offset_s": offset,
        "residual_std_ms": (statistics.pstdev(residuals) * 1000.0) if len(residuals) >= 2 else 0.0,
        "drift_ms_per_s": drift,
        "n_used": len(kept),
        "n_total": len(pairs),
    }


def interpolate_series(
    times: Sequence[float],
    values: Sequence[tuple[float, ...]],
    query: float,
    max_gap_s: float,
) -> tuple[float, ...] | None:
    """Linear interpolation of vector samples at `query`.

    Returns None when the query is outside the sample span or the
    surrounding sample gap exceeds max_gap_s.
    """
    if not times:
        return None
    if query < times[0] or query > times[-1]:
        return None
    # Binary search for the bracketing interval.
    lo, hi = 0, len(times) - 1
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if times[mid] <= query:
            lo = mid
        else:
            hi = mid
    if times[hi] - times[lo] > max_gap_s:
        return None
    if times[hi] == times[lo]:
        return values[lo]
    ratio = (query - times[lo]) / (times[hi] - times[lo])
    return tuple(a + (b - a) * ratio for a, b in zip(values[lo], values[hi]))


def range_sigma_from_covariance(
    position: Sequence[float], covariance: Sequence[float]
) -> float | None:
    """Project the 3x3 position covariance onto the line of sight."""
    norm = math.sqrt(sum(component * component for component in position))
    if norm <= 0.0 or len(covariance) < 9:
        return None
    u = [component / norm for component in position]
    variance = 0.0
    for i in range(3):
        for j in range(3):
            variance += u[i] * covariance[i * 3 + j] * u[j]
    if variance <= 0.0:
        return None
    return math.sqrt(variance)


def error_stats(errors: Sequence[float]) -> dict:
    if not errors:
        return {"count": 0}
    abs_errors = [abs(value) for value in errors]
    return {
        "count": len(errors),
        "mean_m": statistics.fmean(errors),
        "abs_mean_m": statistics.fmean(abs_errors),
        "p50_m": percentile(abs_errors, 0.50),
        "p95_m": percentile(abs_errors, 0.95),
        "max_m": max(abs_errors),
        "rmse_m": math.sqrt(statistics.fmean([value * value for value in errors])),
        "std_m": statistics.pstdev(errors) if len(errors) >= 2 else 0.0,
    }


def layer_index(range_m: float, edges: Sequence[float]) -> int | None:
    """Bin a truth range; edges are upper bounds: [0,e0), [e0,e1), ..., >=e_last."""
    if range_m < 0.0:
        return None
    for i, edge in enumerate(edges):
        if range_m < edge:
            return i
    return len(edges)


def layer_label(index: int, edges: Sequence[float]) -> str:
    if index >= len(edges):
        return f">={edges[-1]:g}m"
    lower = 0.0 if index == 0 else edges[index - 1]
    return f"{lower:g}-{edges[index]:g}m"


def median_position(fixes: Iterable[dict]) -> dict | None:
    """Median lat/lon/alt of RTK fixes (static anchor estimate)."""
    fixes = list(fixes)
    if not fixes:
        return None
    return {
        "lat": statistics.median(f["lat"] for f in fixes),
        "lon": statistics.median(f["lon"] for f in fixes),
        "alt": statistics.median(f["alt"] for f in fixes),
        "count": len(fixes),
    }


def body_lever_to_enu(
    lever_body_flu: Sequence[float], yaw_deg: float
) -> tuple[float, float, float]:
    """Convert a body-frame (FLU: x forward, y left, z up) lever arm to ENU.

    yaw_deg is the compass heading of the body X axis, degrees clockwise
    from north (0 = X points north, 90 = X points east).  The rig is
    assumed level, so the up component passes through unchanged.
    """
    psi = math.radians(yaw_deg)
    lx, ly, lz = (float(v) for v in lever_body_flu)
    east = lx * math.sin(psi) - ly * math.cos(psi)
    north = lx * math.cos(psi) + ly * math.sin(psi)
    return (east, north, lz)
