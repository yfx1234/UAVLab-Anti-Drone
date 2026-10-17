"""Replay the anti-drone 5 m/s geometries through direct and async stereo.

This evaluator intentionally uses the geometry documented in
``anti-drone/scripts/run_predictive_shot_2mps_scenario.sh``: R=50 m at
5 m/s, S=100x20 m, figure-eight=100x30 m, and the 15/17/20/25/35 m capture
distances. It injects the measured board timing quantiles rather than the
ideal same-timestamp simulator input.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT))

from falcon_intercept.async_state_estimator import (  # noqa: E402
    AsyncStereoConfig,
    AsyncStereoStateEstimator,
    PixelObservation,
)


class SyntheticOwnshipPose:
    """Deterministic FRU pose used to exercise ego-motion compensation."""

    def pose_at(self, timestamp_s: float) -> tuple[np.ndarray, np.ndarray]:
        yaw = 0.12 * float(timestamp_s)
        cosine, sine = math.cos(yaw), math.sin(yaw)
        rotation = np.array(
            [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
            dtype=float,
        )
        translation = np.array(
            [0.8 * float(timestamp_s), 0.2 * math.sin(0.2 * float(timestamp_s)), 0.0],
            dtype=float,
        )
        return rotation, translation


TIMING_CASES = {
    "P50": 0.005528,
    "P95": 0.01208854,
    "MAX": 0.01410937,
}
RANGES_M = (15.0, 17.0, 20.0, 25.0, 35.0)
FPS = 30.0
SPEED = 5.0


def trajectory(name: str, initial_range_m: float, time_s: float) -> tuple[np.ndarray, np.ndarray]:
    if name == "straight":
        return np.array([initial_range_m, SPEED * time_s, 0.0]), np.array([0.0, SPEED, 0.0])
    if name == "circle":
        radius = 50.0
        theta = SPEED * time_s / radius
        return (
            np.array([initial_range_m + radius * (math.cos(theta) - 1.0), radius * math.sin(theta), 0.0]),
            np.array([-SPEED * math.sin(theta), SPEED * math.cos(theta), 0.0]),
        )
    if name == "s_curve":
        length, width = 100.0, 20.0
        x = initial_range_m + SPEED * time_s
        phase = 2.0 * math.pi * (x - initial_range_m) / length
        return (
            np.array([x, 0.5 * width * math.sin(phase), 0.0]),
            np.array([SPEED, 0.5 * width * math.cos(phase) * 2.0 * math.pi / length * SPEED, 0.0]),
        )
    if name == "figure_eight":
        length, width = 100.0, 30.0
        # Select phase rate so the maximum analytic speed is 5 m/s.
        phase_grid = np.linspace(0.0, 2.0 * math.pi, 2001)
        dx = 0.5 * length * 2.0 * np.cos(2.0 * phase_grid)
        dy = 0.5 * width * 2.0 * np.cos(2.0 * phase_grid)
        omega = SPEED / float(np.max(np.sqrt(dx * dx + dy * dy)))
        phase = omega * time_s
        return (
            np.array([initial_range_m + 0.5 * length * math.sin(2.0 * phase), 0.5 * width * math.sin(phase), 0.0]),
            np.array([length * math.cos(2.0 * phase) * omega, 0.5 * width * math.cos(phase) * omega, 0.0]),
        )
    raise ValueError(f"unknown scenario: {name}")


def pixels_for_state(
    position: np.ndarray,
    velocity: np.ndarray,
    timestamp_s: float,
    camera_id: str,
    config: AsyncStereoConfig,
    pose_provider=None,
) -> PixelObservation:
    point = position + velocity * timestamp_s
    center_y = -0.5 * config.baseline_m if camera_id == "front" else 0.5 * config.baseline_m
    if pose_provider is None:
        point_camera = point.copy()
        point_camera[1] -= center_y
    else:
        rotation, translation = pose_provider.pose_at(timestamp_s)
        camera_center = translation + rotation @ np.array([0.0, center_y, 0.0])
        point_camera = rotation.T @ (point - camera_center)
    return PixelObservation(
        camera_id=camera_id,
        timestamp_s=timestamp_s,
        u=float(config.fx * point_camera[1] / point_camera[0] + config.cx),
        v=float(config.cy - config.fy * point_camera[2] / point_camera[0]),
    )


def direct_async_position(front: PixelObservation, back: PixelObservation, config: AsyncStereoConfig) -> np.ndarray:
    disparity = front.u - back.u
    if disparity <= 1e-9:
        return np.full(3, np.nan)
    x = config.fx * config.baseline_m / disparity
    y = (front.u - config.cx) * x / config.fx - 0.5 * config.baseline_m
    z = -(front.v - config.cy) * x / config.fy
    return np.array([x, y, z])


def stats(values: list[float]) -> tuple[float, float, float, float]:
    if not values:
        return (math.nan, math.nan, math.nan, math.nan)
    array = np.asarray(values, dtype=float)
    return tuple(float(np.percentile(array, q)) for q in (50, 95, 99)) + (float(np.max(array)),)


def run_case(
    scenario: str,
    initial_range_m: float,
    timing_label: str,
    delta_s: float,
    *,
    ownship_motion: bool = False,
) -> dict:
    config = AsyncStereoConfig(timestamp_sigma_s=0.0015)
    pose_provider = SyntheticOwnshipPose() if ownship_motion else None
    estimator = AsyncStereoStateEstimator(config, pose_provider=pose_provider)
    direct_position_errors: list[float] = []
    direct_range_errors: list[float] = []
    corrected_position_errors: list[float] = []
    corrected_range_errors: list[float] = []
    corrected_velocity_errors: list[float] = []
    reprojection_errors: list[float] = []
    valid_updates = 0
    total_updates = 0
    # Evaluate the capture window around the requested range.  At 5 m/s the
    # anti-drone launcher can run much longer, but after several seconds a
    # deliberately fixed initial range is no longer the same observation
    # condition (especially for the R=50 m circle).  Sixty stereo updates
    # cover a representative two-second capture segment.
    for index in range(60):
        front_time = index / FPS
        back_time = front_time + delta_s
        reference_time = 0.5 * (front_time + back_time)
        front_position, front_velocity = trajectory(scenario, initial_range_m, 0.0)
        # The trajectory helper returns state at a supplied absolute time; use
        # a local tangent around each exposure to generate exact observations.
        front_position, _ = trajectory(scenario, initial_range_m, front_time)
        back_position, _ = trajectory(scenario, initial_range_m, back_time)
        reference_position, reference_velocity = trajectory(scenario, initial_range_m, reference_time)
        front = pixels_for_state(front_position, np.zeros(3), front_time, "front", config, pose_provider)
        back = pixels_for_state(back_position, np.zeros(3), back_time, "back", config, pose_provider)
        front = PixelObservation("front", front_time, front.u, front.v, timestamp_sigma_s=config.timestamp_sigma_s)
        back = PixelObservation("back", back_time, back.u, back.v, timestamp_sigma_s=config.timestamp_sigma_s)
        direct = direct_async_position(front, back, config)
        if np.all(np.isfinite(direct)):
            direct_truth = reference_position
            if pose_provider is not None:
                rotation, translation = pose_provider.pose_at(reference_time)
                camera_center = translation + rotation @ np.array([0.0, -0.5 * config.baseline_m, 0.0])
                direct_truth = rotation.T @ (reference_position - camera_center)
            direct_error = float(np.linalg.norm(direct - direct_truth))
            direct_position_errors.append(direct_error)
            direct_range_errors.append(abs(float(np.linalg.norm(direct) - np.linalg.norm(direct_truth))))
        result = estimator.update(front, back)
        total_updates += 1
        if result.accepted:
            valid_updates += 1
            corrected_position_errors.append(float(np.linalg.norm(result.position - reference_position)))
            corrected_range_errors.append(abs(float(result.range_m - np.linalg.norm(reference_position))))
            corrected_velocity_errors.append(float(np.linalg.norm(result.velocity - reference_velocity)))
            reprojection_errors.append(float(result.reprojection_error_px))
    return {
        "scenario": scenario,
        "range_m": initial_range_m,
        "timing": timing_label,
        "timing_delta_ms": delta_s * 1000.0,
        "direct_position_m": stats(direct_position_errors),
        "direct_range_m": stats(direct_range_errors),
        "corrected_position_m": stats(corrected_position_errors),
        "corrected_range_m": stats(corrected_range_errors),
        "corrected_velocity_mps": stats(corrected_velocity_errors),
        "reprojection_px": stats(reprojection_errors),
        "valid_update_rate": valid_updates / max(total_updates, 1),
        "oosm_count": estimator.oosm_count,
        "oosm_drop_count": estimator.oosm_drop_count,
        "diverged": bool(corrected_position_errors and np.max(corrected_position_errors) > 100.0),
        "ownship_motion": ownship_motion,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", choices=("straight", "circle", "s_curve", "figure_eight"), action="append")
    parser.add_argument("--range", dest="ranges", type=float, action="append")
    parser.add_argument("--ownship-motion", action="store_true")
    args = parser.parse_args()
    scenarios = args.scenario or ("straight", "circle", "s_curve", "figure_eight")
    ranges = args.ranges or RANGES_M
    print("scenario,range_m,timing,dt_ms,direct_pos_P50/P95/P99/max,corrected_pos_P50/P95/P99/max,corrected_range_P95,velocity_P95,reproj_P95,valid_rate")
    for scenario in scenarios:
        for range_m in ranges:
            for label, delta in TIMING_CASES.items():
                result = run_case(
                    scenario,
                    range_m,
                    label,
                    delta,
                    ownship_motion=args.ownship_motion,
                )
                fmt = lambda value: "/".join("nan" if not math.isfinite(item) else f"{item:.4f}" for item in value)
                print(
                    f"{scenario},{range_m:.1f},{label},{result['timing_delta_ms']:.3f},"
                    f"{fmt(result['direct_position_m'])},{fmt(result['corrected_position_m'])},"
                    f"{result['corrected_range_m'][1]:.4f},{result['corrected_velocity_mps'][1]:.4f},"
                    f"{result['reprojection_px'][1]:.4f},{result['valid_update_rate']:.3f}"
                )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
