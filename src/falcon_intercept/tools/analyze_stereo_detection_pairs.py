#!/usr/bin/env python3
"""Analyze nearest-time stereo detection pairs from video_detection_stats JSONL."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path


def stamp(payload: dict) -> float:
    value = payload["header"]["stamp"]
    return float(value["sec"]) + float(value["nanosec"]) / 1e9


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def distribution(values: list[float]) -> dict:
    return {
        "count": len(values),
        "min": min(values) if values else None,
        "mean": statistics.fmean(values) if values else None,
        "stddev": statistics.pstdev(values) if values else None,
        "p05": percentile(values, 0.05),
        "p50": percentile(values, 0.50),
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("jsonl", type=Path)
    parser.add_argument("--json-output", type=Path)
    parser.add_argument("--min-confidence", type=float, default=0.5)
    parser.add_argument("--max-time-delta-ms", type=float, default=50.0)
    parser.add_argument("--max-vertical-error-px", type=float, default=8.0)
    parser.add_argument("--fx", type=float, default=10238.013493164)
    parser.add_argument("--fy", type=float, default=10322.580645161)
    parser.add_argument("--cx", type=float, default=1919.5)
    parser.add_argument("--baseline-m", type=float, default=0.1)
    parser.add_argument("--cy", type=float, default=1079.5)
    parser.add_argument("--reference-range-m", type=float)
    parser.add_argument("--vertical-offset-px", type=float, default=-50.421808192)
    parser.add_argument("--vertical-slope", type=float, default=-0.025377902666)
    parser.add_argument("--vertical-reference-u-px", type=float, default=1919.5)
    parser.add_argument("--horizontal-zero-px", type=float, default=-365.544120787)
    parser.add_argument("--horizontal-sign", type=float, choices=(-1.0, 1.0), default=1.0)
    args = parser.parse_args()

    frames = {"front": [], "back": []}
    for line in args.jsonl.read_text(encoding="utf-8").splitlines():
        row = json.loads(line)
        if row["source"] in frames:
            frames[row["source"]].append(row["payload"])
    for source in frames:
        frames[source].sort(key=stamp)

    back_index = 0
    counts = {
        "front_frames": len(frames["front"]),
        "back_frames": len(frames["back"]),
        "front_with_eligible_detection": 0,
        "nearest_within_time_gate": 0,
        "positive_disparity": 0,
        "vertical_gate_pass": 0,
    }
    time_deltas_ms: list[float] = []
    vertical_errors_px: list[float] = []
    disparities_px: list[float] = []
    ranges_m: list[float] = []
    slant_ranges_m: list[float] = []
    front_centers: list[tuple[float, float]] = []
    back_centers: list[tuple[float, float]] = []

    for front in frames["front"]:
        front_detections = [
            detection
            for detection in front["detections"]
            if float(detection["confidence"]) >= args.min_confidence
        ]
        if not front_detections or not frames["back"]:
            continue
        counts["front_with_eligible_detection"] += 1
        front_time = stamp(front)
        while (
            back_index + 1 < len(frames["back"])
            and abs(stamp(frames["back"][back_index + 1]) - front_time)
            <= abs(stamp(frames["back"][back_index]) - front_time)
        ):
            back_index += 1
        back = frames["back"][back_index]
        delta_ms = abs(stamp(back) - front_time) * 1000.0
        if delta_ms > args.max_time_delta_ms:
            continue
        counts["nearest_within_time_gate"] += 1
        time_deltas_ms.append(delta_ms)
        back_detections = [
            detection
            for detection in back["detections"]
            if float(detection["confidence"]) >= args.min_confidence
        ]
        candidates = []
        for front_detection in front_detections:
            for back_detection in back_detections:
                front_u, front_v = map(float, front_detection["bbox_center"])
                back_u, back_v = map(float, back_detection["bbox_center"])
                corrected_back_u = back_u - args.vertical_slope * (
                    back_v - args.cy
                )
                disparity = args.horizontal_sign * (
                    front_u - corrected_back_u - args.horizontal_zero_px
                )
                if disparity <= 1.0:
                    continue
                expected_vertical = args.vertical_offset_px + args.vertical_slope * (
                    back_u - args.vertical_reference_u_px
                )
                vertical_error = abs(front_v - back_v - expected_vertical)
                candidates.append(
                    (
                        float(front_detection["confidence"])
                        * float(back_detection["confidence"]),
                        vertical_error,
                        disparity,
                        (front_u, front_v),
                        (back_u, back_v),
                    )
                )
        if not candidates:
            continue
        counts["positive_disparity"] += 1
        _, vertical_error, disparity, front_center, back_center = max(
            candidates, key=lambda item: (item[0], -item[1])
        )
        vertical_errors_px.append(vertical_error)
        if vertical_error > args.max_vertical_error_px:
            continue
        counts["vertical_gate_pass"] += 1
        disparities_px.append(disparity)
        forward_range = args.fx * args.baseline_m / disparity
        ranges_m.append(forward_range)
        y_back = (front_center[0] - args.cx) * forward_range / args.fx
        z_up = -(front_center[1] - args.cy) * forward_range / args.fy
        slant_ranges_m.append(
            math.sqrt(forward_range**2 + y_back**2 + z_up**2)
        )
        front_centers.append(front_center)
        back_centers.append(back_center)

    result = {
        "configuration": {
            key: str(value) if isinstance(value, Path) else value
            for key, value in vars(args).items()
        },
        "counts": counts,
        "fractions": {
            "eligible_front_fraction": (
                counts["front_with_eligible_detection"] / counts["front_frames"]
                if counts["front_frames"]
                else 0.0
            ),
            "vertical_gate_pass_per_eligible_front": (
                counts["vertical_gate_pass"] / counts["front_with_eligible_detection"]
                if counts["front_with_eligible_detection"]
                else 0.0
            ),
        },
        "time_delta_ms": distribution(time_deltas_ms),
        "vertical_error_px": distribution(vertical_errors_px),
        "disparity_px": distribution(disparities_px),
        "forward_range_m": distribution(ranges_m),
        "slant_range_m": distribution(slant_ranges_m),
        "slant_error_m": (
            distribution(
                [value - args.reference_range_m for value in slant_ranges_m]
            )
            if args.reference_range_m is not None
            else None
        ),
        "slant_absolute_percentage_error": (
            distribution(
                [
                    abs(value - args.reference_range_m)
                    / args.reference_range_m
                    * 100.0
                    for value in slant_ranges_m
                ]
            )
            if args.reference_range_m is not None
            and args.reference_range_m > 0.0
            else None
        ),
        "accepted_front_center_x_px": distribution([value[0] for value in front_centers]),
        "accepted_front_center_y_px": distribution([value[1] for value in front_centers]),
        "accepted_back_center_x_px": distribution([value[0] for value in back_centers]),
        "accepted_back_center_y_px": distribution([value[1] for value in back_centers]),
    }
    output = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.json_output:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(output, encoding="utf-8")
    print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
