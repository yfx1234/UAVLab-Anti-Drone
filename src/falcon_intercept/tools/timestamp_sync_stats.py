#!/usr/bin/env python3
"""Compute camera timestamp offset and jitter from LED/event observations.

Input CSV columns:
  event_id,camera,timestamp_s

Each event_id should identify the same visible flash/edge in every camera.
All timestamps must use the same clock. The script reports each camera's
offset against the reference camera and writes JSON suitable for the handoff
evidence folder.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = (len(ordered) - 1) * fraction
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    weight = index - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def read_events(path: Path) -> dict[str, dict[str, float]]:
    events: dict[str, dict[str, float]] = defaultdict(dict)
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        required = {"event_id", "camera", "timestamp_s"}
        if not reader.fieldnames or not required.issubset(set(reader.fieldnames)):
            raise ValueError("CSV must contain event_id,camera,timestamp_s columns")
        for row in reader:
            event_id = str(row["event_id"]).strip()
            camera = str(row["camera"]).strip()
            if not event_id or not camera:
                continue
            events[event_id][camera] = float(row["timestamp_s"])
    if not events:
        raise ValueError("CSV contains no timestamp events")
    return dict(events)


def stats(offsets_ms: list[float]) -> dict[str, float | int]:
    absolute = [abs(value) for value in offsets_ms]
    return {
        "count": len(offsets_ms),
        "mean_ms": statistics.fmean(offsets_ms),
        "median_ms": statistics.median(offsets_ms),
        "std_ms": statistics.pstdev(offsets_ms) if len(offsets_ms) > 1 else 0.0,
        "p95_abs_ms": percentile(absolute, 0.95),
        "p99_abs_ms": percentile(absolute, 0.99),
        "max_abs_ms": max(absolute),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", required=True, help="CSV with event_id,camera,timestamp_s")
    parser.add_argument("--reference", default="search", help="reference camera name")
    parser.add_argument("--output", required=True, help="output JSON path")
    args = parser.parse_args()

    events = read_events(Path(args.input))
    offsets: dict[str, list[float]] = defaultdict(list)
    paired_events: dict[str, int] = defaultdict(int)
    for event_id, cameras in events.items():
        if args.reference not in cameras:
            continue
        reference_time = cameras[args.reference]
        for camera, timestamp in cameras.items():
            if camera == args.reference:
                continue
            offsets[camera].append((timestamp - reference_time) * 1000.0)
            paired_events[camera] += 1
    if not offsets:
        raise ValueError(f"no camera events paired against reference {args.reference!r}")
    result = {
        "reference_camera": args.reference,
        "event_count": len(events),
        "cameras": {camera: stats(values) for camera, values in sorted(offsets.items())},
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    for camera, values in sorted(offsets.items()):
        summary = stats(values)
        print(
            f"SYNC {args.reference}->{camera} count={summary['count']} "
            f"median_ms={summary['median_ms']:.3f} p95_abs_ms={summary['p95_abs_ms']:.3f} "
            f"max_abs_ms={summary['max_abs_ms']:.3f}"
        )
    print(f"SYNC_STATS_OK output={output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
