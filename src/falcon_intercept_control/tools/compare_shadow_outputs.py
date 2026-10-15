#!/usr/bin/env python3
"""Compare normalized upstream and Jetson shadow JSONL streams for S0 acceptance.

Each line must contain: timestamp_s, state, velocity_flu[3], yaw_rate_flu,
land_request and reason. A recorder/exporter may add fields; they are ignored.
"""

import argparse
import json
import math
from pathlib import Path


REQUIRED = ("timestamp_s", "state", "velocity_flu", "yaw_rate_flu", "land_request", "reason")


def load_jsonl(path):
    rows = []
    with Path(path).open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                continue
            row = json.loads(line)
            missing = [key for key in REQUIRED if key not in row]
            if missing:
                raise ValueError(f"{path}:{line_number}: missing {missing}")
            velocity = tuple(float(v) for v in row["velocity_flu"])
            values = (float(row["timestamp_s"]), *velocity, float(row["yaw_rate_flu"]))
            if len(velocity) != 3 or not all(math.isfinite(v) for v in values):
                raise ValueError(f"{path}:{line_number}: non-finite or invalid vector")
            rows.append({**row, "timestamp_s": values[0], "velocity_flu": velocity})
    if not rows:
        raise ValueError(f"{path}: no samples")
    if any(b["timestamp_s"] <= a["timestamp_s"] for a, b in zip(rows, rows[1:])):
        raise ValueError(f"{path}: timestamps must be strictly increasing")
    return rows


def align(reference, candidate, tolerance_s):
    pairs = []
    j = 0
    for left in reference:
        while j + 1 < len(candidate) and abs(candidate[j + 1]["timestamp_s"] - left["timestamp_s"]) <= abs(candidate[j]["timestamp_s"] - left["timestamp_s"]):
            j += 1
        delta = candidate[j]["timestamp_s"] - left["timestamp_s"]
        if abs(delta) <= tolerance_s:
            pairs.append((left, candidate[j], delta))
    return pairs


def compare(upstream, jetson, tolerance_s=0.05):
    pairs = align(upstream, jetson, tolerance_s)
    if not pairs:
        raise ValueError("no samples aligned within tolerance")
    velocity_squared = 0.0
    yaw_squared = 0.0
    state_mismatches = land_mismatches = 0
    maximum_time_delta = 0.0
    reason_pairs = {}
    for left, right, delta in pairs:
        velocity_squared += sum((a - b) ** 2 for a, b in zip(left["velocity_flu"], right["velocity_flu"]))
        yaw_squared += (float(left["yaw_rate_flu"]) - float(right["yaw_rate_flu"])) ** 2
        state_mismatches += str(left["state"]) != str(right["state"])
        land_mismatches += bool(left["land_request"]) != bool(right["land_request"])
        maximum_time_delta = max(maximum_time_delta, abs(delta))
        key = f"{left['reason']} -> {right['reason']}"
        reason_pairs[key] = reason_pairs.get(key, 0) + 1
    count = len(pairs)
    return {
        "upstream_samples": len(upstream),
        "jetson_samples": len(jetson),
        "aligned_samples": count,
        "alignment_ratio": count / len(upstream),
        "maximum_time_delta_s": maximum_time_delta,
        "state_mismatch_count": state_mismatches,
        "state_mismatch_ratio": state_mismatches / count,
        "land_mismatch_count": land_mismatches,
        "velocity_vector_rmse_mps": math.sqrt(velocity_squared / count),
        "yaw_rate_rmse_rps": math.sqrt(yaw_squared / count),
        "reason_pair_counts": reason_pairs,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("upstream_jsonl")
    parser.add_argument("jetson_jsonl")
    parser.add_argument("--tolerance-s", type=float, default=0.05)
    parser.add_argument("--output", default="")
    args = parser.parse_args()
    report = compare(load_jsonl(args.upstream_jsonl), load_jsonl(args.jetson_jsonl), args.tolerance_s)
    encoded = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        Path(args.output).write_text(encoded + "\n", encoding="utf-8")
    print(encoded)


if __name__ == "__main__":
    main()
