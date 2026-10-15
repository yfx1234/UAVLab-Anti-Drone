#!/usr/bin/env python3
"""Measure live front/back detection timestamp alignment and gate pass rate."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from collections import Counter
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


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


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("stereo_timestamp_gate_check")
        self.samples: dict[str, list[dict]] = {"front": [], "back": []}
        self.handles = [
            self.create_subscription(
                String,
                "/stereo/front/detections_json",
                lambda message: self.receive("front", message),
                qos_profile_sensor_data,
            ),
            self.create_subscription(
                String,
                "/stereo/back/detections_json",
                lambda message: self.receive("back", message),
                qos_profile_sensor_data,
            ),
        ]

    def receive(self, source: str, message: String) -> None:
        arrival = time.monotonic()
        try:
            payload = json.loads(message.data)
            stamp = payload["header"]["stamp"]
            stamp_s = float(stamp["sec"]) + float(stamp["nanosec"]) / 1e9
            self.samples[source].append(
                {
                    "stamp_s": stamp_s,
                    "arrival_monotonic_s": arrival,
                    "sequence": int(payload["sequence"]),
                    "timestamp_source": str(payload.get("timestamp_source", "missing")),
                }
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"invalid {source} payload: {error}")


def summarize(samples: dict[str, list[dict]], gate_ms: float) -> dict:
    front = samples["front"]
    back = samples["back"]
    back_stamps = [item["stamp_s"] for item in back]
    deltas_ms = []
    for item in front:
        if not back_stamps:
            break
        nearest = min(back_stamps, key=lambda value: abs(value - item["stamp_s"]))
        deltas_ms.append(abs(nearest - item["stamp_s"]) * 1000.0)

    monotonic = {
        source: all(
            current["stamp_s"] > previous["stamp_s"]
            for previous, current in zip(items, items[1:])
        )
        for source, items in samples.items()
    }
    source_counts = {
        source: dict(Counter(item["timestamp_source"] for item in items))
        for source, items in samples.items()
    }
    pass_count = sum(delta <= gate_ms for delta in deltas_ms)
    return {
        "gate_ms": gate_ms,
        "frames": {source: len(items) for source, items in samples.items()},
        "timestamp_sources": source_counts,
        "strictly_monotonic": monotonic,
        "nearest_pair_count": len(deltas_ms),
        "within_gate_count": pass_count,
        "within_gate_fraction": pass_count / len(deltas_ms) if deltas_ms else 0.0,
        "absolute_delta_ms": {
            "min": min(deltas_ms) if deltas_ms else None,
            "mean": statistics.fmean(deltas_ms) if deltas_ms else None,
            "p50": percentile(deltas_ms, 0.50),
            "p90": percentile(deltas_ms, 0.90),
            "p95": percentile(deltas_ms, 0.95),
            "p99": percentile(deltas_ms, 0.99),
            "max": max(deltas_ms) if deltas_ms else None,
        },
        "result": (
            "PASS"
            if deltas_ms
            and pass_count == len(deltas_ms)
            and all(monotonic.values())
            and all(
                counts.get("gstreamer_source_pts", 0) == len(samples[source])
                for source, counts in source_counts.items()
            )
            else "FAIL"
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=20.0)
    parser.add_argument("--gate-ms", type=float, default=50.0)
    parser.add_argument("--json-output", required=True, type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = Collector()
    deadline = time.monotonic() + args.duration_sec
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        result = summarize(node.samples, args.gate_ms)
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result["result"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
