#!/usr/bin/env python3
"""Measure live three-camera detection and stereo-position continuity."""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from falcon_intercept_msgs.msg import TargetCameraPosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


TOPICS = {
"front": "/stereo/front/detections_json",
"back": "/stereo/back/detections_json",
    "search": "/search/detections_json",
}


class StreakStats:
    def __init__(self) -> None:
        self.frames = 0
        self.positive = 0
        self.current_positive = 0
        self.current_miss = 0
        self.longest_positive = 0
        self.longest_miss = 0

    def update(self, is_positive: bool) -> None:
        self.frames += 1
        if is_positive:
            self.positive += 1
            self.current_positive += 1
            self.current_miss = 0
            self.longest_positive = max(self.longest_positive, self.current_positive)
        else:
            self.current_miss += 1
            self.current_positive = 0
            self.longest_miss = max(self.longest_miss, self.current_miss)

    def summary(self) -> dict:
        return {
            "frames": self.frames,
            "positive_frames": self.positive,
            "positive_fraction": self.positive / self.frames if self.frames else 0.0,
            "longest_positive_streak_frames": self.longest_positive,
            "longest_miss_streak_frames": self.longest_miss,
        }


class Collector(Node):
    def __init__(self) -> None:
        super().__init__("perception_continuity_check")
        self.detections = {source: StreakStats() for source in TOPICS}
        self.positions = StreakStats()
        self.valid_ranges_m: list[float] = []
        self.decode_errors = 0
        self.handles = []
        for source, topic in TOPICS.items():
            self.handles.append(
                self.create_subscription(
                    String,
                    topic,
                    lambda message, source_name=source: self.receive_detection(
                        source_name, message
                    ),
                    qos_profile_sensor_data,
                )
            )
        self.handles.append(
            self.create_subscription(
                TargetCameraPosition,
                "/perception/target/camera_position",
                self.receive_position,
                qos_profile_sensor_data,
            )
        )

    def receive_detection(self, source: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
            self.detections[source].update(bool(payload["detections"]))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            self.decode_errors += 1

    def receive_position(self, message: TargetCameraPosition) -> None:
        valid = bool(message.valid)
        self.positions.update(valid)
        if valid:
            position = message.position_camera
            slant = math.sqrt(
                float(position.x) ** 2
                + float(position.y) ** 2
                + float(position.z) ** 2
            )
            if math.isfinite(slant):
                self.valid_ranges_m.append(slant)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-sec", type=float, default=30.0)
    parser.add_argument("--json-output", required=True, type=Path)
    args = parser.parse_args()

    rclpy.init()
    node = Collector()
    started = time.monotonic()
    try:
        deadline = started + args.duration_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        result = {
            "duration_sec": time.monotonic() - started,
            "detections": {
                source: stats.summary()
                for source, stats in node.detections.items()
            },
            "camera_position": node.positions.summary(),
            "valid_slant_range_m": {
                "count": len(node.valid_ranges_m),
                "min": min(node.valid_ranges_m) if node.valid_ranges_m else None,
                "mean": (
                    sum(node.valid_ranges_m) / len(node.valid_ranges_m)
                    if node.valid_ranges_m
                    else None
                ),
                "max": max(node.valid_ranges_m) if node.valid_ranges_m else None,
            },
            "decode_errors": node.decode_errors,
        }
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
