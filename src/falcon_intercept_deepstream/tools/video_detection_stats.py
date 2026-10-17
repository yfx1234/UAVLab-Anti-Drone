#!/usr/bin/env python3
"""Collect per-source DeepStream detection statistics for recorded-video replay."""

import argparse
import json
import math
import signal
import statistics
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


TOPICS = {
"front": "/stereo/front/detections_json",
"back": "/stereo/back/detections_json",
    "search": "/search/detections_json",
}


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


class SourceStats:
    def __init__(self):
        self.frames = 0
        self.positive_frames = 0
        self.detections = 0
        self.confidences = []
        self.bbox_area_ratios = []
        self.first_sequence = None
        self.last_sequence = None
        self.first_stamp = None
        self.last_stamp = None
        self.current_positive_streak = 0
        self.current_miss_streak = 0
        self.longest_positive_streak = 0
        self.longest_miss_streak = 0
        self.decode_errors = 0

    def update(self, payload):
        detections = payload["detections"]
        width = float(payload["image_width"])
        height = float(payload["image_height"])
        sequence = int(payload["sequence"])
        stamp = payload["header"]["stamp"]
        stamp_seconds = float(stamp["sec"]) + float(stamp["nanosec"]) / 1e9

        self.frames += 1
        self.first_sequence = sequence if self.first_sequence is None else self.first_sequence
        self.last_sequence = sequence
        self.first_stamp = stamp_seconds if self.first_stamp is None else self.first_stamp
        self.last_stamp = stamp_seconds
        if detections:
            self.positive_frames += 1
            self.current_positive_streak += 1
            self.current_miss_streak = 0
            self.longest_positive_streak = max(
                self.longest_positive_streak, self.current_positive_streak
            )
        else:
            self.current_miss_streak += 1
            self.current_positive_streak = 0
            self.longest_miss_streak = max(
                self.longest_miss_streak, self.current_miss_streak
            )

        self.detections += len(detections)
        for detection in detections:
            self.confidences.append(float(detection["confidence"]))
            x1, y1, x2, y2 = map(float, detection["bbox_xyxy"])
            self.bbox_area_ratios.append(((x2 - x1) * (y2 - y1)) / (width * height))

    def summary(self):
        duration = None
        if self.first_stamp is not None and self.last_stamp is not None:
            duration = max(0.0, self.last_stamp - self.first_stamp)
        result = {
            "frames": self.frames,
            "positive_frames": self.positive_frames,
            "positive_frame_ratio": self.positive_frames / self.frames if self.frames else 0.0,
            "detections": self.detections,
            "first_sequence": self.first_sequence,
            "last_sequence": self.last_sequence,
            "stream_duration_sec": duration,
            "observed_fps": ((self.frames - 1) / duration) if duration and duration > 0 else None,
            "longest_positive_streak_frames": self.longest_positive_streak,
            "longest_miss_streak_frames": self.longest_miss_streak,
            "decode_errors": self.decode_errors,
        }
        for name, values in (
            ("confidence", self.confidences),
            ("bbox_area_ratio", self.bbox_area_ratios),
        ):
            result[name] = {
                "count": len(values),
                "min": min(values) if values else None,
                "mean": statistics.fmean(values) if values else None,
                "p50": percentile(values, 0.50),
                "p95": percentile(values, 0.95),
                "max": max(values) if values else None,
            }
        return result


class Collector(Node):
    def __init__(self, jsonl_path):
        super().__init__("deepstream_video_detection_stats")
        self.started_monotonic = time.monotonic()
        self.stats = {name: SourceStats() for name in TOPICS}
        self.jsonl = jsonl_path.open("w", encoding="utf-8")
        self.subscription_handles = []
        for source, topic in TOPICS.items():
            self.subscription_handles.append(
                self.create_subscription(
                    String,
                    topic,
                    lambda message, source_name=source: self.receive(source_name, message),
                    qos_profile_sensor_data,
                )
            )

    def receive(self, source, message):
        try:
            payload = json.loads(message.data)
            self.stats[source].update(payload)
            self.jsonl.write(
                json.dumps({"source": source, "payload": payload}, ensure_ascii=False) + "\n"
            )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.stats[source].decode_errors += 1
            self.get_logger().error(f"invalid {source} detection message: {error}")

    def finish(self):
        self.jsonl.flush()
        self.jsonl.close()
        return {
            "wall_duration_sec": time.monotonic() - self.started_monotonic,
            "sources": {name: stats.summary() for name, stats in self.stats.items()},
            "interpretation": (
                "Observational replay statistics only; without frame-level ground truth they "
                "cannot establish precision, recall, false-positive rate, or mAP."
            ),
        }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", required=True)
    args = parser.parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = Collector(output_dir / "detections.jsonl")
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        while rclpy.ok() and not stop:
            rclpy.spin_once(node, timeout_sec=0.25)
    finally:
        summary = node.finish()
        (output_dir / "detection_summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
