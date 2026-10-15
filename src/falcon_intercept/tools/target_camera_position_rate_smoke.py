#!/usr/bin/env python3
from __future__ import annotations

import math
import json
import sys
import time

import rclpy
from falcon_intercept_msgs.msg import TargetCameraPosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class RateSmoke(Node):
    def __init__(self) -> None:
        super().__init__("target_camera_position_rate_smoke")
        self.samples: list[tuple[float, int, float, bool, tuple[float, float, float]]] = []
        self.source_samples: dict[str, list[tuple[int, float]]] = {
            "front": [],
            "back": [],
        }
        self.create_subscription(
            TargetCameraPosition,
            "/perception/target/camera_position",
            self._on_message,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/stereo/front/detections_json",
            lambda msg: self._on_source("front", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/stereo/back/detections_json",
            lambda msg: self._on_source("back", msg),
            qos_profile_sensor_data,
        )

    def _on_source(self, stream: str, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            stamp = payload["header"]["stamp"]
            stamp_s = float(stamp["sec"]) + float(stamp["nanosec"]) / 1e9
            self.source_samples[stream].append((int(payload["sequence"]), stamp_s))
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            pass

    def _on_message(self, msg: TargetCameraPosition) -> None:
        source_stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) / 1e9
        self.samples.append(
            (
                time.monotonic(),
                int(msg.sequence),
                source_stamp,
                bool(msg.valid),
                (
                    float(msg.position_camera.x),
                    float(msg.position_camera.y),
                    float(msg.position_camera.z),
                ),
            )
        )


def main() -> int:
    rclpy.init()
    node = RateSmoke()
    try:
        deadline = time.monotonic() + 7.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if len(node.samples) < 2:
            print(f"CAMERA_POSITION_RATE_NO_DATA count={len(node.samples)}")
            return 1
        elapsed = node.samples[-1][0] - node.samples[0][0]
        rate_hz = (len(node.samples) - 1) / elapsed if elapsed > 0.0 else 0.0
        sequences = [sample[1] for sample in node.samples]
        source_stamps = [sample[2] for sample in node.samples]
        if rate_hz < 20.0:
            print(f"CAMERA_POSITION_RATE_FAIL rate_hz={rate_hz:.3f} count={len(node.samples)}")
            return 1
        if any(current <= previous for previous, current in zip(sequences, sequences[1:])):
            print(f"CAMERA_POSITION_SEQUENCE_FAIL first={sequences[:10]}")
            return 1
        for stream, samples in node.source_samples.items():
            if len(samples) < 2:
                print(f"CAMERA_SOURCE_NO_DATA stream={stream} count={len(samples)}")
                return 1
            if any(
                current[0] <= previous[0] or current[1] <= previous[1]
                for previous, current in zip(samples, samples[1:])
            ):
                print(f"CAMERA_SOURCE_ORDER_FAIL stream={stream} first={samples[:12]}")
                return 1
        if any(current <= previous for previous, current in zip(source_stamps, source_stamps[1:])):
            source_first = {
                name: values[:8] for name, values in node.source_samples.items()
            }
            source_last = {
                name: values[-12:] for name, values in node.source_samples.items()
            }
            source_duplicate_counts = {
                name: sum(
                    current[1] <= previous[1]
                    for previous, current in zip(values, values[1:])
                )
                for name, values in node.source_samples.items()
            }
            print(
                "CAMERA_POSITION_STAMP_FAIL "
                f"first={list(zip(sequences[:12], source_stamps[:12]))} "
                f"source_first={source_first} source_last={source_last} "
                f"source_duplicate_counts={source_duplicate_counts}"
            )
            return 1
        for _arrival, _sequence, _stamp, valid, position in node.samples:
            if not all(math.isfinite(value) for value in position):
                print("CAMERA_POSITION_NONFINITE_FAIL")
                return 1
            if valid and position[2] <= 0.0:
                print(f"CAMERA_POSITION_VALID_Z_FAIL position={position}")
                return 1
            if not valid and position != (0.0, 0.0, 0.0):
                print(f"CAMERA_POSITION_INVALID_REUSE_FAIL position={position}")
                return 1
        invalid_count = sum(1 for sample in node.samples if not sample[3])
        print(
            "TARGET_CAMERA_POSITION_RATE_SMOKE_PASS "
            f"rate_hz={rate_hz:.2f} count={len(node.samples)} invalid={invalid_count} "
            "sequence=STRICT stamp=STRICT source_order=STRICT"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
