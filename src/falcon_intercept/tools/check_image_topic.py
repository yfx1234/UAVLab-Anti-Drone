#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ImageTopicChecker(Node):
    def __init__(self, topic: str, duration_s: float) -> None:
        super().__init__("image_topic_checker")
        self.topic = topic
        self.duration_s = duration_s
        self.count = 0
        self.first_time: float | None = None
        self.last_time: float | None = None
        self.last_msg: Image | None = None
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.first_time is None:
            self.first_time = now
        self.last_time = now
        self.last_msg = msg
        self.count += 1

    def done(self) -> bool:
        return self.first_time is not None and time.monotonic() - self.first_time >= self.duration_s

    def report(self) -> int:
        if self.count == 0 or self.first_time is None or self.last_time is None or self.last_msg is None:
            print(f"topic={self.topic} count=0 status=NO_MESSAGES")
            return 1
        elapsed = max(1e-6, self.last_time - self.first_time)
        fps = (self.count - 1) / elapsed if self.count > 1 else 0.0
        stamp = self.last_msg.header.stamp
        print(
            "topic=%s count=%d fps=%.2f width=%d height=%d encoding=%s frame_id=%s stamp=%d.%09d"
            % (
                self.topic,
                self.count,
                fps,
                self.last_msg.width,
                self.last_msg.height,
                self.last_msg.encoding,
                self.last_msg.header.frame_id,
                stamp.sec,
                stamp.nanosec,
            )
        )
        return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/search/image_raw")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args()

    rclpy.init()
    node = ImageTopicChecker(args.topic, args.duration)
    start = time.monotonic()
    try:
        while rclpy.ok() and not node.done() and time.monotonic() - start < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
        return node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
