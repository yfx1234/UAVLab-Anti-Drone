#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class V4L2ImageLoopbackCheck(Node):
    def __init__(
        self,
        device_path: str,
        topic: str,
        width: int,
        height: int,
        fps: float,
        fourcc: str,
        duration_s: float,
    ) -> None:
        super().__init__("v4l2_image_loopback_check")
        self.topic = topic
        self.duration_s = duration_s
        self.read_count = 0
        self.publish_count = 0
        self.receive_count = 0
        self.first_receive_time: float | None = None
        self.last_receive_time: float | None = None
        self.last_msg: Image | None = None
        self.last_error = "none"

        self.cap = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
        self.cap.set(cv2.CAP_PROP_FPS, float(fps))

        self.pub = self.create_publisher(Image, topic, qos_profile_sensor_data)
        self.create_subscription(Image, topic, self._on_image, qos_profile_sensor_data)
        self.create_timer(1.0 / max(float(fps), 1.0), self._on_timer)

    def _on_timer(self) -> None:
        if not self.cap.isOpened():
            self.last_error = "camera_not_open"
            return
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.last_error = "read_failed"
            return
        self.read_count += 1
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "search_camera"
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(frame.strides[0])
        msg.data = frame.tobytes()
        self.pub.publish(msg)
        self.publish_count += 1
        self.last_error = "none"

    def _on_image(self, msg: Image) -> None:
        now = time.monotonic()
        if self.first_receive_time is None:
            self.first_receive_time = now
        self.last_receive_time = now
        self.last_msg = msg
        self.receive_count += 1

    def done(self) -> bool:
        return self.first_receive_time is not None and time.monotonic() - self.first_receive_time >= self.duration_s

    def report(self) -> int:
        if self.receive_count == 0 or self.last_msg is None:
            print(
                "loopback topic=%s reads=%d publishes=%d receives=0 last_error=%s status=NO_MESSAGES"
                % (self.topic, self.read_count, self.publish_count, self.last_error)
            )
            return 1
        elapsed = max(1e-6, float(self.last_receive_time or 0.0) - float(self.first_receive_time or 0.0))
        fps = (self.receive_count - 1) / elapsed if self.receive_count > 1 else 0.0
        print(
            "loopback topic=%s reads=%d publishes=%d receives=%d fps=%.2f width=%d height=%d encoding=%s frame_id=%s status=OK"
            % (
                self.topic,
                self.read_count,
                self.publish_count,
                self.receive_count,
                fps,
                self.last_msg.width,
                self.last_msg.height,
                self.last_msg.encoding,
                self.last_msg.header.frame_id,
            )
        )
        return 0

    def destroy_node(self) -> bool:
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--topic", default="/search/image_raw")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    rclpy.init()
    node = V4L2ImageLoopbackCheck(
        args.device,
        args.topic,
        args.width,
        args.height,
        args.fps,
        args.fourcc,
        args.duration,
    )
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
