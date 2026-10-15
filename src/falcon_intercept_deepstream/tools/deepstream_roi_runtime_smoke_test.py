#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


class RoiRuntimeSmoke(Node):
    def __init__(self) -> None:
        super().__init__("deepstream_roi_runtime_smoke")
        self.publisher = self.create_publisher(String, "/stereo/rois_json", qos_profile_sensor_data)
        self.frames: dict[str, dict] = {}
        self.status: dict | None = None
        self.create_subscription(
            String,
            "/stereo/front/detections_json",
            lambda msg: self._store_frame("left", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String,
            "/stereo/back/detections_json",
            lambda msg: self._store_frame("right", msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            String, "/stereo/roi_runtime_status", self._store_status, 10
        )

    def _store_frame(self, stream: str, msg: String) -> None:
        try:
            self.frames[stream] = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _store_status(self, msg: String) -> None:
        try:
            self.status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def publish_roi(self, left: list[int], right: list[int]) -> None:
        now = self.get_clock().now().to_msg()
        payload = {
            "header": {
                "stamp": {"sec": int(now.sec), "nanosec": int(now.nanosec)},
                "frame_id": "search_camera",
            },
            "valid": True,
            "reason": "synthetic_runtime_smoke",
            "rois": {
                "left": dict(zip(("x1", "y1", "x2", "y2"), left)),
                "right": dict(zip(("x1", "y1", "x2", "y2"), right)),
            },
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self.publisher.publish(msg)


def wait_for(node: RoiRuntimeSmoke, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def wait_for_roi_subscriber(node: RoiRuntimeSmoke, timeout_s: float) -> bool:
    """Wait for DDS endpoint discovery before starting the short-lived ROI phases."""
    return wait_for(node, lambda: node.publisher.get_subscription_count() > 0, timeout_s)


def publish_until_active(
    node: RoiRuntimeSmoke, left: list[int], right: list[int], timeout_s: float
) -> bool:
    """Keep the ROI alive while waiting for both inference streams to report it."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        node.publish_roi(left, right)
        rclpy.spin_once(node, timeout_sec=0.05)
        if frame_has_roi(node, "left", left, True) and frame_has_roi(
            node, "right", right, True
        ):
            return True
    return False


def frame_has_roi(node: RoiRuntimeSmoke, stream: str, expected: list[int], active: bool) -> bool:
    runtime = node.frames.get(stream, {}).get("roi_runtime", {})
    roi = runtime.get("roi", {})
    actual = [roi.get(key) for key in ("x1", "y1", "x2", "y2")]
    return runtime.get("active") is active and actual == expected


def main() -> int:
    rclpy.init()
    node = RoiRuntimeSmoke()
    try:
        if not wait_for_roi_subscriber(node, 8.0):
            print(
                "ROI_DISCOVERY_FAIL "
                f"subscriber_count={node.publisher.get_subscription_count()}"
            )
            return 1

        phases = [
            ([100, 200, 1100, 1000], [120, 220, 1120, 1020]),
            ([800, 400, 1800, 1400], [820, 420, 1820, 1420]),
        ]
        for phase_index, (left, right) in enumerate(phases, start=1):
            if not publish_until_active(node, left, right, 6.0):
                print(f"ROI_SMOKE_FAIL phase={phase_index} frames={node.frames} status={node.status}")
                return 1

        # Stop publishing. The runtime watchdog must return both streams to full-frame mode.
        if not wait_for(
            node,
            lambda: frame_has_roi(node, "left", [0, 0, 3840, 2160], False)
            and frame_has_roi(node, "right", [0, 0, 3840, 2160], False),
            4.0,
        ):
            print(f"ROI_TIMEOUT_FAIL frames={node.frames} status={node.status}")
            return 1
        if node.status is None or node.status.get("reason") != "roi_timeout":
            print(f"ROI_STATUS_FAIL status={node.status}")
            return 1
        print("DEEPSTREAM_ROI_RUNTIME_SMOKE_PASS dynamic=2 timeout_fallback=PASS")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
