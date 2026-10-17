#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
import time

import rclpy
from falcon_intercept_msgs.msg import TargetCameraPosition
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from falcon_intercept.detection_utils import build_detection_json, stamp_to_dict


class ContractSmoke(Node):
    def __init__(self) -> None:
        super().__init__("target_camera_position_contract_smoke")
        self.front_pub = self.create_publisher(
            String, "/stereo/front/detections_json", qos_profile_sensor_data
        )
        self.back_pub = self.create_publisher(
            String, "/stereo/back/detections_json", qos_profile_sensor_data
        )
        self.camera_messages: list[TargetCameraPosition] = []
        self.create_subscription(
            TargetCameraPosition,
            "/perception/target/camera_position",
            self.camera_messages.append,
            qos_profile_sensor_data,
        )

    def publish_json(self, publisher, payload: str) -> None:
        message = String()
        message.data = payload
        publisher.publish(message)


def wait_for(node: ContractSmoke, predicate, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if predicate():
            return True
    return False


def main() -> int:
    rclpy.init()
    node = ContractSmoke()
    try:
        # Repeat until DDS discovery completes; the solver deduplicates the source frame.
        valid_front = build_detection_json(
            "stereo_front_camera", 100.0, (340.0, 240.0), confidence=0.8, sequence=1258
        )
        valid_back = build_detection_json(
            "stereo_back_camera", 100.01, (300.0, 240.0), confidence=0.9, sequence=2258
        )
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not node.camera_messages:
            node.publish_json(node.front_pub, valid_front)
            node.publish_json(node.back_pub, valid_back)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        if not node.camera_messages:
            print("CAMERA_POSITION_VALID_DISCOVERY_FAIL")
            return 1
        valid = node.camera_messages[-1]
        valid_stamp = float(valid.header.stamp.sec) + float(valid.header.stamp.nanosec) / 1e9
        checks = {
            "frame": valid.header.frame_id == "pursuer_camera_optical",
            "stamp": abs(valid_stamp - 100.0) < 1e-9,
            "sequence": valid.sequence == 1258,
            "position": all(
                math.isclose(actual, expected, abs_tol=1e-6)
                for actual, expected in zip(
                    (
                        valid.position_camera.x,
                        valid.position_camera.y,
                        valid.position_camera.z,
                    ),
                    (0.1, 0.0, 1.6),
                )
            ),
            "covariance": len(valid.position_covariance) == 9
            and all(math.isfinite(value) for value in valid.position_covariance),
            "confidence": math.isclose(valid.confidence, 0.8, abs_tol=1e-6),
            "valid": valid.valid is True,
        }
        if not all(checks.values()):
            print(f"CAMERA_POSITION_VALID_FAIL checks={checks} msg={valid}")
            return 1

        skew_front = build_detection_json(
            "stereo_front_camera", 100.1, (340.0, 240.0), confidence=0.8, sequence=1259
        )
        skew_back = build_detection_json(
            "stereo_back_camera", 100.2, (300.0, 240.0), confidence=0.9, sequence=2259
        )
        before_skew = len(node.camera_messages)
        deadline = time.monotonic() + 0.75
        while time.monotonic() < deadline:
            node.publish_json(node.front_pub, skew_front)
            node.publish_json(node.back_pub, skew_back)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        if len(node.camera_messages) != before_skew:
            print("CAMERA_POSITION_TIME_GATE_REJECT_FAIL")
            return 1

        recovery_front = build_detection_json(
            "stereo_front_camera", 100.3, (340.0, 240.0), confidence=0.8, sequence=1260
        )
        recovery_back = build_detection_json(
            "stereo_back_camera", 100.31, (300.0, 240.0), confidence=0.9, sequence=2260
        )
        before_recovery = len(node.camera_messages)
        deadline = time.monotonic() + 2.0
        while (
            time.monotonic() < deadline
            and not any(message.valid for message in node.camera_messages[before_recovery:])
        ):
            # Publish front first deliberately: recovery must not be blocked by a
            # prior invalid combination with the old back frame.
            node.publish_json(node.front_pub, recovery_front)
            rclpy.spin_once(node, timeout_sec=0.02)
            node.publish_json(node.back_pub, recovery_back)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        recovered = [
            message for message in node.camera_messages[before_recovery:] if message.valid
        ]
        if not recovered:
            print("CAMERA_POSITION_TIME_GATE_RECOVERY_FAIL")
            return 1

        empty_front = json.dumps(
            {
                "header": {
                    "stamp": stamp_to_dict(100.4),
                    "frame_id": "stereo_front_camera",
                },
                "sequence": 1261,
                "detections": [],
            },
            separators=(",", ":"),
        )
        empty_back = json.dumps(
            {
                "header": {
                    "stamp": stamp_to_dict(100.41),
                    "frame_id": "stereo_back_camera",
                },
                "sequence": 2261,
                "detections": [],
            },
            separators=(",", ":"),
        )
        before_empty = len(node.camera_messages)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and len(node.camera_messages) == before_empty:
            node.publish_json(node.front_pub, empty_front)
            node.publish_json(node.back_pub, empty_back)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(0.05)
        if len(node.camera_messages) == before_empty:
            print("CAMERA_POSITION_INVALID_DISCOVERY_FAIL")
            return 1
        invalid = node.camera_messages[-1]
        invalid_stamp = float(invalid.header.stamp.sec) + float(invalid.header.stamp.nanosec) / 1e9
        if not (
            invalid.valid is False
            and invalid.sequence >= 1261
            and abs(invalid_stamp - 100.4) < 1e-9
            and invalid.confidence == 0.0
            and invalid.position_camera.x == 0.0
            and invalid.position_camera.y == 0.0
            and invalid.position_camera.z == 0.0
        ):
            print(f"CAMERA_POSITION_INVALID_FAIL msg={invalid}")
            return 1
        print(
            "TARGET_CAMERA_POSITION_CONTRACT_SMOKE_PASS "
            "optical=PASS covariance=PASS sequence=PASS valid_invalid=PASS "
            "source_stamp=PASS time_gate_reject=PASS time_gate_recovery=PASS"
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
