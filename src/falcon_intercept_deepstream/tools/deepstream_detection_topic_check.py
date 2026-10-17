#!/usr/bin/env python3
"""Verify one JSON detection message from each native DeepStream camera source."""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


EXPECTED = {
    "/search/detections_json": ("search_camera", 2560, 1440),
    "/stereo/front/detections_json": ("stereo_front_camera", 3840, 2160),
    "/stereo/back/detections_json": ("stereo_back_camera", 3840, 2160),
}


class DetectionTopicCheck(Node):
    def __init__(self) -> None:
        super().__init__("deepstream_detection_topic_check")
        self.received = {}
        self.subscription_handles = []
        for topic in EXPECTED:
            subscription = self.create_subscription(
                String,
                topic,
                lambda message, source_topic=topic: self._receive(source_topic, message),
                qos_profile_sensor_data,
            )
            self.subscription_handles.append(subscription)

    def _receive(self, topic: str, message: String) -> None:
        try:
            payload = json.loads(message.data)
            expected_frame, expected_width, expected_height = EXPECTED[topic]
            header = payload["header"]
            stamp = header["stamp"]
            if header["frame_id"] != expected_frame:
                raise ValueError(
                    f"frame_id={header['frame_id']!r}, expected {expected_frame!r}"
                )
            if payload["image_width"] != expected_width:
                raise ValueError(
                    f"image_width={payload['image_width']}, expected {expected_width}"
                )
            if payload["image_height"] != expected_height:
                raise ValueError(
                    f"image_height={payload['image_height']}, expected {expected_height}"
                )
            if int(stamp["sec"]) <= 0:
                raise ValueError("header timestamp is not populated")
            if not isinstance(payload["detections"], list):
                raise ValueError("detections is not a list")
            detection_summaries = []
            for detection in payload["detections"]:
                class_id = int(detection["class_id"])
                class_name = str(detection["class_name"])
                confidence = float(detection["confidence"])
                bbox = [float(value) for value in detection["bbox_xyxy"]]
                if len(bbox) != 4:
                    raise ValueError(f"bbox_xyxy has {len(bbox)} values")
                x1, y1, x2, y2 = bbox
                if not 0.0 <= confidence <= 1.0:
                    raise ValueError(f"confidence={confidence} is outside [0, 1]")
                if not (0.0 <= x1 < x2 <= expected_width):
                    raise ValueError(f"bbox x values are outside source width: {bbox}")
                if not (0.0 <= y1 < y2 <= expected_height):
                    raise ValueError(f"bbox y values are outside source height: {bbox}")
                detection_summaries.append(
                    {
                        "class_id": class_id,
                        "class_name": class_name,
                        "confidence": confidence,
                        "bbox_xyxy": bbox,
                    }
                )
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as error:
            self.get_logger().error(f"invalid message on {topic}: {error}")
            self.received[topic] = {"error": str(error)}
            return

        self.received[topic] = {
            "frame_id": header["frame_id"],
            "image_width": payload["image_width"],
            "image_height": payload["image_height"],
            "stamp": stamp,
            "detection_count": len(payload["detections"]),
            "detections": detection_summaries[:5],
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--require-positive", choices=sorted(EXPECTED))
    arguments = parser.parse_args()

    rclpy.init()
    node = DetectionTopicCheck()
    deadline = time.monotonic() + arguments.timeout
    try:
        while rclpy.ok():
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(node, timeout_sec=0.2)
            all_received = len(node.received) == len(EXPECTED)
            positive_received = (
                arguments.require_positive is None
                or node.received.get(arguments.require_positive, {}).get(
                    "detection_count", 0
                )
                > 0
            )
            if all_received and positive_received:
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

    missing = sorted(set(EXPECTED) - set(node.received))
    errors = {
        topic: result["error"]
        for topic, result in node.received.items()
        if "error" in result
    }
    print(json.dumps(node.received, ensure_ascii=False, sort_keys=True))
    if missing:
        print(f"DEEPSTREAM_TOPIC_CHECK_FAIL missing={','.join(missing)}", file=sys.stderr)
        return 1
    if errors:
        print(f"DEEPSTREAM_TOPIC_CHECK_FAIL invalid={errors}", file=sys.stderr)
        return 1
    if (
        arguments.require_positive is not None
        and node.received.get(arguments.require_positive, {}).get("detection_count", 0) == 0
    ):
        print(
            f"DEEPSTREAM_TOPIC_CHECK_FAIL no positive detection on "
            f"{arguments.require_positive}",
            file=sys.stderr,
        )
        return 1
    print("DEEPSTREAM_TOPIC_CHECK_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
