#!/usr/bin/env python3
"""Probe body-FLU axis conversion through the hardware shadow adapter."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import rclpy
from falcon_intercept_msgs.msg import GuidanceCommand
from rclpy.node import Node
from std_msgs.msg import String


CASES = (
    ("forward", (0.10, 0.0, 0.0), 0.0, False),
    ("left", (0.0, 0.10, 0.0), 0.0, False),
    ("up", (0.0, 0.0, 0.10), 0.0, False),
    ("yaw_ccw", (0.0, 0.0, 0.0), 0.10, False),
    ("land_intent", (0.0, 0.0, 0.0), 0.0, True),
)


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("px4_shadow_axis_probe")
        self.publisher = self.create_publisher(
            GuidanceCommand, "/intercept/guidance_command", 10
        )
        self.samples: dict[int, dict] = {}
        self.create_subscription(String, "/intercept/px4_shadow", self.on_shadow, 10)

    def on_shadow(self, message: String) -> None:
        payload = json.loads(message.data)
        sequence = payload.get("sequence")
        if sequence is not None:
            self.samples[int(sequence)] = payload


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--json-output", required=True, type=Path)
    parser.add_argument("--case-timeout-sec", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = Probe()
    results = []
    try:
        for sequence, (name, velocity, yaw_rate, land_request) in enumerate(
            CASES, start=1
        ):
            message = GuidanceCommand()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "pursuer_base_flu"
            message.sequence = sequence
            message.mission_state = (
                GuidanceCommand.STATE_LAND
                if land_request
                else GuidanceCommand.STATE_TRACK
            )
            message.velocity_body_flu.x = velocity[0]
            message.velocity_body_flu.y = velocity[1]
            message.velocity_body_flu.z = velocity[2]
            message.yaw_rate_flu = yaw_rate
            message.valid_for_s = 1.0
            message.valid = True
            message.land_request = land_request
            message.reason = f"shadow axis probe: {name}"

            deadline = time.monotonic() + args.case_timeout_sec
            while rclpy.ok() and time.monotonic() < deadline:
                node.publisher.publish(message)
                rclpy.spin_once(node, timeout_sec=0.1)
                if sequence in node.samples:
                    break
            payload = node.samples.get(sequence)
            results.append(
                {
                    "name": name,
                    "sequence": sequence,
                    "received": payload is not None,
                    "shadow": payload,
                }
            )

        result = {
            "all_received": all(case["received"] for case in results),
            "all_mode_shadow": all(
                case["shadow"] is not None
                and case["shadow"].get("mode") == "shadow"
                for case in results
            ),
            "cases": results,
        }
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(
            json.dumps(result, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["all_received"] and result["all_mode_shadow"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
