#!/usr/bin/env python3
"""Small ROS 2 probe used only by the isolated PX4 SIH acceptance script."""

import argparse
import json
import time

import rclpy
from falcon_intercept_msgs.msg import GuidanceCommand, VehicleInterfaceStatus
from rclpy.node import Node


def wait_for_state(state: str, timeout_s: float) -> int:
    rclpy.init()
    node = Node("sitl_state_probe")
    latest = None

    def on_status(message):
        nonlocal latest
        latest = message

    node.create_subscription(VehicleInterfaceStatus, "/control/px4_status", on_status, 10)
    deadline = time.monotonic() + timeout_s
    matched = False
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if latest is None:
                continue
            if state == "armed":
                matched = latest.armed
            elif state == "prestreaming":
                matched = latest.armed and not latest.landed and latest.prestreaming
            elif state == "ready":
                matched = latest.ready_for_mission
            elif state == "landed":
                matched = latest.landed
            elif state == "takeover":
                matched = latest.takeover_latched
            if matched:
                break
        if latest is not None:
            print(json.dumps({
                "requested": state,
                "matched": matched,
                "armed": latest.armed,
                "landed": latest.landed,
                "offboard": latest.offboard,
                "failsafe": latest.failsafe,
                "takeover_latched": latest.takeover_latched,
                "prestreaming": latest.prestreaming,
                "ready_for_mission": latest.ready_for_mission,
                "position_ned": [
                    latest.position_ned.x,
                    latest.position_ned.y,
                    latest.position_ned.z,
                ],
                "reason": latest.reason,
            }, separators=(",", ":")))
        return 0 if matched else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


def publish_command(duration_s: float, vx: float, land_request: bool) -> int:
    rclpy.init()
    node = Node("sitl_guidance_probe")
    publisher = node.create_publisher(GuidanceCommand, "/intercept/guidance_command", 10)
    sequence = int(time.time() * 1000.0) & 0xFFFFFFFF
    deadline = time.monotonic() + duration_s
    try:
        while time.monotonic() < deadline:
            message = GuidanceCommand()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "pursuer_base_flu"
            message.sequence = sequence
            sequence = (sequence + 1) & 0xFFFFFFFF
            message.mission_state = message.STATE_LAND if land_request else message.STATE_APPROACH
            message.velocity_body_flu.x = float(vx)
            message.valid_for_s = 0.25
            message.valid = True
            message.land_request = land_request
            message.reason = "isolated SIH acceptance probe"
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    wait_parser = subparsers.add_parser("wait")
    wait_parser.add_argument(
        "state", choices=("armed", "prestreaming", "ready", "landed", "takeover")
    )
    wait_parser.add_argument("--timeout", type=float, default=20.0)
    command_parser = subparsers.add_parser("command")
    command_parser.add_argument("--duration", type=float, default=3.0)
    command_parser.add_argument("--vx", type=float, default=0.0)
    command_parser.add_argument("--land-request", action="store_true")
    args = parser.parse_args()
    if args.command == "wait":
        return wait_for_state(args.state, args.timeout)
    return publish_command(args.duration, args.vx, args.land_request)


if __name__ == "__main__":
    raise SystemExit(main())
