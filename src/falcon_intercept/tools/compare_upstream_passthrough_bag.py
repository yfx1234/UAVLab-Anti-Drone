#!/usr/bin/env python3
"""Audit an isolated real-strike shadow bag for bridge and relay equivalence."""

from __future__ import annotations

import argparse
import json
import math
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


PAIRS = (
    (
        "/upstream/test_fmu/in/offboard_control_mode",
        "/intercept/replay_px4/offboard_control_mode",
    ),
    (
        "/upstream/test_fmu/in/trajectory_setpoint",
        "/intercept/replay_px4/trajectory_setpoint",
    ),
    (
        "/upstream/test_fmu/in/vehicle_attitude_setpoint",
        "/intercept/replay_px4/vehicle_attitude_setpoint",
    ),
)


def load_topics(connection: sqlite3.Connection) -> dict[str, tuple[int, str]]:
    return {
        str(name): (int(topic_id), str(type_name))
        for topic_id, name, type_name in connection.execute(
            "SELECT id, name, type FROM topics"
        )
    }


def payloads(connection: sqlite3.Connection, topic_id: int) -> list[bytes]:
    return [
        bytes(row[0])
        for row in connection.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY id", (topic_id,)
        )
    ]


def messages(connection, topics, topic_name):
    topic_id, type_name = topics[topic_name]
    message_type = get_message(type_name)
    for (data,) in connection.execute(
        "SELECT data FROM messages WHERE topic_id = ? ORDER BY id", (topic_id,)
    ):
        yield deserialize_message(data, message_type)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    databases = sorted(args.bag.glob("*.db3")) if args.bag.is_dir() else [args.bag]
    if len(databases) != 1:
        raise SystemExit(f"expected one db3 file, found {len(databases)}")
    connection = sqlite3.connect(str(databases[0]))
    topics = load_topics(connection)
    result = {"bag": str(args.bag), "pairs": {}, "observations": {}, "mission": {}}
    passed = True

    for source, relay in PAIRS:
        source_payloads = payloads(connection, topics[source][0])
        relay_payloads = payloads(connection, topics[relay][0])
        mismatch_indices = [
            index
            for index, (left, right) in enumerate(zip(source_payloads, relay_payloads))
            if left != right
        ]
        pair_passed = (
            len(source_payloads) == len(relay_payloads) and not mismatch_indices
        )
        passed &= pair_passed
        result["pairs"][source] = {
            "relay": relay,
            "source_count": len(source_payloads),
            "relay_count": len(relay_payloads),
            "cdr_mismatch_count": len(mismatch_indices),
            "passed": pair_passed,
        }

    search = list(messages(connection, topics, "/perception/search/observation"))
    depth = list(messages(connection, topics, "/perception/depth/observation"))
    search_valid = [
        message
        for message in search
        if message.visible and message.angles_valid and message.unit_los_valid
    ]
    depth_valid = [
        message for message in depth if message.visible and message.depth_valid
    ]
    search_finite = all(
        math.isfinite(value)
        for message in search_valid
        for value in (
            message.pixel_u,
            message.pixel_v,
            message.horizontal_angle_rad,
            message.vertical_angle_rad,
            message.unit_los_optical.x,
            message.unit_los_optical.y,
            message.unit_los_optical.z,
        )
    )
    depth_finite = all(
        math.isfinite(value) and value > 0.0
        for message in depth_valid
        for value in (message.depth_z_m, message.range_m)
    )
    result["observations"] = {
        "search_count": len(search),
        "search_valid_count": len(search_valid),
        "search_valid_finite": search_finite,
        "depth_count": len(depth),
        "depth_valid_count": len(depth_valid),
        "depth_valid_positive": depth_finite,
    }
    passed &= bool(search_valid and depth_valid and search_finite and depth_finite)

    statuses = list(messages(connection, topics, "/tracking/pure_interceptor/status"))
    state_sequence = []
    for message in statuses:
        if not state_sequence or state_sequence[-1] != message.state_name:
            state_sequence.append(str(message.state_name))
    result["mission"] = {
        "status_count": len(statuses),
        "state_sequence": state_sequence,
        "required_capture_counts": sorted(
            {int(message.required_capture_count) for message in statuses}
        ),
        "maximum_capture_count": max(
            (int(message.capture_count) for message in statuses), default=0
        ),
        "safe_to_exit_offboard_seen": any(
            bool(message.safe_to_exit_offboard) for message in statuses
        ),
    }
    passed &= bool(statuses)
    passed &= result["mission"]["required_capture_counts"] == [1]

    shadow = list(messages(connection, topics, "/intercept/upstream_px4_shadow"))
    shadow_json = [json.loads(message.data) for message in shadow]
    shadow_safe = all(
        item.get("mode") == "shadow"
        and item.get("vehicle_command_forwarded") is False
        and item.get("would_forward") is False
        for item in shadow_json
    )
    result["shadow"] = {"count": len(shadow_json), "safe": shadow_safe}
    passed &= bool(shadow_json and shadow_safe)
    result["passed"] = passed
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
