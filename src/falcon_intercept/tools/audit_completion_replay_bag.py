#!/usr/bin/env python3
"""Audit the isolated upstream FIRE-to-COMPLETE shadow replay."""

from __future__ import annotations

import argparse
import json
import sqlite3
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def load_topics(connection: sqlite3.Connection) -> dict[str, tuple[int, str]]:
    return {
        str(name): (int(topic_id), str(type_name))
        for topic_id, name, type_name in connection.execute(
            "SELECT id, name, type FROM topics"
        )
    }


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
    statuses = list(
        messages(connection, topics, "/tracking/pure_interceptor/status")
    )
    shots = list(messages(connection, topics, "/tracking/virtual_shot_event"))
    completion = list(messages(connection, topics, "/tracking/mission/complete"))
    shadow = [
        json.loads(message.data)
        for message in messages(
            connection, topics, "/intercept/upstream_px4_shadow"
        )
    ]

    states = []
    for status in statuses:
        if not states or states[-1] != status.state_name:
            states.append(str(status.state_name))
    required_states = [
        "TAKEOFF_SAFE",
        "SEARCH",
        "BEARING_TRACK",
        "DEPTH_HANDOVER",
        "DEPTH_ACQUIRE",
        "INTERCEPT",
        "EGRESS_REATTACK",
        "COMPLETE",
    ]
    state_order_ok = all(state in states for state in required_states)
    state_order_ok &= [states.index(state) for state in required_states] == sorted(
        states.index(state) for state in required_states
    )
    shadow_safe = bool(shadow) and all(
        item.get("mode") == "shadow"
        and item.get("would_forward") is False
        and item.get("vehicle_command_forwarded") is False
        for item in shadow
    )
    real_px4_inputs = sorted(
        name for name in topics if name.startswith("/fmu/in/")
    )
    result = {
        "bag": str(args.bag),
        "status_count": len(statuses),
        "state_sequence": states,
        "state_order_ok": state_order_ok,
        "shot_count": len(shots),
        "shot_ids": [int(message.shot_id) for message in shots],
        "completion_true_seen": any(bool(message.data) for message in completion),
        "maximum_capture_count": max(
            (int(message.capture_count) for message in statuses), default=0
        ),
        "safe_to_exit_offboard_seen": any(
            bool(message.safe_to_exit_offboard) for message in statuses
        ),
        "shadow_count": len(shadow),
        "shadow_safe": shadow_safe,
        "real_px4_input_topics_in_bag": real_px4_inputs,
    }
    result["passed"] = (
        bool(statuses)
        and state_order_ok
        and len(shots) == 1
        and result["shot_ids"] == [1]
        and result["completion_true_seen"]
        and result["maximum_capture_count"] == 1
        and result["safe_to_exit_offboard_seen"]
        and shadow_safe
        and not real_px4_inputs
    )
    print(json.dumps(result, indent=2, ensure_ascii=False))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
