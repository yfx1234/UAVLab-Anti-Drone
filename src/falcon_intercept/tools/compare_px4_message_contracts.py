#!/usr/bin/env python3
"""Compare PX4 uORB and px4_msgs message source trees without modifying them."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Dict


def normalized(data: bytes) -> bytes:
    return data.replace(b"\r\n", b"\n").rstrip() + b"\n"


def load_messages(root: Path) -> Dict[str, bytes]:
    if not root.is_dir():
        raise FileNotFoundError(f"message directory does not exist: {root}")
    return {
        path.relative_to(root).as_posix(): normalized(path.read_bytes())
        for path in sorted(root.rglob("*.msg"))
    }


def tree_sha256(messages: Dict[str, bytes]) -> str:
    digest = hashlib.sha256()
    for relative, data in sorted(messages.items()):
        name = relative.encode("utf-8")
        digest.update(len(name).to_bytes(4, "big"))
        digest.update(name)
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("px4_msg_dir", type=Path, help="PX4-Autopilot/msg directory")
    parser.add_argument("px4_msgs_dir", type=Path, help="px4_msgs/msg directory")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="also fail when either tree contains a message absent from the other",
    )
    args = parser.parse_args()

    firmware = load_messages(args.px4_msg_dir.resolve())
    ros = load_messages(args.px4_msgs_dir.resolve())
    common = sorted(firmware.keys() & ros.keys())
    different = sorted(name for name in common if firmware[name] != ros[name])
    firmware_only = sorted(firmware.keys() - ros.keys())
    ros_only = sorted(ros.keys() - firmware.keys())

    result = {
        "px4_count": len(firmware),
        "px4_msgs_count": len(ros),
        "common_count": len(common),
        "identical_common_count": len(common) - len(different),
        "different": different,
        "px4_only": firmware_only,
        "px4_msgs_only": ros_only,
        "px4_tree_sha256": tree_sha256(firmware),
        "px4_msgs_tree_sha256": tree_sha256(ros),
        "strict": args.strict,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))

    if different:
        return 1
    if args.strict and (firmware_only or ros_only):
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
