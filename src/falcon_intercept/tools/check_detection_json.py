#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys

from falcon_intercept.detection_utils import parse_detections_json


def main() -> int:
    parser = argparse.ArgumentParser(description="Validate a falcon_intercept detection JSON payload.")
    parser.add_argument("json_text", help="Detection JSON string")
    args = parser.parse_args()
    parsed = parse_detections_json(args.json_text)
    print(f"frame_id={parsed.frame_id} stamp={parsed.stamp_s} detections={len(parsed.detections)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
