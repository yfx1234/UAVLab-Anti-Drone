#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

import cv2

from falcon_intercept.detection_utils import parse_detections_json
from falcon_intercept.detector_node import _empty_detection_json


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--frame-id", default="search_camera")
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--fourcc", default="MJPG")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    if args.fourcc:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*args.fourcc[:4]))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    try:
        if not cap.isOpened():
            print(f"device={args.device} status=CAMERA_NOT_OPEN")
            return 1
        ok, frame = cap.read()
        if not ok or frame is None:
            print(f"device={args.device} status=READ_FAILED")
            return 1
        text = _empty_detection_json(args.frame_id, time.time())
        parsed = parse_detections_json(text)
        print(
            "device=%s frame_shape=%s frame_id=%s detections=%d status=OK"
            % (args.device, tuple(frame.shape), parsed.frame_id, len(parsed.detections))
        )
        print(text)
        return 0
    finally:
        cap.release()


if __name__ == "__main__":
    raise SystemExit(main())
