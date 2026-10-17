#!/usr/bin/env python3
"""Extract one frame near a requested timestamp from calibration videos."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--time-sec", type=float, default=4.0)
    args = parser.parse_args()

    capture = cv2.VideoCapture(str(args.input))
    if not capture.isOpened():
        raise RuntimeError(f"cannot open video: {args.input}")
    capture.set(cv2.CAP_PROP_POS_MSEC, args.time_sec * 1000.0)
    ok, frame = capture.read()
    actual_ms = capture.get(cv2.CAP_PROP_POS_MSEC)
    capture.release()
    if not ok or frame is None:
        raise RuntimeError(
            f"cannot read frame near {args.time_sec:.3f}s from {args.input}"
        )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(args.output), frame):
        raise RuntimeError(f"cannot write image: {args.output}")
    print(
        f"{args.input} -> {args.output} "
        f"requested={args.time_sec:.3f}s actual={actual_ms / 1000.0:.3f}s "
        f"shape={frame.shape[1]}x{frame.shape[0]}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
