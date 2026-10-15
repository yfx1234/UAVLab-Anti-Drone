#!/usr/bin/env python3
"""Measure stereo disparity along the centerline of a solid-red target."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def solid_red_mask(path: Path) -> np.ndarray:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read image: {path}")
    blue, green, red = cv2.split(image)
    red_i16 = red.astype(np.int16)
    mask = (
        (red >= 75)
        & (red_i16 - green.astype(np.int16) >= 45)
        & (red_i16 - blue.astype(np.int16) >= 40)
    ).astype(np.uint8)
    return cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)),
    )


def row_center(mask: np.ndarray, y: float, x_min: int, x_max: int) -> tuple[float, int] | None:
    y0 = max(0, int(round(y)) - 2)
    y1 = min(mask.shape[0], int(round(y)) + 3)
    columns = np.nonzero(mask[y0:y1, x_min:x_max])[1]
    if columns.size < 20:
        return None
    unique_columns = np.unique(columns + x_min)
    return float(np.median(unique_columns)), int(unique_columns.size)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--left", required=True, type=Path)
    parser.add_argument("--right", required=True, type=Path)
    parser.add_argument("--left-x", default="700,1250")
    parser.add_argument("--right-x", default="500,1000")
    parser.add_argument("--right-y", default="650,900")
    parser.add_argument("--vertical-offset", type=float, required=True)
    parser.add_argument("--vertical-slope", type=float, required=True)
    parser.add_argument("--reference-u", type=float, required=True)
    parser.add_argument("--cy", type=float, required=True)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    left_mask = solid_red_mask(args.left)
    right_mask = solid_red_mask(args.right)
    left_x = tuple(int(value) for value in args.left_x.split(","))
    right_x = tuple(int(value) for value in args.right_x.split(","))
    right_y = tuple(int(value) for value in args.right_y.split(","))

    samples = []
    for y_right in range(right_y[0], right_y[1] + 1, 10):
        right = row_center(right_mask, y_right, *right_x)
        if right is None:
            continue
        u_right, right_width = right
        y_left = (
            y_right
            + args.vertical_offset
            + args.vertical_slope * (u_right - args.reference_u)
        )
        left = row_center(left_mask, y_left, *left_x)
        if left is None:
            continue
        u_left, left_width = left
        corrected_u_right = u_right - args.vertical_slope * (y_right - args.cy)
        samples.append(
            {
                "right_y": y_right,
                "expected_left_y": y_left,
                "left_u": u_left,
                "right_u": u_right,
                "left_width": left_width,
                "right_width": right_width,
                "corrected_observed_disparity_px": u_left - corrected_u_right,
            }
        )
    if not samples:
        raise RuntimeError("no valid centerline samples")
    disparities = np.asarray(
        [item["corrected_observed_disparity_px"] for item in samples],
        dtype=np.float64,
    )
    result = {
        "sample_count": len(samples),
        "disparity_median_px": float(np.median(disparities)),
        "disparity_p10_px": float(np.percentile(disparities, 10)),
        "disparity_p90_px": float(np.percentile(disparities, 90)),
        "samples": samples,
    }
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.json_output:
        args.json_output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
