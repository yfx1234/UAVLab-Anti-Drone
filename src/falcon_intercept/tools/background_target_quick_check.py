#!/usr/bin/env python3
"""Locate a newly placed calibration target by static-background differencing."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def locate(
    before_path: Path,
    after_path: Path,
    limit: int,
    roi: tuple[int, int, int, int] | None,
) -> dict:
    before = cv2.imread(str(before_path), cv2.IMREAD_COLOR)
    after = cv2.imread(str(after_path), cv2.IMREAD_COLOR)
    if before is None or after is None:
        raise RuntimeError(f"cannot read pair: {before_path}, {after_path}")
    if before.shape != after.shape:
        raise RuntimeError(f"image sizes differ: {before.shape}, {after.shape}")

    origin_x = 0
    origin_y = 0
    if roi is not None:
        origin_x, origin_y, roi_width, roi_height = roi
        before = before[
            origin_y : origin_y + roi_height,
            origin_x : origin_x + roi_width,
        ]
        after = after[
            origin_y : origin_y + roi_height,
            origin_x : origin_x + roi_width,
        ]
        if before.size == 0 or after.size == 0:
            raise RuntimeError(f"empty ROI {roi} for {before_path}")

    # Correct modest global exposure changes per channel before differencing.
    normalized = before.astype(np.float32)
    target = after.astype(np.float32)
    sample = (slice(None, None, 16), slice(None, None, 16))
    for channel in range(3):
        source_values = normalized[sample[0], sample[1], channel].reshape(-1)
        target_values = target[sample[0], sample[1], channel].reshape(-1)
        design = np.column_stack((source_values, np.ones_like(source_values)))
        gain, offset = np.linalg.lstsq(design, target_values, rcond=None)[0]
        normalized[:, :, channel] = gain * normalized[:, :, channel] + offset

    delta = np.max(np.abs(target - normalized), axis=2)
    delta = cv2.GaussianBlur(np.clip(delta, 0, 255).astype(np.uint8), (7, 7), 0)
    otsu_threshold, mask = cv2.threshold(
        delta, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU
    )
    threshold = max(20.0, float(otsu_threshold))
    mask = (delta >= threshold).astype(np.uint8)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (25, 25)),
    )

    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    image_area = int(before.shape[0] * before.shape[1])
    for index in range(1, count):
        x, y, width, height, area = (int(value) for value in stats[index])
        if area < 100 or area > int(0.45 * image_area):
            continue
        found.append(
            {
                "area_px": area,
                "bbox_xywh": [
                    x + origin_x,
                    y + origin_y,
                    width,
                    height,
                ],
                "centroid_xy": [
                    float(centroids[index, 0] + origin_x),
                    float(centroids[index, 1] + origin_y),
                ],
            }
        )
    found.sort(key=lambda item: item["area_px"], reverse=True)
    return {
        "before": str(before_path),
        "after": str(after_path),
        "roi_xywh": [
            origin_x,
            origin_y,
            int(before.shape[1]),
            int(before.shape[0]),
        ],
        "difference_threshold": threshold,
        "components": found[:limit],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--before", nargs="+", type=Path, required=True)
    parser.add_argument("--after", nargs="+", type=Path, required=True)
    parser.add_argument(
        "--roi",
        nargs="*",
        default=[],
        help="optional x,y,width,height ROI for each before/after pair",
    )
    parser.add_argument("--limit", type=int, default=5)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()
    if len(args.before) != len(args.after):
        parser.error("--before and --after must contain the same number of images")
    if args.roi and len(args.roi) != len(args.before):
        parser.error("--roi must be omitted or contain one ROI per image pair")
    rois = []
    for text in args.roi:
        values = tuple(int(value) for value in text.split(","))
        if len(values) != 4 or min(values[2:]) <= 0:
            parser.error(f"invalid ROI: {text}")
        rois.append(values)
    if not rois:
        rois = [None] * len(args.before)

    result = {
        "pairs": [
            locate(before, after, max(1, args.limit), roi)
            for before, after, roi in zip(args.before, args.after, rois)
        ]
    }
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
