#!/usr/bin/env python3
"""Locate dominant red target components in calibration still images."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def components(
    path: Path,
    limit: int,
    roi: tuple[int, int, int, int] | None,
    min_red: int | None,
    red_green_margin: int | None,
    red_blue_margin: int | None,
) -> dict:
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot read image: {path}")
    full_height, full_width = image.shape[:2]
    origin_x = 0
    origin_y = 0
    if roi is not None:
        origin_x, origin_y, roi_width, roi_height = roi
        image = image[
            origin_y : origin_y + roi_height,
            origin_x : origin_x + roi_width,
        ]
        if image.size == 0:
            raise RuntimeError(f"empty ROI {roi} for {path}")
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    mask = (
        ((hue <= 15) | (hue >= 160))
        & (saturation >= 65)
        & (value >= 28)
    )
    if min_red is not None:
        blue, green, red = cv2.split(image)
        red_i16 = red.astype(np.int16)
        mask &= red >= min_red
        if red_green_margin is not None:
            mask &= red_i16 - green.astype(np.int16) >= red_green_margin
        if red_blue_margin is not None:
            mask &= red_i16 - blue.astype(np.int16) >= red_blue_margin
    mask = mask.astype(np.uint8)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)),
    )
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (17, 17)),
    )
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    found = []
    for index in range(1, count):
        x, y, width, height, area = (int(value) for value in stats[index])
        if area < 50:
            continue
        component_y, component_x = np.nonzero(labels == index)
        top_band_height = max(5, int(round(0.03 * height)))
        top_band = component_y <= int(component_y.min()) + top_band_height
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
                "top_center_xy": [
                    float(np.median(component_x[top_band]) + origin_x),
                    float(np.median(component_y[top_band]) + origin_y),
                ],
            }
        )
    found.sort(key=lambda item: item["area_px"], reverse=True)
    return {
        "path": str(path),
        "width": int(full_width),
        "height": int(full_height),
        "roi_xywh": [
            origin_x,
            origin_y,
            int(image.shape[1]),
            int(image.shape[0]),
        ],
        "components": found[:limit],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("images", nargs="+", type=Path)
    parser.add_argument("--limit", type=int, default=8)
    parser.add_argument(
        "--min-red",
        type=int,
        help="optional minimum BGR red-channel value",
    )
    parser.add_argument(
        "--red-green-margin",
        type=int,
        help="optional minimum red-minus-green channel difference",
    )
    parser.add_argument(
        "--red-blue-margin",
        type=int,
        help="optional minimum red-minus-blue channel difference",
    )
    parser.add_argument(
        "--roi",
        nargs="*",
        default=[],
        help="optional x,y,width,height ROI for each image",
    )
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    if args.roi and len(args.roi) != len(args.images):
        parser.error("--roi must be omitted or contain one ROI per image")
    rois = []
    for text in args.roi:
        values = tuple(int(value) for value in text.split(","))
        if len(values) != 4 or min(values[2:]) <= 0:
            parser.error(f"invalid ROI: {text}")
        rois.append(values)
    if not rois:
        rois = [None] * len(args.images)
    result = {
        "images": [
            components(
                path,
                max(1, args.limit),
                roi,
                args.min_red,
                args.red_green_margin,
                args.red_blue_margin,
            )
            for path, roi in zip(args.images, rois)
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
