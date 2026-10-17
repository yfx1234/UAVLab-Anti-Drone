#!/usr/bin/env python3
"""Project a measured search-camera target box and verify observed stereo points."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from falcon_intercept.perception_geometry import (
    load_camera_geometry,
    project_search_bbox_to_stereo_rois,
)


def parse_values(text: str, count: int) -> tuple[float, ...]:
    values = tuple(float(value) for value in text.split(","))
    if len(values) != count:
        raise argparse.ArgumentTypeError(f"expected {count} comma-separated values")
    return values


def roi_payload(roi, observed: tuple[float, float] | None) -> dict | None:
    if roi is None:
        return None
    result = {
        "xyxy": [roi.x1, roi.y1, roi.x2, roi.y2],
        "width": roi.width,
        "height": roi.height,
    }
    if observed is not None:
        result["observed_xy"] = list(observed)
        result["observed_inside"] = (
            roi.x1 <= observed[0] < roi.x2
            and roi.y1 <= observed[1] < roi.y2
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument("--extrinsics", type=Path, required=True)
    parser.add_argument(
        "--search-bbox", type=lambda text: parse_values(text, 4), required=True
    )
    parser.add_argument("--depth-min", type=float, required=True)
    parser.add_argument("--depth-max", type=float, required=True)
    parser.add_argument("--margin-px", type=float, default=32.0)
    parser.add_argument("--observed-front", type=lambda text: parse_values(text, 2))
    parser.add_argument("--observed-back", type=lambda text: parse_values(text, 2))
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    geometry = load_camera_geometry(args.calibration, args.extrinsics)
    rois = project_search_bbox_to_stereo_rois(
        args.search_bbox,
        geometry,
        depth_min_m=args.depth_min,
        depth_max_m=args.depth_max,
        margin_px=args.margin_px,
    )
    result = {
        "search_bbox_xyxy": list(args.search_bbox),
        "depth_min_m": args.depth_min,
        "depth_max_m": args.depth_max,
        "margin_px": args.margin_px,
        "front": roi_payload(rois["front"], args.observed_front),
        "back": roi_payload(rois["back"], args.observed_back),
    }
    result["pass"] = (
        result["front"] is not None
        and result["back"] is not None
        and result["front"].get("observed_inside", True)
        and result["back"].get("observed_inside", True)
    )
    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(text + "\n", encoding="utf-8")
    return 0 if result["pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
