#!/usr/bin/env python3
"""Quick numerical audit for a fixed stereo rig using one static image pair.

This is not a replacement for intrinsic/stereo calibration.  It is a field
check for the minimum acceptance items: readable frames, front/back disparity
sign, and gross vertical/roll misalignment.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def load_gray(
    path: Path,
    roi: tuple[int, int, int, int] | None,
) -> tuple[np.ndarray, tuple[int, int], tuple[int, int]]:
    image = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise RuntimeError(f"cannot read image: {path}")
    full_size = (int(image.shape[1]), int(image.shape[0]))
    if roi is None:
        return image, (0, 0), full_size
    x, y, width, height = roi
    cropped = image[y : y + height, x : x + width]
    if cropped.size == 0:
        raise RuntimeError(f"empty ROI {roi} for {path}")
    return cropped, (x, y), full_size


def parse_roi(text: str) -> tuple[int, int, int, int] | None:
    if not text:
        return None
    values = tuple(int(value) for value in text.split(","))
    if len(values) != 4 or min(values[2:]) <= 0:
        raise argparse.ArgumentTypeError("ROI must be x,y,width,height")
    return values


def create_detector():
    if hasattr(cv2, "SIFT_create"):
        return "SIFT", cv2.SIFT_create(nfeatures=10000), cv2.NORM_L2
    return "ORB", cv2.ORB_create(nfeatures=12000), cv2.NORM_HAMMING


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--front", type=Path, required=True)
    parser.add_argument("--back", type=Path, required=True)
    parser.add_argument("--front-roi", type=parse_roi)
    parser.add_argument("--back-roi", type=parse_roi)
    parser.add_argument("--ratio", type=float, default=0.72)
    parser.add_argument("--alignment-offset", type=float)
    parser.add_argument("--alignment-slope", type=float, default=0.0)
    parser.add_argument("--alignment-reference-u", type=float, default=0.0)
    parser.add_argument("--cy", type=float, default=0.0)
    parser.add_argument("--max-alignment-residual", type=float, default=8.0)
    parser.add_argument("--json-output", type=Path)
    args = parser.parse_args()

    front, left_offset, left_full_size = load_gray(args.front, args.left_roi)
    back, right_offset, right_full_size = load_gray(args.back, args.right_roi)
    if left_full_size != right_full_size:
        raise RuntimeError(
            f"stereo full image sizes differ: front={left_full_size}, back={right_full_size}"
        )
    if front.shape != back.shape:
        raise RuntimeError(
            f"stereo ROI sizes differ: front={front.shape[::-1]}, back={back.shape[::-1]}"
        )

    detector_name, detector, norm = create_detector()
    left_keypoints, left_descriptors = detector.detectAndCompute(front, None)
    right_keypoints, right_descriptors = detector.detectAndCompute(back, None)
    if left_descriptors is None or right_descriptors is None:
        raise RuntimeError("not enough texture for stereo feature matching")

    pairs = cv2.BFMatcher(norm).knnMatch(left_descriptors, right_descriptors, k=2)
    matches = [first for first, second in pairs if first.distance < args.ratio * second.distance]
    if len(matches) < 20:
        raise RuntimeError(f"too few ratio-test matches: {len(matches)}")

    left_points = np.float32([left_keypoints[item.queryIdx].pt for item in matches])
    right_points = np.float32([right_keypoints[item.trainIdx].pt for item in matches])
    left_points += np.float32(left_offset)
    right_points += np.float32(right_offset)
    _, mask = cv2.findFundamentalMat(
        left_points,
        right_points,
        cv2.FM_RANSAC,
        1.5,
        0.995,
        10000,
    )
    if mask is None:
        raise RuntimeError("fundamental-matrix RANSAC failed")

    inliers = mask.reshape(-1).astype(bool)
    delta = left_points[inliers] - right_points[inliers]
    if len(delta) < 15:
        raise RuntimeError(f"too few geometric inliers: {len(delta)}")

    dx = delta[:, 0]
    dy = delta[:, 1]
    # Fit the physical back -> physical front field-alignment transform. A small in-plane
    # rotation gives dv ~= offset + slope * (u_right - cx).
    centered_u = (
        right_points[inliers, 0].astype(np.float64)
        - 0.5 * float(left_full_size[0] - 1)
    )
    fit_mask = np.ones(len(dy), dtype=bool)
    slope = 0.0
    intercept = float(np.median(dy))
    for _ in range(4):
        design = np.column_stack(
            (centered_u[fit_mask], np.ones(int(fit_mask.sum()), dtype=np.float64))
        )
        slope, intercept = np.linalg.lstsq(
            design,
            dy[fit_mask].astype(np.float64),
            rcond=None,
        )[0]
        residual = dy.astype(np.float64) - (slope * centered_u + intercept)
        median_residual = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median_residual)))
        threshold = max(3.0, 3.5 * 1.4826 * mad)
        fit_mask = np.abs(residual - median_residual) <= threshold
        if int(fit_mask.sum()) < 15:
            break
    residual = dy.astype(np.float64) - (slope * centered_u + intercept)
    result = {
        "detector": detector_name,
        "image_width": int(left_full_size[0]),
        "image_height": int(left_full_size[1]),
        "left_roi_xywh": list(args.left_roi) if args.left_roi else None,
        "right_roi_xywh": list(args.right_roi) if args.right_roi else None,
        "left_keypoints": len(left_keypoints),
        "right_keypoints": len(right_keypoints),
        "ratio_matches": len(matches),
        "fundamental_inliers": int(inliers.sum()),
        "left_minus_right_dx_median_px": float(np.median(dx)),
        "left_minus_right_dx_p10_px": float(np.percentile(dx, 10)),
        "left_minus_right_dx_p90_px": float(np.percentile(dx, 90)),
        "left_minus_right_dy_median_px": float(np.median(dy)),
        "left_minus_right_dy_p10_px": float(np.percentile(dy, 10)),
        "left_minus_right_dy_p90_px": float(np.percentile(dy, 90)),
        "left_minus_right_abs_dy_p90_px": float(np.percentile(np.abs(dy), 90)),
        "vertical_offset_at_image_center_px": float(intercept),
        "vertical_slope_px_per_px": float(slope),
        "relative_roll_estimate_deg": float(np.degrees(np.arctan(slope))),
        "vertical_model_inliers": int(fit_mask.sum()),
        "vertical_model_abs_residual_p90_px": float(
            np.percentile(np.abs(residual[fit_mask]), 90)
        ),
        "positive_disparity_fraction": float(np.mean(dx > 0.0)),
    }
    if args.alignment_offset is not None:
        configured_expected_dy = (
            float(args.alignment_offset)
            + float(args.alignment_slope)
            * (right_points[inliers, 0] - float(args.alignment_reference_u))
        )
        configured_residual = dy - configured_expected_dy
        configured_mask = (
            np.abs(configured_residual) <= float(args.max_alignment_residual)
        )
        configured_disparity = (
            left_points[inliers, 0]
            - (
                right_points[inliers, 0]
                - float(args.alignment_slope)
                * (right_points[inliers, 1] - float(args.cy))
            )
        )
        accepted_disparity = configured_disparity[configured_mask]
        result["configured_alignment"] = {
            "offset_px": float(args.alignment_offset),
            "slope": float(args.alignment_slope),
            "reference_u_px": float(args.alignment_reference_u),
            "max_residual_px": float(args.max_alignment_residual),
            "accepted_matches": int(configured_mask.sum()),
            "residual_abs_p90_px": (
                float(np.percentile(np.abs(configured_residual[configured_mask]), 90))
                if configured_mask.any()
                else None
            ),
            "corrected_disparity_median_px": (
                float(np.median(accepted_disparity))
                if configured_mask.any()
                else None
            ),
            "corrected_disparity_p10_px": (
                float(np.percentile(accepted_disparity, 10))
                if configured_mask.any()
                else None
            ),
            "corrected_disparity_p90_px": (
                float(np.percentile(accepted_disparity, 90))
                if configured_mask.any()
                else None
            ),
            "corrected_disparity_values_px": [
                float(value) for value in np.sort(accepted_disparity)
            ],
            "accepted_match_samples": [
                {
                    "left_xy": [
                        float(left_points[inliers][configured_mask][index, 0]),
                        float(left_points[inliers][configured_mask][index, 1]),
                    ],
                    "right_xy": [
                        float(right_points[inliers][configured_mask][index, 0]),
                        float(right_points[inliers][configured_mask][index, 1]),
                    ],
                    "corrected_disparity_px": float(
                        accepted_disparity[index]
                    ),
                    "vertical_residual_px": float(
                        configured_residual[configured_mask][index]
                    ),
                }
                for index in range(len(accepted_disparity))
            ],
        }
    result["mapping_check"] = (
        "PHYSICAL_LEFT_RIGHT_ORDER_SUPPORTED"
        if result["left_minus_right_dx_median_px"] > 0.0
        and result["positive_disparity_fraction"] >= 0.7
        else "PHYSICAL_LEFT_RIGHT_ORDER_NOT_SUPPORTED"
    )

    text = json.dumps(result, ensure_ascii=False, indent=2)
    print(text)
    if args.json_output is not None:
        args.json_output.parent.mkdir(parents=True, exist_ok=True)
        args.json_output.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
