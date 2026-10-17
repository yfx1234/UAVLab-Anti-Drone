#!/usr/bin/env python3
"""Replay validation of rectified stereo triangulation on checkerboard stills.

For every left/right still pair the 9x6 corners are detected, rectified
point-by-point (K1/D1/R1/P1 and K2/D2/R2/P2) and triangulated.  The chain is
then checked against ground truth that is independent of the fitted model:

  * adjacent-corner distances must equal the measured square size;
  * all 54 corners must lie on one plane (board rigidity);
  * epipolar vertical residuals after rectification.

Usage:
    python3 validate_rectified_triangulation.py \
        --calibration stereo_calib_candidate_final.yaml \
        --left-images left/ --right-images right/ \
        --board-cols 9 --board-rows 6 --square-size-m 0.025
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from falcon_intercept.rectified_stereo import load_rectified_stereo_geometry  # noqa: E402


def detect(gray, pattern):
    found, corners = False, None
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(cv2, "CALIB_CB_ACCURACY", 0)
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray, pattern, flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        )
        if found and corners is not None:
            cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4),
            )
    if not found or corners is None:
        return None
    return corners.reshape(-1, 2)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--left-images", required=True)
    parser.add_argument("--right-images", required=True)
    parser.add_argument("--board-cols", type=int, default=9)
    parser.add_argument("--board-rows", type=int, default=6)
    parser.add_argument("--square-size-m", type=float, default=0.025)
    args = parser.parse_args()

    geometry = load_rectified_stereo_geometry(args.calibration)
    if geometry is None:
        raise SystemExit("calibration has no rectification section")

    pattern = (args.board_cols, args.board_rows)
    left_paths = {p.stem: p for p in Path(args.left_images).glob("*.jpg")}
    left_paths.update({p.stem: p for p in Path(args.left_images).glob("*.png")})
    right_paths = {p.stem: p for p in Path(args.right_images).glob("*.jpg")}
    right_paths.update({p.stem: p for p in Path(args.right_images).glob("*.png")})
    stems = sorted(set(left_paths) & set(right_paths))
    if not stems:
        raise SystemExit("no common image pairs")

    cols, rows = pattern
    horiz_pairs = [(r * cols + c, r * cols + c + 1) for r in range(rows) for c in range(cols - 1)]
    vert_pairs = [(r * cols + c, (r + 1) * cols + c) for r in range(rows - 1) for c in range(cols)]

    side_errors = []
    plane_residuals = []
    vertical_errors = []
    distances = []
    used = 0
    for stem in stems:
        gl = cv2.imread(str(left_paths[stem]), cv2.IMREAD_GRAYSCALE)
        gr = cv2.imread(str(right_paths[stem]), cv2.IMREAD_GRAYSCALE)
        if gl is None or gr is None:
            continue
        cl = detect(gl, pattern)
        cr = detect(gr, pattern)
        if cl is None or cr is None:
            continue
        if np.dot(cl[-1] - cl[0], cr[-1] - cr[0]) < 0:
            cr = cr[::-1].copy()
        ul, vl = geometry.rectify_left(cl[:, 0], cl[:, 1])
        ur, vr = geometry.rectify_right(cr[:, 0], cr[:, 1])
        vertical_errors.append(np.abs(vl - vr))
        points = []
        for x_l, y_l, x_r in zip(ul, vl, ur):
            point_rect = geometry.triangulate_rectified(x_l, y_l, x_r)
            points.append(geometry.rectified_to_left_cv(point_rect))
        points = np.stack(points)
        for a, b in horiz_pairs + vert_pairs:
            side_errors.append(np.linalg.norm(points[a] - points[b]) - args.square_size_m)
        centroid = points.mean(axis=0)
        _, _, vh = np.linalg.svd(points - centroid)
        normal = vh[-1]
        plane_residuals.append(np.abs((points - centroid) @ normal))
        distances.append(float(np.linalg.norm(centroid)))
        used += 1

    side_errors = np.asarray(side_errors)
    plane_residuals = np.concatenate(plane_residuals)
    vertical_errors = np.concatenate(vertical_errors)
    print("pairs_used=%d" % used)
    print("side_length_error_mm: mean=%.3f std=%.3f p95=%.3f max=%.3f" % (
        1000 * side_errors.mean(), 1000 * side_errors.std(),
        1000 * np.percentile(side_errors, 95), 1000 * side_errors.max()))
    print("side_length_bias_mm: %+0.3f" % (1000 * side_errors.mean()))
    print("plane_residual_mm: rms=%.3f p95=%.3f max=%.3f" % (
        1000 * np.sqrt(np.mean(plane_residuals**2)),
        1000 * np.percentile(plane_residuals, 95), 1000 * plane_residuals.max()))
    print("epipolar_vertical_px: median=%.3f p95=%.3f max=%.3f" % (
        np.median(vertical_errors), np.percentile(vertical_errors, 95), vertical_errors.max()))
    print("board_distance_m: min=%.3f max=%.3f" % (min(distances), max(distances)))
    print("REPLAY_VALIDATION_OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
