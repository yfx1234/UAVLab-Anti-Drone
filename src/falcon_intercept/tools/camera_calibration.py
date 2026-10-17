#!/usr/bin/env python3
"""Offline checkerboard calibration tools for the assembled camera rig.

The tool deliberately consumes image directories instead of opening a camera
device. Capture can therefore use the final Jetson Argus/USB pipeline and the
calibration is performed on the exact output mode used at runtime.

Subcommands:
  mono    calibrate one camera and write/update a calibration YAML section
  stereo  calibrate the front/back stereo pair and write R/T + rectification
  rig     estimate search->stereo extrinsics from a common board
  rectify create side-by-side rectified preview images

OpenCV camera coordinates are converted to the project convention when rig
extrinsics are written: x-forward, y-right, z-up.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np
import yaml


IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


def image_map(directory: Path) -> dict[str, Path]:
    paths = sorted(
        path for path in directory.rglob("*")
        if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES
    )
    if not paths:
        raise ValueError(f"no images found under {directory}")
    result: dict[str, Path] = {}
    for path in paths:
        result.setdefault(path.stem, path)
    return result


def board_object_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    if cols < 2 or rows < 2 or square_size_m <= 0.0:
        raise ValueError("board dimensions must be >= 2 and square_size_m must be positive")
    points = np.zeros((rows * cols, 3), dtype=np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= float(square_size_m)
    return points


def detect_corners(image: np.ndarray, pattern: tuple[int, int]) -> np.ndarray | None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    corners: np.ndarray | None = None
    found = False
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = getattr(cv2, "CALIB_CB_EXHAUSTIVE", 0) | getattr(cv2, "CALIB_CB_ACCURACY", 0)
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags)
    if not found:
        found, corners = cv2.findChessboardCorners(
            gray,
            pattern,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE,
        )
        if found and corners is not None:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                40,
                1e-4,
            )
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    if not found or corners is None:
        return None
    return np.asarray(corners, dtype=np.float32).reshape(-1, 2)


def collect_views(
    paths: dict[str, Path],
    object_points: np.ndarray,
    pattern: tuple[int, int],
) -> tuple[list[np.ndarray], list[np.ndarray], tuple[int, int], list[str]]:
    object_views: list[np.ndarray] = []
    image_views: list[np.ndarray] = []
    accepted: list[str] = []
    image_size: tuple[int, int] | None = None
    for stem, path in sorted(paths.items()):
        image = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if image is None:
            continue
        height, width = image.shape[:2]
        current_size = (width, height)
        if image_size is None:
            image_size = current_size
        if current_size != image_size:
            raise ValueError(
                f"mixed image sizes in {path.parent}: expected {image_size}, got {current_size}"
            )
        corners = detect_corners(image, pattern)
        if corners is None or len(corners) != len(object_points):
            continue
        object_views.append(object_points.copy())
        image_views.append(corners)
        accepted.append(stem)
    if image_size is None:
        raise ValueError("could not read any images")
    return object_views, image_views, image_size, accepted


def load_yaml(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)


def matrix_list(value: np.ndarray) -> list[list[float]]:
    return np.asarray(value, dtype=float).tolist()


def vector_list(value: np.ndarray) -> list[float]:
    return np.asarray(value, dtype=float).reshape(-1).tolist()


def mono(args: argparse.Namespace) -> None:
    object_points = board_object_points(args.board_cols, args.board_rows, args.square_size_m)
    paths = image_map(Path(args.images))
    object_views, image_views, image_size, accepted = collect_views(
        paths, object_points, (args.board_cols, args.board_rows)
    )
    if len(object_views) < args.min_views:
        raise RuntimeError(f"only {len(object_views)} valid views; need at least {args.min_views}")
    rms, camera_matrix, distortion, rvecs, tvecs = cv2.calibrateCamera(
        object_views,
        [view.reshape(-1, 1, 2) for view in image_views],
        image_size,
        None,
        None,
    )
    errors: list[float] = []
    for obj, img, rvec, tvec in zip(object_views, image_views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, distortion)
        errors.append(float(np.mean(np.linalg.norm(projected.reshape(-1, 2) - img, axis=1))))
    section = {
        "calibration_status": "measured",
        "calibration_method": "opencv_chessboard",
        "image_width": image_size[0],
        "image_height": image_size[1],
        "fx": float(camera_matrix[0, 0]),
        "fy": float(camera_matrix[1, 1]),
        "cx": float(camera_matrix[0, 2]),
        "cy": float(camera_matrix[1, 2]),
        "distortion": vector_list(distortion),
        "rms_reprojection_error_px": float(rms),
        "mean_corner_error_px": float(np.mean(errors)),
        "max_corner_error_px": float(np.max(errors)),
        "valid_view_count": len(accepted),
        "accepted_image_stems": accepted,
        "board": {
            "inner_corners": [args.board_cols, args.board_rows],
            "square_size_m": float(args.square_size_m),
        },
    }
    data = load_yaml(Path(args.output))
    data[args.section] = section
    data.setdefault("calibration_status", "measured")
    write_yaml(Path(args.output), data)
    print(f"MONO_CALIBRATION_OK section={args.section} views={len(accepted)} rms_px={rms:.6f}")


def intrinsic_section(data: dict[str, Any], section: str) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    item = data.get(section)
    if not isinstance(item, dict):
        raise ValueError(f"missing calibration section {section!r}")
    width = int(item.get("image_width", data.get("image_width", 0)))
    height = int(item.get("image_height", data.get("image_height", 0)))
    matrix = np.array(
        [[float(item["fx"]), 0.0, float(item["cx"])],
         [0.0, float(item["fy"]), float(item["cy"])],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion = np.asarray(item.get("distortion", []), dtype=np.float64).reshape(-1, 1)
    if width <= 0 or height <= 0:
        raise ValueError(f"section {section!r} has invalid image size")
    return matrix, distortion, (width, height)


def stereo(args: argparse.Namespace) -> None:
    object_points = board_object_points(args.board_cols, args.board_rows, args.square_size_m)
    front_paths = image_map(Path(args.front_images))
    back_paths = image_map(Path(args.back_images))
    common = {stem: front_paths[stem] for stem in front_paths.keys() & back_paths.keys()}
    back_common = {stem: back_paths[stem] for stem in common}
    front_objects, front_views, front_size, front_ok = collect_views(
        common, object_points, (args.board_cols, args.board_rows)
    )
    back_objects, back_views, back_size, back_ok = collect_views(
        back_common, object_points, (args.board_cols, args.board_rows)
    )
    valid = sorted(set(front_ok) & set(back_ok))
    if len(valid) < args.min_views:
        raise RuntimeError(f"only {len(valid)} stereo pairs have valid corners; need at least {args.min_views}")
    front_by_stem = {stem: view for stem, view in zip(front_ok, front_views)}
    back_by_stem = {stem: view for stem, view in zip(back_ok, back_views)}
    object_by_stem = {stem: obj for stem, obj in zip(front_ok, front_objects)}
    front_data = load_yaml(Path(args.intrinsics))
    back_data = front_data if not args.back_intrinsics else load_yaml(Path(args.back_intrinsics))
    k1, d1, size1 = intrinsic_section(front_data, args.front_section)
    k2, d2, size2 = intrinsic_section(back_data, args.back_section)
    if size1 != front_size or size2 != back_size or front_size != back_size:
        raise ValueError("intrinsics and stereo image sizes must match exactly")
    obj = [object_by_stem[stem] for stem in valid]
    img1 = [front_by_stem[stem].reshape(-1, 1, 2) for stem in valid]
    img2 = [back_by_stem[stem].reshape(-1, 1, 2) for stem in valid]
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-7)
    rms, k1, d1, k2, d2, rotation, translation, essential, fundamental = cv2.stereoCalibrate(
        obj, img1, img2, k1, d1, k2, d2, size1, criteria=criteria, flags=cv2.CALIB_FIX_INTRINSIC
    )
    r1, r2, p1, p2, q, roi1, roi2 = cv2.stereoRectify(
        k1, d1, k2, d2, size1, rotation, translation, alpha=0
    )
    data = load_yaml(Path(args.output))
    data.setdefault(args.front_section, front_data[args.front_section])
    data.setdefault(args.back_section, back_data[args.back_section])
    data["stereo_pair"] = {
        "calibration_status": "measured",
        "image_width": size1[0],
        "image_height": size1[1],
        "valid_pair_count": len(valid),
        "rms_reprojection_error_px": float(rms),
        "rotation_left_to_right_opencv": matrix_list(rotation),
        "translation_left_to_right_m": vector_list(translation),
        "baseline_m": float(np.linalg.norm(translation)),
        "essential_matrix": matrix_list(essential),
        "fundamental_matrix": matrix_list(fundamental),
        "rectification": {
            "R1": matrix_list(r1), "R2": matrix_list(r2),
            "P1": matrix_list(p1), "P2": matrix_list(p2),
            "Q": matrix_list(q), "valid_roi_front": list(map(int, roi1)),
            "valid_roi_back": list(map(int, roi2)),
        },
        "accepted_image_stems": valid,
        "board": {"inner_corners": [args.board_cols, args.board_rows], "square_size_m": args.square_size_m},
    }
    data["calibration_status"] = "measured"
    write_yaml(Path(args.output), data)
    print(f"STEREO_CALIBRATION_OK pairs={len(valid)} baseline_m={np.linalg.norm(translation):.6f} rms_px={rms:.6f}")


CV_TO_PROJECT = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def average_rotation(rotations: list[np.ndarray]) -> np.ndarray:
    u, _, vt = np.linalg.svd(np.sum(rotations, axis=0))
    result = u @ vt
    if np.linalg.det(result) < 0:
        u[:, -1] *= -1
        result = u @ vt
    return result


def rotation_to_rpy_deg(rotation: np.ndarray) -> list[float]:
    value = float(np.clip(-rotation[2, 0], -1.0, 1.0))
    pitch = math.asin(value)
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rotation[2, 1], rotation[2, 2])
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rotation[0, 1], rotation[1, 1])
    return [math.degrees(roll), math.degrees(pitch), math.degrees(yaw)]


def camera_board_transform(image: np.ndarray, object_points: np.ndarray, k: np.ndarray, d: np.ndarray, pattern: tuple[int, int]) -> tuple[np.ndarray, np.ndarray] | None:
    corners = detect_corners(image, pattern)
    if corners is None:
        return None
    ok, rvec, tvec = cv2.solvePnP(object_points, corners, k, d, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    rotation_cv, _ = cv2.Rodrigues(rvec)
    # OpenCV: x-right, y-down, z-forward. Project: x-forward, y-right, z-up.
    rotation_project = CV_TO_PROJECT @ rotation_cv
    translation_project = CV_TO_PROJECT @ tvec.reshape(3)
    return rotation_project, translation_project


def rig(args: argparse.Namespace) -> None:
    object_points = board_object_points(args.board_cols, args.board_rows, args.square_size_m)
    search = image_map(Path(args.search_images))
    front = image_map(Path(args.front_images))
    back = image_map(Path(args.back_images))
    stems = sorted(search.keys() & front.keys() & back.keys())
    if len(stems) < args.min_views:
        raise RuntimeError(f"only {len(stems)} common image stems; need at least {args.min_views}")
    calibration = load_yaml(Path(args.intrinsics))
    camera_data = {
        "search_camera": intrinsic_section(calibration, args.search_section),
        "stereo_front_camera": intrinsic_section(calibration, args.front_section),
        "stereo_back_camera": intrinsic_section(calibration, args.back_section),
    }
    images = {"search_camera": search, "stereo_front_camera": front, "stereo_back_camera": back}
    transforms: dict[str, list[tuple[np.ndarray, np.ndarray]]] = {name: [] for name in camera_data}
    pattern = (args.board_cols, args.board_rows)
    for stem in stems:
        per_camera: dict[str, tuple[np.ndarray, np.ndarray]] = {}
        for name, (k, d, _) in camera_data.items():
            image = cv2.imread(str(images[name][stem]), cv2.IMREAD_COLOR)
            if image is None:
                break
            pose = camera_board_transform(image, object_points, k, d, pattern)
            if pose is None:
                break
            per_camera[name] = pose
        if len(per_camera) != 3:
            continue
        ref_r, ref_t = per_camera["search_camera"]
        ref_from_board = (ref_r, ref_t)
        board_from_ref = (ref_r.T, -ref_r.T @ ref_t)
        for name, (cam_r, cam_t) in per_camera.items():
            if name == "search_camera":
                transforms[name].append((np.eye(3), np.zeros(3)))
                continue
            # T_search_camera_from_camera = T_search_from_board * T_board_from_camera.
            rotation = ref_from_board[0] @ cam_r.T
            translation = ref_from_board[0] @ (-cam_r.T @ cam_t) + ref_from_board[1]
            transforms[name].append((rotation, translation))
    valid_count = len(transforms["stereo_front_camera"])
    if valid_count < args.min_views:
        raise RuntimeError(f"only {valid_count} valid three-camera views; need at least {args.min_views}")
    output: dict[str, Any] = {
        "calibration_status": "measured",
        "parent_frame": "rig_search_camera",
        "source": "opencv_chessboard_solvepnp",
        "valid_view_count": valid_count,
        "board": {"inner_corners": [args.board_cols, args.board_rows], "square_size_m": args.square_size_m},
        "frames": {"rig_search_camera": "search_camera"},
    }
    for name, values in transforms.items():
        rotation = average_rotation([item[0] for item in values])
        translation = np.median(np.stack([item[1] for item in values]), axis=0)
        output[name] = {
            "parent": "rig_search_camera",
            "translation_m": vector_list(translation),
            "rpy_deg": rotation_to_rpy_deg(rotation),
        }
    write_yaml(Path(args.output), output)
    print(f"RIG_EXTRINSICS_OK views={valid_count} output={args.output}")


def rectify(args: argparse.Namespace) -> None:
    data = load_yaml(Path(args.calibration))
    k1, d1, size = intrinsic_section(data, args.front_section)
    k2, d2, size2 = intrinsic_section(data, args.back_section)
    if size != size2:
        raise ValueError("front/back image sizes must match for rectification")
    pair = data.get("stereo_pair", {})
    rectification = pair.get("rectification", {})
    required = ("R1", "R2", "P1", "P2")
    if any(key not in rectification for key in required):
        raise ValueError("calibration YAML has no stereo_pair.rectification matrices")
    maps = [
        cv2.initUndistortRectifyMap(k1, d1, np.asarray(rectification["R1"]), np.asarray(rectification["P1"]), size, cv2.CV_32FC1),
        cv2.initUndistortRectifyMap(k2, d2, np.asarray(rectification["R2"]), np.asarray(rectification["P2"]), size, cv2.CV_32FC1),
    ]
    front_paths = image_map(Path(args.front_images))
    back_paths = image_map(Path(args.back_images))
    stems = sorted(front_paths.keys() & back_paths.keys())[: args.max_pairs]
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    for stem in stems:
        front_image = cv2.imread(str(front_paths[stem]), cv2.IMREAD_COLOR)
        back_image = cv2.imread(str(back_paths[stem]), cv2.IMREAD_COLOR)
        if front_image is None or back_image is None:
            continue
        front_rect = cv2.remap(front_image, maps[0][0], maps[0][1], cv2.INTER_LINEAR)
        back_rect = cv2.remap(back_image, maps[1][0], maps[1][1], cv2.INTER_LINEAR)
        canvas = np.hstack([front_rect, back_rect])
        for y in range(0, canvas.shape[0], max(1, args.line_step)):
            cv2.line(canvas, (0, y), (canvas.shape[1] - 1, y), (0, 255, 0), 1)
        cv2.imwrite(str(output / f"{stem}_rectified.png"), canvas)
    print(f"RECTIFY_PREVIEW_OK pairs={len(stems)} output={output}")


def common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--board-cols", type=int, required=True, help="checkerboard inner-corner columns")
    parser.add_argument("--board-rows", type=int, required=True, help="checkerboard inner-corner rows")
    parser.add_argument("--square-size-m", type=float, required=True)
    parser.add_argument("--min-views", type=int, default=12)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    item = sub.add_parser("mono", help="calibrate one camera")
    common_arguments(item)
    item.add_argument("--images", required=True)
    item.add_argument("--section", required=True)
    item.add_argument("--output", required=True)
    item.set_defaults(handler=mono)

    item = sub.add_parser("stereo", help="calibrate the front/back stereo pair")
    common_arguments(item)
    item.add_argument("--front-images", required=True)
    item.add_argument("--back-images", required=True)
    item.add_argument("--intrinsics", required=True)
    item.add_argument("--back-intrinsics")
    item.add_argument("--front-section", default="stereo_front")
    item.add_argument("--back-section", default="stereo_back")
    item.add_argument("--output", required=True)
    item.set_defaults(handler=stereo)

    item = sub.add_parser("rig", help="estimate search-to-stereo extrinsics")
    common_arguments(item)
    item.add_argument("--search-images", required=True)
    item.add_argument("--front-images", required=True)
    item.add_argument("--back-images", required=True)
    item.add_argument("--intrinsics", required=True)
    item.add_argument("--search-section", default="search")
    item.add_argument("--front-section", default="stereo_front")
    item.add_argument("--back-section", default="stereo_back")
    item.add_argument("--output", required=True)
    item.set_defaults(handler=rig)

    item = sub.add_parser("rectify", help="write rectified side-by-side previews")
    item.add_argument("--calibration", required=True)
    item.add_argument("--front-images", required=True)
    item.add_argument("--back-images", required=True)
    item.add_argument("--front-section", default="stereo_front")
    item.add_argument("--back-section", default="stereo_back")
    item.add_argument("--output", required=True)
    item.add_argument("--max-pairs", type=int, default=20)
    item.add_argument("--line-step", type=int, default=40)
    item.set_defaults(handler=rectify)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    args.handler(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
