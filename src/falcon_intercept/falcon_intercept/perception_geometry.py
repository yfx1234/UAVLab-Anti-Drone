"""Camera geometry utilities for search-to-stereo ROI projection.

The functions in this module are deliberately independent of ROS, DeepStream,
and a trained detector. They provide the geometry contract that the runtime
ROI pipeline can consume once measured calibration is available.

Coordinate convention: camera/body coordinates use x-forward, y-right, z-up;
image coordinates use u-back, v-down. A camera pose is represented as
``T_parent_camera`` so that ``p_parent = R @ p_camera + t``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Sequence

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: tuple[float, ...] = ()

    def pixel_ray(self, u: float, v: float) -> np.ndarray:
        """Return an unnormalised x-forward ray in camera coordinates."""

        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")
        return np.array(
            [1.0, (float(u) - self.cx) / self.fx, -(float(v) - self.cy) / self.fy],
            dtype=float,
        )

    def project_point(self, point_camera: Sequence[float]) -> tuple[float, float] | None:
        """Project one x-forward camera point to image pixels."""

        point = np.asarray(point_camera, dtype=float).reshape(3)
        if not np.all(np.isfinite(point)) or point[0] <= 1e-6:
            return None
        u = self.fx * point[1] / point[0] + self.cx
        v = self.cy - self.fy * point[2] / point[0]
        if not math.isfinite(float(u)) or not math.isfinite(float(v)):
            return None
        return float(u), float(v)


@dataclass(frozen=True)
class RigidTransform:
    """Rigid transform ``T_parent_child`` in the project coordinate convention."""

    rotation_parent_from_child: np.ndarray
    translation_parent_from_child: np.ndarray

    def __post_init__(self) -> None:
        rotation = np.asarray(self.rotation_parent_from_child, dtype=float)
        translation = np.asarray(self.translation_parent_from_child, dtype=float)
        if rotation.shape != (3, 3) or translation.shape != (3,):
            raise ValueError("rigid transform must contain a 3x3 rotation and 3-vector translation")
        if not np.all(np.isfinite(rotation)) or not np.all(np.isfinite(translation)):
            raise ValueError("rigid transform contains non-finite values")
        object.__setattr__(self, "rotation_parent_from_child", rotation)
        object.__setattr__(self, "translation_parent_from_child", translation)

    @staticmethod
    def identity() -> "RigidTransform":
        return RigidTransform(np.eye(3, dtype=float), np.zeros(3, dtype=float))

    def apply(self, point_child: Sequence[float]) -> np.ndarray:
        point = np.asarray(point_child, dtype=float).reshape(3)
        return self.rotation_parent_from_child @ point + self.translation_parent_from_child

    def inverse(self) -> "RigidTransform":
        rotation = self.rotation_parent_from_child.T
        translation = -(rotation @ self.translation_parent_from_child)
        return RigidTransform(rotation, translation)


@dataclass(frozen=True)
class ProjectedRoi:
    x1: int
    y1: int
    x2: int
    y2: int
    image_width: int
    image_height: int
    depth_min_m: float
    depth_max_m: float

    @property
    def width(self) -> int:
        return self.x2 - self.x1

    @property
    def height(self) -> int:
        return self.y2 - self.y1


@dataclass(frozen=True)
class CameraGeometry:
    search: CameraIntrinsics
    stereo_front: CameraIntrinsics
    stereo_back: CameraIntrinsics
    search_to_front: RigidTransform
    search_to_back: RigidTransform


def rpy_deg_to_rotation(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Build ``R_parent_from_child`` using Rz(yaw) Ry(pitch) Rx(roll)."""

    roll_rad, pitch_rad, yaw_rad = [math.radians(float(v)) for v in (roll, pitch, yaw)]
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def _mapping_compat(data: Any, primary: str, legacy: str) -> dict[str, Any]:
    if isinstance(data, dict) and primary not in data and legacy in data:
        return _mapping(data, legacy)
    return _mapping(data, primary)


def _mapping(data: Any, name: str) -> dict[str, Any]:
    value = data.get(name) if isinstance(data, dict) else None
    if not isinstance(value, dict):
        raise ValueError("calibration is missing camera section %r" % name)
    return value


def _intrinsics_from_mapping(data: dict[str, Any], root: dict[str, Any]) -> CameraIntrinsics:
    width = int(data.get("image_width", root.get("image_width", 0)))
    height = int(data.get("image_height", root.get("image_height", 0)))
    required = ("fx", "fy", "cx", "cy")
    if width <= 0 or height <= 0 or any(key not in data for key in required):
        raise ValueError("camera calibration requires image size and fx/fy/cx/cy")
    distortion = tuple(float(value) for value in data.get("distortion", ()))
    return CameraIntrinsics(
        width=width,
        height=height,
        fx=float(data["fx"]),
        fy=float(data["fy"]),
        cx=float(data["cx"]),
        cy=float(data["cy"]),
        distortion=distortion,
    )


def _pose_from_mapping(data: dict[str, Any], name: str) -> tuple[str, RigidTransform]:
    parent = str(data.get("parent", "")).strip()
    translation = data.get("translation_m")
    rpy = data.get("rpy_deg")
    if not parent or not isinstance(translation, (list, tuple)) or len(translation) != 3:
        raise ValueError("extrinsics %r requires parent and translation_m[3]" % name)
    if not isinstance(rpy, (list, tuple)) or len(rpy) != 3:
        raise ValueError("extrinsics %r requires rpy_deg[3]" % name)
    return parent, RigidTransform(
        rpy_deg_to_rotation(*[float(value) for value in rpy]),
        np.asarray([float(value) for value in translation], dtype=float),
    )


def load_camera_geometry(
    calibration_path: str | Path, extrinsics_path: str | Path
) -> CameraGeometry:
    """Load measured intrinsics and search/stereo extrinsics from YAML.

    Required calibration sections are ``search``, ``stereo_front`` and
    ``stereo_back``. Required extrinsics sections are ``search_camera``,
    ``stereo_front_camera`` and ``stereo_back_camera``. The existing placeholder
    files intentionally fail this loader until real measurements are written.
    """

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load camera geometry") from exc

    with Path(calibration_path).open("r", encoding="utf-8") as stream:
        calibration = yaml.safe_load(stream) or {}
    with Path(extrinsics_path).open("r", encoding="utf-8") as stream:
        extrinsics = yaml.safe_load(stream) or {}

    search = _intrinsics_from_mapping(_mapping(calibration, "search"), calibration)
    stereo_front = _intrinsics_from_mapping(_mapping_compat(calibration, "stereo_front", "stereo_left"), calibration)
    stereo_back = _intrinsics_from_mapping(_mapping_compat(calibration, "stereo_back", "stereo_right"), calibration)
    search_pose = _pose_from_mapping(
        _mapping(extrinsics, "search_camera"), "search_camera"
    )
    front_pose = _pose_from_mapping(
        _mapping_compat(extrinsics, "stereo_front_camera", "stereo_left_camera"), "stereo_front_camera"
    )
    back_pose = _pose_from_mapping(
        _mapping_compat(extrinsics, "stereo_back_camera", "stereo_right_camera"), "stereo_back_camera"
    )

    return CameraGeometry(
        search=search,
        stereo_front=stereo_front,
        stereo_back=stereo_back,
        search_to_front=_compose_search_to_target(search_pose, front_pose, "stereo_front_camera"),
        search_to_back=_compose_search_to_target(search_pose, back_pose, "stereo_back_camera"),
    )


def _compose_search_to_target(
    search_pose: tuple[str, RigidTransform],
    target_pose: tuple[str, RigidTransform],
    target_name: str,
) -> RigidTransform:
    search_parent, body_from_search = search_pose
    target_parent, body_from_target = target_pose
    if search_parent != target_parent:
        raise ValueError(
            "search and %s extrinsics must share a parent frame, got %r and %r"
            % (target_name, search_parent, target_parent)
        )
    target_from_body = body_from_target.inverse()
    return RigidTransform(
        target_from_body.rotation_parent_from_child @ body_from_search.rotation_parent_from_child,
        target_from_body.rotation_parent_from_child @ body_from_search.translation_parent_from_child
        + target_from_body.translation_parent_from_child,
    )


def project_search_bbox_to_roi(
    search_bbox_xyxy: Sequence[float],
    search_intrinsics: CameraIntrinsics,
    target_intrinsics: CameraIntrinsics,
    search_to_target: RigidTransform,
    depth_min_m: float,
    depth_max_m: float,
    margin_px: float = 16.0,
) -> ProjectedRoi | None:
    """Project a search bbox over a depth interval into one stereo-camera ROI."""

    if len(search_bbox_xyxy) != 4:
        raise ValueError("search bbox must contain x1,y1,x2,y2")
    if depth_min_m <= 0.0 or depth_max_m < depth_min_m:
        raise ValueError("depth interval must satisfy 0 < min <= max")
    if margin_px < 0.0:
        raise ValueError("ROI margin must be non-negative")

    x1, y1, x2, y2 = [float(value) for value in search_bbox_xyxy]
    x1 = max(0.0, min(float(search_intrinsics.width), x1))
    y1 = max(0.0, min(float(search_intrinsics.height), y1))
    x2 = max(0.0, min(float(search_intrinsics.width), x2))
    y2 = max(0.0, min(float(search_intrinsics.height), y2))
    if x2 <= x1 or y2 <= y1:
        return None

    projected: list[tuple[float, float]] = []
    for depth_m in (float(depth_min_m), float(depth_max_m)):
        for u, v in ((x1, y1), (x2, y1), (x2, y2), (x1, y2)):
            ray = search_intrinsics.pixel_ray(u, v)
            point_search = ray * (depth_m / ray[0])
            point_target = search_to_target.apply(point_search)
            pixel = target_intrinsics.project_point(point_target)
            if pixel is not None:
                projected.append(pixel)

    if not projected:
        return None
    projected_u = [pixel[0] for pixel in projected]
    projected_v = [pixel[1] for pixel in projected]
    roi_x1 = max(0, int(math.floor(min(projected_u) - float(margin_px))))
    roi_y1 = max(0, int(math.floor(min(projected_v) - float(margin_px))))
    roi_x2 = min(target_intrinsics.width, int(math.ceil(max(projected_u) + float(margin_px))))
    roi_y2 = min(target_intrinsics.height, int(math.ceil(max(projected_v) + float(margin_px))))
    if roi_x2 <= roi_x1 or roi_y2 <= roi_y1:
        return None
    return ProjectedRoi(
        x1=roi_x1,
        y1=roi_y1,
        x2=roi_x2,
        y2=roi_y2,
        image_width=target_intrinsics.width,
        image_height=target_intrinsics.height,
        depth_min_m=float(depth_min_m),
        depth_max_m=float(depth_max_m),
    )


def project_search_bbox_to_stereo_rois(
    search_bbox_xyxy: Sequence[float],
    geometry: CameraGeometry,
    depth_min_m: float,
    depth_max_m: float,
    margin_px: float = 16.0,
) -> dict[str, ProjectedRoi | None]:
    """Project one search bbox to both stereo images."""

    return {
        "front": project_search_bbox_to_roi(
            search_bbox_xyxy,
            geometry.search,
            geometry.stereo_front,
            geometry.search_to_front,
            depth_min_m,
            depth_max_m,
            margin_px,
        ),
        "back": project_search_bbox_to_roi(
            search_bbox_xyxy,
            geometry.search,
            geometry.stereo_back,
            geometry.search_to_back,
            depth_min_m,
            depth_max_m,
            margin_px,
        ),
    }
