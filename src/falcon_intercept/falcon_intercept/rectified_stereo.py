"""Point-level rectified stereo geometry (numpy only, no cv2 dependency).

The production detection chain must not remap whole 4K frames.  Instead the
detection centres are undistorted and rectified point-by-point with the
checkerboard calibration (K1/D1/K2/D2/R/T/R1/R2/P1/P2/Q), matched in the
rectified domain and triangulated there.  Triangulated points are rotated
back into the raw front-camera frame so downstream consumers keep the
existing x-forward/y-right/z-up (FRU) convention.

The rectified virtual pair shares one pinhole model (P1 fx/fy/cx/cy) with
the back camera translated along +x by ``baseline_rect_m``; this is exactly
the model assumed by AsyncStereoStateEstimator, so the asynchronous EKF can
be reused unchanged by feeding it rectified pixels.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np

# OpenCV camera coords (x-right, y-down, z-forward) -> project FRU
# (x-forward, y-right, z-up).
CV_TO_FRU = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=float
)


def _as_matrix33(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 3):
        raise ValueError("expected a 3x3 matrix")
    return matrix


def _as_matrix34(value) -> np.ndarray:
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (3, 4):
        raise ValueError("expected a 3x4 matrix")
    return matrix


def undistort_normalized(u, v, camera_matrix, distortion, iterations: int = 8):
    """Invert the OpenCV 5-parameter distortion for pixel coordinates.

    Returns normalized (ideal pinhole) coordinates x/y.  Uses the same
    fixed-point iteration as cv2.undistortPoints, iterated to convergence.
    """

    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    cx = float(camera_matrix[0, 2])
    cy = float(camera_matrix[1, 2])
    d = np.asarray(distortion, dtype=float).reshape(-1)
    k1 = d[0] if len(d) > 0 else 0.0
    k2 = d[1] if len(d) > 1 else 0.0
    p1 = d[2] if len(d) > 2 else 0.0
    p2 = d[3] if len(d) > 3 else 0.0
    k3 = d[4] if len(d) > 4 else 0.0
    x0 = (np.asarray(u, dtype=float) - cx) / fx
    y0 = (np.asarray(v, dtype=float) - cy) / fy
    x = x0.copy()
    y = y0.copy()
    for _ in range(max(1, int(iterations))):
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        delta_x = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        delta_y = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        x = (x0 - delta_x) / radial
        y = (y0 - delta_y) / radial
    return x, y


@dataclass(frozen=True)
class RectifiedStereoGeometry:
    """Measured stereo rig geometry with rectification matrices."""

    k_front: np.ndarray
    d_front: np.ndarray
    k_back: np.ndarray
    d_back: np.ndarray
    rotation_left_to_right: np.ndarray
    translation_left_to_right_m: np.ndarray
    r1: np.ndarray
    r2: np.ndarray
    p1: np.ndarray
    p2: np.ndarray
    q: np.ndarray
    baseline_m: float
    # Field-measured constant offset applied to back rectified u (2026-08-06):
    # RTK-anchored disparity residual over the 10-20 m working range was
    # -1.65 px (measured disparity systematically small).  Applied as
    # u_back_rectified + offset in rectify_back.
    back_u_offset_px: float = 0.0
    # Field-fitted mechanical-drift correction (2026-08-13): the rig twisted
    # after the 08-04 calibration; rectified vertical error was ~66 px median.
    # A small rotation (wx, wz) of the back camera fitted from 3824 live
    # target correspondences brings it back to ~8.5 px (holdout-verified).
    # wy is unobservable from vertical residuals (degenerate with disparity)
    # and stays zero.  Applied to the back rectified point in rectify_back.
    # Refit or zero after the next checkerboard re-calibration.
    back_drift_rotation_rad: tuple = (0.0, 0.0, 0.0)

    @property
    def fx_rect(self) -> float:
        return float(self.p1[0, 0])

    @property
    def fy_rect(self) -> float:
        return float(self.p1[1, 1])

    @property
    def cx_rect(self) -> float:
        return float(self.p1[0, 2])

    @property
    def cy_rect(self) -> float:
        return float(self.p1[1, 2])

    @property
    def baseline_rect_m(self) -> float:
        """Effective rectified baseline: back camera at +x in rectified frame."""

        return float(-self.p2[0, 3] / self.p2[0, 0])

    def _rectify(self, u, v, camera_matrix, distortion, rotation, projection):
        xn, yn = undistort_normalized(u, v, camera_matrix, distortion)
        ones = np.ones_like(xn)
        points = np.stack([xn, yn, ones], axis=0)
        rotated = rotation @ points
        z = rotated[2]
        u_rect = projection[0, 0] * rotated[0] / z + projection[0, 2]
        v_rect = projection[1, 1] * rotated[1] / z + projection[1, 2]
        return u_rect, v_rect

    def rectify_front(self, u, v):
        return self._rectify(u, v, self.k_front, self.d_front, self.r1, self.p1)

    def rectify_back(self, u, v):
        u_rect, v_rect = self._rectify(u, v, self.k_back, self.d_back, self.r2, self.p2)
        u_rect = u_rect + self.back_u_offset_px
        wx, wy, wz = (float(value) for value in self.back_drift_rotation_rad)
        if wx or wy or wz:
            x = (u_rect - self.cx_rect) / self.fx_rect
            y = (v_rect - self.cy_rect) / self.fy_rect
            cross = np.cross(np.array([wx, wy, wz]), np.array([x, y, 1.0]))
            x_corr = x + float(cross[0])
            y_corr = y + float(cross[1])
            z_corr = 1.0 + float(cross[2])
            u_rect = self.fx_rect * x_corr / z_corr + self.cx_rect
            v_rect = self.fy_rect * y_corr / z_corr + self.cy_rect
        return u_rect, v_rect

    def pair_metrics(self, u_front, v_front, u_back, v_back) -> tuple[float, float]:
        """Return (disparity_px, vertical_error_px) in the rectified domain."""

        ul, vl = self.rectify_front(u_front, v_front)
        ur, vr = self.rectify_back(u_back, v_back)
        return float(ul - ur), float(abs(vl - vr))

    def triangulate_rectified(self, u_front, v_front, u_back) -> np.ndarray:
        """Triangulate one correspondence into the rectified front CV frame."""

        disparity = float(u_front) - float(u_back)
        if disparity <= 0.0:
            raise ValueError("disparity must be positive")
        z = -float(self.p2[0, 3]) / disparity
        x = (float(u_front) - float(self.p1[0, 2])) * z / float(self.p1[0, 0])
        y = (float(v_front) - float(self.p1[1, 2])) * z / float(self.p1[1, 1])
        return np.array([x, y, z], dtype=float)

    def rectified_to_front_cv(self, point_rectified) -> np.ndarray:
        """Rotate a rectified-frame point back into the raw front CV frame."""

        return self.r1.T @ np.asarray(point_rectified, dtype=float).reshape(3)

    def triangulate_fru(self, u_front, v_front, u_back, v_back) -> np.ndarray:
        """Full chain: raw pixels -> rectified triangulation -> raw front FRU."""

        ul, vl = self.rectify_front(u_front, v_front)
        ur, _ = self.rectify_back(u_back, v_back)
        point_rect = self.triangulate_rectified(ul, vl, ur)
        point_front_cv = self.rectified_to_front_cv(point_rect)
        return CV_TO_FRU @ point_front_cv

    def rotation_fru_rectified_to_front(self) -> np.ndarray:
        """FRU rotation mapping rectified-front-frame vectors to raw-front FRU."""

        return CV_TO_FRU @ self.r1.T @ CV_TO_FRU.T


def geometry_from_dict(data: dict) -> RectifiedStereoGeometry:
    front = data.get("stereo_front", data.get("stereo_left"))
    back = data.get("stereo_back", data.get("stereo_right"))
    pair = data.get("stereo_pair")
    if not isinstance(front, dict) or not isinstance(back, dict) or not isinstance(pair, dict):
        raise ValueError(
        "calibration requires stereo_front/stereo_back (or legacy stereo_left/stereo_right)/stereo_pair"
    )
    rectification = pair.get("rectification")
    if not isinstance(rectification, dict):
        raise ValueError("stereo_pair.rectification is missing")
    for name in ("R1", "R2", "P1", "P2", "Q"):
        if name not in rectification:
            raise ValueError(f"stereo_pair.rectification.{name} is missing")
    if "rotation_left_to_right_opencv" not in pair or "translation_left_to_right_m" not in pair:
        raise ValueError("stereo_pair rotation/translation is missing")

    def camera(section):
        matrix = np.array(
            [
                [float(section["fx"]), 0.0, float(section["cx"])],
                [0.0, float(section["fy"]), float(section["cy"])],
                [0.0, 0.0, 1.0],
            ],
            dtype=float,
        )
        distortion = np.asarray(section.get("distortion", []), dtype=float).reshape(-1)
        return matrix, distortion

    k_front, d_front = camera(front)
    k_back, d_back = camera(back)
    geometry = RectifiedStereoGeometry(
        k_front=k_front,
        d_front=d_front,
        k_back=k_back,
        d_back=d_back,
        rotation_left_to_right=_as_matrix33(pair["rotation_left_to_right_opencv"]),
        translation_left_to_right_m=np.asarray(pair["translation_left_to_right_m"], dtype=float).reshape(3),
        r1=_as_matrix33(rectification["R1"]),
        r2=_as_matrix33(rectification["R2"]),
        p1=_as_matrix34(rectification["P1"]),
        p2=_as_matrix34(rectification["P2"]),
        q=np.asarray(rectification["Q"], dtype=float).reshape(4, 4),
        baseline_m=float(pair.get("baseline_m", 0.0)),
        back_drift_rotation_rad=tuple(
            float(value)
            for value in pair.get("drift_rotation_back_rad", (0.0, 0.0, 0.0))
        ),
    )
    if geometry.fx_rect <= 0.0 or geometry.fy_rect <= 0.0 or geometry.baseline_rect_m <= 0.0:
        raise ValueError("rectified intrinsics and baseline must be positive")
    return geometry


def load_rectified_stereo_geometry(calibration_path: str | Path) -> RectifiedStereoGeometry | None:
    """Load rectified geometry; return None when the YAML is the legacy format."""

    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to load stereo calibration") from exc

    with Path(calibration_path).open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    pair = data.get("stereo_pair")
    if not isinstance(pair, dict) or not isinstance(pair.get("rectification"), dict):
        return None
    return geometry_from_dict(data)
