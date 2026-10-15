#!/usr/bin/env python3
"""双机记录整理以及双逻辑相机、姿态和打击轨迹绘图。"""

from __future__ import annotations

import csv
import math
import os
import shutil
from pathlib import Path
from typing import Dict, List

import matplotlib

if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Ellipse  # noqa: E402
import numpy as np  # noqa: E402


REQUIRED_COLUMNS = (
    "time_s",
    "target_n_m",
    "target_e_m",
    "target_d_m",
    "pursuer_n_m",
    "pursuer_e_m",
    "pursuer_d_m",
    "camera_u_px",
    "camera_v_px",
    "camera_depth_m",
    "camera_visible",
    "estimate_valid",
)

OPTIONAL_COLUMN_DEFAULTS = {
    "mission_state": np.nan,
    "pursuer_yaw_rad": np.nan,
    "pursuer_q_w": np.nan,
    "pursuer_q_x": np.nan,
    "pursuer_q_y": np.nan,
    "pursuer_q_z": np.nan,
    "pursuer_body_x_n": np.nan,
    "pursuer_body_x_e": np.nan,
    "pursuer_body_x_d": np.nan,
    "pursuer_body_minus_z_n": np.nan,
    "pursuer_body_minus_z_e": np.nan,
    "pursuer_body_minus_z_d": np.nan,
    "capture_held": 0.0,
    "capture_count": 0.0,
    "capture_pixel_error_px": np.nan,
    "capture_alignment_error_rad": np.nan,
}

DEPTH_HORIZONTAL_FOV_RAD = math.radians(17.0)
DEPTH_VERTICAL_FOV_RAD = math.radians(20.0)
SEARCH_HORIZONTAL_FOV_RAD = math.radians(35.0)
SEARCH_VERTICAL_FOV_RAD = math.radians(60.0)
SEARCH_SIN_TILT = math.sin(math.radians(10.0))
SEARCH_COS_TILT = math.cos(math.radians(10.0))


def _configure_chinese_font() -> None:
    matplotlib.rcParams["font.sans-serif"] = [
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "Microsoft YaHei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False


_configure_chinese_font()


def focal_lengths_from_fov(
    width: int, height: int, horizontal_fov_rad: float, vertical_fov_rad: float
) -> tuple[float, float]:
    """Return pinhole focal lengths whose image edges match the requested FOV."""
    if width <= 0 or height <= 0:
        raise ValueError("camera dimensions must be positive")
    return (
        0.5 * width / math.tan(0.5 * horizontal_fov_rad),
        0.5 * height / math.tan(0.5 * vertical_fov_rad),
    )


def load_record(path: Path) -> Dict[str, np.ndarray]:
    names = REQUIRED_COLUMNS + tuple(OPTIONAL_COLUMN_DEFAULTS)
    columns: Dict[str, List[float]] = {name: [] for name in names}
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        missing = set(REQUIRED_COLUMNS) - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"CSV missing columns: {sorted(missing)}")
        for row in reader:
            for name in REQUIRED_COLUMNS:
                columns[name].append(float(row[name]))
            for name, default in OPTIONAL_COLUMN_DEFAULTS.items():
                value = row.get(name, "")
                if value in (None, ""):
                    columns[name].append(default)
                elif name == "mission_state" and value.startswith("B"):
                    columns[name].append(float(value[1:].split("_", 1)[0]))
                else:
                    columns[name].append(float(value))
    if not columns["time_s"]:
        raise ValueError("CSV contains no trajectory samples")
    return {name: np.asarray(values, dtype=float) for name, values in columns.items()}


def _finite_xyz(data: Dict[str, np.ndarray], prefix: str) -> np.ndarray:
    points = np.column_stack(
        (data[f"{prefix}_n_m"], data[f"{prefix}_e_m"], -data[f"{prefix}_d_m"])
    )
    return points[np.all(np.isfinite(points), axis=1)]


def capture_event_indices(data: Dict[str, np.ndarray]) -> np.ndarray:
    """Return one sample per independent trigger event."""
    held = np.asarray(data["capture_held"] > 0.5, dtype=bool)
    previous = np.concatenate(([False], held[:-1]))
    held_edges = np.flatnonzero(held & ~previous)
    count = np.nan_to_num(data["capture_count"], nan=0.0)
    count_edges = np.flatnonzero(np.concatenate(([False], np.diff(count) > 0.5)))
    events = list(count_edges)
    for edge in held_edges:
        if not any(abs(int(edge) - int(existing)) <= 2 for existing in events):
            events.append(edge)
    return np.asarray(sorted(events), dtype=int)


def _quaternion_body_to_ned_matrix(quaternion: np.ndarray) -> np.ndarray | None:
    if not np.all(np.isfinite(quaternion)):
        return None
    norm = np.linalg.norm(quaternion)
    if norm <= 1e-9:
        return None
    w, x, y, z = quaternion / norm
    return np.asarray(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
            [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
            [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


def simulated_camera_tracks(
    data: Dict[str, np.ndarray], camera_width: int = 3840, camera_height: int = 2160
) -> Dict[str, np.ndarray]:
    """Project truth to ideal YOLO center pixels without exposing search depth."""
    sample_count = len(data["time_s"])
    depth_u = np.full(sample_count, np.nan)
    depth_v = np.full(sample_count, np.nan)
    depth_visible = np.zeros(sample_count, dtype=bool)
    search_u = np.full(sample_count, np.nan)
    search_v = np.full(sample_count, np.nan)
    search_visible = np.zeros(sample_count, dtype=bool)
    depth_fx, depth_fy = focal_lengths_from_fov(
        camera_width, camera_height, DEPTH_HORIZONTAL_FOV_RAD, DEPTH_VERTICAL_FOV_RAD
    )
    search_fx, search_fy = focal_lengths_from_fov(
        camera_width, camera_height, SEARCH_HORIZONTAL_FOV_RAD, SEARCH_VERTICAL_FOV_RAD
    )
    cx = 0.5 * camera_width - 0.5
    cy = 0.5 * camera_height - 0.5
    target_ned = np.column_stack(
        (data["target_n_m"], data["target_e_m"], data["target_d_m"])
    )
    pursuer_ned = np.column_stack(
        (data["pursuer_n_m"], data["pursuer_e_m"], data["pursuer_d_m"])
    )
    quaternions = np.column_stack(
        (
            data["pursuer_q_w"],
            data["pursuer_q_x"],
            data["pursuer_q_y"],
            data["pursuer_q_z"],
        )
    )
    for index in range(sample_count):
        rotation = _quaternion_body_to_ned_matrix(quaternions[index])
        relative_ned = target_ned[index] - pursuer_ned[index]
        if rotation is None or not np.all(np.isfinite(relative_ned)):
            continue
        x_body, y_body, z_body = rotation.T @ relative_ned

        x_depth, y_depth, z_depth = y_body, x_body, -z_body
        if z_depth > 0.0:
            horizontal = math.atan2(x_depth, z_depth)
            vertical = math.atan2(y_depth, z_depth)
            if (
                abs(horizontal) <= 0.5 * DEPTH_HORIZONTAL_FOV_RAD
                and abs(vertical) <= 0.5 * DEPTH_VERTICAL_FOV_RAD
            ):
                depth_u[index] = depth_fx * x_depth / z_depth + cx
                depth_v[index] = depth_fy * y_depth / z_depth + cy
                depth_visible[index] = True

        x_search = y_body
        y_search = SEARCH_COS_TILT * x_body + SEARCH_SIN_TILT * z_body
        z_search = SEARCH_SIN_TILT * x_body - SEARCH_COS_TILT * z_body
        if z_search > 0.0:
            horizontal = math.atan2(x_search, z_search)
            vertical = math.atan2(y_search, z_search)
            if (
                abs(horizontal) <= 0.5 * SEARCH_HORIZONTAL_FOV_RAD
                and abs(vertical) <= 0.5 * SEARCH_VERTICAL_FOV_RAD
            ):
                search_u[index] = search_fx * x_search / z_search + cx
                search_v[index] = search_fy * y_search / z_search + cy
                search_visible[index] = True
    return {
        "depth_u_px": depth_u,
        "depth_v_px": depth_v,
        "depth_visible": depth_visible,
        "search_u_px": search_u,
        "search_v_px": search_v,
        "search_visible": search_visible,
    }


def _plot_trajectory_3d(data: Dict[str, np.ndarray], output_path: Path) -> None:
    target = _finite_xyz(data, "target")
    pursuer = _finite_xyz(data, "pursuer")
    if len(target) == 0 or len(pursuer) == 0:
        raise ValueError("CSV has no finite Target/Pursuer trajectory")
    figure = plt.figure(figsize=(11, 8))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(*target.T, label="目标机", color="tab:red")
    axes.plot(*pursuer.T, label="拦截机", color="tab:blue")
    pose = np.column_stack(
        (data["pursuer_n_m"], data["pursuer_e_m"], -data["pursuer_d_m"])
    )
    attack_axis = np.column_stack(
        (
            data["pursuer_body_minus_z_n"],
            data["pursuer_body_minus_z_e"],
            -data["pursuer_body_minus_z_d"],
        )
    )
    valid = np.flatnonzero(
        np.all(np.isfinite(pose), axis=1) & np.all(np.isfinite(attack_axis), axis=1)
    )
    if len(valid):
        sampled = valid[
            np.unique(np.linspace(0, len(valid) - 1, min(32, len(valid))).astype(int))
        ]
        for number, index in enumerate(sampled):
            direction = attack_axis[index]
            norm = np.linalg.norm(direction)
            if norm <= 1e-9:
                continue
            end = pose[index] + 1.25 * direction / norm
            axes.plot(
                [pose[index, 0], end[0]],
                [pose[index, 1], end[1]],
                [pose[index, 2], end[2]],
                color="darkviolet",
                alpha=0.65,
                linewidth=1.1,
                label="机体 -Z / 攻击轴" if number == 0 else None,
            )
    for number, index in enumerate(capture_event_indices(data), start=1):
        target_event = np.asarray(
            [data["target_n_m"][index], data["target_e_m"][index], -data["target_d_m"][index]]
        )
        pursuer_event = np.asarray(
            [
                data["pursuer_n_m"][index],
                data["pursuer_e_m"][index],
                -data["pursuer_d_m"][index],
            ]
        )
        axes.scatter(*target_event, marker="*", s=190, color="gold", edgecolor="black")
        axes.plot(
            [pursuer_event[0], target_event[0]],
            [pursuer_event[1], target_event[1]],
            [pursuer_event[2], target_event[2]],
            "k:",
        )
        axes.text(*target_event, f"  打击{number}")
    axes.set(xlabel="北 [m]", ylabel="东 [m]", zlabel="高度 [m]")
    axes.set_title("双机三维轨迹与拦截机攻击轴")
    axes.set_box_aspect((1.0, 1.0, 1.5))
    axes.legend()
    axes.grid(True)
    figure.tight_layout()
    figure.savefig(output_path, dpi=170)
    plt.close(figure)


def _plot_camera(
    data: Dict[str, np.ndarray],
    u: np.ndarray,
    v: np.ndarray,
    visible: np.ndarray,
    output_path: Path,
    title: str,
    camera_width: int,
    camera_height: int,
    fx: float,
    fy: float,
    show_trigger_gate: bool,
) -> None:
    figure, axes = plt.subplots(figsize=(12, 7))
    axes.set_xlim(0, camera_width)
    axes.set_ylim(camera_height, 0)
    axes.set_aspect("equal", adjustable="box")
    axes.set(xlabel="像素 u [px]", ylabel="像素 v [px]", title=title)
    axes.grid(True, alpha=0.25)
    cx, cy = 0.5 * camera_width - 0.5, 0.5 * camera_height - 0.5
    axes.scatter(cx, cy, marker="+", s=140, linewidth=2, color="red", label="光学中心")
    if show_trigger_gate:
        half_angle = math.radians(2.0)
        axes.add_patch(
            Ellipse(
                (cx, cy),
                width=2.0 * fx * math.tan(half_angle),
                height=2.0 * fy * math.tan(half_angle),
                fill=False,
                linestyle="--",
                linewidth=1.6,
                edgecolor="red",
                label="2°打击门限",
            )
        )
    valid = visible & np.isfinite(u) & np.isfinite(v)
    if np.any(valid):
        elapsed = data["time_s"][valid] - data["time_s"][valid][0]
        scatter = axes.scatter(
            u[valid], v[valid], c=elapsed, cmap="viridis", s=11, label="YOLO目标中心"
        )
        figure.colorbar(scatter, ax=axes, label="可见后经过时间 [s]")
    else:
        axes.text(0.5, 0.5, "没有有效可见样本", transform=axes.transAxes, ha="center")
    for number, index in enumerate(capture_event_indices(data), start=1):
        if index >= len(visible) or not visible[index]:
            continue
        axes.scatter(
            u[index], v[index], marker="*", s=250, color="gold", edgecolor="black", zorder=8
        )
        axes.annotate(
            f"打击{number}",
            (u[index], v[index]),
            xytext=(10, -18 - 24 * (number - 1)),
            textcoords="offset points",
            arrowprops={"arrowstyle": "->", "color": "black"},
        )
    axes.legend(loc="best")
    figure.tight_layout()
    figure.savefig(output_path, dpi=170)
    plt.close(figure)


def _plot_attack_summary(data: Dict[str, np.ndarray], output_path: Path) -> None:
    figure, (path_axes, error_axes) = plt.subplots(2, 1, figsize=(11, 10))
    path_axes.plot(data["target_n_m"], data["target_e_m"], color="tab:red", label="目标机")
    path_axes.plot(data["pursuer_n_m"], data["pursuer_e_m"], color="tab:blue", label="拦截机")
    event_indices = capture_event_indices(data)
    for number, index in enumerate(event_indices, start=1):
        path_axes.scatter(
            data["target_n_m"][index],
            data["target_e_m"][index],
            marker="*",
            s=220,
            color="gold",
            edgecolor="black",
            zorder=8,
        )
        path_axes.annotate(
            f"打击{number}",
            (data["target_n_m"][index], data["target_e_m"][index]),
            xytext=(8, 8),
            textcoords="offset points",
        )
    path_axes.set(title="打击事件平面轨迹", xlabel="北 [m]", ylabel="东 [m]")
    path_axes.axis("equal")
    path_axes.grid(True)
    path_axes.legend()

    alignment_deg = np.degrees(data["capture_alignment_error_rad"])
    error_axes.plot(data["time_s"], alignment_deg, label="攻击轴/LOS误差")
    error_axes.axhline(1.0, color="tab:green", linestyle="--", label="优选1°")
    error_axes.axhline(2.0, color="tab:red", linestyle="--", label="硬门限2°")
    for number, index in enumerate(event_indices, start=1):
        error_axes.axvline(
            data["time_s"][index],
            color="goldenrod",
            linestyle=":",
            label="打击时刻" if number == 1 else None,
        )
    error_axes.set(
        title="瞄准误差与打击时刻", xlabel="时间 [s]", ylabel="角度误差 [deg]"
    )
    error_axes.set_ylim(bottom=0)
    error_axes.grid(True)
    error_axes.legend()
    figure.tight_layout()
    figure.savefig(output_path, dpi=170)
    plt.close(figure)


def plot_record(
    csv_path: Path,
    output_directory: Path,
    camera_width: int = 3840,
    camera_height: int = 2160,
    show: bool = False,
) -> tuple[Path, Path, Path, Path]:
    """Generate images in the outer run directory; never mutate the CSV."""
    data = load_record(csv_path)
    output_directory.mkdir(parents=True, exist_ok=True)
    projections = simulated_camera_tracks(data, camera_width, camera_height)
    depth_fx, depth_fy = focal_lengths_from_fov(
        camera_width, camera_height, DEPTH_HORIZONTAL_FOV_RAD, DEPTH_VERTICAL_FOV_RAD
    )
    search_fx, search_fy = focal_lengths_from_fov(
        camera_width, camera_height, SEARCH_HORIZONTAL_FOV_RAD, SEARCH_VERTICAL_FOV_RAD
    )
    depth_visible = (
        (data["camera_visible"] > 0.5)
        & np.isfinite(data["camera_u_px"])
        & np.isfinite(data["camera_v_px"])
    )
    # New records contain the synchronized camera projection. Old records remain
    # visually reproducible because their logged pixels are plotted unchanged.
    outputs = (
        output_directory / "01_三维轨迹与攻击轴.png",
        output_directory / "02_双目相机视野_YOLO中心.png",
        output_directory / "03_搜索相机视野_YOLO中心.png",
        output_directory / "04_打击轨迹与误差.png",
    )
    _plot_trajectory_3d(data, outputs[0])
    _plot_camera(
        data,
        data["camera_u_px"],
        data["camera_v_px"],
        depth_visible,
        outputs[1],
        "模拟双目深度相机：目标中心轨迹",
        camera_width,
        camera_height,
        depth_fx,
        depth_fy,
        True,
    )
    _plot_camera(
        data,
        projections["search_u_px"],
        projections["search_v_px"],
        projections["search_visible"],
        outputs[2],
        "搜索相机：理想YOLO目标中心轨迹（无深度输出）",
        camera_width,
        camera_height,
        search_fx,
        search_fy,
        False,
    )
    _plot_attack_summary(data, outputs[3])
    if show:
        plt.show()
    return outputs


def organize_run_files(csv_path: Path, run_directory: Path) -> Path:
    """Move raw CSV/log/metadata one level inward while keeping images outside."""
    run_directory.mkdir(parents=True, exist_ok=True)
    raw_root = run_directory / "原始数据"
    csv_directory = raw_root / "CSV"
    log_directory = raw_root / "日志"
    metadata_directory = raw_root / "其他"
    csv_directory.mkdir(parents=True, exist_ok=True)
    log_directory.mkdir(parents=True, exist_ok=True)
    metadata_directory.mkdir(parents=True, exist_ok=True)

    source_csv = csv_path.resolve()
    destination_csv = csv_directory / source_csv.name
    if source_csv != destination_csv.resolve():
        if destination_csv.exists():
            raise FileExistsError(f"refusing to overwrite archived CSV: {destination_csv}")
        shutil.move(str(source_csv), destination_csv)

    for child in list(run_directory.iterdir()):
        if not child.is_file() or child == destination_csv:
            continue
        suffix = child.suffix.lower()
        destination_directory = None
        if suffix == ".log":
            destination_directory = log_directory
        elif suffix in {".json", ".yaml", ".yml", ".txt", ".ulg"}:
            destination_directory = metadata_directory
        if destination_directory is None:
            continue
        destination = destination_directory / child.name
        if destination.exists():
            raise FileExistsError(f"refusing to overwrite archived file: {destination}")
        shutil.move(str(child), destination)
    return destination_csv


def plot_run(
    csv_path: Path,
    run_directory: Path,
    camera_width: int = 3840,
    camera_height: int = 2160,
    organize: bool = True,
    show: bool = False,
) -> tuple[Path, Path, Path, Path]:
    """Organize one completed run and place all four images in its outer directory."""
    archived_csv = organize_run_files(csv_path, run_directory) if organize else csv_path
    return plot_record(archived_csv, run_directory, camera_width, camera_height, show)
