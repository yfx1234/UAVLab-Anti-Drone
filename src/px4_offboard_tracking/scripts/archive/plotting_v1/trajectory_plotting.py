#!/usr/bin/env python3
"""CSV loading and plotting helpers for dual-aircraft tracking runs."""

from __future__ import annotations

import csv
import os
from pathlib import Path
from typing import Dict, List

import matplotlib

if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
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
    "pursuer_body_x_n": np.nan,
    "pursuer_body_x_e": np.nan,
    "pursuer_body_x_d": np.nan,
    "pursuer_body_minus_z_n": np.nan,
    "pursuer_body_minus_z_e": np.nan,
    "pursuer_body_minus_z_d": np.nan,
    "capture_held": 0.0,
    "capture_count": 0.0,
    "capture_pixel_error_px": np.nan,
}


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
    """Return one index per capture, tolerating an asynchronously missed held pulse."""
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


def _xyz_at(data: Dict[str, np.ndarray], prefix: str, index: int) -> np.ndarray:
    return np.asarray(
        [
            data[f"{prefix}_n_m"][index],
            data[f"{prefix}_e_m"][index],
            -data[f"{prefix}_d_m"][index],
        ],
        dtype=float,
    )


def plot_record(
    csv_path: Path,
    output_directory: Path,
    camera_width: int = 3840,
    camera_height: int = 2160,
    show: bool = False,
) -> tuple[Path, Path]:
    if camera_width <= 0 or camera_height <= 0:
        raise ValueError("camera dimensions must be positive")
    data = load_record(csv_path)
    output_directory.mkdir(parents=True, exist_ok=True)

    target = _finite_xyz(data, "target")
    pursuer = _finite_xyz(data, "pursuer")
    if len(target) == 0 or len(pursuer) == 0:
        raise ValueError("CSV has no finite Target/Pursuer trajectory")

    trajectory_path = output_directory / "trajectory_3d.png"
    figure = plt.figure(figsize=(10, 8))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(target[:, 0], target[:, 1], target[:, 2], label="Target", color="tab:red")
    axes.plot(
        pursuer[:, 0], pursuer[:, 1], pursuer[:, 2], label="Pursuer", color="tab:blue"
    )
    pose_points = np.column_stack(
        (data["pursuer_n_m"], data["pursuer_e_m"], -data["pursuer_d_m"])
    )
    minus_z_vectors = np.column_stack(
        (
            data["pursuer_body_minus_z_n"],
            data["pursuer_body_minus_z_e"],
            -data["pursuer_body_minus_z_d"],
        )
    )
    attitude_valid = np.all(np.isfinite(pose_points), axis=1) & np.all(
        np.isfinite(minus_z_vectors), axis=1
    )
    attitude_indices = np.flatnonzero(attitude_valid)
    if len(attitude_indices):
        sample_count = min(32, len(attitude_indices))
        sampled = attitude_indices[
            np.unique(np.linspace(0, len(attitude_indices) - 1, sample_count).astype(int))
        ]
        for line_number, index in enumerate(sampled):
            direction = minus_z_vectors[index]
            norm = np.linalg.norm(direction)
            if norm <= 1e-9:
                continue
            endpoint = pose_points[index] + 1.25 * direction / norm
            axes.plot(
                [pose_points[index, 0], endpoint[0]],
                [pose_points[index, 1], endpoint[1]],
                [pose_points[index, 2], endpoint[2]],
                color="darkviolet",
                linewidth=1.25,
                alpha=0.75,
                label="Pursuer body -Z / camera axis" if line_number == 0 else None,
            )
    axes.scatter(*target[0], color="darkred", marker="o", label="Target start")
    axes.scatter(*pursuer[0], color="navy", marker="^", label="Pursuer start")
    event_indices = capture_event_indices(data)
    for event_number, index in enumerate(event_indices, start=1):
        target_event = _xyz_at(data, "target", int(index))
        pursuer_event = _xyz_at(data, "pursuer", int(index))
        if not np.all(np.isfinite(target_event)) or not np.all(np.isfinite(pursuer_event)):
            continue
        axes.scatter(
            *target_event,
            color="red",
            edgecolor="black",
            marker="*",
            s=180,
            label="Target at trigger signal" if event_number == 1 else None,
            zorder=8,
        )
        axes.scatter(
            *pursuer_event,
            color="deepskyblue",
            edgecolor="black",
            marker="X",
            s=110,
            label="Pursuer at trigger signal" if event_number == 1 else None,
            zorder=8,
        )
        axes.plot(
            [pursuer_event[0], target_event[0]],
            [pursuer_event[1], target_event[1]],
            [pursuer_event[2], target_event[2]],
            color="black",
            linestyle=":",
            alpha=0.65,
        )
        body_x = np.asarray(
            [
                data["pursuer_body_x_n"][index],
                data["pursuer_body_x_e"][index],
                -data["pursuer_body_x_d"][index],
            ]
        )
        if not np.all(np.isfinite(body_x)):
            yaw = data["pursuer_yaw_rad"][index]
            if not np.isfinite(yaw):
                yaw = 0.0
            body_x = np.asarray([np.cos(yaw), np.sin(yaw), 0.0])
        axes.quiver(
            pursuer_event[0],
            pursuer_event[1],
            pursuer_event[2],
            body_x[0],
            body_x[1],
            body_x[2],
            length=1.5,
            normalize=True,
            color="limegreen",
            linewidth=2.0,
            arrow_length_ratio=0.25,
            label="Pursuer body +X (nose)" if event_number == 1 else None,
        )
        body_minus_z = np.asarray(
            [
                data["pursuer_body_minus_z_n"][index],
                data["pursuer_body_minus_z_e"][index],
                -data["pursuer_body_minus_z_d"][index],
            ]
        )
        alignment_text = ""
        if np.all(np.isfinite(body_minus_z)):
            axes.quiver(
                pursuer_event[0],
                pursuer_event[1],
                pursuer_event[2],
                body_minus_z[0],
                body_minus_z[1],
                body_minus_z[2],
                length=1.5,
                normalize=True,
                color="darkviolet",
                linewidth=2.0,
                arrow_length_ratio=0.25,
                label=None,
            )
            los = target_event - pursuer_event
            denominator = np.linalg.norm(los) * np.linalg.norm(body_minus_z)
            if denominator > 1e-9:
                cosine = np.clip(np.dot(los, body_minus_z) / denominator, -1.0, 1.0)
                asynchronous_truth_angle = np.degrees(np.arccos(cosine))
                pixel_error = data["capture_pixel_error_px"][index]
                if np.isfinite(pixel_error):
                    trigger_center_angle = np.degrees(
                        np.arctan2(pixel_error, camera_width / 2.0)
                    )
                    alignment_text = (
                        f"\ntrigger center {trigger_center_angle:.1f} deg"
                        f"\nasync truth {asynchronous_truth_angle:.1f} deg"
                    )
                else:
                    alignment_text = f"\nasync -Z/LOS {asynchronous_truth_angle:.1f} deg"
        axes.text(
            target_event[0],
            target_event[1],
            target_event[2] + 0.25,
            f"T{event_number} signal: Target",
            color="darkred",
        )
        axes.text(
            pursuer_event[0],
            pursuer_event[1],
            pursuer_event[2] - 0.35,
            f"T{event_number} signal: Pursuer{alignment_text}",
            color="navy",
        )
    axes.set_xlabel("North [m]")
    axes.set_ylabel("East [m]")
    axes.set_zlabel("Altitude (-Down) [m]")
    axes.set_title("Target–Pursuer trajectory in common NED")
    # Keep exact axis values but avoid a 30 m vertical span visually crushing the horizontal path.
    axes.set_box_aspect((1.0, 1.0, 1.5))
    axes.legend()
    axes.grid(True)
    figure.tight_layout()
    figure.savefig(trajectory_path, dpi=160)

    camera_path = output_directory / "camera_track.png"
    visible = (
        (data["camera_visible"] > 0.5)
        & np.isfinite(data["camera_u_px"])
        & np.isfinite(data["camera_v_px"])
    )
    figure_camera, camera_axes = plt.subplots(figsize=(10, 7.5))
    camera_axes.set_xlim(0, camera_width)
    camera_axes.set_ylim(camera_height, 0)
    camera_axes.set_aspect("equal", adjustable="box")
    camera_axes.set_xlabel("pixel u [px]")
    camera_axes.set_ylabel("pixel v [px]")
    camera_axes.set_title("Target trajectory in upward-camera image (active tracking highlighted)")
    camera_axes.grid(True, alpha=0.3)
    camera_axes.scatter(
        camera_width / 2.0 - 0.5,
        camera_height / 2.0 - 0.5,
        marker="+",
        s=120,
        linewidth=1.8,
        color="red",
        label="Optical center",
    )
    acceptance_radius_px = camera_width / 2.0 * np.tan(np.deg2rad(2.0))
    camera_axes.add_patch(
        plt.Circle(
            (camera_width / 2.0 - 0.5, camera_height / 2.0 - 0.5),
            acceptance_radius_px,
            fill=False,
            linestyle="--",
            linewidth=1.5,
            color="red",
            label="2 deg trigger region",
        )
    )
    active_tracking = visible & (data["mission_state"] >= 3.0) & (
        data["mission_state"] <= 10.0
    )
    inactive_visible = visible & ~active_tracking
    if np.any(inactive_visible):
        camera_axes.scatter(
            data["camera_u_px"][inactive_visible],
            data["camera_v_px"][inactive_visible],
            color="lightgray",
            s=7,
            alpha=0.45,
            label="Pre-tracking / reacquisition",
            zorder=1,
        )
    if np.any(active_tracking):
        elapsed = data["time_s"][active_tracking] - data["time_s"][active_tracking][0]
        scatter = camera_axes.scatter(
            data["camera_u_px"][active_tracking],
            data["camera_v_px"][active_tracking],
            c=elapsed,
            cmap="viridis",
            s=12,
            zorder=2,
        )
        figure_camera.colorbar(scatter, ax=camera_axes, label="active-tracking time [s]")
    elif not np.any(visible):
        camera_axes.text(
            0.5,
            0.5,
            "No visible camera samples",
            transform=camera_axes.transAxes,
            ha="center",
            va="center",
        )
    for event_number, index in enumerate(event_indices, start=1):
        u = data["camera_u_px"][index]
        v = data["camera_v_px"][index]
        if not data["camera_visible"][index] > 0.5 or not np.isfinite(u + v):
            continue
        camera_axes.scatter(
            u,
            v,
            marker="*",
            s=260,
            color="gold",
            edgecolor="black",
            linewidth=1.2,
            zorder=10,
            label="Trigger-signal target" if event_number == 1 else None,
        )
        camera_axes.annotate(
            f"T{event_number} ({u:.0f}, {v:.0f})",
            xy=(u, v),
            xytext=(12, -18 - 22 * (event_number - 1)),
            textcoords="offset points",
            arrowprops={"arrowstyle": "->", "color": "black"},
            fontsize=9,
            fontweight="bold",
        )
    if len(event_indices):
        camera_axes.legend(loc="best")
    figure_camera.tight_layout()
    figure_camera.savefig(camera_path, dpi=160)

    if show:
        plt.show()
    plt.close("all")
    return trajectory_path, camera_path
