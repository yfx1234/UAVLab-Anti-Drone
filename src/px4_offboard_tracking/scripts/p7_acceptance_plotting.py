#!/usr/bin/env python3
"""P7纯控制闭环验收记录的离线可视化。"""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path
from typing import Dict

import matplotlib

if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
    matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from pyulog import ULog  # noqa: E402


STATE_NAMES = (
    "INIT",
    "TAKEOFF_SAFE",
    "SEARCH",
    "BEARING_HANDOVER",
    "DEPTH_ACQUIRE",
    "INTERCEPT",
    "CAPTURE_CONFIRM",
    "EGRESS_REATTACK",
    "COMPLETE",
    "SAFETY_ABORT",
)


def _configure_chinese_font() -> None:
    matplotlib.rcParams["font.sans-serif"] = [
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    ]
    matplotlib.rcParams["axes.unicode_minus"] = False


_configure_chinese_font()


def _as_bool(value: str) -> bool:
    return value.strip().lower() in {"true", "1"}


def load_status_csv(path: Path) -> Dict[str, np.ndarray]:
    """读取ros2 topic echo --csv输出的PureInterceptorStatus。"""
    rows = []
    with path.open("r", newline="", encoding="utf-8") as stream:
        for row in csv.reader(stream):
            if len(row) >= 21:
                rows.append(row)
    if not rows:
        raise ValueError(f"状态CSV没有有效数据: {path}")

    stamp = np.asarray(
        [float(row[0]) + 1e-9 * float(row[1]) for row in rows], dtype=float
    )
    time_s = stamp - stamp[0]
    legacy_axis_error = np.degrees(
        np.asarray([float(row[17]) for row in rows], dtype=float)
    )
    has_expanded_axis_fields = all(len(row) >= 25 for row in rows)
    commanded_error = (
        np.degrees(np.asarray([float(row[21]) for row in rows], dtype=float))
        if has_expanded_axis_fields
        else legacy_axis_error.copy()
    )
    actual_error = (
        np.degrees(np.asarray([float(row[22]) for row in rows], dtype=float))
        if has_expanded_axis_fields
        else np.full(len(rows), np.nan)
    )
    tracking_error = (
        np.degrees(np.asarray([float(row[23]) for row in rows], dtype=float))
        if has_expanded_axis_fields
        else np.full(len(rows), np.nan)
    )
    estimate_coasting = (
        np.asarray([_as_bool(row[24]) for row in rows])
        if has_expanded_axis_fields
        else np.zeros(len(rows), dtype=bool)
    )
    return {
        "time_s": time_s,
        "state": np.asarray([int(row[3]) for row in rows], dtype=int),
        "state_name": np.asarray([row[4] for row in rows], dtype=object),
        "search_visible": np.asarray([_as_bool(row[8]) for row in rows]),
        "depth_visible": np.asarray([_as_bool(row[9]) for row in rows]),
        "estimate_valid": np.asarray([_as_bool(row[10]) for row in rows]),
        "guidance_valid": np.asarray([_as_bool(row[11]) for row in rows]),
        "capture_gate": np.asarray([_as_bool(row[12]) for row in rows]),
        "capture_count": np.asarray([int(row[13]) for row in rows], dtype=int),
        "required_capture_count": np.asarray(
            [int(row[14]) for row in rows], dtype=int
        ),
        "range_m": np.asarray([float(row[15]) for row in rows], dtype=float),
        "closing_speed_mps": np.asarray(
            [float(row[16]) for row in rows], dtype=float
        ),
        "attack_axis_error_deg": actual_error,
        "commanded_attack_axis_error_deg": commanded_error,
        "actual_attack_axis_error_deg": actual_error,
        "attack_axis_tracking_error_deg": tracking_error,
        "estimate_coasting": estimate_coasting,
        "expanded_axis_fields": np.asarray([has_expanded_axis_fields]),
        "search_age_s": np.asarray([float(row[18]) for row in rows], dtype=float),
        "depth_age_s": np.asarray([float(row[19]) for row in rows], dtype=float),
        "estimate_age_s": np.asarray([float(row[20]) for row in rows], dtype=float),
    }


def capture_indices(status: Dict[str, np.ndarray]) -> np.ndarray:
    count = status["capture_count"]
    return np.flatnonzero(np.concatenate(([False], np.diff(count) > 0)))


def _ulog_dataset(path: Path, name: str) -> Dict[str, np.ndarray]:
    data = ULog(str(path), message_name_filter_list=[name]).get_dataset(name).data
    return {key: np.asarray(value) for key, value in data.items()}


def load_vehicle_ulog(path: Path) -> Dict[str, np.ndarray]:
    position = _ulog_dataset(path, "vehicle_local_position_groundtruth")
    attitude = _ulog_dataset(path, "vehicle_attitude_groundtruth")
    time_s = position["timestamp"].astype(float) * 1e-6
    time_s -= time_s[0]
    attitude_time = attitude["timestamp"].astype(float) * 1e-6
    attitude_time -= attitude_time[0]
    quaternion = np.column_stack(
        [
            np.interp(time_s, attitude_time, attitude[f"q[{axis}]"])
            for axis in range(4)
        ]
    )
    quaternion /= np.linalg.norm(quaternion, axis=1)[:, None]
    position_ned = np.column_stack((position["x"], position["y"], position["z"]))
    position_ned -= position_ned[0]
    velocity_ned = np.column_stack((position["vx"], position["vy"], position["vz"]))
    speed = np.linalg.norm(velocity_ned, axis=1)
    global_position = _ulog_dataset(path, "vehicle_global_position_groundtruth")
    global_timestamp = global_position["timestamp_sample"].astype(float) * 1e-6
    return {
        "time_s": time_s,
        "absolute_time_s": position["timestamp"].astype(float) * 1e-6,
        "position_ned": position_ned,
        "velocity_ned": velocity_ned,
        "speed_mps": speed,
        "quaternion": quaternion,
        "global_time_s": global_timestamp,
        "latitude_deg": global_position["lat"].astype(float),
        "longitude_deg": global_position["lon"].astype(float),
        "altitude_m": global_position["alt"].astype(float),
    }


def attack_axis_ned(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = quaternion.T
    return np.column_stack(
        (
            -2.0 * (x * z + w * y),
            -2.0 * (y * z - w * x),
            -(1.0 - 2.0 * (x * x + y * y)),
        )
    )


def body_axes_ned(quaternion: np.ndarray) -> np.ndarray:
    """Return body FRD basis vectors expressed in local NED."""
    w, x, y, z = quaternion.T
    rotation = np.empty((len(quaternion), 3, 3), dtype=float)
    rotation[:, 0, 0] = 1.0 - 2.0 * (y * y + z * z)
    rotation[:, 0, 1] = 2.0 * (x * y - w * z)
    rotation[:, 0, 2] = 2.0 * (x * z + w * y)
    rotation[:, 1, 0] = 2.0 * (x * y + w * z)
    rotation[:, 1, 1] = 1.0 - 2.0 * (x * x + z * z)
    rotation[:, 1, 2] = 2.0 * (y * z - w * x)
    rotation[:, 2, 0] = 2.0 * (x * z - w * y)
    rotation[:, 2, 1] = 2.0 * (y * z + w * x)
    rotation[:, 2, 2] = 1.0 - 2.0 * (x * x + y * y)
    return rotation


def quaternion_euler_deg(quaternion: np.ndarray) -> tuple[float, float, float]:
    """PX4 body-FRD to NED quaternion as roll, pitch and yaw in degrees."""
    w, x, y, z = quaternion
    roll = math.atan2(
        2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)
    )
    pitch = math.asin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = math.atan2(
        2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)
    )
    return tuple(np.degrees((roll, pitch, yaw)))


def infer_profile(run_directory: Path, config_path: Path) -> Dict[str, float]:
    profile_name = "v0"
    summary = run_directory / "summary.txt"
    if summary.exists():
        for line in summary.read_text(encoding="utf-8").splitlines():
            if line.startswith("profile="):
                profile_name = line.split("=", 1)[1].strip()
                break
    config = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    if profile_name not in config["profiles"]:
        raise ValueError(f"配置中不存在档位: {profile_name}")
    return {"name": profile_name, **config["profiles"][profile_name]}


def _mark_captures(axes, status: Dict[str, np.ndarray]) -> None:
    for number, index in enumerate(capture_indices(status), start=1):
        axes.axvline(
            status["time_s"][index],
            color="goldenrod",
            linestyle=":",
            linewidth=1.8,
            label="打击事件" if number == 1 else None,
        )


def common_global_trajectories(
    target: Dict[str, np.ndarray], pursuer: Dict[str, np.ndarray]
) -> tuple[np.ndarray, np.ndarray]:
    """Convert both global groundtruth streams into one common NED frame."""
    earth_radius_m = 6378137.0
    latitude_0 = float(target["latitude_deg"][0])
    longitude_0 = float(target["longitude_deg"][0])
    altitude_0 = float(target["altitude_m"][0])

    def convert(vehicle: Dict[str, np.ndarray]) -> np.ndarray:
        north = np.radians(vehicle["latitude_deg"] - latitude_0) * earth_radius_m
        east = (
            np.radians(vehicle["longitude_deg"] - longitude_0)
            * earth_radius_m
            * math.cos(math.radians(latitude_0))
        )
        down = -(vehicle["altitude_m"] - altitude_0)
        return np.column_stack((north, east, down))

    return convert(target), convert(pursuer)


def capture_absolute_times(
    status: Dict[str, np.ndarray], pursuer: Dict[str, np.ndarray]
) -> np.ndarray:
    indices = capture_indices(status)
    if not len(indices):
        return np.empty(0)
    offset = pursuer["absolute_time_s"][-1] - status["time_s"][-1]
    return status["time_s"][indices] + offset


def _nearest_indices(sample_time: np.ndarray, query_time: np.ndarray) -> np.ndarray:
    return np.asarray(
        [int(np.argmin(np.abs(sample_time - value))) for value in query_time],
        dtype=int,
    )


def _plot_trajectory(
    status: Dict[str, np.ndarray],
    target: Dict[str, np.ndarray],
    pursuer: Dict[str, np.ndarray],
    horizontal_offset_m: float,
    output: Path,
) -> None:
    del horizontal_offset_m
    target_xyz, pursuer_xyz = common_global_trajectories(target, pursuer)
    target_xyz[:, 2] *= -1.0
    pursuer_xyz[:, 2] *= -1.0
    attitude_indices = _nearest_indices(
        pursuer["absolute_time_s"], pursuer["global_time_s"]
    )
    global_quaternion = pursuer["quaternion"][attitude_indices]
    axes_body = attack_axis_ned(global_quaternion)
    axes_body[:, 2] *= -1.0
    body_axes = body_axes_ned(global_quaternion)
    body_axes[:, 2, :] *= -1.0

    figure = plt.figure(figsize=(12, 9))
    axes = figure.add_subplot(111, projection="3d")
    axes.plot(*target_xyz.T, color="tab:red", linewidth=2, label="目标机 x500")
    axes.plot(
        *pursuer_xyz.T, color="tab:blue", linewidth=2, label="拦截机 22001"
    )
    valid = np.flatnonzero(np.all(np.isfinite(axes_body), axis=1))
    if len(valid):
        sampled = valid[
            np.unique(np.linspace(0, len(valid) - 1, min(40, len(valid))).astype(int))
        ]
        length = max(1.5, 0.035 * np.ptp(pursuer_xyz[:, 1]))
        for number, index in enumerate(sampled):
            end = pursuer_xyz[index] + length * axes_body[index]
            axes.plot(
                [pursuer_xyz[index, 0], end[0]],
                [pursuer_xyz[index, 1], end[1]],
                [pursuer_xyz[index, 2], end[2]],
                color="darkviolet",
                alpha=0.55,
                linewidth=1.0,
                label="机体 -Z / 攻击轴" if number == 0 else None,
            )
    capture_times = capture_absolute_times(status, pursuer)
    capture_vehicle_indices = _nearest_indices(
        pursuer["global_time_s"], capture_times
    )
    capture_axis_length = max(2.0, 0.05 * np.ptp(pursuer_xyz[:, 1]))
    capture_colors = ("tab:red", "tab:green", "darkviolet")
    capture_labels = ("机体 +X", "机体 +Y", "攻击轴 -Z")
    for number, vehicle_index in enumerate(capture_vehicle_indices, start=1):
        origin = pursuer_xyz[vehicle_index]
        vectors = (
            body_axes[vehicle_index, :, 0],
            body_axes[vehicle_index, :, 1],
            -body_axes[vehicle_index, :, 2],
        )
        axes.scatter(
            *origin,
            marker="*",
            s=180,
            color="gold",
            edgecolor="black",
            zorder=12,
        )
        roll, pitch, yaw = quaternion_euler_deg(
            global_quaternion[vehicle_index]
        )
        axes.text(
            origin[0],
            origin[1],
            origin[2],
            f"  打击{number}\n  R/P/Y={roll:.1f}°/{pitch:.1f}°/{yaw:.1f}°",
            fontsize=8,
            zorder=13,
        )
        for vector, color, label in zip(
            vectors, capture_colors, capture_labels
        ):
            end = origin + capture_axis_length * vector
            axes.plot(
                [origin[0], end[0]],
                [origin[1], end[1]],
                [origin[2], end[2]],
                color=color,
                linewidth=3.0,
                zorder=11,
                label=f"打击姿态：{label}" if number == 1 else None,
            )
    axes.scatter(*target_xyz[0], color="tab:red", marker="o", s=60)
    axes.scatter(*pursuer_xyz[0], color="tab:blue", marker="o", s=60)
    axes.set(
        title="P7闭环三维轨迹、连续攻击轴与两次打击姿态",
        xlabel="北 [m]",
        ylabel="东 [m]",
        zlabel="相对起始高度 [m]",
    )
    axes.legend()
    axes.grid(True)
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def _plot_trajectory_projections(
    status: Dict[str, np.ndarray],
    target: Dict[str, np.ndarray],
    pursuer: Dict[str, np.ndarray],
    output: Path,
) -> None:
    target_ned, pursuer_ned = common_global_trajectories(target, pursuer)
    target_xyz = target_ned.copy()
    pursuer_xyz = pursuer_ned.copy()
    target_xyz[:, 2] *= -1.0
    pursuer_xyz[:, 2] *= -1.0
    capture_times = capture_absolute_times(status, pursuer)
    target_capture = _nearest_indices(target["global_time_s"], capture_times)
    pursuer_capture = _nearest_indices(pursuer["global_time_s"], capture_times)

    figure, projection_axes = plt.subplots(1, 3, figsize=(18, 6))
    projections = (
        (0, 1, "北-东俯视投影", "北 [m]", "东 [m]"),
        (1, 2, "东-高度侧视投影", "东 [m]", "高度 [m]"),
        (0, 2, "北-高度侧视投影", "北 [m]", "高度 [m]"),
    )
    for axes, (horizontal, vertical, title, xlabel, ylabel) in zip(
        projection_axes, projections
    ):
        axes.plot(
            target_xyz[:, horizontal],
            target_xyz[:, vertical],
            color="tab:red",
            linewidth=2,
            label="目标机 x500",
        )
        axes.plot(
            pursuer_xyz[:, horizontal],
            pursuer_xyz[:, vertical],
            color="tab:blue",
            linewidth=2,
            label="拦截机 22001",
        )
        for number, (target_index, pursuer_index) in enumerate(
            zip(target_capture, pursuer_capture), start=1
        ):
            axes.scatter(
                pursuer_xyz[pursuer_index, horizontal],
                pursuer_xyz[pursuer_index, vertical],
                marker="*",
                s=180,
                color="gold",
                edgecolor="black",
                zorder=8,
            )
            axes.plot(
                [
                    pursuer_xyz[pursuer_index, horizontal],
                    target_xyz[target_index, horizontal],
                ],
                [
                    pursuer_xyz[pursuer_index, vertical],
                    target_xyz[target_index, vertical],
                ],
                color="darkviolet",
                linestyle="--",
                linewidth=1.5,
                label="打击时LOS" if number == 1 else None,
            )
            axes.annotate(
                f"打击{number}",
                (
                    pursuer_xyz[pursuer_index, horizontal],
                    pursuer_xyz[pursuer_index, vertical],
                ),
                xytext=(6, 6),
                textcoords="offset points",
            )
        axes.set(title=title, xlabel=xlabel, ylabel=ylabel)
        axes.grid(True)
        axes.axis("equal")
    projection_axes[0].legend()
    figure.suptitle("双机轨迹正交投影与两次打击LOS")
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def _plot_camera_status(status: Dict[str, np.ndarray], output: Path) -> None:
    time_s = status["time_s"]
    figure, (visibility, error) = plt.subplots(
        2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [1, 2]}
    )
    visibility.fill_between(
        time_s, 0.0, status["search_visible"], step="post", alpha=0.55, label="搜索相机"
    )
    visibility.fill_between(
        time_s,
        1.15,
        1.15 + status["depth_visible"],
        step="post",
        alpha=0.55,
        label="双目深度相机",
    )
    visibility.fill_between(
        time_s,
        2.30,
        2.30 + status["estimate_valid"],
        step="post",
        alpha=0.55,
        label="IMM-EKF有效",
    )
    visibility.fill_between(
        time_s,
        2.30,
        2.30 + status["estimate_coasting"],
        step="post",
        color="tab:purple",
        alpha=0.65,
        label="IMM预测续航",
    )
    for index in capture_indices(status):
        visibility.scatter(
            [time_s[index]] * 3,
            [0.5, 1.65, 2.8],
            marker="*",
            s=100,
            color="gold",
            edgecolor="black",
            zorder=9,
        )
    visibility.set(
        title="双相机可见性与估计交接",
        yticks=[0.5, 1.65, 2.8],
        yticklabels=["搜索", "双目", "估计"],
        ylim=(0, 3.35),
    )
    visibility.grid(True, axis="x", alpha=0.25)
    visibility.legend(ncol=3, loc="upper right")

    actual = status["actual_attack_axis_error_deg"]
    commanded = status["commanded_attack_axis_error_deg"]
    tracking = status["attack_axis_tracking_error_deg"]
    actual_valid = np.isfinite(actual)
    commanded_valid = np.isfinite(commanded)
    tracking_valid = np.isfinite(tracking)
    error.plot(
        time_s[actual_valid],
        actual[actual_valid],
        color="tab:purple",
        linewidth=1.5,
        label="实际机体-Z/LOS误差",
    )
    error.plot(
        time_s[commanded_valid],
        commanded[commanded_valid],
        color="tab:gray",
        linewidth=1.0,
        linestyle="-.",
        label="制导期望轴/LOS误差",
    )
    error.plot(
        time_s[tracking_valid],
        tracking[tracking_valid],
        color="tab:cyan",
        linewidth=1.0,
        label="实际-Z/期望轴跟踪误差",
    )
    error.axhline(1.0, color="tab:green", linestyle="--", label="优选1°")
    error.axhline(2.0, color="tab:red", linestyle="--", label="硬门2°")
    _mark_captures(error, status)
    finite_errors = np.concatenate(
        [
            values[np.isfinite(values)]
            for values in (actual, commanded, tracking)
            if np.any(np.isfinite(values))
        ]
    )
    upper_error = max(5.0, float(np.percentile(finite_errors, 98))) \
        if len(finite_errors) else 5.0
    error.set(
        title="双目瞄准误差（相机中心即0°）",
        xlabel="任务时间 [s]",
        ylabel="角误差 [deg]",
        ylim=(0, upper_error),
    )
    error.grid(True)
    error.legend(ncol=4)
    if not bool(status["expanded_axis_fields"][0]):
        error.text(
            0.01,
            0.93,
            "旧记录未保存实际-Z误差；灰线仅为旧版制导命令字段",
            transform=error.transAxes,
            color="tab:red",
        )
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def _plot_guidance(status: Dict[str, np.ndarray], output: Path) -> None:
    time_s = status["time_s"]
    figure, (distance, closing) = plt.subplots(2, 1, figsize=(13, 9), sharex=True)
    distance.axhspan(15.0, 35.0, color="tab:green", alpha=0.14, label="15～35 m打击区")
    distance.axhline(25.0, color="tab:green", linestyle="--", label="25 m调节中心")
    distance.plot(time_s, status["range_m"], color="tab:blue", label="双目/估计距离")
    _mark_captures(distance, status)
    for number, index in enumerate(capture_indices(status), start=1):
        distance.scatter(
            time_s[index],
            status["range_m"][index],
            marker="*",
            s=220,
            color="gold",
            edgecolor="black",
            zorder=8,
        )
        distance.annotate(
            f"打击{number}: {status['range_m'][index]:.2f} m",
            (time_s[index], status["range_m"][index]),
            xytext=(8, 10),
            textcoords="offset points",
        )
    distance.set(title="打击距离窗口", ylabel="距离 [m]")
    distance.grid(True)
    distance.legend(ncol=3)

    closing.axhline(0.0, color="black", linewidth=0.8)
    closing.plot(
        time_s,
        status["closing_speed_mps"],
        color="tab:orange",
        label="闭合速度（正值为接近）",
    )
    _mark_captures(closing, status)
    closing.set(
        title="径向闭合速度", xlabel="任务时间 [s]", ylabel="闭合速度 [m/s]"
    )
    closing.grid(True)
    closing.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def _plot_state_and_motion(
    status: Dict[str, np.ndarray],
    target: Dict[str, np.ndarray],
    pursuer: Dict[str, np.ndarray],
    profile: Dict[str, float],
    output: Path,
) -> None:
    figure, (state_axes, speed_axes, tilt_axes) = plt.subplots(
        3, 1, figsize=(14, 11), gridspec_kw={"height_ratios": [1.3, 1, 1]}
    )
    state_axes.step(status["time_s"], status["state"], where="post", color="tab:blue")
    _mark_captures(state_axes, status)
    present_states = sorted(set(status["state"].tolist()))
    state_axes.set(
        title="任务状态机时序",
        ylabel="状态",
        yticks=present_states,
        yticklabels=[STATE_NAMES[state] for state in present_states],
    )
    state_axes.grid(True)

    speed_axes.plot(target["time_s"], target["speed_mps"], label="目标机速度")
    speed_axes.plot(pursuer["time_s"], pursuer["speed_mps"], label="拦截机速度")
    speed_axes.axhline(
        float(profile["pursuer_command_speed_mps"]),
        color="tab:orange",
        linestyle="--",
        label="指令软上限",
    )
    speed_axes.axhline(
        float(profile["pursuer_safety_speed_mps"]),
        color="tab:red",
        linestyle="--",
        label="安全上限",
    )
    speed_axes.set(title="飞行速度", ylabel="速度 [m/s]")
    speed_axes.grid(True)
    speed_axes.legend(ncol=4)

    q = pursuer["quaternion"]
    body_z_down = 1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2)
    tilt_deg = np.degrees(np.arccos(np.clip(body_z_down, -1.0, 1.0)))
    tilt_axes.plot(pursuer["time_s"], tilt_deg, color="tab:purple", label="拦截机倾角")
    tilt_axes.axhline(
        math.degrees(float(profile["maximum_tilt_rad"])),
        color="tab:red",
        linestyle="--",
        label="配置倾角上限",
    )
    tilt_axes.set(
        title="拦截机姿态平滑性", xlabel="ULog飞行时间 [s]", ylabel="倾角 [deg]"
    )
    tilt_axes.grid(True)
    tilt_axes.legend()
    figure.tight_layout()
    figure.savefig(output, dpi=180)
    plt.close(figure)


def plot_p7_run(
    run_directory: Path,
    config_path: Path,
) -> tuple[Path, Path, Path, Path, Path]:
    data_directory = run_directory / "data"
    status = load_status_csv(data_directory / "status.csv")
    target = load_vehicle_ulog(data_directory / "px4_0.ulg")
    pursuer = load_vehicle_ulog(data_directory / "px4_1.ulg")
    profile = infer_profile(run_directory, config_path)
    outputs = (
        run_directory / "01_三维轨迹与攻击轴.png",
        run_directory / "02_双相机可见性与瞄准.png",
        run_directory / "03_距离窗口与闭合速度.png",
        run_directory / "04_状态机速度与姿态.png",
        run_directory / "05_轨迹正交投影.png",
    )
    _plot_trajectory(
        status,
        target,
        pursuer,
        float(profile["initial_horizontal_offset_m"]),
        outputs[0],
    )
    _plot_camera_status(status, outputs[1])
    _plot_guidance(status, outputs[2])
    _plot_state_and_motion(status, target, pursuer, profile, outputs[3])
    _plot_trajectory_projections(status, target, pursuer, outputs[4])
    return outputs
