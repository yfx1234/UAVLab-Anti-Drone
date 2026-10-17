#!/usr/bin/env python3
"""Analyze and plot one predictive-shot simulation rosbag."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def _strict_json_value(value):
    """Replace non-finite numeric diagnostics with JSON null recursively."""
    if isinstance(value, dict):
        return {key: _strict_json_value(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_strict_json_value(item) for item in value]
    if isinstance(value, (float, np.floating)) and not math.isfinite(float(value)):
        return None
    if isinstance(value, np.integer):
        return int(value)
    return value
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


class AnalysisInputError(RuntimeError):
    """The rosbag does not contain the minimum data needed for analysis."""


def _stamp_seconds(message) -> float:
    stamp = message.header.stamp
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _read_bag(bag_directory: Path) -> dict[str, list[tuple[float, object]]]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    output = {topic: [] for topic in topic_types}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        message_type = get_message(topic_types[topic])
        output[topic].append(
            (float(timestamp_ns) * 1.0e-9, deserialize_message(serialized, message_type))
        )
    return output


def _series(records, extractor):
    if not records:
        return np.empty(0), np.empty((0, 1))
    times = np.asarray([record[0] for record in records], dtype=float)
    values = np.asarray([extractor(record[1]) for record in records], dtype=float)
    return times, values


def _interp_vector(source_time, source_values, destination_time):
    if source_time.size < 2 or destination_time.size == 0:
        return np.empty((0, source_values.shape[1] if source_values.ndim == 2 else 1))
    return np.column_stack(
        [
            np.interp(destination_time, source_time, source_values[:, axis])
            for axis in range(source_values.shape[1])
        ]
    )


def _first_sustained_time(
    times: np.ndarray,
    condition: np.ndarray,
    start_time: float,
    hold_s: float,
) -> float:
    """Return seconds from start until a sampled condition remains true."""
    since = math.nan
    for timestamp, passed in zip(times, condition):
        if timestamp < start_time:
            continue
        if not passed:
            since = math.nan
            continue
        if not math.isfinite(since):
            since = float(timestamp)
        if timestamp - since >= hold_s:
            return max(0.0, float(since - start_time))
    return math.nan


def _configure_chinese_font() -> None:
    candidates = [
        "Noto Sans CJK SC",
        "Source Han Sans SC",
        "WenQuanYi Micro Hei",
        "Microsoft YaHei",
        "SimHei",
    ]
    available = {font.name for font in matplotlib.font_manager.fontManager.ttflist}
    for candidate in candidates:
        if candidate in available:
            plt.rcParams["font.sans-serif"] = [candidate, "DejaVu Sans"]
            break
    plt.rcParams["axes.unicode_minus"] = False


def _read_run_parameters(run_directory: Path) -> dict[str, str]:
    summary_path = run_directory / "原始数据" / "run_summary.txt"
    if not summary_path.is_file():
        summary_path = run_directory / "run_summary.txt"
    parameters: dict[str, str] = {}
    if not summary_path.is_file():
        return parameters
    for line in summary_path.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition("=")
        if separator:
            parameters[key.strip()] = value.strip()
    return parameters


def _theoretical_target_peak_acceleration(
    scenario: str,
    speed_mps: float,
    radius_m: float,
    path_length_m: float,
    path_width_m: float,
) -> float:
    if scenario == "circle":
        return speed_mps * speed_mps / max(radius_m, 1.0e-6)
    if scenario == "s_curve":
        amplitude = 0.5 * path_width_m
        wave_number = 2.0 * math.pi / max(path_length_m, 1.0e-6)
        tangent_norm = math.sqrt(
            1.0 + (amplitude * wave_number) ** 2
        )
        forward_speed = speed_mps / tangent_norm
        return (
            amplitude
            * wave_number
            * wave_number
            * forward_speed
            * forward_speed
        )
    if scenario == "figure_eight":
        phase = np.linspace(0.0, 2.0 * math.pi, 8192, endpoint=False)
        amplitude_x = 0.5 * path_length_m
        amplitude_y = 0.5 * path_width_m
        derivative_norm = np.hypot(
            amplitude_x * np.sin(phase),
            2.0 * amplitude_y * np.cos(2.0 * phase),
        )
        phase_rate = speed_mps / max(float(np.max(derivative_norm)), 1.0e-6)
        acceleration_norm = phase_rate * phase_rate * np.hypot(
            amplitude_x * np.cos(phase),
            4.0 * amplitude_y * np.sin(2.0 * phase),
        )
        return float(np.max(acceleration_norm))
    if scenario == "rectangle":
        corner_radius = 0.25 * min(path_length_m, path_width_m)
        return speed_mps * speed_mps / max(corner_radius, 1.0e-6)
    return 0.0


def _estimate_values(message) -> tuple[float, ...]:
    covariance = np.asarray(message.full_covariance, dtype=float).reshape(9, 9)
    position_sigma = math.sqrt(
        max(0.0, float(np.trace(covariance[0:3, 0:3])))
    )
    velocity_sigma = math.sqrt(
        max(0.0, float(np.trace(covariance[3:6, 3:6])))
    )
    acceleration_sigma = math.sqrt(
        max(0.0, float(np.max(np.diag(covariance[6:9, 6:9]))))
    )
    return (
        message.position.x,
        message.position.y,
        message.position.z,
        message.velocity.x,
        message.velocity.y,
        message.velocity.z,
        1.0 if message.valid else 0.0,
        1.0 if message.measurement_accepted else 0.0,
        message.coast_time_s,
        position_sigma,
        velocity_sigma,
        message.acceleration.x,
        message.acceleration.y,
        message.acceleration.z,
        message.model_probabilities[1],
        acceleration_sigma,
    )


def _body_minus_z_from_quaternion(quaternion: np.ndarray) -> np.ndarray:
    w = quaternion[:, 0]
    x = quaternion[:, 1]
    y = quaternion[:, 2]
    z = quaternion[:, 3]
    body_plus_z = np.column_stack(
        (
            2.0 * (x * z + w * y),
            2.0 * (y * z - w * x),
            1.0 - 2.0 * (x * x + y * y),
        )
    )
    return -body_plus_z


def _referee_result_dict(result) -> dict:
    return {
        "shot_id": int(result.shot_id),
        "hit": bool(result.hit),
        "result": int(result.result),
        "hit_radius_m": float(result.hit_radius_m),
        "minimum_miss_distance_m": float(result.minimum_miss_distance_m),
        "time_to_closest_approach_s": float(
            result.time_to_closest_approach_s
        ),
        "reason": result.reason,
    }


def analyze(run_directory: Path) -> dict:
    run_parameters = _read_run_parameters(run_directory)
    analysis_plot_count = int(run_parameters.get("analysis_plot_count", "6"))
    target_speed_setting_mps = float(
        run_parameters.get("target_speed_mps", "2.0")
    )
    command_speed_setting_mps = float(
        run_parameters.get("command_maximum_speed_mps", "6.0")
    )
    feedforward_gain_setting = float(
        run_parameters.get("target_acceleration_feedforward_gain", "0.2")
    )
    feedforward_minimum_ca_probability = float(
        run_parameters.get(
            "target_acceleration_feedforward_minimum_ca_probability", "0.25"
        )
    )
    feedforward_full_ca_probability = float(
        run_parameters.get(
            "target_acceleration_feedforward_full_ca_probability", "0.75"
        )
    )
    feedforward_maximum_stddev_mps2 = float(
        run_parameters.get(
            "target_acceleration_feedforward_maximum_stddev_mps2", "5.0"
        )
    )
    preferred_range_setting_m = float(
        run_parameters.get("preferred_capture_range_m", "17.0")
    )
    minimum_height_setting_m = float(
        run_parameters.get("minimum_flight_height_m", "3.0")
    )
    depth_fov_preferred_angle_deg = float(
        run_parameters.get("depth_fov_preferred_angle_deg", "5.0")
    )
    depth_fov_normal_angle_deg = float(
        run_parameters.get("depth_fov_normal_angle_deg", "5.0")
    )
    depth_fov_recovery_exit_angle_deg = float(
        run_parameters.get("depth_fov_recovery_exit_angle_deg", "6.0")
    )
    depth_fov_hard_angle_deg = float(
        run_parameters.get("depth_fov_hard_angle_deg", "8.5")
    )
    recoil_duration_setting_s = float(
        run_parameters.get("recoil_duration_s", "0.2")
    )
    recoil_opening_speed_setting_mps = float(
        run_parameters.get("recoil_opening_speed_mps", "1.0")
    )
    reattack_cooldown_setting_s = float(
        run_parameters.get("reattack_cooldown_s", "3.0")
    )
    target_scenario_setting = run_parameters.get("scenario", "未知")
    target_radius_setting_m = float(
        run_parameters.get("target_radius_m", "20.0")
    )
    target_path_length_setting_m = float(
        run_parameters.get("target_path_length_m", "40.0")
    )
    target_path_width_setting_m = float(
        run_parameters.get("target_path_width_m", "12.0")
    )
    theoretical_target_peak_acceleration_mps2 = (
        _theoretical_target_peak_acceleration(
            target_scenario_setting,
            target_speed_setting_mps,
            target_radius_setting_m,
            target_path_length_setting_m,
            target_path_width_setting_m,
        )
    )
    nested_bag_directory = run_directory / "原始数据" / "rosbag"
    bag_directory = (
        nested_bag_directory
        if nested_bag_directory.is_dir()
        else run_directory / "rosbag"
    )
    if not bag_directory.is_dir():
        raise FileNotFoundError(f"找不到rosbag目录: {bag_directory}")
    records = _read_bag(bag_directory)
    pursuer_prefix = run_parameters.get("pursuer_prefix", "/px4_1/fmu").rstrip("/")

    local_time, local = _series(
        records.get(f"{pursuer_prefix}/out/vehicle_local_position", []),
        lambda msg: (
            msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz,
            msg.ax, msg.ay, msg.az,
        ),
    )
    task_origin_time, task_origin = _series(
        records.get("/tracking/task_origin_ned", []),
        lambda msg: (msg.point.x, msg.point.y, msg.point.z),
    )
    if local.size and task_origin.size:
        # PX4 local coordinates remain the native setpoint frame.  All plots
        # and reconstructed absolute target positions use the latched
        # propeller-start task origin published by the controller.  A later
        # PX4 estimator reset republishes the compensated numeric origin, so
        # select the most recent origin for each local-position sample.
        origin_indices = np.searchsorted(
            task_origin_time, local_time, side="right"
        ) - 1
        origin_indices = np.clip(origin_indices, 0, task_origin.shape[0] - 1)
        local[:, 0:3] -= task_origin[origin_indices, 0:3]
    truth_time, truth = _series(
        records.get("/evaluation/relative_truth", []),
        lambda msg: (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ),
    )
    estimate_time, estimate = _series(
        records.get("/tracking/relative/estimate", []),
        _estimate_values,
    )
    status_time, status = _series(
        records.get("/tracking/pure_interceptor/status", []),
        lambda msg: (
            msg.state,
            msg.range_m,
            msg.actual_attack_axis_error_rad,
            msg.actual_shot_axis_error_rad,
            msg.predicted_shot_time_of_flight_s,
            1.0 if msg.search_visible else 0.0,
            1.0 if msg.depth_visible else 0.0,
            1.0 if msg.estimate_valid else 0.0,
            1.0 if msg.guidance_valid else 0.0,
            msg.commanded_attack_axis_error_rad,
            msg.attack_axis_tracking_error_rad,
            1.0 if msg.estimate_coasting else 0.0,
            msg.height_m,
            msg.minimum_flight_height_m,
            1.0 if msg.minimum_height_constraint_active else 0.0,
            msg.minimum_height_acceleration_limit_down_mps2,
            1.0 if msg.capture_gate_passed else 0.0,
            1.0 if msg.shot_solution_valid else 0.0,
            msg.predicted_shot_lateral_sigma_m,
            msg.closing_speed_mps,
            msg.capture_count,
            msg.fov_barrier_zone,
            msg.fov_horizontal_error_rad,
            msg.fov_vertical_error_rad,
            msg.fov_predicted_horizontal_error_rad,
            msg.fov_predicted_vertical_error_rad,
            msg.fov_predicted_maximum_error_rad,
            1.0 if msg.fov_approach_paused else 0.0,
            msg.active_range_reference_m,
            msg.active_range_lower_bound_m,
            msg.active_range_upper_bound_m,
            1.0 if msg.recoil_active else 0.0,
            1.0 if msg.reattack_cooldown_active else 0.0,
            msg.reattack_cooldown_remaining_s,
            1.0 if msg.tracking_ready else 0.0,
            msg.continuous_tracking_duration_s,
            1.0 if msg.terminal_alignment_active else 0.0,
            msg.terminal_alignment_strength,
            1.0 if msg.fire_authorized else 0.0,
            msg.attack_enable_delay_remaining_s,
            msg.first_attack_tracking_hold_remaining_s,
            msg.fov_approach_scale,
            1.0 if msg.shot_hard_gate_passed else 0.0,
            1.0 if msg.shot_timing_gate_passed else 0.0,
            msg.shot_timing_predicted_axis_error_rad,
            msg.shot_timing_axis_error_rate_rad_s,
            msg.shot_timing_score,
            1.0 if msg.launch_planner_enabled else 0.0,
            1.0 if msg.launch_planner_active else 0.0,
            msg.launch_planner_blend,
            msg.planned_launch_horizon_s,
            msg.planned_launch_range_m,
            msg.planned_launch_cost,
            msg.planned_acceleration_ned.x,
            msg.planned_acceleration_ned.y,
            msg.planned_acceleration_ned.z,
            msg.planned_relative_velocity_at_launch_ned.x,
            msg.planned_relative_velocity_at_launch_ned.y,
            msg.planned_relative_velocity_at_launch_ned.z,
            msg.radial_control_source,
            1.0 if msg.height_recovery_active else 0.0,
            msg.hover_thrust,
            msg.maximum_thrust,
            msg.commanded_thrust,
            msg.requested_thrust_before_saturation,
            1.0 if msg.thrust_saturated else 0.0,
            msg.commanded_acceleration_ned.x,
            msg.commanded_acceleration_ned.y,
            msg.commanded_acceleration_ned.z,
            msg.local_velocity_ned.x,
            msg.local_velocity_ned.y,
            msg.local_velocity_ned.z,
            1.0 if msg.attack_complete else 0.0,
            1.0 if msg.completion_velocity_braking_active else 0.0,
            1.0 if msg.completion_position_hold_active else 0.0,
            1.0 if msg.safe_to_exit_offboard else 0.0,
            1.0 if msg.attack_height_gate_passed else 0.0,
        ),
    )
    attitude_time, attitude = _series(
        records.get(f"{pursuer_prefix}/out/vehicle_attitude", []),
        lambda msg: (msg.q[0], msg.q[1], msg.q[2], msg.q[3]),
    )
    offboard_mode_time, offboard_mode = _series(
        records.get(f"{pursuer_prefix}/in/offboard_control_mode", []),
        lambda msg: (
            1.0 if msg.position else 0.0,
            1.0 if msg.attitude else 0.0,
        ),
    )
    search_time, search = _series(
        records.get("/perception/search/observation", []),
        lambda msg: (
            msg.pixel_u,
            msg.pixel_v,
            msg.image_width,
            msg.image_height,
            1.0 if msg.visible else 0.0,
        ),
    )
    depth_time, depth = _series(
        records.get("/perception/depth/observation", []),
        lambda msg: (
            msg.pixel_u,
            msg.pixel_v,
            msg.image_width,
            msg.image_height,
            msg.range_m,
            1.0 if msg.visible else 0.0,
        ),
    )
    shots = [
        message
        for _, message in records.get("/tracking/virtual_shot_event", [])
    ]
    hits = [
        message
        for _, message in records.get("/evaluation/virtual_hit_result", [])
    ]
    solver_hits = [
        message
        for _, message in records.get(
            "/evaluation/virtual_solver_hit_result", []
        )
    ]
    referee_mode = (
        "physical-axis-with-separate-solver"
        if "/evaluation/virtual_solver_hit_result" in records
        else "legacy-solver-direction"
    )
    mission_success = any(
        message.data
        for _, message in records.get("/evaluation/mission_success", [])
    )

    required_series = {
        f"{pursuer_prefix}/out/vehicle_local_position": local_time,
        "/evaluation/relative_truth": truth_time,
        "/tracking/relative/estimate": estimate_time,
        "/tracking/pure_interceptor/status": status_time,
        f"{pursuer_prefix}/out/vehicle_attitude": attitude_time,
        "/perception/search/observation": search_time,
        "/perception/depth/observation": depth_time,
    }
    missing_topics = [
        topic for topic, timestamps in required_series.items()
        if timestamps.size == 0
    ]
    if missing_topics:
        raise AnalysisInputError(
            "rosbag缺少分析必需消息（可能是核心节点启动失败）: "
            + ", ".join(missing_topics)
        )

    all_nonempty_times = [
        series
        for series in (local_time, truth_time, estimate_time, status_time)
        if series.size
    ]
    time_origin = min(series[0] for series in all_nonempty_times)
    local_elapsed = local_time - time_origin
    truth_elapsed = truth_time - time_origin
    estimate_elapsed = estimate_time - time_origin
    status_elapsed = status_time - time_origin

    pursuer_at_truth = _interp_vector(local_time, local[:, :6], truth_time)
    target = pursuer_at_truth[:, :3] + truth[:, :3]
    target_velocity = pursuer_at_truth[:, 3:6] + truth[:, 3:6]
    maximum_target_speed_mps = float(
        np.max(np.linalg.norm(target_velocity, axis=1))
    )
    # The target PX4's automatic takeoff contributes about 3 m/s vertically.
    # It is unrelated to the configured horizontal trajectory speed and made
    # every low-speed case fail strict acceptance before interception began.
    maximum_target_horizontal_speed_mps = float(
        np.max(np.linalg.norm(target_velocity[:, :2], axis=1))
    )
    valid_estimate_mask = estimate[:, 6] > 0.5
    valid_estimate_time = estimate_time[valid_estimate_mask]
    valid_estimate = estimate[valid_estimate_mask, :6]
    estimate_at_truth = _interp_vector(
        valid_estimate_time, valid_estimate, truth_time
    )
    valid_time_mask = (
        (truth_time >= valid_estimate_time[0])
        & (truth_time <= valid_estimate_time[-1])
        if valid_estimate_time.size
        else np.zeros(truth_time.size, dtype=bool)
    )
    relative_position_rmse_m = math.nan
    relative_velocity_rmse_mps = math.nan
    if valid_time_mask.any() and estimate_at_truth.shape[0] == truth.shape[0]:
        position_error = estimate_at_truth[valid_time_mask, :3] - truth[
            valid_time_mask, :3
        ]
        velocity_error = estimate_at_truth[valid_time_mask, 3:6] - truth[
            valid_time_mask, 3:6
        ]
        relative_position_rmse_m = float(
            np.sqrt(np.mean(np.sum(position_error * position_error, axis=1)))
        )
        relative_velocity_rmse_mps = float(
            np.sqrt(np.mean(np.sum(velocity_error * velocity_error, axis=1)))
        )

    shot_times = [_stamp_seconds(shot) for shot in shots]
    shot_interval_s = (
        shot_times[1] - shot_times[0] if len(shot_times) >= 2 else math.nan
    )
    speed = np.linalg.norm(local[:, 3:6], axis=1)
    horizontal_speed = np.linalg.norm(local[:, 3:5], axis=1)
    maximum_vertical_speed_mps = float(np.max(np.abs(local[:, 5])))
    states = status[:, 0].astype(int) if status.size else np.empty(0, dtype=int)
    transitions = int(np.count_nonzero(np.diff(states))) if states.size else 0
    search_returns = int(
        np.count_nonzero((states[1:] == 2) & (states[:-1] != 2))
    ) if states.size > 1 else 0
    mission_indices = np.flatnonzero(states == 2)
    mission_start_time = (
        float(status_time[mission_indices[0]])
        if mission_indices.size else float(status_time[0])
    )
    first_horizontal_crossing_time = math.nan
    time_to_first_horizontal_crossing_s = math.nan
    mission_truth_indices = np.flatnonzero(truth_time >= mission_start_time)
    if mission_truth_indices.size:
        first_truth_index = int(mission_truth_indices[0])
        initial_horizontal_relative = truth[first_truth_index, :2]
        initial_horizontal_range = float(np.linalg.norm(initial_horizontal_relative))
        if initial_horizontal_range > 1.0e-3:
            initial_approach_axis = initial_horizontal_relative / initial_horizontal_range
            signed_along_track = truth[:, :2] @ initial_approach_axis
            previous_index = first_truth_index
            for current_index in mission_truth_indices[1:]:
                current_index = int(current_index)
                previous_value = float(signed_along_track[previous_index])
                current_value = float(signed_along_track[current_index])
                if previous_value > 0.0 and current_value <= 0.0:
                    denominator = previous_value - current_value
                    fraction = previous_value / denominator if denominator > 1.0e-9 else 1.0
                    first_horizontal_crossing_time = float(
                        truth_time[previous_index] + fraction *
                        (truth_time[current_index] - truth_time[previous_index])
                    )
                    time_to_first_horizontal_crossing_s = max(
                        0.0, first_horizontal_crossing_time - mission_start_time
                    )
                    break
                previous_index = current_index
    bearing_indices = np.flatnonzero((states == 3) & (status_time >= mission_start_time))
    time_to_first_bearing_s = (
        float(status_time[bearing_indices[0]] - mission_start_time)
        if bearing_indices.size else math.nan
    )
    shot_heights_m = [
        float(np.interp(shot_time, status_time, status[:, 12]))
        for shot_time in shot_times
    ]
    minimum_tracking_height_m = math.nan
    maximum_tracking_vertical_speed_mps = math.nan
    depth_visible_rate = math.nan
    fov_approach_paused_rate = math.nan
    mean_fov_approach_scale = math.nan
    ca_probability_p90 = math.nan
    acceleration_stddev_p90_mps2 = math.nan
    feedforward_confidence_p90 = math.nan
    feedforward_active_rate = math.nan
    launch_planner_active_rate = math.nan
    launch_planner_mean_blend = math.nan
    launch_planner_mean_horizon_s = math.nan
    launch_planner_mean_range_m = math.nan
    launch_planner_maximum_acceleration_mps2 = math.nan
    launch_planner_mean_terminal_relative_speed_mps = math.nan
    intercept_indices = np.flatnonzero(states == 6)
    time_to_first_intercept_s = (
        float(status_time[intercept_indices[0]] - mission_start_time)
        if intercept_indices.size else math.nan
    )
    target_speed_at_local = np.linalg.norm(
        _interp_vector(truth_time, target_velocity, local_time), axis=1
    )
    time_to_speed_match_s = _first_sustained_time(
        local_time,
        np.abs(speed - target_speed_at_local) <= 1.0,
        mission_start_time,
        1.0,
    )
    fire_indices = np.flatnonzero(
        (status[:, 38] > 0.5) & (status_time >= mission_start_time)
    )
    time_to_fire_authorized_s = (
        float(status_time[fire_indices[0]] - mission_start_time)
        if fire_indices.size else math.nan
    )
    first_shot_before_horizontal_crossing = bool(
        shot_times and math.isfinite(first_horizontal_crossing_time) and
        shot_times[0] <= first_horizontal_crossing_time
    )
    first_hit_before_horizontal_crossing = bool(
        first_shot_before_horizontal_crossing and hits and bool(hits[0].hit)
    )
    head_on_first_pass_success = first_hit_before_horizontal_crossing
    time_to_first_shot_s = (
        max(0.0, shot_times[0] - mission_start_time) if shot_times else math.nan
    )
    first_shot_margin_to_crossing_s = (
        first_horizontal_crossing_time - shot_times[0]
        if shot_times and math.isfinite(first_horizontal_crossing_time) else math.nan
    )
    crossing_state = -1
    crossing_actual_shot_axis_error_deg = math.nan
    crossing_capture_gate_passed = False
    crossing_fire_authorized = False
    crossing_shot_hard_gate_passed = False
    crossing_shot_timing_gate_passed = False
    crossing_predicted_shot_axis_error_deg = math.nan
    crossing_shot_axis_error_rate_deg_s = math.nan
    if math.isfinite(first_horizontal_crossing_time) and status_time.size:
        crossing_status_index = int(
            np.argmin(np.abs(status_time - first_horizontal_crossing_time))
        )
        crossing_state = int(states[crossing_status_index])
        crossing_actual_shot_axis_error_deg = math.degrees(
            float(status[crossing_status_index, 3])
        )
        crossing_capture_gate_passed = bool(status[crossing_status_index, 16] > 0.5)
        crossing_fire_authorized = bool(status[crossing_status_index, 38] > 0.5)
        crossing_shot_hard_gate_passed = bool(status[crossing_status_index, 42] > 0.5)
        crossing_shot_timing_gate_passed = bool(status[crossing_status_index, 43] > 0.5)
        crossing_predicted_shot_axis_error_deg = math.degrees(
            float(status[crossing_status_index, 44])
        )
        crossing_shot_axis_error_rate_deg_s = math.degrees(
            float(status[crossing_status_index, 45])
        )
    if intercept_indices.size:
        tracking_start = status_time[intercept_indices[0]]
        tracking_end = shot_times[-1] if shot_times else status_time[-1]
        status_tracking_mask = (
            (status_time >= tracking_start) & (status_time <= tracking_end)
        )
        if status_tracking_mask.any():
            minimum_tracking_height_m = float(
                np.min(status[status_tracking_mask, 12])
            )
            depth_visible_rate = float(
                np.mean(status[status_tracking_mask, 6])
            )
            fov_approach_paused_rate = float(
                np.mean(status[status_tracking_mask, 27])
            )
            mean_fov_approach_scale = float(
                np.mean(status[status_tracking_mask, 41])
            )
            planner_valid_mask = status_tracking_mask & (status[:, 48] > 0.5)
            launch_planner_active_rate = float(
                np.mean(status[status_tracking_mask, 48])
            )
            launch_planner_mean_blend = float(
                np.mean(status[status_tracking_mask, 49])
            )
            if planner_valid_mask.any():
                launch_planner_mean_horizon_s = float(
                    np.mean(status[planner_valid_mask, 50])
                )
                launch_planner_mean_range_m = float(
                    np.mean(status[planner_valid_mask, 51])
                )
                launch_planner_maximum_acceleration_mps2 = float(
                    np.max(np.linalg.norm(status[planner_valid_mask, 53:56], axis=1))
                )
                launch_planner_mean_terminal_relative_speed_mps = float(
                    np.mean(np.linalg.norm(status[planner_valid_mask, 56:59], axis=1))
                )
        local_tracking_mask = (
            (local_time >= tracking_start) & (local_time <= tracking_end)
        )
        if local_tracking_mask.any():
            maximum_tracking_vertical_speed_mps = float(
                np.max(np.abs(local[local_tracking_mask, 5]))
            )
        estimate_tracking_mask = (
            (estimate_time >= tracking_start)
            & (estimate_time <= tracking_end)
            & (estimate[:, 6] > 0.5)
        )
        if estimate_tracking_mask.any():
            tracking_estimate = estimate[estimate_tracking_mask]
            tracking_estimate_time = estimate_time[estimate_tracking_mask]
            ca_probability = tracking_estimate[:, 14]
            acceleration_stddev = tracking_estimate[:, 15]
            probability_scale = np.clip(
                (
                    ca_probability - feedforward_minimum_ca_probability
                ) / max(
                    feedforward_full_ca_probability
                    - feedforward_minimum_ca_probability,
                    1.0e-9,
                ),
                0.0,
                1.0,
            )
            probability_confidence = probability_scale * probability_scale * (
                3.0 - 2.0 * probability_scale
            )
            normalized_stddev = (
                acceleration_stddev / feedforward_maximum_stddev_mps2
            )
            uncertainty_confidence = np.clip(
                1.0 - normalized_stddev * normalized_stddev, 0.0, 1.0
            )
            feedforward_confidence = (
                probability_confidence * uncertainty_confidence
            )
            ca_probability_p90 = float(np.quantile(ca_probability, 0.90))
            acceleration_stddev_p90_mps2 = float(
                np.quantile(acceleration_stddev, 0.90)
            )
            feedforward_confidence_p90 = float(
                np.quantile(feedforward_confidence, 0.90)
            )
            status_indices = np.clip(
                np.searchsorted(
                    status_time, tracking_estimate_time, side="right"
                ) - 1,
                0,
                status.shape[0] - 1,
            )
            status_at_estimate = status[status_indices]
            application_gate = (
                (tracking_estimate[:, 7] > 0.5)
                & (status_at_estimate[:, 0].astype(int) == 6)
                & (status_at_estimate[:, 6] > 0.5)
                & (status_at_estimate[:, 21].astype(int) == 0)
            )
            feedforward_active_rate = float(
                np.mean(
                    (feedforward_confidence > 1.0e-3)
                    & application_gate
                )
            )
    intercept_mask = states == 6
    valid_shot_solution_mask = intercept_mask & (status[:, 17] > 0.5)
    finite_shot_axis_mask = valid_shot_solution_mask & np.isfinite(status[:, 3])
    minimum_valid_shot_axis_error_deg = (
        float(np.degrees(np.min(status[finite_shot_axis_mask, 3])))
        if finite_shot_axis_mask.any() else math.nan
    )
    capture_gate_pass_rate_during_intercept = (
        float(np.mean(status[intercept_mask, 16]))
        if intercept_mask.any() else 0.0
    )
    shot_solution_valid_rate_during_intercept = (
        float(np.mean(status[intercept_mask, 17]))
        if intercept_mask.any() else 0.0
    )
    post_recoil_cooldown_mask = np.zeros(status_time.shape, dtype=bool)
    if shot_times:
        first_cooldown_end = shot_times[0] + reattack_cooldown_setting_s
        if len(shot_times) >= 2:
            first_cooldown_end = min(first_cooldown_end, shot_times[1])
        post_recoil_cooldown_mask = (
            (status_time >= shot_times[0] + recoil_duration_setting_s) &
            (status_time < first_cooldown_end) &
            (status[:, 32] > 0.5) &
            (status[:, 31] < 0.5)
        )
    cooldown_post_recoil_sample_count = int(
        np.count_nonzero(post_recoil_cooldown_mask)
    )
    cooldown_post_recoil_guidance_valid_rate = (
        float(np.mean(status[post_recoil_cooldown_mask, 8]))
        if post_recoil_cooldown_mask.any() else math.nan
    )
    cooldown_post_recoil_depth_visible_rate = (
        float(np.mean(status[post_recoil_cooldown_mask, 6]))
        if post_recoil_cooldown_mask.any() else math.nan
    )
    active_reacquisition_states = np.isin(states, [2, 3, 4, 5, 6, 9, 10])
    cooldown_post_recoil_active_response_rate = (
        float(np.mean(active_reacquisition_states[post_recoil_cooldown_mask]))
        if post_recoil_cooldown_mask.any() else math.nan
    )
    recoil_durations_s = []
    for shot_time in shot_times:
        after_shot = np.flatnonzero(
            (status_time >= shot_time) &
            (status_time <= shot_time + max(1.0, reattack_cooldown_setting_s)) &
            (status[:, 31] > 0.5)
        )
        if after_shot.size:
            recoil_durations_s.append(
                max(0.0, float(status_time[after_shot[-1]] - shot_time))
            )

    completion_hold_duration_s = 0.0
    completion_horizontal_drift_m = math.nan
    completion_vertical_drift_m = math.nan
    completion_position_mode_seen = False
    completion_position_hold_delay_s = math.nan
    completion_position_hold_duration_s = 0.0
    completion_braking_horizontal_displacement_m = math.nan
    completion_settled_hold_duration_s = 0.0
    completion_settled_horizontal_drift_m = math.nan
    completion_settled_vertical_drift_m = math.nan
    complete_indices = np.flatnonzero(states == 11)
    if complete_indices.size:
        completion_start = status_time[complete_indices[0]]
        completion_hold_duration_s = max(0.0, float(status_time[-1] - completion_start))
        complete_local = local[local_time >= completion_start, :3]
        if complete_local.shape[0] >= 2:
            displacement = complete_local - complete_local[0]
            completion_horizontal_drift_m = float(
                np.max(np.linalg.norm(displacement[:, :2], axis=1))
            )
            completion_vertical_drift_m = float(
                np.max(np.abs(displacement[:, 2]))
            )
        complete_mode_mask = offboard_mode_time >= completion_start
        if complete_mode_mask.any():
            position_mode_indices = np.flatnonzero(
                complete_mode_mask & (offboard_mode[:, 0] > 0.5)
            )
            completion_position_mode_seen = bool(position_mode_indices.size)
            if position_mode_indices.size:
                position_hold_start = offboard_mode_time[position_mode_indices[0]]
                position_hold_end = offboard_mode_time[position_mode_indices[-1]]
                completion_position_hold_delay_s = max(
                    0.0, float(position_hold_start - completion_start)
                )
                completion_position_hold_duration_s = max(
                    0.0, float(position_hold_end - position_hold_start)
                )
                complete_position = np.column_stack(
                    [
                        np.interp(
                            [completion_start, position_hold_start],
                            local_time,
                            local[:, axis],
                        )
                        for axis in range(3)
                    ]
                )
                completion_braking_horizontal_displacement_m = float(
                    np.linalg.norm(complete_position[1, :2] - complete_position[0, :2])
                )
                held_local = local[
                    (local_time >= position_hold_start) &
                    (local_time <= position_hold_end), :3
                ]
                if held_local.shape[0] >= 2:
                    held_displacement = held_local - held_local[0]
                    completion_horizontal_drift_m = float(
                        np.max(np.linalg.norm(held_displacement[:, :2], axis=1))
                    )
                    completion_vertical_drift_m = float(
                        np.max(np.abs(held_displacement[:, 2]))
                    )
                settled_start = position_hold_start + 3.0
                settled_local = local[
                    (local_time >= settled_start) &
                    (local_time <= position_hold_end), :3
                ]
                if settled_local.shape[0] >= 2:
                    completion_settled_hold_duration_s = max(
                        0.0, float(position_hold_end - settled_start)
                    )
                    settled_displacement = settled_local - settled_local[0]
                    completion_settled_horizontal_drift_m = float(
                        np.max(np.linalg.norm(settled_displacement[:, :2], axis=1))
                    )
                    completion_settled_vertical_drift_m = float(
                        np.max(np.abs(settled_displacement[:, 2]))
                    )

    controller_complete = bool(np.any(states == 11))
    safety_abort_seen = bool(np.any(states == 12))
    radial_source_names = {
        0: "FOV_HOLD",
        1: "SHOT_HOLD",
        2: "LAUNCH_PLANNER",
        3: "APPROACH_17M",
        4: "RECOIL",
    }
    radial_source_counts = {
        radial_source_names.get(int(source), f"UNKNOWN_{int(source)}"): int(count)
        for source, count in zip(*np.unique(status[:, 59].astype(int), return_counts=True))
    }
    parsed_hover_thrust = float(np.median(status[:, 61]))
    parsed_maximum_thrust = float(np.median(status[:, 62]))
    commanded_thrust_minimum = float(np.min(status[:, 63]))
    commanded_thrust_maximum = float(np.max(status[:, 63]))
    requested_thrust_maximum = float(np.max(status[:, 64]))
    thrust_saturation_rate = float(np.mean(status[:, 65] > 0.5))
    maximum_commanded_acceleration_mps2 = float(
        np.max(np.linalg.norm(status[:, 66:69], axis=1))
    )
    height_recovery_active_rate = float(np.mean(status[:, 60] > 0.5))
    completion_velocity_braking_seen = bool(np.any(status[:, 73] > 0.5))
    safe_to_exit_offboard_seen = bool(np.any(status[:, 75] > 0.5))
    hit_count = sum(bool(hit.hit) for hit in hits)
    solver_hit_count = sum(bool(hit.hit) for hit in solver_hits)
    strict_acceptance_passed = bool(
        controller_complete
        and mission_success
        and len(shots) >= 2
        and hit_count >= 2
        and not safety_abort_seen
        and math.isfinite(shot_interval_s)
        and shot_interval_s >= reattack_cooldown_setting_s
        # PX4 trajectory following can overshoot the commanded target speed by
        # a few percent during ramp-in. Use one scale-aware tolerance instead
        # of making the same fixed 0.5 m/s allowance disproportionately strict
        # at 7--8 m/s. This gate still rejects >10% target-speed excursions.
        and maximum_target_horizontal_speed_mps <= max(
            target_speed_setting_mps + 0.5,
            target_speed_setting_mps * 1.1,
        )
        and completion_position_mode_seen
        and completion_position_hold_duration_s >= 10.0
        and completion_settled_hold_duration_s >= 10.0
        and math.isfinite(completion_settled_horizontal_drift_m)
        and completion_settled_horizontal_drift_m <= 0.5
        and math.isfinite(completion_settled_vertical_drift_m)
        and completion_settled_vertical_drift_m <= 0.3
    )
    early_tracking_acceptance_passed = bool(
        math.isfinite(time_to_first_bearing_s)
        and time_to_first_bearing_s <= 0.5
        and math.isfinite(time_to_first_intercept_s)
        and time_to_first_intercept_s <= 3.0
        and math.isfinite(depth_visible_rate)
        and depth_visible_rate >= 0.95
        and math.isfinite(fov_approach_paused_rate)
        and fov_approach_paused_rate <= 0.30
    )
    early_window_end = mission_start_time + 10.0
    early_window_mask = (
        (status_time >= mission_start_time) &
        (status_time <= early_window_end)
    )
    early_states = states[early_window_mask]
    early_search_return_count = int(
        np.count_nonzero(
            (early_states[1:] == 2) & (early_states[:-1] != 2)
        )
    ) if early_states.size > 1 else 0
    summary = {
        "configured_target_speed_mps": target_speed_setting_mps,
        "configured_command_maximum_speed_mps": command_speed_setting_mps,
        "configured_target_acceleration_feedforward_gain": (
            feedforward_gain_setting
        ),
        "configured_preferred_capture_range_m": preferred_range_setting_m,
        "configured_minimum_flight_height_m": minimum_height_setting_m,
        "configured_target_radius_m": target_radius_setting_m,
        "configured_target_path_length_m": target_path_length_setting_m,
        "configured_target_path_width_m": target_path_width_setting_m,
        "configured_recoil_duration_s": recoil_duration_setting_s,
        "configured_recoil_opening_speed_mps": (
            recoil_opening_speed_setting_mps
        ),
        "configured_reattack_cooldown_s": reattack_cooldown_setting_s,
        "theoretical_target_peak_acceleration_mps2": (
            theoretical_target_peak_acceleration_mps2
        ),
        "mission_success": bool(mission_success),
        "shot_count": len(shots),
        "hit_count": hit_count,
        "physical_hit_count": (
            hit_count if referee_mode == "physical-axis-with-separate-solver"
            else None
        ),
        "solver_hit_count": (
            solver_hit_count
            if referee_mode == "physical-axis-with-separate-solver"
            else hit_count
        ),
        "referee_mode": referee_mode,
        "strict_acceptance_passed": strict_acceptance_passed,
        "early_tracking_acceptance_passed": early_tracking_acceptance_passed,
        "time_to_first_bearing_s": time_to_first_bearing_s,
        "time_to_first_intercept_s": time_to_first_intercept_s,
        "time_to_speed_match_s": time_to_speed_match_s,
        "time_to_fire_authorized_s": time_to_fire_authorized_s,
        "time_to_first_shot_s": time_to_first_shot_s,
        "time_to_first_horizontal_crossing_s": (
            time_to_first_horizontal_crossing_s
        ),
        "first_shot_before_horizontal_crossing": (
            first_shot_before_horizontal_crossing
        ),
        "first_hit_before_horizontal_crossing": (
            first_hit_before_horizontal_crossing
        ),
        "head_on_first_pass_success": head_on_first_pass_success,
        "first_shot_margin_to_crossing_s": first_shot_margin_to_crossing_s,
        "crossing_state": crossing_state,
        "crossing_actual_shot_axis_error_deg": crossing_actual_shot_axis_error_deg,
        "crossing_capture_gate_passed": crossing_capture_gate_passed,
        "crossing_fire_authorized": crossing_fire_authorized,
        "crossing_shot_hard_gate_passed": crossing_shot_hard_gate_passed,
        "crossing_shot_timing_gate_passed": crossing_shot_timing_gate_passed,
        "crossing_predicted_shot_axis_error_deg": (
            crossing_predicted_shot_axis_error_deg
        ),
        "crossing_shot_axis_error_rate_deg_s": crossing_shot_axis_error_rate_deg_s,
        "returns_to_search_in_first_10s": early_search_return_count,
        "shot_interval_s": shot_interval_s,
        "shot_ranges_m": [float(shot.estimated_range_m) for shot in shots],
        "shot_axis_errors_deg": [
            math.degrees(float(shot.actual_attack_axis_error_rad)) for shot in shots
        ],
        "shot_pursuer_heights_m": shot_heights_m,
        "predicted_flight_times_s": [
            float(shot.predicted_time_of_flight_s) for shot in shots
        ],
        "minimum_miss_distances_m": [
            float(hit.minimum_miss_distance_m) for hit in hits
        ],
        "referee_results": [
            _referee_result_dict(hit)
            for hit in hits
        ],
        "solver_minimum_miss_distances_m": [
            float(hit.minimum_miss_distance_m) for hit in solver_hits
        ],
        "solver_referee_results": [
            _referee_result_dict(hit) for hit in solver_hits
        ],
        "referee_invalid_count": sum(
            int(hit.result) == 4 for hit in hits
        ),
        "solver_referee_invalid_count": sum(
            int(hit.result) == 4 for hit in solver_hits
        ),
        "maximum_pursuer_speed_mps": float(np.max(speed)),
        "maximum_target_speed_mps": maximum_target_speed_mps,
        "maximum_target_horizontal_speed_mps": maximum_target_horizontal_speed_mps,
        "maximum_pursuer_horizontal_speed_mps": float(
            np.max(horizontal_speed)
        ),
        "maximum_pursuer_vertical_speed_mps": maximum_vertical_speed_mps,
        "maximum_tracking_vertical_speed_mps": maximum_tracking_vertical_speed_mps,
        "minimum_pursuer_height_during_tracking_m": minimum_tracking_height_m,
        "depth_visible_rate_during_tracking": depth_visible_rate,
        "ca_probability_p90_during_tracking": ca_probability_p90,
        "acceleration_stddev_p90_during_tracking_mps2": (
            acceleration_stddev_p90_mps2
        ),
        "feedforward_confidence_p90_during_tracking": (
            feedforward_confidence_p90
        ),
        "feedforward_active_rate_during_tracking": feedforward_active_rate,
        "launch_planner_active_rate_during_tracking": (
            launch_planner_active_rate
        ),
        "launch_planner_mean_blend_during_tracking": (
            launch_planner_mean_blend
        ),
        "launch_planner_mean_horizon_s": launch_planner_mean_horizon_s,
        "launch_planner_mean_range_m": launch_planner_mean_range_m,
        "launch_planner_maximum_acceleration_mps2": (
            launch_planner_maximum_acceleration_mps2
        ),
        "launch_planner_mean_terminal_relative_speed_mps": (
            launch_planner_mean_terminal_relative_speed_mps
        ),
        "minimum_valid_shot_axis_error_deg": (
            minimum_valid_shot_axis_error_deg
        ),
        "capture_gate_pass_rate_during_intercept": (
            capture_gate_pass_rate_during_intercept
        ),
        "shot_solution_valid_rate_during_intercept": (
            shot_solution_valid_rate_during_intercept
        ),
        "observed_recoil_durations_s": recoil_durations_s,
        "cooldown_post_recoil_sample_count": (
            cooldown_post_recoil_sample_count
        ),
        "cooldown_post_recoil_guidance_valid_rate": (
            cooldown_post_recoil_guidance_valid_rate
        ),
        "cooldown_post_recoil_depth_visible_rate": (
            cooldown_post_recoil_depth_visible_rate
        ),
        "cooldown_post_recoil_active_response_rate": (
            cooldown_post_recoil_active_response_rate
        ),
        "minimum_height_constraint_active_rate": float(
            np.mean(status[:, 14])
        ),
        "height_recovery_active_rate": height_recovery_active_rate,
        "radial_control_source_counts": radial_source_counts,
        "parsed_hover_thrust": parsed_hover_thrust,
        "parsed_maximum_thrust": parsed_maximum_thrust,
        "commanded_thrust_minimum": commanded_thrust_minimum,
        "commanded_thrust_maximum": commanded_thrust_maximum,
        "requested_thrust_maximum": requested_thrust_maximum,
        "thrust_saturation_rate": thrust_saturation_rate,
        "maximum_commanded_acceleration_mps2": (
            maximum_commanded_acceleration_mps2
        ),
        "relative_position_rmse_m": relative_position_rmse_m,
        "relative_velocity_rmse_mps": relative_velocity_rmse_mps,
        "state_transition_count": transitions,
        "returns_to_search_count": search_returns,
        "controller_complete": controller_complete,
        "safety_abort_seen": safety_abort_seen,
        "completion_hold_duration_s": completion_hold_duration_s,
        "completion_horizontal_drift_m": completion_horizontal_drift_m,
        "completion_vertical_drift_m": completion_vertical_drift_m,
        "completion_position_mode_seen": completion_position_mode_seen,
        "completion_velocity_braking_seen": completion_velocity_braking_seen,
        "safe_to_exit_offboard_seen": safe_to_exit_offboard_seen,
        "completion_position_hold_delay_s": completion_position_hold_delay_s,
        "completion_position_hold_duration_s": completion_position_hold_duration_s,
        "completion_braking_horizontal_displacement_m": (
            completion_braking_horizontal_displacement_m
        ),
        "completion_settled_hold_duration_s": completion_settled_hold_duration_s,
        "completion_settled_horizontal_drift_m": (
            completion_settled_horizontal_drift_m
        ),
        "completion_settled_vertical_drift_m": completion_settled_vertical_drift_m,
        "fov_recovery_state_rate": float(np.mean(states == 9)),
        "imm_coast_state_rate": float(np.mean(states == 10)),
        "fov_approach_paused_rate_during_tracking": fov_approach_paused_rate,
        "mean_fov_approach_scale_during_tracking": mean_fov_approach_scale,
    }
    strict_summary = _strict_json_value(summary)
    (run_directory / "结果摘要.json").write_text(
        json.dumps(strict_summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    _configure_chinese_font()
    shot_elapsed = [shot_time - time_origin for shot_time in shot_times]
    crossing_elapsed = (
        first_horizontal_crossing_time - time_origin
        if math.isfinite(first_horizontal_crossing_time) else math.nan
    )
    all_end_times = [
        series[-1]
        for series in (
            local_time,
            truth_time,
            estimate_time,
            status_time,
            attitude_time,
            search_time,
            depth_time,
        )
        if series.size
    ]
    recording_end_time = max(all_end_times)
    plot_end_time = min(
        recording_end_time,
        shot_times[-1] + 2.0 if shot_times else recording_end_time,
    )
    plot_end_elapsed = plot_end_time - time_origin
    local_plot_mask = local_time <= plot_end_time
    truth_plot_mask = truth_time <= plot_end_time

    def mark_shots(axes) -> None:
        if math.isfinite(crossing_elapsed):
            axes.axvline(
                crossing_elapsed, color="black", linestyle=":", linewidth=1.5,
                alpha=0.85, label="首次水平交错",
            )
        for number, elapsed in enumerate(shot_elapsed, start=1):
            axes.axvline(
                elapsed, color="gold", linewidth=1.4, alpha=0.85,
                label=f"射击{number}",
            )

    local_z_at_status = np.interp(status_time, local_time, local[:, 2])
    local_origin_down_m = float(np.nanmedian(local_z_at_status + status[:, 12]))
    pursuer_xyz = np.column_stack(
        (local[:, 0], local[:, 1], local_origin_down_m - local[:, 2])
    )
    target_xyz = np.column_stack(
        (target[:, 0], target[:, 1], local_origin_down_m - target[:, 2])
    )
    body_minus_z = _body_minus_z_from_quaternion(attitude)
    attitude_position = _interp_vector(
        local_time, pursuer_xyz, attitude_time
    )

    fig = plt.figure(figsize=(17, 12))
    grid = fig.add_gridspec(2, 2)
    trajectory_3d = fig.add_subplot(grid[0, 0], projection="3d")
    trajectory_3d.plot(
        target_xyz[truth_plot_mask, 0],
        target_xyz[truth_plot_mask, 1],
        target_xyz[truth_plot_mask, 2],
        color="tab:red", linewidth=2.0, label="目标机x500",
    )
    trajectory_3d.plot(
        pursuer_xyz[local_plot_mask, 0],
        pursuer_xyz[local_plot_mask, 1],
        pursuer_xyz[local_plot_mask, 2],
        color="tab:blue", linewidth=1.8, label="拦截机22001",
    )
    if math.isfinite(first_horizontal_crossing_time):
        crossing_truth_index = int(
            np.argmin(np.abs(truth_time - first_horizontal_crossing_time))
        )
        crossing_local_index = int(
            np.argmin(np.abs(local_time - first_horizontal_crossing_time))
        )
        trajectory_3d.scatter(
            *target_xyz[crossing_truth_index], marker="o", s=80,
            facecolors="none", edgecolors="black", linewidths=1.5,
            label="首次水平交错",
        )
        trajectory_3d.scatter(
            *pursuer_xyz[crossing_local_index], marker="o", s=80,
            facecolors="none", edgecolors="black", linewidths=1.5,
        )
    if attitude_time.size:
        active_start = (
            status_time[intercept_indices[0]]
            if intercept_indices.size else attitude_time[0]
        )
        active_end = shot_times[-1] if shot_times else attitude_time[-1]
        active_axis = np.flatnonzero(
            (attitude_time >= active_start) & (attitude_time <= active_end)
        )[::80]
        for axis_index in active_axis:
            origin = attitude_position[axis_index]
            endpoint = origin + 2.0 * body_minus_z[axis_index]
            trajectory_3d.plot(
                [origin[0], endpoint[0]],
                [origin[1], endpoint[1]],
                [origin[2], endpoint[2]],
                color="darkviolet", alpha=0.45, linewidth=0.8,
            )
    for number, shot_time in enumerate(shot_times, start=1):
        local_index = int(np.argmin(np.abs(local_time - shot_time)))
        attitude_index = int(np.argmin(np.abs(attitude_time - shot_time)))
        origin = pursuer_xyz[local_index]
        endpoint = origin + 5.0 * body_minus_z[attitude_index]
        trajectory_3d.scatter(
            *origin, marker="*", s=190, color="gold", edgecolor="black",
        )
        trajectory_3d.plot(
            [origin[0], endpoint[0]],
            [origin[1], endpoint[1]],
            [origin[2], endpoint[2]],
            color="darkviolet", linewidth=3.0,
            label="实际Body -Z" if number == 1 else None,
        )
        trajectory_3d.text(*origin, f"射击{number}")
    trajectory_3d.set(
        title="三维轨迹、连续攻击轴与射击姿态",
        xlabel="北 [m]", ylabel="东 [m]", zlabel="高度 [m]",
    )
    trajectory_3d.legend()
    trajectory_3d.grid(True)
    combined = np.vstack(
        (target_xyz[truth_plot_mask], pursuer_xyz[local_plot_mask])
    )
    spans = np.maximum(np.ptp(combined, axis=0), 1.0)
    trajectory_3d.set_box_aspect(spans)

    projections = (
        (fig.add_subplot(grid[0, 1]), 0, 1, "北-东俯视投影", "北 [m]", "东 [m]"),
        (fig.add_subplot(grid[1, 0]), 1, 2, "东-高度投影", "东 [m]", "高度 [m]"),
        (fig.add_subplot(grid[1, 1]), 0, 2, "北-高度投影", "北 [m]", "高度 [m]"),
    )
    for axes, horizontal, vertical, title, xlabel, ylabel in projections:
        axes.plot(
            target_xyz[truth_plot_mask, horizontal],
            target_xyz[truth_plot_mask, vertical],
            color="tab:red", linewidth=2.0, label="目标机",
        )
        axes.plot(
            pursuer_xyz[local_plot_mask, horizontal],
            pursuer_xyz[local_plot_mask, vertical],
            color="tab:blue", linewidth=1.7, label="拦截机",
        )
        if math.isfinite(first_horizontal_crossing_time):
            crossing_truth_index = int(
                np.argmin(np.abs(truth_time - first_horizontal_crossing_time))
            )
            crossing_local_index = int(
                np.argmin(np.abs(local_time - first_horizontal_crossing_time))
            )
            axes.scatter(
                target_xyz[crossing_truth_index, horizontal],
                target_xyz[crossing_truth_index, vertical],
                marker="o", s=65, facecolors="none", edgecolors="black",
                linewidths=1.3, label="首次水平交错",
            )
            axes.scatter(
                pursuer_xyz[crossing_local_index, horizontal],
                pursuer_xyz[crossing_local_index, vertical],
                marker="o", s=65, facecolors="none", edgecolors="black",
                linewidths=1.3,
            )
        for number, shot_time in enumerate(shot_times, start=1):
            pursuer_index = int(np.argmin(np.abs(local_time - shot_time)))
            target_index = int(np.argmin(np.abs(truth_time - shot_time)))
            axes.scatter(
                pursuer_xyz[pursuer_index, horizontal],
                pursuer_xyz[pursuer_index, vertical],
                marker="*", s=150, color="gold", edgecolor="black",
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
                color="darkviolet", linestyle="--", linewidth=1.2,
                label="射击LOS" if number == 1 else None,
            )
        axes.set(title=title, xlabel=xlabel, ylabel=ylabel)
        axes.grid(True, alpha=0.3)
        axes.axis("equal")
        axes.legend()
    fig.tight_layout()
    fig.savefig(run_directory / "01_三维轨迹攻击轴与正交投影.png", dpi=180)
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(17, 11))
    for camera_axes, camera_time, camera, title in (
        (axes[0, 0], search_time, search, "搜索相机像面轨迹"),
        (axes[0, 1], depth_time, depth, "双目深度相机像面轨迹"),
    ):
        visible_column = 4 if camera.shape[1] == 5 else 5
        visible = (
            (camera[:, visible_column] > 0.5)
            & (camera_time <= plot_end_time)
        )
        camera_axes.scatter(
            camera[visible, 0], camera[visible, 1],
            c=camera_time[visible] - time_origin,
            s=5, cmap="viridis",
        )
        # Invisible observations still carry the image dimensions. Using only
        # visible rows made a legitimate all-missed search run produce NaN
        # axis limits and abort the whole diagnostic report.
        dimension_rows = camera[visible] if np.any(visible) else camera
        width = float(np.nanmedian(dimension_rows[:, 2]))
        height = float(np.nanmedian(dimension_rows[:, 3]))
        if not np.isfinite(width) or width <= 0.0:
            width = 3840.0
        if not np.isfinite(height) or height <= 0.0:
            height = 2160.0
        camera_axes.scatter(
            [0.5 * (width - 1.0)], [0.5 * (height - 1.0)],
            marker="+", s=180, color="red", linewidth=2.0,
            label="光学中心",
        )
        for number, shot_time in enumerate(shot_times, start=1):
            nearest = int(np.argmin(np.abs(camera_time - shot_time)))
            camera_axes.scatter(
                [camera[nearest, 0]], [camera[nearest, 1]],
                marker="*", s=160, edgecolor="black",
                label=f"射击{number}",
            )
        camera_axes.set(
            title=title, xlabel="像素u [px]", ylabel="像素v [px]",
            xlim=(0.0, width), ylim=(height, 0.0),
        )
        camera_axes.grid(True, alpha=0.3)
        camera_axes.legend()
    visibility = axes[1, 0]
    visibility.fill_between(
        status_elapsed, 0.0, status[:, 5], step="post", alpha=0.55,
        label="搜索相机",
    )
    visibility.fill_between(
        status_elapsed, 1.2, 1.2 + status[:, 6], step="post", alpha=0.55,
        label="双目相机",
    )
    visibility.fill_between(
        status_elapsed, 2.4, 2.4 + status[:, 7], step="post", alpha=0.55,
        label="IMM-EKF有效",
    )
    visibility.fill_between(
        status_elapsed, 2.4, 2.4 + status[:, 11], step="post",
        color="tab:purple", alpha=0.55, label="IMM预测续航",
    )
    visibility.step(
        status_elapsed,
        0.08 + 0.16 * status[:, 21],
        where="post",
        color="black",
        linewidth=1.0,
        label="FOV区编号(0正常/1居中/2恢复/3续航/4丢失)",
    )
    mark_shots(visibility)
    visibility.set(
        title="双相机可见性与估计交接", xlabel="时间 [s]",
        yticks=[0.5, 1.7, 2.9], yticklabels=["搜索", "双目", "估计"],
        ylim=(0.0, 3.6),
    )
    visibility.grid(True, axis="x", alpha=0.3)
    visibility.legend(ncol=2)
    visibility.set_xlim(0.0, plot_end_elapsed)
    aim = axes[1, 1]
    aim.plot(
        status_elapsed, np.degrees(np.abs(status[:, 22])),
        color="tab:blue", label="实际横向轴角",
    )
    aim.plot(
        status_elapsed, np.degrees(np.abs(status[:, 23])),
        color="tab:green", label="实际纵向轴角",
    )
    aim.plot(
        status_elapsed, np.degrees(status[:, 26]),
        color="darkviolet", linewidth=1.8, label="0.5s预测最大分量轴角",
    )
    aim.plot(
        status_elapsed, np.degrees(status[:, 3]),
        color="tab:orange", label="实际Body -Z/预测射线",
    )
    aim.axhline(1.0, color="green", linestyle="--", label="1°优选")
    aim.axhline(2.0, color="red", linestyle="--", label="2°硬门限")
    aim.axhline(
        depth_fov_normal_angle_deg,
        color="tab:cyan",
        linestyle=":",
        label=f"{depth_fov_normal_angle_deg:g}°正常区上界",
    )
    aim.axhline(
        depth_fov_recovery_exit_angle_deg,
        color="tab:olive",
        linestyle=":",
        label=f"{depth_fov_recovery_exit_angle_deg:g}°恢复退出",
    )
    aim.axhline(
        depth_fov_preferred_angle_deg,
        color="tab:orange",
        linestyle=":",
        label=f"{depth_fov_preferred_angle_deg:g}°恢复区入口",
    )
    aim.axhline(
        depth_fov_hard_angle_deg,
        color="black",
        linestyle=":",
        label=f"{depth_fov_hard_angle_deg:g}°相机硬边界",
    )
    mark_shots(aim)
    aim.set(title="实际攻击轴与预测射击瞄准", xlabel="时间 [s]", ylabel="角度 [deg]")
    aim.set_ylim(bottom=0.0)
    aim.grid(True, alpha=0.3)
    aim.legend(ncol=2)
    aim.set_xlim(0.0, plot_end_elapsed)
    fig.tight_layout()
    fig.savefig(run_directory / "02_双相机可见性与预测瞄准.png", dpi=180)
    plt.close(fig)

    truth_range = np.linalg.norm(truth[:, :3], axis=1)
    truth_closing = -np.sum(truth[:, :3] * truth[:, 3:6], axis=1) / np.maximum(
        truth_range, 1.0e-6
    )
    truth_los_rate = np.linalg.norm(
        np.cross(truth[:, :3], truth[:, 3:6]), axis=1
    ) / np.maximum(truth_range * truth_range, 1.0e-6)
    fig, axes = plt.subplots(3, 1, figsize=(15, 11), sharex=True)
    axes[0].axhspan(15.0, 35.0, color="green", alpha=0.12, label="15–35m射击窗")
    axes[0].plot(truth_elapsed, truth_range, label="真值距离")
    axes[0].plot(status_elapsed, status[:, 1], linestyle="--", label="IMM/双目距离")
    axes[0].plot(
        status_elapsed, status[:, 28],
        color="green", linestyle=":", label="阶段进入/软偏好参考",
    )
    axes[0].fill_between(
        status_elapsed, status[:, 29], status[:, 30],
        color="tab:blue", alpha=0.10, label="当前阶段距离区间",
    )
    axes[0].set(title="相对距离", ylabel="距离 [m]")
    axes[1].plot(
        truth_elapsed,
        truth_closing,
        color="tab:orange",
        label="径向闭合速度",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8)
    axes[1].set(title="真值径向闭合速度", ylabel="闭合速度 [m/s]")
    axes[2].plot(
        truth_elapsed,
        np.degrees(truth_los_rate),
        color="tab:purple",
        label="LOS角速度",
    )
    axes[2].set(
        title="三维LOS角速度", xlabel="时间 [s]", ylabel="角速度 [deg/s]"
    )
    for plot_axes in axes:
        mark_shots(plot_axes)
        plot_axes.grid(True, alpha=0.3)
        plot_axes.legend()
        plot_axes.set_xlim(0.0, plot_end_elapsed)
    fig.tight_layout()
    fig.savefig(run_directory / "03_距离闭合速度与LOS角速度.png", dpi=180)
    plt.close(fig)

    target_speed = np.linalg.norm(target_velocity, axis=1)
    tilt_deg = np.degrees(
        np.arccos(np.clip(-body_minus_z[:, 2], -1.0, 1.0))
    )
    state_names = {
        0: "INIT", 1: "TAKEOFF", 2: "SEARCH", 3: "BEARING_TRACK",
        4: "DEPTH_HANDOVER", 5: "DEPTH_ACQUIRE", 6: "INTERCEPT",
        7: "SHOT_READY", 8: "EGRESS", 9: "FOV_RECOVERY",
        10: "IMM_COAST", 11: "COMPLETE", 12: "ABORT",
    }
    fig, axes = plt.subplots(4, 1, figsize=(15, 14), sharex=False)
    axes[0].step(status_elapsed, states, where="post", label="任务状态")
    present_states = sorted(set(states.tolist()))
    axes[0].set(
        title="任务状态机", ylabel="状态", yticks=present_states,
        yticklabels=[state_names.get(state, str(state)) for state in present_states],
    )
    axes[1].plot(truth_elapsed, target_speed, label="目标机速度")
    axes[1].plot(local_elapsed, speed, label="拦截机速度")
    axes[1].axhline(
        command_speed_setting_mps,
        color="red",
        linestyle="--",
        label=f"{command_speed_setting_mps:g}m/s指令上限",
    )
    axes[1].set(title="双机速度", ylabel="速度 [m/s]")
    axes[2].plot(
        attitude_time - time_origin,
        tilt_deg,
        color="tab:purple",
        label="实际倾角",
    )
    axes[2].set(title="拦截机倾角", ylabel="倾角 [deg]")
    axes[3].plot(status_elapsed, status[:, 12], label="实际高度")
    axes[3].plot(
        status_elapsed, status[:, 13], color="red", linestyle="--",
        label=f"{minimum_height_setting_m:g}m最低高度",
    )
    axes[3].fill_between(
        status_elapsed, 0.0, status[:, 12],
        where=status[:, 14] > 0.5, color="orange", alpha=0.22,
        label="最低高度约束激活",
    )
    axes[3].set(title="最低高度控制屏障", xlabel="时间 [s]", ylabel="高度 [m]")
    for plot_axes in axes:
        mark_shots(plot_axes)
        plot_axes.grid(True, alpha=0.3)
        plot_axes.legend()
        plot_axes.set_xlim(0.0, plot_end_elapsed)
    fig.tight_layout()
    fig.savefig(run_directory / "04_状态机速度姿态与最低高度.png", dpi=180)
    plt.close(fig)

    valid_estimate_elapsed = valid_estimate_time - time_origin
    if valid_time_mask.any() and estimate_at_truth.shape[0] == truth.shape[0]:
        position_error_norm = np.linalg.norm(
            estimate_at_truth[valid_time_mask, :3] - truth[valid_time_mask, :3],
            axis=1,
        )
        velocity_error_norm = np.linalg.norm(
            estimate_at_truth[valid_time_mask, 3:6] - truth[valid_time_mask, 3:6],
            axis=1,
        )
        position_three_sigma = 3.0 * np.interp(
            truth_time[valid_time_mask], valid_estimate_time,
            estimate[valid_estimate_mask, 9],
        )
        velocity_three_sigma = 3.0 * np.interp(
            truth_time[valid_time_mask], valid_estimate_time,
            estimate[valid_estimate_mask, 10],
        )
    else:
        position_error_norm = np.empty(0)
        velocity_error_norm = np.empty(0)
        position_three_sigma = np.empty(0)
        velocity_three_sigma = np.empty(0)
    fig, axes = plt.subplots(3, 1, figsize=(16, 13), sharex=True)
    axis_names = ("北", "东", "下")
    colors = ("tab:blue", "tab:orange", "tab:green")
    for component, (name, color) in enumerate(zip(axis_names, colors)):
        axes[0].plot(
            truth_elapsed, truth[:, component], color=color,
            label=f"真值{name}",
        )
        axes[0].plot(
            valid_estimate_elapsed, valid_estimate[:, component],
            color=color, linestyle="--", label=f"IMM-EKF{name}",
        )
        axes[1].plot(
            truth_elapsed, truth[:, component + 3], color=color,
            label=f"真值{name}",
        )
        axes[1].plot(
            valid_estimate_elapsed, valid_estimate[:, component + 3],
            color=color, linestyle="--", label=f"IMM-EKF{name}",
        )
    error_elapsed = truth_elapsed[valid_time_mask]
    axes[2].plot(error_elapsed, position_error_norm, label="位置误差范数")
    axes[2].plot(error_elapsed, position_three_sigma, linestyle="--", label="位置3σ")
    axes[2].plot(error_elapsed, velocity_error_norm, label="速度误差范数")
    axes[2].plot(error_elapsed, velocity_three_sigma, linestyle="--", label="速度3σ")
    axes[0].set(title="IMM-EKF相对位置：真值与估计", ylabel="相对位置 [m]")
    axes[1].set(title="IMM-EKF相对速度：真值与估计", ylabel="相对速度 [m/s]")
    axes[2].set(
        title=(
            f"估计误差与协方差包络：位置RMSE={relative_position_rmse_m:.3f}m，"
            f"速度RMSE={relative_velocity_rmse_mps:.3f}m/s"
        ),
        xlabel="时间 [s]", ylabel="误差/3σ",
    )
    for plot_axes in axes:
        mark_shots(plot_axes)
        plot_axes.grid(True, alpha=0.3)
        plot_axes.legend(ncol=3)
        plot_axes.set_xlim(0.0, plot_end_elapsed)
    fig.tight_layout()
    fig.savefig(run_directory / "05_IMM_EKF真值预测与误差.png", dpi=180)
    plt.close(fig)

    fig = plt.figure(figsize=(17, 11))
    grid = fig.add_gridspec(2, 2)
    overview = fig.add_subplot(grid[0, 0])
    miss_axes = fig.add_subplot(grid[0, 1])
    timing_axes = fig.add_subplot(grid[1, 0])
    closest_axes = fig.add_subplot(grid[1, 1], projection="3d")
    overview.axis("off")
    overview.set_title("独立裁判判定摘要")

    if hits:
        hits_by_id = {int(hit.shot_id): hit for hit in hits}
        solver_hits_by_id = {
            int(hit.shot_id): hit for hit in solver_hits
        }
        shots_by_id = {int(shot.shot_id): shot for shot in shots}
        physical_results = (
            referee_mode == "physical-axis-with-separate-solver"
        )
        result_lines = [
            f"任务裁判结论：{'成功' if mission_success else '未完成'}",
            f"射击事件：{len(shots)}，"
            f"{'实际攻击轴' if physical_results else '旧版理想射线'}裁判结果："
            f"{len(hits)}，命中："
            f"{sum(bool(hit.hit) for hit in hits)}",
            "",
        ]
        shot_ids = [int(hit.shot_id) for hit in hits]
        minimum_miss_mm = [
            1000.0 * float(hit.minimum_miss_distance_m) for hit in hits
        ]
        hit_radius_mm = [1000.0 * float(hit.hit_radius_m) for hit in hits]
        colors = ["tab:green" if hit.hit else "tab:red" for hit in hits]
        positions = np.arange(len(hits), dtype=float)
        physical_bar_width = 0.36 if solver_hits else 0.72
        physical_positions = (
            positions + 0.5 * physical_bar_width if solver_hits else positions
        )
        miss_axes.bar(
            physical_positions,
            minimum_miss_mm,
            width=physical_bar_width,
            color=colors,
            alpha=0.8,
            label=(
                "实际 Body -Z 脱靶量" if physical_results
                else "旧版理想射线脱靶量"
            ),
        )
        if solver_hits:
            solver_miss_by_id = {
                int(hit.shot_id): 1000.0 * float(hit.minimum_miss_distance_m)
                for hit in solver_hits
            }
            solver_values = [
                solver_miss_by_id.get(shot_id, math.nan)
                for shot_id in shot_ids
            ]
            miss_axes.bar(
                positions - 0.5 * physical_bar_width,
                solver_values,
                width=physical_bar_width,
                color="tab:blue",
                alpha=0.55,
                label="解算器理想射线脱靶量",
            )
        miss_axes.plot(
            positions,
            hit_radius_mm,
            color="black",
            linestyle="--",
            marker="o",
            label="命中半径",
        )
        miss_axes.set(
            title="裁判真值脱靶量与命中阈值",
            xlabel="射击编号",
            ylabel="距离 [mm]",
            xticks=positions,
            xticklabels=[str(shot_id) for shot_id in shot_ids],
        )
        miss_axes.grid(True, axis="y", alpha=0.3)
        miss_axes.legend()

        predicted_times = []
        closest_times = []
        for hit in hits:
            shot = shots_by_id.get(int(hit.shot_id))
            predicted_times.append(
                float(shot.predicted_time_of_flight_s)
                if shot is not None else math.nan
            )
            closest_times.append(float(hit.time_to_closest_approach_s))
            result_lines.append(
                f"射击{int(hit.shot_id)}："
                f"{'命中' if hit.hit else '未命中'}，"
                f"最小脱靶量 {1000.0 * hit.minimum_miss_distance_m:.1f} mm，"
                f"命中半径 {1000.0 * hit.hit_radius_m:.1f} mm"
            )
            result_lines.append(
                f"  预测飞行 {predicted_times[-1] * 1000.0:.1f} ms，"
                f"真值最近接近 {closest_times[-1] * 1000.0:.1f} ms；"
                + (
                    "真值脱靶量小于命中半径"
                    if hit.hit else f"裁判原因：{hit.reason}"
                )
            )
            solver_hit = solver_hits_by_id.get(int(hit.shot_id))
            if solver_hit is not None:
                result_lines.append(
                    f"  解算器理想射线："
                    f"{'命中' if solver_hit.hit else '未命中'}，"
                    f"脱靶 {1000.0 * solver_hit.minimum_miss_distance_m:.1f} mm"
                )
        overview.text(
            0.03,
            0.95,
            "\n".join(result_lines),
            va="top",
            ha="left",
            fontsize=12,
            linespacing=1.55,
        )

        width = 0.36
        timing_axes.bar(
            positions - 0.5 * width,
            np.asarray(predicted_times) * 1000.0,
            width=width,
            label="控制器预测飞行时间",
        )
        timing_axes.bar(
            positions + 0.5 * width,
            np.asarray(closest_times) * 1000.0,
            width=width,
            label="裁判真值最近接近时间",
        )
        timing_axes.set(
            title="预测时间与裁判真值对照",
            xlabel="射击编号",
            ylabel="时间 [ms]",
            xticks=positions,
            xticklabels=[str(shot_id) for shot_id in shot_ids],
        )
        timing_axes.grid(True, axis="y", alpha=0.3)
        timing_axes.legend()

        miss_vectors_mm = []
        for result_index, hit in enumerate(hits):
            projectile_ned = np.asarray(
                (
                    hit.projectile_closest_position_ned.x,
                    hit.projectile_closest_position_ned.y,
                    hit.projectile_closest_position_ned.z,
                ),
                dtype=float,
            )
            target_ned = np.asarray(
                (
                    hit.target_closest_position_ned.x,
                    hit.target_closest_position_ned.y,
                    hit.target_closest_position_ned.z,
                ),
                dtype=float,
            )
            miss_vector_mm = 1000.0 * (projectile_ned - target_ned)
            # Plot vertical error positive upwards rather than NED positive down.
            miss_vector_mm[2] = -miss_vector_mm[2]
            miss_vectors_mm.append(miss_vector_mm)
            color = "tab:green" if hit.hit else "tab:red"
            closest_axes.scatter(
                *miss_vector_mm,
                marker="o",
                s=80,
                color=color,
                label="弹丸相对目标最近点" if result_index == 0 else None,
            )
            closest_axes.scatter(
                0.0,
                0.0,
                0.0,
                marker="x",
                s=100,
                color="black",
                label="目标中心" if result_index == 0 else None,
            )
            closest_axes.plot(
                [0.0, miss_vector_mm[0]],
                [0.0, miss_vector_mm[1]],
                [0.0, miss_vector_mm[2]],
                color=color,
                linewidth=2.0,
            )
            closest_axes.text(*miss_vector_mm, f"射击{int(hit.shot_id)}")
        closest_axes.set(
            title="真值最近接近误差向量（目标中心为原点）",
            xlabel="北向误差 [mm]",
            ylabel="东向误差 [mm]",
            zlabel="高度误差 [mm]",
        )
        closest_axes.grid(True)
        closest_axes.legend()
        closest_points_array = np.vstack(
            (np.zeros(3), np.asarray(miss_vectors_mm))
        )
        closest_axes.set_box_aspect(
            np.maximum(np.ptp(closest_points_array, axis=0), 0.1)
        )
    else:
        overview.text(
            0.5,
            0.55,
            "本轮没有产生射击事件\n裁判系统无命中/未命中结果",
            ha="center",
            va="center",
            fontsize=18,
            color="tab:red",
        )
        miss_axes.text(
            0.5, 0.5, "无裁判脱靶量数据", ha="center", va="center",
        )
        timing_axes.text(
            0.5, 0.5, "无弹丸飞行时间数据", ha="center", va="center",
        )
        closest_axes.text2D(
            0.5,
            0.5,
            "无最近接近几何数据",
            transform=closest_axes.transAxes,
            ha="center",
            va="center",
        )
        for empty_axes in (miss_axes, timing_axes):
            empty_axes.set_xticks([])
            empty_axes.set_yticks([])
        closest_axes.set_xticks([])
        closest_axes.set_yticks([])
        closest_axes.set_zticks([])
    fig.suptitle("虚拟弹丸独立裁判系统可视化（裁判真值不反馈控制器）")
    fig.tight_layout()
    fig.savefig(run_directory / "06_裁判系统命中效果.png", dpi=180)
    plt.close(fig)

    state_names = {
        0: "INIT", 1: "TAKEOFF", 2: "SEARCH", 3: "BEARING_TRACK",
        4: "DEPTH_HANDOVER", 5: "DEPTH_ACQUIRE", 6: "INTERCEPT",
        7: "SHOT_READY", 8: "EGRESS_REATTACK", 9: "FOV_RECOVERY",
        10: "IMM_COAST", 11: "COMPLETE", 12: "SAFETY_ABORT",
    }
    transition_states = (
        states[np.r_[True, np.diff(states) != 0]]
        if states.size else np.empty(0, dtype=int)
    )
    transition_text = " → ".join(
        state_names.get(int(state), str(int(state)))
        for state in transition_states
    )

    def metric(value: float, digits: int = 3) -> str:
        return f"{value:.{digits}f}" if math.isfinite(value) else "无有效数据"

    report_lines = [
        "# 仿真效果分析报告",
        "",
        "## 配置",
        "",
        f"- 轨迹：{target_scenario_setting}",
        f"- 目标速度：{target_speed_setting_mps:g} m/s",
        f"- 目标轨迹几何：半径 {target_radius_setting_m:g} m，"
        f"长度 {target_path_length_setting_m:g} m，"
        f"宽度 {target_path_width_setting_m:g} m",
        f"- 理论目标峰值曲率加速度："
        f"{theoretical_target_peak_acceleration_mps2:.3f} m/s²",
        f"- 双目预测视场四区：0–{depth_fov_normal_angle_deg:g}°正常，"
        f"{depth_fov_normal_angle_deg:g}–{depth_fov_preferred_angle_deg:g}°"
        f"增强居中，{depth_fov_preferred_angle_deg:g}–"
        f"{depth_fov_hard_angle_deg:g}°恢复，超过硬边界进入IMM续航",
        f"- FOV恢复退出迟滞：小于{depth_fov_recovery_exit_angle_deg:g}°",
        f"- 拦截机水平指令速度上限：{command_speed_setting_mps:g} m/s",
        f"- 距离管理：交接/深度建立20–30 m，稳定拦截17–25 m，"
        f"{preferred_range_setting_m:g} m仅为软偏好",
        "- 有效射击距离窗口：15–35 m",
        f"- 最低飞行高度：{minimum_height_setting_m:g} m",
        "- 弹丸初速：400 m/s",
        "",
        "## 总体结论",
        "",
    ]
    if strict_acceptance_passed:
        report_lines.extend(
            [
                f"本轮通过严格验收，产生 {len(shots)} 次射击事件，"
                f"独立裁判判定 {sum(bool(hit.hit) for hit in hits)}/{len(hits)} 次命中。",
                "控制器只根据相机观测、IMM-EKF估计和拦截机自身PX4状态决策；"
                "裁判真值只用于射后判定和本报告，不反馈控制器。",
            ]
        )
    elif mission_success:
        report_lines.extend(
            [
                f"本轮裁判判定 {sum(bool(hit.hit) for hit in hits)}/{len(hits)} 次命中，"
                "但未通过严格整轮验收。",
                "命中不是唯一成功条件；还必须满足冷却间隔、无安全中止，以及"
                "完成后至少10秒的稳定位置保持。",
            ]
        )
    elif shots:
        report_lines.append(
            f"本轮产生 {len(shots)} 次射击事件，但独立裁判没有判定任务成功；"
            "必须结合第6张图中的脱靶量与命中半径检查射击质量。"
        )
    else:
        reached_intercept = bool(intercept_mask.any())
        report_lines.extend(
            [
                "本轮任务未完成，并且没有产生射击事件，因此裁判系统没有命中或"
                "未命中结果。",
                (
                    "状态机曾进入 INTERCEPT，但没有同时满足预测射击与捕获门限。"
                    if reached_intercept else
                    "状态机没有进入 INTERCEPT，故障发生在搜索、相机交接或估计建立阶段。"
                ),
                f"INTERCEPT内预测射击解有效率为 "
                f"{100.0 * shot_solution_valid_rate_during_intercept:.1f}%，"
                f"捕获门通过率为 "
                f"{100.0 * capture_gate_pass_rate_during_intercept:.1f}%，"
                f"有效预测解下最小实际射击轴误差为 "
                f"{metric(minimum_valid_shot_axis_error_deg, 2)}°。",
            ]
        )

    report_lines.extend(
        [
            "",
            "## 关键量化结果",
            "",
            f"- 状态机路径：{transition_text or '无状态数据'}",
            f"- 前段跟踪验收：{'通过' if early_tracking_acceptance_passed else '不通过'}",
            f"- 首次搜索方位建立：{metric(time_to_first_bearing_s)} s（要求≤0.5 s）",
            f"- 首次进入INTERCEPT：{metric(time_to_first_intercept_s)} s（要求≤3 s）",
            f"- 双机速度差稳定进入1 m/s：{metric(time_to_speed_match_s)} s"
            "（仅作诊断，不是发射位置规划或开火硬门）",
            f"- 首次开火授权：{metric(time_to_fire_authorized_s)} s",
            f"- 首次实际射击：{metric(time_to_first_shot_s)} s；"
            f"相对交错裕度 {metric(first_shot_margin_to_crossing_s)} s（正值为交错前）",
            f"- 首次水平交错：{metric(time_to_first_horizontal_crossing_s)} s",
            f"- 交错前首发：{'是' if first_shot_before_horizontal_crossing else '否'}；"
            f"交错前首发命中：{'是' if first_hit_before_horizontal_crossing else '否'}",
            f"- 迎头首次交错验收：{'通过' if head_on_first_pass_success else '不通过'}",
            f"- 交错时状态编号：{crossing_state}；开火授权："
            f"{'是' if crossing_fire_authorized else '否'}；捕获门："
            f"{'通过' if crossing_capture_gate_passed else '未通过'}；"
            f"实际射击轴误差：{metric(crossing_actual_shot_axis_error_deg, 2)}°",
            f"- 交错时射击硬门：{'通过' if crossing_shot_hard_gate_passed else '未通过'}；"
            f"时机门：{'通过' if crossing_shot_timing_gate_passed else '未通过'}；"
            f"预测轴误差：{metric(crossing_predicted_shot_axis_error_deg, 2)}°；"
            f"轴误差变化率：{metric(crossing_shot_axis_error_rate_deg_s, 2)}°/s",
            f"- 前10秒退回完整搜索：{early_search_return_count} 次",
            f"- 状态跳变次数：{transitions}",
            f"- 返回 SEARCH 次数（含首次进入）：{search_returns}",
            f"- FOV_RECOVERY状态占比：{100.0 * summary['fov_recovery_state_rate']:.1f}%",
            f"- IMM_COAST状态占比：{100.0 * summary['imm_coast_state_rate']:.1f}%",
            f"- 跟踪段FOV完全暂停逼近占比：{100.0 * fov_approach_paused_rate:.1f}%",
            f"- 跟踪段FOV连续逼近平均比例：{100.0 * mean_fov_approach_scale:.1f}%",
            f"- 发射位置规划器有效占比："
            f"{metric(100.0 * launch_planner_active_rate, 1)}%；"
            f"平均融合：{metric(100.0 * launch_planner_mean_blend, 1)}%",
            f"- 规划平均到达时域/距离："
            f"{metric(launch_planner_mean_horizon_s)} s / "
            f"{metric(launch_planner_mean_range_m)} m；"
            f"规划最大加速度："
            f"{metric(launch_planner_maximum_acceleration_mps2)} m/s²",
            f"- 规划发射时平均相对速度："
            f"{metric(launch_planner_mean_terminal_relative_speed_mps)} m/s"
            "（非共速硬约束）",
            f"- 后坐力配置：{recoil_duration_setting_s:.2f} s / "
            f"{recoil_opening_speed_setting_mps:.2f} m/s；"
            f"再次开火冷却：{reattack_cooldown_setting_s:.2f} s",
            f"- 冷却期去除后坐力后的有效样本："
            f"{cooldown_post_recoil_sample_count}",
            f"- 冷却期去除后坐力后的控制响应占比："
            f"{metric(100.0 * cooldown_post_recoil_active_response_rate, 1)}%",
            f"- 冷却期去除后坐力后的引导有效率："
            f"{metric(100.0 * cooldown_post_recoil_guidance_valid_rate, 1)}%",
            f"- 冷却期去除后坐力后的双目可见率："
            f"{metric(100.0 * cooldown_post_recoil_depth_visible_rate, 1)}%",
            f"- 拦截机最大三维速度：{metric(float(np.max(speed)))} m/s",
            f"- 靶机实际最大三维速度（含起飞）："
            f"{metric(maximum_target_speed_mps)} m/s",
            f"- 靶机任务最大水平速度："
            f"{metric(maximum_target_horizontal_speed_mps)} m/s",
            f"- 拦截机最大水平速度："
            f"{metric(float(np.max(horizontal_speed)))} m/s",
            f"- 追踪阶段最低高度：{metric(minimum_tracking_height_m)} m",
            f"- 径向控制来源采样数：{radial_source_counts}",
            f"- 实际解析悬停/最大推力：{parsed_hover_thrust:.3f} / "
            f"{parsed_maximum_thrust:.3f}",
            f"- 实际指令推力范围：{commanded_thrust_minimum:.3f}～"
            f"{commanded_thrust_maximum:.3f}；请求峰值 "
            f"{requested_thrust_maximum:.3f}；饱和占比 "
            f"{100.0 * thrust_saturation_rate:.1f}%",
            f"- 最终控制加速度峰值："
            f"{maximum_commanded_acceleration_mps2:.3f} m/s²；"
            f"高度恢复占比：{100.0 * height_recovery_active_rate:.1f}%",
            f"- 追踪阶段双目可见率："
            f"{metric(100.0 * depth_visible_rate, 1)}%",
            f"- IMM CA概率P90：{metric(ca_probability_p90)}；"
            f"加速度标准差P90：{metric(acceleration_stddev_p90_mps2)} m/s²",
            f"- 机动前馈置信度P90：{metric(feedforward_confidence_p90)}；"
            f"实际门控启用占比：{metric(100.0 * feedforward_active_rate, 1)}%",
            f"- IMM-EKF相对位置RMSE：{metric(relative_position_rmse_m)} m",
            f"- IMM-EKF相对速度RMSE：{metric(relative_velocity_rmse_mps)} m/s",
            f"- 完成后位置模式：{'已进入' if completion_position_mode_seen else '未进入'}",
            f"- 完成停车链：零速度刹车"
            f"{'已进入' if completion_velocity_braking_seen else '未进入'}；"
            f"安全退出标志{'已置位' if safe_to_exit_offboard_seen else '未置位'}",
            f"- 完成后稳定保持：{completion_settled_hold_duration_s:.2f} s，"
            f"水平漂移 {metric(completion_settled_horizontal_drift_m)} m，"
            f"垂直漂移 {metric(completion_settled_vertical_drift_m)} m",
            f"- 严格整轮验收：{'通过' if strict_acceptance_passed else '不通过'}",
        ]
    )
    if shots:
        report_lines.extend(
            [
                f"- 射击距离："
                + " / ".join(f"{shot.estimated_range_m:.2f}" for shot in shots)
                + " m",
                f"- 实际射击轴误差："
                + " / ".join(
                    f"{math.degrees(shot.actual_attack_axis_error_rad):.2f}"
                    for shot in shots
                )
                + "°",
                f"- 两次射击间隔：{metric(shot_interval_s)} s",
            ]
        )
    if hits:
        report_lines.extend(
            [
                f"- {'实际 Body -Z' if referee_mode == 'physical-axis-with-separate-solver' else '旧版理想射线'}裁判最小脱靶量："
                + " / ".join(
                    f"{1000.0 * hit.minimum_miss_distance_m:.1f}"
                    for hit in hits
                )
                + " mm",
                f"- 裁判结果："
                + " / ".join("命中" if hit.hit else "未命中" for hit in hits),
            ]
        )
    if solver_hits:
        report_lines.extend(
            [
                "- 解算器理想射线最小脱靶量："
                + " / ".join(
                    f"{1000.0 * hit.minimum_miss_distance_m:.1f}"
                    for hit in solver_hits
                )
                + " mm",
                "- 解算器理想射线结果："
                + " / ".join(
                    "命中" if hit.hit else "未命中" for hit in solver_hits
                ),
            ]
        )

    plot_description_lines = [
            "1. `01_三维轨迹攻击轴与正交投影.png`：检查目标与拦截机空间轨迹、"
            "实际 Body -Z 攻击轴连续性，以及射击位置的三向投影。",
            "2. `02_双相机可见性与预测瞄准.png`：检查搜索相机到双目相机的交接、"
            "IMM-EKF续航，以及实际攻击轴对当前LOS和预测弹道方向的误差。",
            "3. `03_距离闭合速度与LOS角速度.png`：检查15–35 m射击窗、"
            "当前阶段距离区间和无跳变参考，以及闭合速度和LOS角速度是否收敛。",
            "4. `04_状态机速度姿态与最低高度.png`：检查状态跳变、速度峰值、"
            "姿态摆动和3 m最低高度约束是否介入。",
            "5. `05_IMM_EKF真值预测与误差.png`：仅用于离线验收，比较估计与"
            "仿真真值，并检查误差是否落在协方差包络内。",
    ]
    if analysis_plot_count >= 6:
        plot_description_lines.append(
            "6. `06_裁判系统命中效果.png`：新版日志并列显示实际 Body -Z 与"
            "解算器理想射线的脱靶量，并给出命中半径、最近接近时间和空间几何；"
            "这些真值不参与控制。旧版日志只能显示原有理想射线结果。"
        )
    report_lines.extend(
        [
            "",
            f"## {analysis_plot_count}张图片解读",
            "",
            *plot_description_lines,
            "",
            "## 当前主要问题",
            "",
        ]
    )
    if strict_acceptance_passed:
        report_lines.append(
            "本轮没有阻止任务完成的问题；后续仍应重点观察速度是否超过正常指令"
            "上限、状态机是否出现非必要回退，以及IMM-EKF速度误差。"
        )
    elif mission_success:
        report_lines.append(
            "裁判命中已经完成，但整轮验收仍失败；应优先检查完成后位置模式、"
            "稳定保持时长与漂移，不得把该轮登记为成功样本。"
        )
    elif shots:
        report_lines.append(
            "控制器能够触发射击，但射后真值判定未形成完整成功。应优先检查弹道"
            "预测误差和估计协方差，不应通过扩大命中半径掩盖问题。"
        )
    else:
        report_lines.append(
            "当前直接失败点是闭环丢失后无法再次建立可射击的 INTERCEPT。应根据"
            "第2、3、5张图区分相机视场丢失、攻击轴未对准和预测解无效；本报告"
            "不把该轮视为有效打击测试。"
        )
    (run_directory / "仿真效果分析报告.md").write_text(
        "\n".join(report_lines) + "\n",
        encoding="utf-8",
    )

    return summary


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("run_directory", type=Path)
    args = parser.parse_args()
    try:
        summary = analyze(args.run_directory.expanduser().resolve())
    except (AnalysisInputError, FileNotFoundError) as error:
        print(f"分析输入无效: {error}", file=sys.stderr)
        raise SystemExit(2) from None
    print(json.dumps(_strict_json_value(summary), ensure_ascii=False, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
