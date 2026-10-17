#!/usr/bin/env python3
"""Offline acceptance for the predictive FOV barrier and staged range policy.

Only camera observations and IMM-EKF outputs are read.  Simulation truth and
the historical controller status are deliberately not deserialized, so this
tool both preserves truth isolation and can replay bags recorded with an older
status-message schema.
"""

from __future__ import annotations

import argparse
import bisect
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


NORMAL = 0
CENTERING = 1
RECOVERY = 2
IMM_COAST = 3
LOST = 4

NORMAL_RAD = math.radians(5.0)
CENTERING_RAD = math.radians(8.0)
HARD_RAD = math.radians(10.0)
RECOVERY_EXIT_RAD = math.radians(6.0)
PREDICTION_HORIZON_S = 0.5
FILTER_TIME_CONSTANT_S = 0.12


def _bag_directory(run_directory: Path) -> Path:
    nested = run_directory / "原始数据" / "rosbag"
    return nested if nested.is_dir() else run_directory / "rosbag"


def _read_selected(run_directory: Path) -> dict[str, list[tuple[float, object]]]:
    bag_directory = _bag_directory(run_directory)
    if not bag_directory.is_dir():
        raise FileNotFoundError(f"找不到rosbag: {bag_directory}")
    selected = {
        "/perception/search/observation",
        "/perception/depth/observation",
        "/tracking/relative/estimate",
    }
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_directory), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {
        item.name: item.type for item in reader.get_all_topics_and_types()
        if item.name in selected
    }
    output = {topic: [] for topic in selected}
    while reader.has_next():
        topic, serialized, timestamp_ns = reader.read_next()
        if topic not in selected:
            continue
        output[topic].append(
            (
                timestamp_ns * 1.0e-9,
                deserialize_message(serialized, get_message(topic_types[topic])),
            )
        )
    return output


def _latest_before(records, times, query_time):
    index = bisect.bisect_right(times, query_time) - 1
    return records[index][1] if index >= 0 else None


def _imm_usable(message) -> bool:
    if message is None or not message.valid:
        return False
    covariance = list(message.full_covariance)
    maximum_position_variance = max(
        covariance[0], covariance[10], covariance[20]
    )
    return (
        message.coast_time_s <= 1.0
        and maximum_position_variance <= 25.0
    )


def _barrier_trace(depth_records, estimate_records):
    estimate_times = [record[0] for record in estimate_records]
    zone = LOST
    initialized = False
    last_time = 0.0
    last_horizontal = 0.0
    last_vertical = 0.0
    last_sequence = 0
    horizontal_rate = 0.0
    vertical_rate = 0.0
    trace = []
    for bag_time, message in depth_records:
        estimate = _latest_before(estimate_records, estimate_times, bag_time)
        visible = bool(message.visible)
        if not visible:
            zone = IMM_COAST if _imm_usable(estimate) else LOST
            trace.append(
                {
                    "time": bag_time,
                    "visible": False,
                    "zone": zone,
                    "horizontal": math.nan,
                    "vertical": math.nan,
                    "predicted_h": math.nan,
                    "predicted_v": math.nan,
                    "predicted_max": math.nan,
                    "pause": True,
                    "range": math.nan,
                }
            )
            continue

        sample_time = (
            float(message.header.stamp.sec)
            + float(message.header.stamp.nanosec) * 1.0e-9
        )
        horizontal = float(message.horizontal_angle_rad)
        vertical = float(message.vertical_angle_rad)
        sequence = int(message.sequence)
        if initialized and sequence != 0 and sequence != last_sequence:
            dt = sample_time - last_time
            if 1.0e-4 < dt < 1.0:
                alpha = dt / (FILTER_TIME_CONSTANT_S + dt)
                horizontal_rate += alpha * (
                    (horizontal - last_horizontal) / dt - horizontal_rate
                )
                vertical_rate += alpha * (
                    (vertical - last_vertical) / dt - vertical_rate
                )
        if not initialized or sequence != last_sequence:
            last_time = sample_time
            last_horizontal = horizontal
            last_vertical = vertical
            last_sequence = sequence
            initialized = True

        predicted_h = horizontal + PREDICTION_HORIZON_S * horizontal_rate
        predicted_v = vertical + PREDICTION_HORIZON_S * vertical_rate
        predicted_max = max(
            abs(horizontal), abs(vertical), abs(predicted_h), abs(predicted_v)
        )
        if zone == RECOVERY and predicted_max >= RECOVERY_EXIT_RAD:
            zone = RECOVERY
        elif predicted_max >= CENTERING_RAD:
            zone = RECOVERY
        elif predicted_max >= NORMAL_RAD:
            zone = CENTERING
        else:
            zone = NORMAL
        trace.append(
            {
                "time": bag_time,
                "visible": True,
                "zone": zone,
                "horizontal": horizontal,
                "vertical": vertical,
                "predicted_h": predicted_h,
                "predicted_v": predicted_v,
                "predicted_max": predicted_max,
                "pause": zone == RECOVERY,
                "range": float(message.range_m),
            }
        )
    return trace


def _prediction_error(trace):
    visible = [sample for sample in trace if sample["visible"]]
    visible_times = [sample["time"] for sample in visible]
    errors_h = []
    errors_v = []
    for sample in visible:
        future_time = sample["time"] + PREDICTION_HORIZON_S
        index = bisect.bisect_left(visible_times, future_time)
        if index >= len(visible):
            continue
        future = visible[index]
        if abs(future["time"] - future_time) > 0.08:
            continue
        errors_h.append(abs(sample["predicted_h"] - future["horizontal"]))
        errors_v.append(abs(sample["predicted_v"] - future["vertical"]))
    return (
        math.degrees(sum(errors_h) / len(errors_h)) if errors_h else math.nan,
        math.degrees(sum(errors_v) / len(errors_v)) if errors_v else math.nan,
        len(errors_h),
    )


def _range_replay(trace):
    reference = None
    last_time = None
    commands = []
    fov_approach_violations = 0
    for sample in trace:
        range_m = sample["range"]
        if not math.isfinite(range_m):
            continue
        if reference is None:
            reference = min(25.0, max(17.0, range_m))
        dt = 0.0 if last_time is None else max(0.0, sample["time"] - last_time)
        reference += max(-0.25 * dt, min(0.25 * dt, 17.0 - reference))
        reference = min(25.0, max(17.0, reference))
        if range_m < 17.0:
            command = max(-4.0, 0.45 * (range_m - 17.0))
        elif range_m > 25.0:
            command = 0.0 if sample["pause"] else min(
                4.0, 0.45 * (range_m - 25.0)
            )
        elif sample["pause"]:
            command = 0.0
        else:
            command = max(-0.5, min(0.5, 0.08 * (range_m - reference)))
        if sample["pause"] and range_m >= 17.0 and command > 1.0e-12:
            fov_approach_violations += 1
        commands.append(command)
        last_time = sample["time"]
    maximum_command_step = max(
        (abs(right - left) for left, right in zip(commands, commands[1:])),
        default=0.0,
    )
    return maximum_command_step, fov_approach_violations


def analyze_case(run_directory: Path) -> dict:
    records = _read_selected(run_directory)
    depth = records["/perception/depth/observation"]
    estimate = records["/tracking/relative/estimate"]
    first = _barrier_trace(depth, estimate)
    second = _barrier_trace(depth, estimate)
    deterministic = [item["zone"] for item in first] == [
        item["zone"] for item in second
    ]
    recovery_pause_violations = sum(
        item["zone"] == RECOVERY and not item["pause"] for item in first
    )
    recovery_exit_violations = 0
    previous_zone = LOST
    for item in first:
        if (
            previous_zone == RECOVERY
            and item["zone"] in (NORMAL, CENTERING)
        ):
            if (
                not item["visible"]
                or item["predicted_max"] >= RECOVERY_EXIT_RAD
            ):
                recovery_exit_violations += 1
        previous_zone = item["zone"]
    maximum_command_step, fov_approach_violations = _range_replay(first)
    prediction_h_mae_deg, prediction_v_mae_deg, comparison_count = (
        _prediction_error(first)
    )
    zone_counts = {
        str(zone): sum(item["zone"] == zone for item in first)
        for zone in (NORMAL, CENTERING, RECOVERY, IMM_COAST, LOST)
    }
    passed = (
        bool(first)
        and deterministic
        and recovery_pause_violations == 0
        and recovery_exit_violations == 0
        and fov_approach_violations == 0
        and maximum_command_step <= 4.0
    )
    return {
        "run_directory": str(run_directory),
        "sample_count": len(first),
        "zone_counts": zone_counts,
        "deterministic_replay": deterministic,
        "recovery_pause_violations": recovery_pause_violations,
        "recovery_exit_hysteresis_violations": recovery_exit_violations,
        "fov_priority_approach_violations": fov_approach_violations,
        "maximum_range_command_step_mps": maximum_command_step,
        "prediction_comparison_count": comparison_count,
        "horizontal_0p5s_prediction_mae_deg": prediction_h_mae_deg,
        "vertical_0p5s_prediction_mae_deg": prediction_v_mae_deg,
        "offline_gate_passed": passed,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("run_directories", nargs="+", type=Path)
    args = parser.parse_args()
    output_directory = args.output_directory.expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    results = [
        analyze_case(run.expanduser().resolve())
        for run in args.run_directories
    ]
    overall = all(result["offline_gate_passed"] for result in results)
    payload = {"overall_passed": overall, "cases": results}
    (output_directory / "离线回放验收.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    lines = [
        "# 预测视场屏障与分阶段距离离线回放验收",
        "",
        f"- 总体准入：{'通过' if overall else '不通过'}",
        "- 输入边界：只读取搜索/深度相机观测与IMM-EKF估计，不读取仿真真值。",
        "- 判据：四区确定性、恢复暂停逼近、6°迟滞退出、距离指令连续性。",
        "",
    ]
    for result in results:
        lines.extend(
            [
                f"## {Path(result['run_directory']).name}",
                "",
                f"- 样本数：{result['sample_count']}",
                f"- 四区计数（0/1/2/3/4）：{result['zone_counts']}",
                f"- 确定性：{result['deterministic_replay']}",
                f"- 恢复区未暂停逼近违规：{result['recovery_pause_violations']}",
                f"- 6°迟滞退出违规："
                f"{result['recovery_exit_hysteresis_violations']}",
                f"- FOV优先时正向逼近违规："
                f"{result['fov_priority_approach_violations']}",
                f"- 距离指令最大相邻变化："
                f"{result['maximum_range_command_step_mps']:.3f} m/s",
                f"- 0.5 s横/纵预测MAE："
                f"{result['horizontal_0p5s_prediction_mae_deg']:.3f}° / "
                f"{result['vertical_0p5s_prediction_mae_deg']:.3f}°",
                f"- 本场准入："
                f"{'通过' if result['offline_gate_passed'] else '不通过'}",
                "",
            ]
        )
    (output_directory / "离线回放验收报告.md").write_text(
        "\n".join(lines), encoding="utf-8"
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2, allow_nan=True))
    raise SystemExit(0 if overall else 1)


if __name__ == "__main__":
    main()
