#!/usr/bin/env python3
"""PX4 僚机版自动循环测试、平滑性审查与优选图册生成。"""

from __future__ import annotations

import argparse
import csv
import html
import json
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE = PACKAGE_ROOT.parents[1]
SINGLE_RUNNER = PACKAGE_ROOT / "scripts" / "run_hybrid_2v1.sh"
DEFAULT_OUTPUT_PARENT = Path("/home/key/codex_work")

FIGURES = (
    ("01_三机三维轨迹_高度向上.png", "01_三机三维轨迹_高度向上", "h=-z_NED，高度正方向向上"),
    ("02_XY平面等边三角形包围.png", "02_XY平面等边三角形包围", "全程轨迹与最佳近似等边包围"),
    ("03_高度曲线与敌机垂直间距.png", "03_高度曲线与敌机垂直间距", "P2/P3敌机下方15–25 m高度层"),
    ("04_P1敌机距离与开火事件.png", "04_P1敌机距离与开火事件", "17 m参考、15–35 m窗口与独立裁判"),
    ("05_P2P3敌机距离.png", "05_P2P3敌机距离", "僚机防穿越与稳定包围距离"),
    ("06_三架拦截机两两安全距离.png", "06_三架拦截机两两安全距离", "15 m硬安全门槛"),
    ("07_包围质心误差与方位角误差.png", "07_包围质心误差与方位角误差", "中心性及P2/P3相对±120°误差"),
    ("08_关键时刻三机包围快照.png", "08_关键时刻三机包围快照", "两次开火与最佳包围快照"),
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="自动循环运行 PX4 2虚1实协同拦截，增量汇总并生成编号优选图册。"
    )
    parser.add_argument("--repeats", type=int, default=3, help="每个速度/场景重复次数，默认3")
    parser.add_argument("--target-speeds", default="5.0", help="逗号分隔的目标速度，默认5.0")
    parser.add_argument(
        "--scenarios", default="straight",
        help="逗号分隔的 straight,circle,s_curve,figure_eight,rectangle",
    )
    geometry = parser.add_mutually_exclusive_group()
    geometry.add_argument("--head-on", action="store_true", default=True, help="迎头几何（默认）")
    geometry.add_argument("--tail-chase", action="store_true", help="尾追几何")
    parser.add_argument("--duration", type=float, default=60.0, help="单轮记录上限秒数")
    parser.add_argument("--inter-case-settle", type=float, default=3.0, help="轮间等待秒数")
    parser.add_argument("--smooth-min-target-distance", type=float, default=15.0,
                        help="P2/P3 任务全程最小目标距离门限")
    parser.add_argument("--smooth-min-inter-distance", type=float, default=15.0,
                        help="三机任务全程最小两两间距门限")
    parser.add_argument("--select-top", type=int, default=3, help="优选代表案例数，默认3")
    parser.add_argument("--max-infra-retries", type=int, default=1,
                        help="目标/P1未正常启动等基础设施失败的自动重试次数")
    parser.add_argument("--output-root", type=Path, help="批次根目录")
    parser.add_argument("--resume", action="store_true", help="续跑已有批次，跳过已完成案例")
    parser.add_argument("--rerun-failed", action="store_true", help="续跑时重跑非平滑/失败案例")
    parser.add_argument("--dry-run", action="store_true", help="只生成计划，不启动仿真")
    parser.add_argument(
        "--curate-existing", type=Path, nargs="+",
        help="不运行仿真，直接导入已有 hybrid 结果目录验证汇总/优选",
    )
    return parser.parse_args()


def atomic_write_text(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def atomic_write_json(path: Path, data: Any) -> None:
    atomic_write_text(path, json.dumps(data, ensure_ascii=False, indent=2) + "\n")


def finite(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def parse_list(text: str) -> list[str]:
    return [item.strip() for item in text.split(",") if item.strip()]


def validate_args(args: argparse.Namespace) -> tuple[list[float], list[str]]:
    if args.repeats < 1 or args.select_top < 1 or args.max_infra_retries < 0:
        raise ValueError("--repeats 和 --select-top 必须大于0")
    if args.duration <= 0.0 or args.inter_case_settle < 0.0:
        raise ValueError("--duration 必须大于0，--inter-case-settle 不能为负")
    try:
        speeds = [float(item) for item in parse_list(args.target_speeds)]
    except ValueError as error:
        raise ValueError("--target-speeds 必须是逗号分隔的数值") from error
    if not speeds or any(not math.isfinite(item) or item <= 0.0 for item in speeds):
        raise ValueError("目标速度必须是有限正数")
    allowed = {"straight", "circle", "s_curve", "figure_eight", "rectangle"}
    scenarios = parse_list(args.scenarios)
    if not scenarios or any(item not in allowed for item in scenarios):
        raise ValueError("--scenarios 包含不支持的轨迹")
    return speeds, scenarios


def speed_label(speed: float) -> str:
    return f"{speed:g}".replace(".", "p")


def ros_double(value: float) -> str:
    """保留 ROS 2 double 参数类型，避免 5.0 被序列化为整数 5。"""
    text = format(value, ".15g")
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text


def build_cases(args: argparse.Namespace, speeds: list[float], scenarios: list[str]) -> list[dict[str, Any]]:
    cases: list[dict[str, Any]] = []
    index = 1
    geometry = "tail_chase" if args.tail_chase else "head_on"
    for speed in speeds:
        for scenario in scenarios:
            for repeat in range(1, args.repeats + 1):
                case_id = (
                    f"{index:03d}_{speed_label(speed)}mps_{scenario}_"
                    f"{geometry}_r{repeat:02d}"
                )
                cases.append({
                    "index": index,
                    "case_id": case_id,
                    "target_speed_mps": speed,
                    "scenario": scenario,
                    "geometry": geometry,
                    "repeat": repeat,
                })
                index += 1
    return cases


def final_distances(trajectory_path: Path) -> tuple[float | None, float | None, float | None]:
    if not trajectory_path.is_file():
        return None, None, None
    last: dict[str, str] | None = None
    with trajectory_path.open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            last = row
    if last is None:
        return None, None, None
    return finite(last.get("dist_p1")), finite(last.get("dist_p2")), finite(last.get("dist_p3"))


def record_from_metrics(
    case: dict[str, Any], metrics_dir: Path, run_dir: Path | None,
    return_code: int, args: argparse.Namespace,
) -> dict[str, Any]:
    metrics_path = metrics_dir / "metrics.json"
    if not metrics_path.is_file():
        return {
            **case, "status": "failed", "return_code": return_code,
            "error": f"缺少 {metrics_path}", "run_dir": str(run_dir or ""),
            "metrics_dir": str(metrics_dir), "smooth_pass": False,
        }
    metrics = json.loads(metrics_path.read_text(encoding="utf-8"))
    final_p1, final_p2, final_p3 = final_distances(metrics_dir / "trajectory.csv")
    min_p2 = finite(metrics.get("min_distance_p2"))
    min_p3 = finite(metrics.get("min_distance_p3"))
    min_inter = finite(metrics.get("min_inter_pursuer_distance"))
    centroid_error = finite(metrics.get("centroid_error_at_best_m"))
    mission_success = bool(metrics.get("success"))
    smooth_pass = bool(
        mission_success
        and min_p2 is not None and min_p2 >= args.smooth_min_target_distance
        and min_p3 is not None and min_p3 >= args.smooth_min_target_distance
        and min_inter is not None and min_inter >= args.smooth_min_inter_distance
        and not metrics.get("speed_limit_violation", False)
        and metrics.get("p1_shot_count", 0) >= 2
        and metrics.get("p1_hit_count", 0) >= 2
        and metrics.get("finite_data", False)
        and metrics.get("reference_smooth", False)
        and centroid_error is not None
    )
    # 新包围是水平18~25 m、垂向15~25 m，三维理想距离约32 m。
    ideal = 32.0
    final_error = sum(
        abs(value - ideal) for value in (final_p2, final_p3) if value is not None
    )
    quality_score = (
        (100.0 if smooth_pass else 0.0)
        - final_error
        + 0.1 * min(min_inter or 0.0, 40.0)
        - 0.02 * (finite(metrics.get("intercept_time")) or 99.0)
    )
    return {
        **case,
        "status": "completed" if return_code == 0 else "failed",
        "return_code": return_code,
        "run_dir": str(run_dir or ""),
        "metrics_dir": str(metrics_dir),
        "mission_success": mission_success,
        "smooth_pass": smooth_pass,
        "quality_score": round(quality_score, 4),
        "p1_complete_time": finite(metrics.get("p1_complete_time")),
        "wings_encircled_time": finite(metrics.get("wings_encircled_time")),
        "intercept_time": finite(metrics.get("intercept_time")),
        "min_distance_p1": finite(metrics.get("min_distance_p1")),
        "min_distance_p2": min_p2,
        "min_distance_p3": min_p3,
        "final_distance_p1": final_p1,
        "final_distance_p2": final_p2,
        "final_distance_p3": final_p3,
        "min_inter_pursuer_distance": min_inter,
        "target_max_speed": finite(metrics.get("target_max_speed")),
        "p1_max_speed": finite(metrics.get("p1_max_speed")),
        "p2_max_speed": finite(metrics.get("p2_max_speed")),
        "p3_max_speed": finite(metrics.get("p3_max_speed")),
        "speed_limit_violation": bool(metrics.get("speed_limit_violation", False)),
        "p1_shot_count": int(metrics.get("p1_shot_count", 0)),
        "p1_hit_count": int(metrics.get("p1_hit_count", 0)),
        "p1_actual_shot_distances_m": metrics.get("p1_actual_shot_distances_m", []),
        "centroid_error_at_best_m": centroid_error,
        "best_formation_time_s": finite(metrics.get("best_formation_time_s")),
        "max_reference_accel_mps2": finite(metrics.get("max_reference_accel_mps2")),
        "max_reference_position_rate_mps": finite(
            metrics.get("max_reference_position_rate_mps")),
        "p2_max_height_delta_to_p1_m": finite(
            metrics.get("p2_max_height_delta_to_p1_m")),
        "p3_max_height_delta_to_p1_m": finite(
            metrics.get("p3_max_height_delta_to_p1_m")),
        "reference_smooth": bool(metrics.get("reference_smooth", False)),
        "finite_data": bool(metrics.get("finite_data", False)),
    }


def find_paths(output: str, case_output: Path) -> tuple[Path | None, Path | None]:
    run_matches = re.findall(r"(?:结果目录|result_dir):\s*([^\r\n]+)", output)
    run_dir = Path(run_matches[-1].strip()) if run_matches else None
    if run_dir is None or not run_dir.is_dir():
        candidates = sorted(path.parent for path in case_output.glob("*/运行结果.txt"))
        run_dir = candidates[-1] if candidates else None
    metrics_dir = None
    if run_dir is not None:
        tracking = run_dir / "原始数据" / "tracking.log"
        if tracking.is_file():
            text = tracking.read_text(encoding="utf-8", errors="replace")
            matches = re.findall(r"结果目录:\s*([^\r\n]+)", text)
            if matches:
                metrics_dir = Path(matches[-1].strip())
    return run_dir, metrics_dir


def run_case(case: dict[str, Any], batch_root: Path, args: argparse.Namespace) -> dict[str, Any]:
    case_output = batch_root / "飞行日志" / case["case_id"]
    case_output.mkdir(parents=True, exist_ok=True)
    environment = os.environ.copy()
    environment.update({
        "HEAD_ON_MODE": "false" if args.tail_chase else "true",
        "TARGET_SPEED_MPS": ros_double(float(case["target_speed_mps"])),
        "HYBRID_DURATION_S": ros_double(float(args.duration)),
        "OUTPUT_ROOT": str(case_output),
    })
    command = ["bash", str(SINGLE_RUNNER), case["scenario"]]
    log_path = batch_root / "调度日志" / f"{case['case_id']}.log"
    log_path.parent.mkdir(parents=True, exist_ok=True)
    lines: list[str] = []
    with log_path.open("w", encoding="utf-8") as log_stream:
        process = subprocess.Popen(
            command, cwd=WORKSPACE, env=environment,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            text=True, bufsize=1, start_new_session=True,
        )
        try:
            assert process.stdout is not None
            for line in process.stdout:
                print(line, end="", flush=True)
                log_stream.write(line)
                log_stream.flush()
                lines.append(line)
            return_code = process.wait()
        except KeyboardInterrupt:
            print(f"\n[{case['case_id']}] 收到中断，通知单轮脚本清理…", flush=True)
            os.killpg(process.pid, signal.SIGINT)
            try:
                return_code = process.wait(timeout=40)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                return_code = process.wait(timeout=20)
            raise
    output = "".join(lines)
    run_dir, metrics_dir = find_paths(output, case_output)
    if metrics_dir is None:
        return {
            **case, "status": "failed", "return_code": return_code,
            "error": "未能从 tracking.log 解析 metrics 目录",
            "run_dir": str(run_dir or ""), "metrics_dir": "", "smooth_pass": False,
        }
    return record_from_metrics(case, metrics_dir, run_dir, return_code, args)


def is_infrastructure_failure(record: dict[str, Any], commanded_speed: float) -> bool:
    """识别PX4/目标未正常启动；不把此类轮次伪装成算法样本。"""
    target_speed = finite(record.get("target_max_speed"))
    wing_height_delta = max(
        finite(record.get("p2_max_height_delta_to_p1_m")) or 0.0,
        finite(record.get("p3_max_height_delta_to_p1_m")) or 0.0)
    return bool(
        record.get("status") != "completed"
        or not record.get("metrics_dir")
        or (not record.get("p1_complete_time")
            and int(record.get("p1_shot_count", 0)) == 0
            and target_speed is not None
            and target_speed < 0.8 * float(commanded_speed))
        or (record.get("p1_complete_time") is not None
            and not record.get("wings_encircled_time")
            and wing_height_delta > 8.0)
    )


def fmt(value: Any, digits: int = 2) -> str:
    number = finite(value)
    return "—" if number is None else f"{number:.{digits}f}"


def write_aggregates(batch_root: Path, records: list[dict[str, Any]]) -> None:
    ordered = sorted(records, key=lambda item: int(item.get("index", 0)))
    fields = (
        "index", "case_id", "target_speed_mps", "scenario", "geometry", "repeat",
        "status", "mission_success", "smooth_pass", "quality_score",
        "p1_complete_time", "wings_encircled_time", "intercept_time",
        "min_distance_p1", "min_distance_p2", "min_distance_p3",
        "final_distance_p1", "final_distance_p2", "final_distance_p3",
        "min_inter_pursuer_distance", "target_max_speed", "p1_max_speed",
        "p2_max_speed", "p3_max_speed", "speed_limit_violation",
        "p1_shot_count", "p1_hit_count", "p1_actual_shot_distances_m",
        "centroid_error_at_best_m", "best_formation_time_s",
        "max_reference_accel_mps2", "max_reference_position_rate_mps",
        "p2_max_height_delta_to_p1_m", "p3_max_height_delta_to_p1_m",
        "reference_smooth", "finite_data",
        "return_code", "run_dir", "metrics_dir", "error",
    )
    csv_path = batch_root / "批次汇总.csv"
    temporary = csv_path.with_suffix(".csv.tmp")
    with temporary.open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(ordered)
    temporary.replace(csv_path)

    completed = sum(item.get("status") == "completed" for item in ordered)
    success = sum(bool(item.get("mission_success")) for item in ordered)
    smooth = sum(bool(item.get("smooth_pass")) for item in ordered)
    lines = [
        "# PX4 三机协同拦截自动测试汇总", "",
        f"- 计划/完成：{len(ordered)} / {completed}",
        f"- 任务 SUCCESS：{success}/{len(ordered)}",
        f"- 平滑通过：{smooth}/{len(ordered)}",
        "- 平滑门槛同时要求任务 SUCCESS、P2/P3 全程不近于目标、三机不近距穿越。",
        "", "| # | 案例 | 任务 | 平滑 | 开火/命中 | P1 COMPLETE | 僚机包围 | min d2/d3 | min 两两 | 质心误差 |",
        "|---:|---|:---:|:---:|---:|---:|---:|---:|---:|---:|",
    ]
    for item in ordered:
        lines.append(
            f"| {item.get('index', '—')} | {item.get('case_id', '—')} | "
            f"{'SUCCESS' if item.get('mission_success') else 'FAIL'} | "
            f"{'✓' if item.get('smooth_pass') else '✗'} | "
            f"{item.get('p1_shot_count', 0)}/{item.get('p1_hit_count', 0)} | "
            f"{fmt(item.get('p1_complete_time'))} | {fmt(item.get('wings_encircled_time'))} | "
            f"{fmt(item.get('min_distance_p2'))}/{fmt(item.get('min_distance_p3'))} | "
            f"{fmt(item.get('min_inter_pursuer_distance'))} | "
            f"{fmt(item.get('centroid_error_at_best_m'))} |"
        )
    atomic_write_text(batch_root / "批次汇总.md", "\n".join(lines) + "\n")
    atomic_write_json(batch_root / "records.json", ordered)


def scenario_desc(item: dict[str, Any]) -> str:
    geometry = "直线迎头" if item.get("geometry") == "head_on" else "尾追"
    return f"{item.get('target_speed_mps', 0):g} m/s {item.get('scenario')} {geometry}"


def curate(batch_root: Path, records: list[dict[str, Any]], select_top: int) -> list[dict[str, Any]]:
    candidates = [item for item in records if item.get("smooth_pass")]
    selected = sorted(
        candidates,
        key=lambda item: (-float(item.get("quality_score", -1e9)), int(item.get("index", 0))),
    )[:select_top]
    preferred_root = batch_root / "优选案例"
    image_root = preferred_root / "图片"
    image_root.mkdir(parents=True, exist_ok=True)
    items: list[dict[str, Any]] = []
    cards: list[str] = []
    index_lines = ["# 优选平滑案例", ""]
    for rank, record in enumerate(selected, start=1):
        metrics_dir = Path(record["metrics_dir"])
        tag = f"案例{rank:02d}_{record['target_speed_mps']:g}mps_直线迎头"
        case_root = preferred_root / tag
        case_root.mkdir(parents=True, exist_ok=True)
        for filename in ("metrics.json", "summary.txt", "params_snapshot.yaml",
                         "trajectory.csv", "p1_shot_events.csv"):
            source = metrics_dir / filename
            if source.is_file():
                shutil.copy2(source, case_root / filename)
        image_items = []
        for source_name, numbered_name, caption in FIGURES:
            source = metrics_dir / source_name
            if not source.is_file():
                continue
            destination_name = f"{tag}_{source_name}"
            destination = image_root / destination_name
            shutil.copy2(source, destination)
            shutil.copy2(source, case_root / source_name)
            image_items.append({"file": f"图片/{destination_name}", "label": numbered_name,
                                "caption": caption})
        item = {
            "rank": rank, "tag": tag, "description": scenario_desc(record),
            "case_id": record["case_id"], "quality_score": record["quality_score"],
            "p1_complete_time": record.get("p1_complete_time"),
            "wings_encircled_time": record.get("wings_encircled_time"),
            "min_distance": [record.get("min_distance_p1"), record.get("min_distance_p2"),
                             record.get("min_distance_p3")],
            "final_distance": [record.get("final_distance_p1"), record.get("final_distance_p2"),
                               record.get("final_distance_p3")],
            "min_inter_pursuer_distance": record.get("min_inter_pursuer_distance"),
            "p1_actual_shot_distances_m": record.get("p1_actual_shot_distances_m"),
            "centroid_error_at_best_m": record.get("centroid_error_at_best_m"),
            "images": image_items,
        }
        items.append(item)
        index_lines += [
            f"## {rank:02d}. {scenario_desc(record)}", "",
            f"- 原案例：`{record['case_id']}`",
            f"- P1 COMPLETE / 僚机包围：{fmt(record.get('p1_complete_time'))} / "
            f"{fmt(record.get('wings_encircled_time'))} s",
            f"- P2/P3 全程最小目标距离：{fmt(record.get('min_distance_p2'))} / "
            f"{fmt(record.get('min_distance_p3'))} m",
            f"- 三机最小两两间距：{fmt(record.get('min_inter_pursuer_distance'))} m", "",
            f"- P1实际开火距离：{record.get('p1_actual_shot_distances_m', [])} m",
            f"- 最佳包围质心误差：{fmt(record.get('centroid_error_at_best_m'))} m", "",
        ]
        figure_cards = "".join(
            f'<div class="card"><img src="{html.escape(image["file"])}" loading="lazy" '
            f'onclick="this.classList.toggle(\'zoom\')"><div class="meta"><b>'
            f'{html.escape(image["label"])}</b><span>{html.escape(image["caption"])}</span></div></div>'
            for image in image_items
        )
        cards.append(
            f'<h2>{rank:02d}. {html.escape(scenario_desc(record))}</h2><div class="stat">'
            f'P1 COMPLETE {fmt(record.get("p1_complete_time"))} s · 僚机包围 '
            f'{fmt(record.get("wings_encircled_time"))} s · min d2/d3 '
            f'{fmt(record.get("min_distance_p2"))}/{fmt(record.get("min_distance_p3"))} m · '
            f'min 两两 {fmt(record.get("min_inter_pursuer_distance"))} m</div>'
            f'<div class="grid">{figure_cards}</div>'
        )
    atomic_write_json(preferred_root / "items.json", items)
    atomic_write_text(preferred_root / "案例索引.md", "\n".join(index_lines) + "\n")
    page = f"""<!DOCTYPE html><html lang="zh"><head><meta charset="utf-8">
<title>PX4 三机协同拦截优选案例</title><style>
body{{font-family:"Microsoft YaHei",sans-serif;margin:0;background:#f5f6f8;color:#222}}
header{{background:#1f2d3d;color:#fff;padding:18px 24px}} h1{{margin:0 0 6px;font-size:21px}}
h2{{padding:18px 24px 0;font-size:17px}} .stat{{padding:6px 24px;color:#444}}
.grid{{display:grid;grid-template-columns:repeat(auto-fill,minmax(430px,1fr));gap:14px;padding:12px 24px}}
.card{{background:#fff;border:1px solid #ddd;border-radius:8px;overflow:hidden;box-shadow:0 1px 3px #0002}}
.card img{{width:100%;display:block;cursor:zoom-in}} .card img.zoom{{position:fixed;inset:0;margin:auto;
width:auto;max-width:94vw;max-height:94vh;z-index:10;background:#fff;cursor:zoom-out;box-shadow:0 10px 40px #0008}}
.meta{{padding:8px 12px;display:flex;gap:12px;flex-wrap:wrap}} .meta span{{color:#666;font-size:12px}}
</style></head><body><header><h1>PX4 三机协同拦截 · 自动优选平滑案例</h1>
<div>实机 P1 原闭环零修改 · P2/P3 真值算法 + PX4 SITL/Gazebo 实控 · 案例和图片均带序号</div></header>
{''.join(cards)}</body></html>"""
    atomic_write_text(preferred_root / "index.html", page)
    return items


def load_existing(args: argparse.Namespace, batch_root: Path) -> list[dict[str, Any]]:
    records = []
    for index, metrics_dir in enumerate(args.curate_existing or [], start=1):
        case = {
            "index": index, "case_id": f"{index:03d}_existing_{metrics_dir.name}",
            "target_speed_mps": 5.0, "scenario": "straight",
            "geometry": "head_on", "repeat": index,
        }
        records.append(record_from_metrics(case, metrics_dir.resolve(), None, 0, args))
    write_aggregates(batch_root, records)
    curate(batch_root, records, args.select_top)
    return records


def main() -> int:
    args = parse_args()
    try:
        speeds, scenarios = validate_args(args)
    except ValueError as error:
        print(f"参数错误: {error}", file=sys.stderr)
        return 2
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    batch_root = (args.output_root or
                  DEFAULT_OUTPUT_PARENT / f"05_PX4三机协同自动测试_{timestamp}").resolve()
    batch_root.mkdir(parents=True, exist_ok=True)
    if args.curate_existing:
        records = load_existing(args, batch_root)
        print(f"已导入 {len(records)} 个结果：{batch_root}")
        return 0

    cases = build_cases(args, speeds, scenarios)
    atomic_write_json(batch_root / "测试计划.json", cases)
    if args.dry_run:
        print(f"已生成 {len(cases)} 轮计划：{batch_root / '测试计划.json'}")
        return 0

    records_by_id: dict[str, dict[str, Any]] = {}
    records_path = batch_root / "records.json"
    if args.resume and records_path.is_file():
        for item in json.loads(records_path.read_text(encoding="utf-8")):
            records_by_id[item["case_id"]] = item

    try:
        for offset, case in enumerate(cases):
            old = records_by_id.get(case["case_id"])
            if old and old.get("status") == "completed" and (
                old.get("smooth_pass") or not args.rerun_failed
            ):
                print(f"[{case['case_id']}] 已有结果，跳过")
                continue
            print(f"\n[{case['case_id']}] 开始：{scenario_desc(case)}", flush=True)
            record = run_case(case, batch_root, args)
            infra_attempts = []
            for retry in range(1, args.max_infra_retries + 1):
                if not is_infrastructure_failure(
                        record, float(case["target_speed_mps"])):
                    break
                infra_attempts.append(record)
                print(
                    f"[{case['case_id']}] 基础设施失败，保留原日志并自动重试 "
                    f"{retry}/{args.max_infra_retries}", flush=True)
                time.sleep(max(args.inter_case_settle, 2.0))
                record = run_case(case, batch_root, args)
            if infra_attempts:
                record["infrastructure_retry_count"] = len(infra_attempts)
                record["infrastructure_failed_attempts"] = [
                    {key: item.get(key) for key in
                     ("run_dir", "metrics_dir", "target_max_speed", "error")}
                    for item in infra_attempts]
            records_by_id[case["case_id"]] = record
            records = list(records_by_id.values())
            write_aggregates(batch_root, records)
            print(
                f"[{case['case_id']}] mission={'SUCCESS' if record.get('mission_success') else 'FAIL'} "
                f"smooth={'PASS' if record.get('smooth_pass') else 'FAIL'} "
                f"min_d2/d3={fmt(record.get('min_distance_p2'))}/"
                f"{fmt(record.get('min_distance_p3'))} m",
                flush=True,
            )
            if offset + 1 < len(cases) and args.inter_case_settle > 0.0:
                time.sleep(args.inter_case_settle)
    except KeyboardInterrupt:
        write_aggregates(batch_root, list(records_by_id.values()))
        curate(batch_root, list(records_by_id.values()), args.select_top)
        print(f"\n批次已中断，已保留可续跑状态：{batch_root}")
        return 130

    records = list(records_by_id.values())
    write_aggregates(batch_root, records)
    selected = curate(batch_root, records, args.select_top)
    smooth_count = sum(bool(item.get("smooth_pass")) for item in records)
    print(
        f"\n批次完成：{len(records)} 轮，平滑通过 {smooth_count} 轮，"
        f"优选 {len(selected)} 例\n结果：{batch_root}"
    )
    return 0 if records and smooth_count == len(records) else 1


if __name__ == "__main__":
    raise SystemExit(main())
