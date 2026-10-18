#!/usr/bin/env python3
"""Regression tests for the workspace-level interception batch runner."""

from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

WORKSPACE = Path(__file__).resolve().parents[3]
SCRIPT = WORKSPACE / "scripts" / "run_interception_batch.py"
# H-4：run_interception_batch.py 属纯拦截工作台脚本，不在本仓布局中——
# 缺失时全部用例 skipif 跳过（保留收集数，避免 pytest rc=5 被判失败）。
RUNNER_AVAILABLE = SCRIPT.is_file()
_SKIP_REASON = "run_interception_batch.py 不属于本仓（纯拦截工作台脚本）"

if RUNNER_AVAILABLE:
    SPEC = importlib.util.spec_from_file_location("run_interception_batch", SCRIPT)
    assert SPEC is not None and SPEC.loader is not None
    BATCH = importlib.util.module_from_spec(SPEC)
    SPEC.loader.exec_module(BATCH)


def _arguments(target_speed: float) -> SimpleNamespace:
    return SimpleNamespace(
        preset="planner-ab",
        repeats=1,
        target_speed=target_speed,
        trajectories="straight,circle,s_curve,figure_eight",
        mission_timeout=None,
        post_complete_hold=None,
        inter_case_settle=0.0,
    )


def _case() -> dict:
    return {
        "case_id": "001_head_on_planner_on_r1",
        "index": 1,
        "pair_id": "head_on_r1",
        "geometry": "head_on",
        "scenario": "straight",
        "repeat": 1,
        "planner_enabled": True,
    }


@pytest.mark.skipif(not RUNNER_AVAILABLE, reason=_SKIP_REASON)
def test_integral_target_speed_remains_a_ros_double() -> None:
    cases = BATCH.build_cases(_arguments(5.0))
    assert len(cases) == 6
    assert all(
        item["environment"]["TARGET_SPEED_MPS"] == "5.0" for item in cases
    )
    assert BATCH.ros_double(1.2) == "1.2"
    assert BATCH.ros_double(6.0) == "6.0"


@pytest.mark.skipif(not RUNNER_AVAILABLE, reason=_SKIP_REASON)
def test_speed_scan_uses_one_case_matrix_per_requested_speed() -> None:
    arguments = _arguments(5.0)
    arguments.preset = "encounters"
    arguments.target_speeds = "1.2,5,8"
    cases = BATCH.build_cases(arguments)
    assert len(cases) == 9
    assert [case["target_speed_mps"] for case in cases[::3]] == [1.2, 5.0, 8.0]
    assert all(case["environment"]["TARGET_SPEED_MPS"].endswith(".0")
               for case in cases if case["target_speed_mps"] in {5.0, 8.0})
    by_geometry = {case["geometry"]: case for case in cases[-3:]}
    assert (
        by_geometry["head_on"]["environment"]["KNOWN_TARGET_COURSE_ENABLED"]
        == "true"
    )
    assert (
        by_geometry["head_on"]["environment"]
        ["KNOWN_TARGET_SEARCH_VELOCITY_MPS"] == "8.0"
    )
    assert (
        by_geometry["crossing"]["environment"]
        ["KNOWN_TARGET_SEARCH_VELOCITY_MPS"] == "8.0"
    )
    assert (
        by_geometry["tail_chase"]["environment"]["KNOWN_TARGET_VELOCITY_GAIN"]
        == "1.0"
    )


@pytest.mark.skipif(not RUNNER_AVAILABLE, reason=_SKIP_REASON)
def test_critical_launch_failure_stops_the_batch(tmp_path: Path) -> None:
    raw_directory = tmp_path / "原始数据"
    raw_directory.mkdir()
    (raw_directory / "run_summary.txt").write_text(
        "wait_status=12\n"
        "critical_launch_failure=true\n"
        "critical_launch_failure_reason=pure_interceptor_controller process has died\n"
        "analysis_status=2\n",
        encoding="utf-8",
    )
    result = BATCH.normalized_result(
        _case(), "failed", 12, tmp_path, "start", "finish"
    )
    assert result["infrastructure_failure"] is True
    assert "process has died" in result["failure_reason"]


@pytest.mark.skipif(not RUNNER_AVAILABLE, reason=_SKIP_REASON)
def test_valid_analysis_failure_is_not_mislabeled_as_infrastructure(
    tmp_path: Path,
) -> None:
    raw_directory = tmp_path / "原始数据"
    raw_directory.mkdir()
    (raw_directory / "run_summary.txt").write_text(
        "wait_status=0\nanalysis_status=0\nacceptance_status=1\n",
        encoding="utf-8",
    )
    (tmp_path / "结果摘要.json").write_text(
        '{"strict_acceptance_passed": false, "mission_success": false, '
        '"referee_results": []}\n',
        encoding="utf-8",
    )
    result = BATCH.normalized_result(
        _case(), "failed", 1, tmp_path, "start", "finish"
    )
    assert result["summary_available"] is True
    assert result["infrastructure_failure"] is False
