"""Unit tests for the RTK truth field-test toolchain.

Pure stdlib; runs on the dev laptop and on the Jetson. Imports the tools
directory directly because tools/ is not a ROS python package.
"""

from __future__ import annotations

import math
import random
import sys
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parents[1] / "tools" / "rtk_field_test"
sys.path.insert(0, str(TOOLS_DIR))

from rtk_truth_core import (  # noqa: E402
    fit_clock_offset,
    geodetic_to_enu,
    interpolate_series,
    layer_index,
    layer_label,
    median_position,
    percentile,
    range_sigma_from_covariance,
)
from compare_rtk_stereo_truth import run_comparison  # noqa: E402

HOME_LAT = 30.5278
HOME_LON = 114.3567
HOME_ALT = 25.0
# Measured against geodetic_to_enu at the test site (approximation only
# used to generate synthetic geodetic coordinates).
M_PER_DEG_LAT = 110_862.0
M_PER_DEG_LON = 95_972.0


def enu_to_geodetic_approx(east: float, north: float, up: float = 0.0) -> tuple[float, float, float]:
    return (
        HOME_LAT + north / M_PER_DEG_LAT,
        HOME_LON + east / M_PER_DEG_LON,
        HOME_ALT + up,
    )


def test_geodetic_to_enu_north_axis():
    lat, lon, alt = enu_to_geodetic_approx(0.0, 100.0)
    e, n, u = geodetic_to_enu(lat, lon, alt, HOME_LAT, HOME_LON, HOME_ALT)
    assert abs(e) < 0.01
    assert abs(n - 100.0) < 0.01
    assert abs(u) < 0.01


def test_geodetic_to_enu_east_axis():
    lat, lon, alt = enu_to_geodetic_approx(50.0, 0.0)
    e, n, u = geodetic_to_enu(lat, lon, alt, HOME_LAT, HOME_LON, HOME_ALT)
    assert abs(e - 50.0) < 0.05
    assert abs(n) < 0.01


def test_geodetic_to_enu_up():
    e, n, u = geodetic_to_enu(HOME_LAT, HOME_LON, HOME_ALT + 10.0, HOME_LAT, HOME_LON, HOME_ALT)
    assert abs(u - 10.0) < 0.01
    assert abs(e) < 0.01 and abs(n) < 0.01


def test_fit_clock_offset_recovers_constant_with_outliers():
    rng = random.Random(7)
    pairs = [(1000.0 + t + 2.5 + rng.gauss(0, 0.003), 1000.0 + t) for t in range(60)]
    pairs.append((1010.0 + 2.5 + 1.0, 1010.0))  # 1 s outlier
    result = fit_clock_offset(pairs)
    assert abs(result["offset_s"] - 2.5) < 0.01
    assert result["residual_std_ms"] < 5.0
    assert result["n_used"] == 60
    assert result["n_total"] == 61


def test_interpolate_series_linear_and_gap_reject():
    times = [1.0, 2.0, 3.0]
    values = [(10.0,), (20.0,), (30.0,)]
    assert interpolate_series(times, values, 1.5, 2.0) == (15.0,)
    assert interpolate_series(times, values, 1.0, 2.0) == (10.0,)
    assert interpolate_series(times, values, 0.5, 2.0) is None
    assert interpolate_series(times, values, 3.5, 2.0) is None
    assert interpolate_series(times, values, 2.5, 0.5) is None  # gap 1.0 > 0.5


def test_range_sigma_isotropic():
    covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]
    sigma = range_sigma_from_covariance([3.0, 4.0, 0.0], covariance)
    assert sigma is not None and abs(sigma - 0.2) < 1e-9


def test_layers_and_median_position():
    edges = (5.0, 10.0, 20.0)
    assert layer_index(3.0, edges) == 0
    assert layer_index(7.0, edges) == 1
    assert layer_index(25.0, edges) == 3
    assert layer_label(0, edges) == "0-5m"
    assert layer_label(1, edges) == "5-10m"
    assert layer_label(3, edges) == ">=20m"
    anchor = median_position(
        [{"lat": 1.0, "lon": 2.0, "alt": 10.0}, {"lat": 1.1, "lon": 1.9, "alt": 10.2}]
    )
    assert anchor["count"] == 2 and abs(anchor["lat"] - 1.05) < 1e-9


def _synthetic_scenario(offset_true: float = 2.5, radio_lag: float = 0.0, include_own: bool = True):
    """Target approaches along the north axis 30 m -> 10 m at 5 m/s.

    Returns (enemy_records, measurement_records). Wall axis is offset from
    GPS utc by offset_true, like a Jetson that was never NTP-synced. Own
    RTK fixes (static at HOME) span the whole session, as the real
    interceptor DDS stream would.
    """
    rng = random.Random(11)
    enemy = []
    measurements = []
    wall0 = 1_700_000_000.0
    for step in range(400):  # 10 Hz over 40 s
        t = step / 10.0
        utc = wall0 + t
        north = 30.0 - 5.0 * t
        north = max(north, 10.0)
        lat, lon, alt = enu_to_geodetic_approx(2.0, north)
        wall = utc + offset_true
        enemy.append(
            {
                "kind": "gps_raw_int",
                "wall": wall + radio_lag + rng.gauss(0, 0.002),
                "boot_us": int(t * 1e6),
                "lat": lat,
                "lon": lon,
                "alt": alt,
                "fix": 6,
            }
        )
        if step % 2 == 0:  # SYSTEM_TIME at 5 Hz
            enemy.append(
                {
                    "kind": "system_time",
                    "wall": wall + rng.gauss(0, 0.003),
                    "boot_ms": int(t * 1000),
                    "utc": utc,
                }
            )
        if include_own and step % 2 == 0:  # own RTK at 5 Hz, static
            measurements.append(
                {
                    "kind": "own_rtk",
                    "wall": wall,
                    "utc": utc,
                    "lat": HOME_LAT,
                    "lon": HOME_LON,
                    "alt": HOME_ALT,
                    "fix": 6,
                }
            )
        # async measurement at 10 Hz equivalent
        true_range = math.sqrt(north * north + 2.0 * 2.0)
        noise = rng.uniform(-0.03, 0.03)
        meas_range = true_range + noise
        ux, uy, uz = 2.0 / true_range, north / true_range, 0.0
        measurements.append(
            {
                "kind": "async_meas",
                "wall": wall,
                "valid": True,
                "reason": "accepted",
                "position": [ux * meas_range, uy * meas_range, uz * meas_range],
                "covariance": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                "timing_delta": 0.012,
            }
        )
    measurements.sort(key=lambda r: r["wall"])
    return enemy, measurements


def test_comparison_recovers_clock_and_noise_floor():
    enemy, measurements = _synthetic_scenario()
    result = run_comparison(enemy, measurements, max_gap_s=0.3)
    assert result.get("result") == "OK", result.get("error")
    assert result["own"]["mode"] == "track"  # both-fly path, static here
    overall = result["overall"]
    # injected noise is uniform +-0.03 m; a broken clock map would add
    # up to ~5 m/s * offset jitter, far above this floor.
    assert overall["abs_mean_m"] < 0.05
    assert overall["p95_m"] < 0.10
    assert abs(result["clock_jetson_wall_vs_utc"]["offset_s"] - 2.5) < 0.02
    coverage = result["sigma_coverage"]
    assert coverage["fraction"] > 0.95  # sigma=0.1, noise<=0.03 -> full cover
    layers = result["per_layer"]
    assert any(label.startswith("20-30") or "20" in label for label in layers)


def test_comparison_decoupled_logger_machines():
    """Both-fly regression: the enemy logger runs on a machine whose clock
    is 97.5 s ahead of the Jetson. GPS-UTC alignment must be unaffected;
    recv (same-wall) mode must visibly break."""
    enemy, measurements = _synthetic_scenario()
    skew = 97.5
    for record in enemy:
        record["wall"] = record["wall"] + skew
    result = run_comparison(enemy, measurements, max_gap_s=0.3)
    assert result.get("result") == "OK", result.get("error")
    overall = result["overall"]
    assert overall["abs_mean_m"] < 0.05
    assert overall["p95_m"] < 0.10
    # diagnostic: the logger machine's large skew is visible but unused
    assert result["clock_logger_wall_vs_utc"]["offset_s"] > 90.0
    # negative control: recv mode uses logger wall -> everything rejected
    # (span mismatch) or metres of fake error
    result_recv = run_comparison(enemy, measurements, max_gap_s=0.3, time_mode="recv")
    assert result_recv.get("result") == "OK"
    recv_overall = result_recv["overall"]
    assert recv_overall["count"] == 0 or recv_overall.get("max_m", 0.0) > 1.0


def test_comparison_moving_interceptor():
    """Both-fly case: interceptor moves east at 1 m/s, target hovers 20 m north."""
    rng = random.Random(5)
    enemy = []
    measurements = []
    wall0 = 1_700_000_000.0
    for step in range(400):
        t = step / 10.0
        utc = wall0 + t
        wall = utc  # same machine: no offset
        own_east = 1.0 * t
        own_lat, own_lon, own_alt = enu_to_geodetic_approx(own_east, 0.0)
        enemy_lat, enemy_lon, enemy_alt = enu_to_geodetic_approx(2.0, 20.0)
        if step % 2 == 0:
            enemy.append(
                {"kind": "system_time", "wall": wall, "boot_ms": int(t * 1000), "utc": utc}
            )
            measurements.append(
                {"kind": "own_rtk", "wall": wall, "utc": utc, "lat": own_lat, "lon": own_lon, "alt": own_alt, "fix": 6}
            )
        enemy.append(
            {"kind": "gps_raw_int", "wall": wall, "boot_us": int(t * 1e6),
             "lat": enemy_lat, "lon": enemy_lon, "alt": enemy_alt, "fix": 6}
        )
        true_range = math.sqrt((20.0) ** 2 + (2.0 - own_east) ** 2)
        noise = rng.uniform(-0.03, 0.03)
        meas_range = true_range + noise
        measurements.append(
            {
                "kind": "async_meas",
                "wall": wall,
                "valid": True,
                "reason": "accepted",
                "position": [0.0, 0.0, meas_range],
                "covariance": [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01],
                "timing_delta": 0.012,
            }
        )
    measurements.sort(key=lambda r: r["wall"])
    result = run_comparison(enemy, measurements, max_gap_s=0.3)
    assert result.get("result") == "OK", result.get("error")
    overall = result["overall"]
    # if the own track were treated as static, the 40 m of own motion
    # would show up as metres of fake error.
    assert overall["abs_mean_m"] < 0.05
    assert overall["p95_m"] < 0.10
    assert result["own"]["mode"] == "track"


def test_comparison_recv_mode_shows_radio_lag_bias():
    enemy, measurements = _synthetic_scenario(radio_lag=0.15)
    result = run_comparison(
        enemy,
        measurements,
        anchor_override=(HOME_LAT, HOME_LON, HOME_ALT),
        time_mode="recv",
    )
    assert result.get("result") == "OK"
    # 5 m/s * 0.15 s = 0.75 m bias during the moving segment must be
    # visible in recv mode; this is why the field test defaults to utc.
    assert result["overall"]["p95_m"] > 0.4
    assert result["overall"]["max_m"] > 0.5


def test_comparison_requires_anchor():
    enemy, measurements = _synthetic_scenario(include_own=False)
    result = run_comparison(enemy, measurements)
    assert "error" in result


def test_percentile():
    assert percentile([1.0, 2.0, 3.0], 0.5) == 2.0
    assert percentile([], 0.5) is None


def test_logger_sysid_lock_rules():
    """The logger must never auto-lock a GCS sysid (255), and must only
    lock on autopilot-originated message types. Regression test for the
    MAVProxy/GCS-heartbeat starvation failure mode."""
    from rtk_truth_logger import should_lock_sysid

    # GCS heartbeat arriving first must not win the lock
    assert not should_lock_sysid("HEARTBEAT", 255, 0)
    # autopilot heartbeat alone does not lock either (no GPS evidence yet)
    assert not should_lock_sysid("HEARTBEAT", 1, 0)
    # GPS/SYSTEM_TIME from an autopilot sysid locks
    assert should_lock_sysid("GPS_RAW_INT", 1, 0)
    assert should_lock_sysid("SYSTEM_TIME", 42, 0)
    assert should_lock_sysid("GLOBAL_POSITION_INT", 2, 0)
    # even with GPS evidence, 255 never locks automatically
    assert not should_lock_sysid("GPS_RAW_INT", 255, 0)
    # explicit --source-system locks only that sysid
    assert should_lock_sysid("GPS_RAW_INT", 7, 7)
    assert not should_lock_sysid("GPS_RAW_INT", 8, 7)


def test_body_lever_to_enu():
    from rtk_truth_core import body_lever_to_enu

    lever = (0.133, 0.0, 0.0462)
    # 机头朝北：X_b -> 北
    e, n, u = body_lever_to_enu(lever, 0.0)
    assert abs(e) < 1e-9 and abs(n - 0.133) < 1e-9 and abs(u - 0.0462) < 1e-9
    # 机头朝东：X_b -> 东
    e, n, u = body_lever_to_enu(lever, 90.0)
    assert abs(e - 0.133) < 1e-9 and abs(n) < 1e-9
    # 机头朝南：X_b -> 南(负北)
    e, n, _ = body_lever_to_enu(lever, 180.0)
    assert abs(e) < 1e-6 and abs(n + 0.133) < 1e-6
    # FLU 的 Y（左）在机头朝北时应指向西（负东）
    e, n, _ = body_lever_to_enu((0.0, 0.1, 0.0), 0.0)
    assert abs(e + 0.1) < 1e-9 and abs(n) < 1e-9


def test_monitor_stale_warning():
    """The live monitor must flag frozen input files instead of repeating
    the last sample forever (field confusion on 2026-07-31)."""
    import time
    from types import SimpleNamespace

    from live_range_monitor import LiveMonitor

    args = SimpleNamespace(
        lever_enu="0,0,0",
        anchor=None,
        clock_offset=None,
        out_csv=None,
        out_jsonl=None,
        min_fix=6,
        max_gap_s=0.3,
        layers="5,10,15,20,30",
        window_s=10.0,
        interval_s=2.0,
        anchor_min_fixes=10,
        stale_s=10.0,
    )
    monitor = LiveMonitor(args)
    monitor.last_input_monotonic = time.monotonic() - 60.0
    assert "STALE" in monitor.status_line()
    monitor.last_input_monotonic = time.monotonic()
    assert "STALE" not in monitor.status_line()


def test_live_monitor_once_matches_batch(tmp_path):
    """live_range_monitor --once must reproduce the batch comparator's numbers."""
    import csv
    import subprocess

    enemy, measurements = _synthetic_scenario()
    enemy_path = tmp_path / "enemy.jsonl"
    meas_path = tmp_path / "meas.jsonl"
    csv_path = tmp_path / "live.csv"
    import json

    enemy_path.write_text("".join(json.dumps(r) + "\n" for r in enemy))
    meas_path.write_text("".join(json.dumps(r) + "\n" for r in measurements))

    proc = subprocess.run(
        [
            sys.executable,
            str(TOOLS_DIR / "live_range_monitor.py"),
            "--enemy", str(enemy_path),
            "--measurements", str(meas_path),
            "--out-csv", str(csv_path),
            "--once",
        ],
        capture_output=True,
        text=True,
        timeout=60,
    )
    assert proc.returncode == 0, proc.stderr
    with open(csv_path, newline="") as handle:
        rows = list(csv.DictReader(handle))
    assert len(rows) >= 390  # batch comparator: 399; live lags boundary samples
    errors = [abs(float(r["error_m"])) for r in rows]
    assert sum(errors) / len(errors) < 0.05
    assert all(r["within_2sigma"] == "1" for r in rows)
