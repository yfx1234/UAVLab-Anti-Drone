#!/usr/bin/env python3
"""Offline comparison: RTK truth range vs stereo/async measured range.

Inputs (all JSONL, may be recorded on DIFFERENT machines):
  --enemy         output of rtk_truth_logger.py (target drone telemetry;
                  in the flight architecture this runs on laptop A)
  --measurements  output of field_measurement_logger.py (interceptor Jetson,
                  airborne or on the ground bench)

Method (utc mode, default):
  1. Common axis = GPS UTC. Enemy fixes are stamped at the source via the
     PX4 boot->UTC mapping from SYSTEM_TIME; radio latency and the logger
     machine's wall clock drop out of the math entirely.
  2. Own track: own RTK fixes carry GPS UTC (SensorGps.time_utc_usec).
     Measurement wall stamps are mapped to UTC via the robust offset
     fitted from the own RTK (wall, utc) pairs. Laptop-A and Jetson clock
     skew is therefore irrelevant. --clock-offset forces this mapping for
     bench tests without GPS UTC.
  3. For every valid stereo measurement: interpolate BOTH ENU tracks at
     the measurement UTC (rejected if either side's gap > --max-gap-s),
     compute range_truth (3D) and compare with |position_camera|.
  4. Report overall + per-layer error stats and the 2-sigma coverage using
     the filter's own line-of-sight range sigma.

recv mode (--time-mode recv) uses logger receive wall times and is only
valid when both loggers run on the same machine (ground-bench setup).

No ROS or third-party dependencies; unit-tested on the dev laptop.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

from rtk_truth_core import (
    body_lever_to_enu,
    error_stats,
    fit_clock_offset,
    geodetic_to_enu,
    interpolate_series,
    layer_index,
    layer_label,
    median_position,
    percentile,
    range_sigma_from_covariance,
)


def load_jsonl(path: str | Path) -> list[dict]:
    records = []
    with open(path, "r", encoding="utf-8") as source:
        for line in source:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def build_enemy_track(enemy_records: list[dict], min_fix: int, time_mode: str) -> dict:
    """Return clock diagnostics + geodetic fixes stamped on the common axis.

    utc mode (default, both-fly capable): fix times are GPS UTC derived
    from the PX4 boot->UTC mapping in SYSTEM_TIME. The logger machine's
    wall clock is NOT part of the measurement axis, so the enemy logger
    may run on any computer with any clock skew. The (wall, utc) pairs
    are still fitted and reported as a diagnostic of the logger clock.

    recv mode (fallback): fix times are logger receive wall times. Only
    valid when the enemy logger and the measurement logger run on the
    SAME machine (same wall clock), e.g. the ground-bench setup.
    """
    system_time = [r for r in enemy_records if r.get("kind") == "system_time" and r.get("utc")]
    gps = [
        r
        for r in enemy_records
        if r.get("kind") == "gps_raw_int" and r.get("fix", 0) >= min_fix
    ]
    clock = fit_clock_offset([(r["wall"], r["utc"]) for r in system_time])

    fixes = []
    if time_mode == "utc" and system_time:
        # Source-side timestamp: PX4 boot time -> UTC via nearest SYSTEM_TIME.
        sys_sorted = sorted(system_time, key=lambda r: r["boot_ms"])
        sys_boot = [r["boot_ms"] / 1e3 for r in sys_sorted]
        sys_utc = [r["utc"] for r in sys_sorted]
        for record in gps:
            boot_s = record["boot_us"] / 1e6
            idx = min(range(len(sys_boot)), key=lambda i: abs(sys_boot[i] - boot_s))
            if abs(sys_boot[idx] - boot_s) > 2.0:
                continue
            utc = sys_utc[idx] + (boot_s - sys_boot[idx])
            fixes.append({"t": utc, "lat": record["lat"], "lon": record["lon"], "alt": record["alt"]})
    else:
        time_mode = "recv"
        for record in gps:
            fixes.append({"t": record["wall"], "lat": record["lat"], "lon": record["lon"], "alt": record["alt"]})

    fixes.sort(key=lambda f: f["t"])
    # Drop duplicate/rewound timestamps.
    deduped = []
    for fix in fixes:
        if deduped and fix["t"] <= deduped[-1]["t"]:
            continue
        deduped.append(fix)
    return {"clock": clock, "time_mode": time_mode, "fixes": deduped, "n_gps_raw": len(gps), "n_system_time": len(system_time)}


def run_comparison(
    enemy_records: list[dict],
    measurement_records: list[dict],
    lever_enu: tuple[float, float, float] = (0.0, 0.0, 0.0),
    min_fix: int = 6,
    max_gap_s: float = 0.3,
    layers: tuple[float, ...] = (5.0, 10.0, 15.0, 20.0, 30.0),
    source: str = "async",
    time_mode: str = "utc",
    anchor_override: tuple[float, float, float] | None = None,
    clock_offset_override: float | None = None,
) -> dict:
    """Compare measured ranges against RTK truth on a common time axis.

    Both-fly architecture (utc mode, default): everything is aligned on
    GPS UTC. The enemy track uses PX4 boot->UTC; own RTK carries GPS UTC
    (SensorGps.time_utc_usec); measurement wall stamps are mapped to UTC
    via the robust offset fitted from the own RTK (wall, utc) pairs. The
    enemy logger and the interceptor Jetson may therefore run on any
    computers with arbitrary clock skew (they do in the flight setup:
    laptop A vs airborne Jetson).

    Own side is a TIME-VARYING ENU track interpolated at each measurement
    time, exactly like the enemy track; a stationary interceptor
    degenerates to the static-anchor case. --anchor forces a fixed own
    point (bench/plumbing tests). --clock-offset (seconds, wall-minus-UTC)
    forces the measurement time mapping when own RTK UTC is unavailable
    (e.g. indoor bench tests without any GPS lock).
    """
    track = build_enemy_track(enemy_records, min_fix, time_mode)
    fixes = track["fixes"]

    own_all = [
        {"wall": r["wall"], "utc": r.get("utc"), "lat": r["lat"], "lon": r["lon"], "alt": r["alt"]}
        for r in measurement_records
        if r.get("kind") == "own_rtk" and r.get("fix", 0) >= min_fix
    ]
    own_all.sort(key=lambda f: f["wall"])
    own_deduped = []
    for fix in own_all:
        if own_deduped and fix["wall"] <= own_deduped[-1]["wall"]:
            continue
        own_deduped.append(fix)

    static_anchor = None
    if anchor_override is not None:
        static_anchor = {"lat": anchor_override[0], "lon": anchor_override[1], "alt": anchor_override[2], "count": -1}
    origin = None
    if static_anchor:
        origin = (static_anchor["lat"], static_anchor["lon"], static_anchor["alt"])
    elif own_deduped:
        med = median_position(own_deduped)
        origin = (med["lat"], med["lon"], med["alt"])

    # Measurement wall -> common axis mapping.
    jetson_clock: dict = {"offset_s": 0.0, "n_used": 0, "n_total": 0, "residual_std_ms": None, "drift_ms_per_s": None}
    offset_j = 0.0
    if time_mode == "utc":
        if clock_offset_override is not None:
            offset_j = clock_offset_override
            jetson_clock["offset_s"] = offset_j
        else:
            pairs = [(r["wall"], r["utc"]) for r in own_deduped if r.get("utc")]
            jetson_clock = fit_clock_offset(pairs)
            if not pairs:
                origin = None  # force the error path below with a clear message
            else:
                offset_j = jetson_clock["offset_s"]

    result: dict = {
        "axis": "gps_utc" if time_mode == "utc" else "logger_recv_wall",
        "clock_logger_wall_vs_utc": track["clock"],
        "clock_jetson_wall_vs_utc": jetson_clock,
        "time_mode": time_mode,
        "enemy": {
            "fixes_used": len(fixes),
            "gps_raw_total": track["n_gps_raw"],
            "system_time_total": track["n_system_time"],
        },
        "own": {
            "mode": "static_override" if static_anchor else "track",
            "fixes_used": len(own_deduped),
        },
        "lever_enu": list(lever_enu),
        "params": {"min_fix": min_fix, "max_gap_s": max_gap_s, "layers": list(layers), "source": source},
    }
    if origin is None:
        result["error"] = (
            "no usable own RTK: need fixes with fix>=min_fix and GPS UTC "
            "(time_utc_usec); indoors pass --anchor lat,lon,alt and --clock-offset <s>"
        )
        return result
    if len(fixes) < 2:
        result["error"] = "not enough enemy RTK fixes to build a track"
        return result

    # Own track on the common axis.
    if time_mode == "utc":
        own_track = [f for f in own_deduped if f.get("utc")]
        own_times = [f["utc"] for f in own_track]
    else:
        own_track = own_deduped
        own_times = [f["wall"] for f in own_track]
    if not static_anchor and len(own_track) < 2:
        result["error"] = "not enough own RTK fixes (with UTC) to build a track"
        return result
    own_enu = [geodetic_to_enu(f["lat"], f["lon"], f["alt"], *origin) for f in own_track]
    if own_enu:
        norms = [math.sqrt(sum(c * c for c in enu)) for enu in own_enu]
        result["own"]["spread_m"] = max(norms) - min(norms)

    kind = "async_meas" if source == "async" else "cam_pos"
    measurements = [
        r
        for r in measurement_records
        if r.get("kind") == kind and r.get("valid") and r.get("position")
    ]
    enemy_times = [f["t"] for f in fixes]
    enemy_enu = [geodetic_to_enu(f["lat"], f["lon"], f["alt"], *origin) for f in fixes]

    samples = []
    rejected_enemy_gap = 0
    rejected_own_gap = 0
    for meas in measurements:
        t_query = (meas["wall"] - offset_j) if time_mode == "utc" else meas["wall"]
        truth = interpolate_series(enemy_times, enemy_enu, t_query, max_gap_s)
        if truth is None:
            rejected_enemy_gap += 1
            continue
        if static_anchor:
            own_pos = (0.0, 0.0, 0.0)
        else:
            own_pos = interpolate_series(own_times, own_enu, t_query, max_gap_s)
            if own_pos is None:
                rejected_own_gap += 1
                continue
        dx = truth[0] - (own_pos[0] + lever_enu[0])
        dy = truth[1] - (own_pos[1] + lever_enu[1])
        dz = truth[2] - (own_pos[2] + lever_enu[2])
        range_truth = math.sqrt(dx * dx + dy * dy + dz * dz)
        position = meas["position"]
        range_meas = math.sqrt(sum(c * c for c in position))
        sigma = range_sigma_from_covariance(position, meas.get("covariance", []))
        samples.append(
            {
                "wall": meas["wall"],
                "range_truth": range_truth,
                "range_meas": range_meas,
                "error": range_meas - range_truth,
                "sigma": sigma,
                "timing_delta": meas.get("timing_delta"),
                "reason": meas.get("reason"),
            }
        )

    errors = [s["error"] for s in samples]
    result["measurements"] = {
        "total_valid_input": len(measurements),
        "compared": len(samples),
        "rejected_enemy_gap": rejected_enemy_gap,
        "rejected_own_gap": rejected_own_gap,
    }
    result["overall"] = error_stats(errors)
    with_sigma = [s for s in samples if s["sigma"]]
    if with_sigma:
        covered = [s for s in with_sigma if abs(s["error"]) <= 2.0 * s["sigma"]]
        result["sigma_coverage"] = {
            "count": len(with_sigma),
            "within_2sigma": len(covered),
            "fraction": len(covered) / len(with_sigma),
            "mean_sigma_m": sum(s["sigma"] for s in with_sigma) / len(with_sigma),
        }
    timing = [s["timing_delta"] for s in samples if s["timing_delta"] is not None]
    if timing:
        result["timing_delta_ms"] = {
            "p50": percentile(timing, 0.50) * 1000.0,
            "p95": percentile(timing, 0.95) * 1000.0,
            "max": max(timing) * 1000.0,
        }

    per_layer: dict[str, dict] = {}
    for sample in samples:
        idx = layer_index(sample["range_truth"], layers)
        if idx is None:
            continue
        label = layer_label(idx, layers)
        per_layer.setdefault(label, []).append(sample["error"])
    result["per_layer"] = {label: error_stats(values) for label, values in per_layer.items()}
    result["result"] = "OK"
    return result


def format_summary(result: dict) -> str:
    lines = []
    logger_clock = result.get("clock_logger_wall_vs_utc", {})
    jetson_clock = result.get("clock_jetson_wall_vs_utc", {})
    lines.append(
        f"axis={result.get('axis')} | logger wall-vs-utc: offset={logger_clock.get('offset_s', 0.0):+.3f}s "
        f"residual={logger_clock.get('residual_std_ms')}ms used={logger_clock.get('n_used')}/{logger_clock.get('n_total')} "
        f"| jetson wall-vs-utc: offset={jetson_clock.get('offset_s', 0.0):+.3f}s "
        f"residual={jetson_clock.get('residual_std_ms')}ms used={jetson_clock.get('n_used')}/{jetson_clock.get('n_total')}"
    )
    own = result.get("own") or {}
    if own:
        lines.append(
            f"own: mode={own.get('mode')} fixes={own.get('fixes_used')} "
            f"spread={own.get('spread_m', 0.0):.2f}m"
        )
    if "error" in result:
        lines.append(f"ERROR: {result['error']}")
        return "\n".join(lines)
    meas = result["measurements"]
    lines.append(
        f"measurements: compared={meas['compared']}/{meas['total_valid_input']} "
        f"(enemy-gap rejects={meas['rejected_enemy_gap']} own-gap rejects={meas['rejected_own_gap']})"
    )
    overall = result["overall"]
    if overall.get("count"):
        lines.append(
            "overall: n={count} mean={mean_m:+.3f}m |err| mean={abs_mean_m:.3f} p50={p50_m:.3f} "
            "p95={p95_m:.3f} max={max_m:.3f} rmse={rmse_m:.3f}".format(**overall)
        )
    coverage = result.get("sigma_coverage")
    if coverage:
        lines.append(
            f"sigma coverage: {coverage['within_2sigma']}/{coverage['count']} within 2σ "
            f"({coverage['fraction'] * 100:.1f}%), mean σ={coverage['mean_sigma_m']:.3f}m"
        )
    timing = result.get("timing_delta_ms")
    if timing:
        lines.append(
            f"timing_delta: p50={timing['p50']:.2f}ms p95={timing['p95']:.2f}ms max={timing['max']:.2f}ms"
        )
    for label, stats in result.get("per_layer", {}).items():
        if stats.get("count"):
            lines.append(
                f"layer {label}: n={stats['count']} mean={stats['mean_m']:+.3f} "
                f"p50={stats['p50_m']:.3f} p95={stats['p95_m']:.3f} max={stats['max_m']:.3f}"
            )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--enemy", required=True, help="rtk_truth_logger JSONL")
    parser.add_argument("--measurements", required=True, help="field_measurement_logger JSONL")
    parser.add_argument("--output", help="write full result JSON here")
    parser.add_argument("--lever-enu", default="0,0,0", help="antenna->camera offset ENU [m], e.g. 0.15,0,-0.05")
    parser.add_argument("--lever-body", default=None,
                        help="antenna->camera lever in body frame (FLU: x fwd, y left, z up), "
                             "e.g. 0.133,0,0.0462; converted to ENU with --rig-yaw-deg")
    parser.add_argument("--rig-yaw-deg", type=float, default=0.0,
                        help="compass heading of the body X axis at setup, degrees from north")
    parser.add_argument("--anchor", help="override own track with a fixed point lat,lon,alt (bench/plumbing tests)")
    parser.add_argument("--clock-offset", type=float, default=None,
                        help="force measurement wall->UTC offset in seconds (wall-minus-UTC); "
                             "use when own RTK has no GPS UTC, e.g. indoor bench tests")
    parser.add_argument("--min-fix", type=int, default=6, help="min fix_type on BOTH sides (6=RTK fixed)")
    parser.add_argument("--max-gap-s", type=float, default=0.3)
    parser.add_argument("--layers", default="5,10,15,20,30")
    parser.add_argument("--source", choices=["async", "camera"], default="async")
    parser.add_argument("--time-mode", choices=["utc", "recv"], default="utc",
                        help="utc: GPS-UTC axis, loggers may be on different machines (default, flight-capable); "
                             "recv: logger receive wall, only valid when both loggers share one machine")
    args = parser.parse_args()

    lever = tuple(float(v) for v in args.lever_enu.split(","))
    if args.lever_body:
        if args.lever_enu != "0,0,0":
            parser.error("--lever-body and --lever-enu are mutually exclusive")
        lever = body_lever_to_enu(
            tuple(float(v) for v in args.lever_body.split(",")), args.rig_yaw_deg
        )
        print(
            f"lever: body FLU {args.lever_body} @yaw {args.rig_yaw_deg:g}deg -> "
            f"ENU {lever[0]:+.4f},{lever[1]:+.4f},{lever[2]:+.4f} m"
        )
    anchor = tuple(float(v) for v in args.anchor.split(",")) if args.anchor else None
    layers = tuple(float(v) for v in args.layers.split(","))

    result = run_comparison(
        load_jsonl(args.enemy),
        load_jsonl(args.measurements),
        lever_enu=lever,
        min_fix=args.min_fix,
        max_gap_s=args.max_gap_s,
        layers=layers,
        source=args.source,
        time_mode=args.time_mode,
        anchor_override=anchor,
        clock_offset_override=args.clock_offset,
    )
    print(format_summary(result))
    if args.output:
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w", encoding="utf-8") as sink:
            json.dump(result, sink, indent=2)
    return 0 if "error" not in result else 1


if __name__ == "__main__":
    sys.exit(main())
