#!/usr/bin/env python3
"""Live range-truth monitor for the RTK field test.

Tails the two JSONL streams written by rtk_truth_logger.py and
field_measurement_logger.py while the test is running, aligns both sides
on the GPS UTC axis and compares each valid stereo measurement against
the interpolated RTK truth in near real time:

- console: one status line per --interval-s (latest truth/meas/error,
  rolling window stats, RTK fix health, clock residual) so the crew can
  adjust the test on the spot;
- export: every compared sample is appended to a CSV (Excel/pandas
  friendly) and a JSONL immediately, so results survive even if the
  session is interrupted; post-flight analysis uses the same files.

Time axis (both-fly capable): enemy fixes use PX4 boot->UTC from
SYSTEM_TIME; own RTK carries GPS UTC (SensorGps.time_utc_usec);
measurement wall stamps are mapped to UTC via the robust offset fitted
from own RTK (wall, utc) pairs. The two input files may therefore be
recorded on different machines with arbitrary clock skew. --clock-offset
(seconds, wall-minus-UTC) forces the measurement mapping for bench tests
without GPS UTC.

Own side is a time-varying ENU track interpolated at each measurement
time (a stationary interceptor degenerates to the static-anchor case).
--anchor forces a fixed own point for bench/plumbing tests.

Note: in the flight architecture the enemy file is logged on laptop A,
so this monitor is primarily a ground-bench tool (both files on the
Jetson); it also works on any machine after the files are copied over.

Run standalone (files may still be growing):
    python3 live_range_monitor.py \
        --enemy <run>/enemy_rtk.jsonl --measurements <run>/measurements.jsonl \
        --lever-enu 0.15,0,-0.05 --out-csv <run>/live_comparison.csv

--once processes the current file contents and exits (used by tests and
for a quick mid-test check). Reuses rtk_truth_core math; numbers match
compare_rtk_stereo_truth.py (batch) within interpolation/span effects.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime, timezone
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
    range_sigma_from_covariance,
)


class JsonlTailer:
    """Incremental reader that tolerates partially written last lines."""

    def __init__(self, path: str) -> None:
        self.path = path
        self.offset = 0
        self.remainder = ""

    def read_new(self) -> list[dict]:
        if not os.path.exists(self.path):
            return []
        with open(self.path, "r", encoding="utf-8") as source:
            source.seek(self.offset)
            chunk = source.read()
            self.offset = source.tell()
        if not chunk:
            return []
        data = self.remainder + chunk
        lines = data.split("\n")
        self.remainder = lines.pop()  # last element is incomplete (or "")
        records = []
        for line in lines:
            line = line.strip()
            if line:
                try:
                    records.append(json.loads(line))
                except json.JSONDecodeError:
                    continue
        return records


class LiveMonitor:
    def __init__(self, args) -> None:
        self.args = args
        if getattr(args, "lever_body", None):
            self.lever = body_lever_to_enu(
                tuple(float(v) for v in args.lever_body.split(",")),
                float(getattr(args, "rig_yaw_deg", 0.0)),
            )
        else:
            self.lever = tuple(float(v) for v in args.lever_enu.split(","))
        self.layers = tuple(float(v) for v in args.layers.split(","))
        self.static_anchor = (
            tuple(float(v) for v in args.anchor.split(",")) if args.anchor else None
        )
        self.sys_pairs_diag: list[tuple[float, float]] = []
        self.sys_boot_utc: list[tuple[float, float]] = []  # (boot_s, utc)
        self.enemy_fixes: list[dict] = []  # dicts with utc, lat, lon, alt
        self.own_fixes: list[dict] = []  # dicts with wall, utc, lat, lon, alt
        self.offset_j: float | None = args.clock_offset  # jetson wall - utc
        self.offset_j_residual_ms: float | None = None
        self.logger_residual_ms: float | None = None
        self.origin: tuple[float, float, float] | None = None
        # incremental caches (ENU recomputed only for newly arrived fixes)
        self._enemy_times: list[float] = []  # utc
        self._enemy_enu: list[tuple[float, ...]] = []
        self._own_times: list[float] = []  # utc
        self._own_enu: list[tuple[float, ...]] = []
        self.samples: list[dict] = []
        self.window: list[dict] = []
        self.pending: list[dict] = []  # measurements waiting for future fixes
        self.total_meas = 0
        self.skipped_clock = 0
        self.skipped_gap = 0
        self.last_print = 0.0
        self.last_input_monotonic = time.monotonic()
        self.csv_file = open(args.out_csv, "a", newline="", encoding="utf-8") if args.out_csv else None
        self.csv_writer = None
        self.jsonl_file = open(args.out_jsonl, "a", encoding="utf-8") if args.out_jsonl else None

    # ---- data intake -------------------------------------------------
    def feed_enemy(self, record: dict) -> None:
        self.last_input_monotonic = time.monotonic()
        kind = record.get("kind")
        if kind == "system_time" and record.get("utc"):
            self.sys_pairs_diag.append((record["wall"], record["utc"]))
            self.sys_boot_utc.append((record["boot_ms"] / 1e3, record["utc"]))
            self.sys_boot_utc.sort()
            if len(self.sys_pairs_diag) >= 5:
                clock = fit_clock_offset(self.sys_pairs_diag)
                self.logger_residual_ms = clock["residual_std_ms"]
        elif kind == "gps_raw_int" and record.get("fix", 0) >= self.args.min_fix:
            if self.sys_boot_utc:
                boot_s = record["boot_us"] / 1e6
                idx = min(range(len(self.sys_boot_utc)), key=lambda i: abs(self.sys_boot_utc[i][0] - boot_s))
                if abs(self.sys_boot_utc[idx][0] - boot_s) <= 2.0:
                    utc = self.sys_boot_utc[idx][1] + (boot_s - self.sys_boot_utc[idx][0])
                    self.enemy_fixes.append(
                        {"utc": utc, "lat": record["lat"], "lon": record["lon"], "alt": record["alt"]}
                    )

    def feed_measurements(self, record: dict) -> None:
        self.last_input_monotonic = time.monotonic()
        kind = record.get("kind")
        if kind == "own_rtk" and record.get("fix", 0) >= self.args.min_fix:
            self.own_fixes.append(
                {
                    "wall": record["wall"],
                    "utc": record.get("utc"),
                    "lat": record["lat"],
                    "lon": record["lon"],
                    "alt": record["alt"],
                }
            )
            if self.origin is None:
                if self.static_anchor is not None:
                    self.origin = self.static_anchor
                elif len(self.own_fixes) >= self.args.anchor_min_fixes:
                    med = median_position(self.own_fixes)
                    self.origin = (med["lat"], med["lon"], med["alt"])
            if self.offset_j is None:
                pairs = [(f["wall"], f["utc"]) for f in self.own_fixes if f.get("utc")]
                if len(pairs) >= 5:
                    clock = fit_clock_offset(pairs)
                    self.offset_j = clock["offset_s"]
                    self.offset_j_residual_ms = clock["residual_std_ms"]
            return
        if kind != "async_meas" or not record.get("valid"):
            return
        self.total_meas += 1
        self.pending.append(record)

    # ---- incremental track caches --------------------------------------
    def _refresh_caches(self) -> None:
        if self.origin is None:
            return
        if len(self._enemy_enu) < len(self.enemy_fixes):
            for fix in self.enemy_fixes[len(self._enemy_enu):]:
                self._enemy_times.append(fix["utc"])
                self._enemy_enu.append(
                    geodetic_to_enu(fix["lat"], fix["lon"], fix["alt"], *self.origin)
                )
        own_with_utc_count = sum(1 for f in self.own_fixes if f.get("utc"))
        if len(self._own_enu) < own_with_utc_count:
            converted = 0
            for fix in self.own_fixes:
                if not fix.get("utc"):
                    continue
                if converted < len(self._own_enu):
                    converted += 1
                    continue
                self._own_times.append(fix["utc"])
                self._own_enu.append(
                    geodetic_to_enu(fix["lat"], fix["lon"], fix["alt"], *self.origin)
                )
                converted += 1

    def latest_track_utc(self) -> tuple[float | None, float | None]:
        self._refresh_caches()
        enemy_latest = self._enemy_times[-1] if self._enemy_times else None
        own_latest = self._own_times[-1] if self._own_times else None
        if self.static_anchor is not None:
            own_latest = float("inf")
        return enemy_latest, own_latest

    def drain_pending(self) -> list[dict]:
        """Compare buffered measurements that are now inside both track spans."""
        ready = []
        enemy_latest, own_latest = self.latest_track_utc()
        still_pending = []
        for record in self.pending:
            if self.offset_j is None:
                still_pending.append(record)  # wall->UTC mapping not fitted yet
                continue
            t_query = record["wall"] - self.offset_j
            if enemy_latest is not None and t_query > enemy_latest:
                still_pending.append(record)  # wait for a future enemy fix
                continue
            if own_latest is not None and t_query > own_latest:
                still_pending.append(record)  # wait for a future own fix
                continue
            sample = self.compare_one(record, t_query)
            if sample is None:
                if enemy_latest is None or own_latest is None:
                    still_pending.append(record)  # tracks not ready yet
                continue
            ready.append(sample)
        self.pending = still_pending
        for sample in ready:
            self.samples.append(sample)
            self.window.append(sample)
            cutoff = sample["t_utc"] - self.args.window_s
            self.window = [s for s in self.window if s["t_utc"] >= cutoff]
        return ready

    # ---- comparison ---------------------------------------------------
    def compare_one(self, meas: dict, t_query: float) -> dict | None:
        own_ready = self.static_anchor is not None or len(self._own_times) >= 2
        if self.offset_j is None or self.origin is None or len(self._enemy_times) < 2 or not own_ready:
            self.skipped_clock += 1
            return None
        self._refresh_caches()
        truth = interpolate_series(self._enemy_times, self._enemy_enu, t_query, self.args.max_gap_s)
        if truth is None:
            self.skipped_gap += 1
            return None
        if self.static_anchor is not None:
            own_pos = (0.0, 0.0, 0.0)
        else:
            own_pos = interpolate_series(self._own_times, self._own_enu, t_query, self.args.max_gap_s)
            if own_pos is None:
                self.skipped_gap += 1
                return None
        dx = truth[0] - (own_pos[0] + self.lever[0])
        dy = truth[1] - (own_pos[1] + self.lever[1])
        dz = truth[2] - (own_pos[2] + self.lever[2])
        range_truth = math.sqrt(dx * dx + dy * dy + dz * dz)
        position = meas["position"]
        range_meas = math.sqrt(sum(c * c for c in position))
        sigma = range_sigma_from_covariance(position, meas.get("covariance", []))
        idx = layer_index(range_truth, self.layers)
        return {
            "wall": meas["wall"],
            "t_utc": t_query,
            "layer": layer_label(idx, self.layers) if idx is not None else "n/a",
            "range_truth": range_truth,
            "range_meas": range_meas,
            "error": range_meas - range_truth,
            "sigma": sigma,
            "timing_delta_ms": (meas.get("timing_delta") or 0.0) * 1000.0,
            "reason": meas.get("reason", ""),
        }

    # ---- output --------------------------------------------------------
    def export_sample(self, sample: dict) -> None:
        row = {
            "iso_time": datetime.fromtimestamp(sample["t_utc"], tz=timezone.utc).isoformat(timespec="milliseconds"),
            "t_utc": f"{sample['t_utc']:.3f}",
            "layer": sample["layer"],
            "range_truth_m": f"{sample['range_truth']:.3f}",
            "range_meas_m": f"{sample['range_meas']:.3f}",
            "error_m": f"{sample['error']:+.3f}",
            "sigma_m": f"{sample['sigma']:.3f}" if sample["sigma"] else "",
            "within_2sigma": int(abs(sample["error"]) <= 2.0 * sample["sigma"]) if sample["sigma"] else "",
            "timing_delta_ms": f"{sample['timing_delta_ms']:.1f}",
            "reason": sample["reason"],
        }
        if self.csv_file is not None:
            if self.csv_writer is None:
                self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=list(row.keys()))
                self.csv_writer.writeheader()
            self.csv_writer.writerow(row)
            self.csv_file.flush()
        if self.jsonl_file is not None:
            self.jsonl_file.write(json.dumps(sample) + "\n")
            self.jsonl_file.flush()

    def status_line(self) -> str:
        clock = "offset:waiting" if self.offset_j is None else f"offset_j={self.offset_j:+.2f}s"
        own = "own:waiting" if self.origin is None else f"own_fixes={len(self.own_fixes)}"
        rtkf = f"enemy_fixes={len(self.enemy_fixes)}"
        if self.window:
            last = self.window[-1]
            stats = error_stats([s["error"] for s in self.window])
            meas_part = (
                f"truth={last['range_truth']:.2f}m meas={last['range_meas']:.2f}m "
                f"err={last['error']:+.3f}m layer={last['layer']} "
                f"win{self.args.window_s:.0f}s: n={stats['count']} |err|mean={stats.get('abs_mean_m', 0):.3f} "
                f"p95={stats.get('p95_m', 0):.3f}"
            )
        else:
            meas_part = f"no samples yet (valid_meas={self.total_meas} skip_clock={self.skipped_clock} skip_gap={self.skipped_gap})"
        stale_s = time.monotonic() - self.last_input_monotonic
        stale_tag = (
            f"  !!STALE {stale_s:.0f}s: input files not growing - check recorders / run dir!!"
            if stale_s > self.args.stale_s
            else ""
        )
        return f"[{datetime.now().strftime('%H:%M:%S')}] {clock} {own} {rtkf} compared={len(self.samples)} | {meas_part}{stale_tag}"

    def summary(self) -> str:
        lines = [self.status_line(), "---- session summary ----"]
        stats = error_stats([s["error"] for s in self.samples])
        if stats.get("count"):
            lines.append(
                "overall: n={count} mean={mean_m:+.3f}m |err|mean={abs_mean_m:.3f} "
                "p50={p50_m:.3f} p95={p95_m:.3f} max={max_m:.3f}".format(**stats)
            )
            with_sigma = [s for s in self.samples if s["sigma"]]
            if with_sigma:
                covered = [s for s in with_sigma if abs(s["error"]) <= 2.0 * s["sigma"]]
                lines.append(
                    f"sigma coverage: {len(covered)}/{len(with_sigma)} within 2σ "
                    f"({len(covered) / len(with_sigma) * 100:.1f}%)"
                )
            per_layer: dict[str, list[float]] = {}
            for s in self.samples:
                per_layer.setdefault(s["layer"], []).append(s["error"])
            for label, values in per_layer.items():
                st = error_stats(values)
                lines.append(
                    f"layer {label}: n={st['count']} mean={st['mean_m']:+.3f} "
                    f"p95={st['p95_m']:.3f} max={st['max_m']:.3f}"
                )
        else:
            lines.append("no compared samples")
        return "\n".join(lines)

    def close(self) -> None:
        if self.csv_file is not None:
            self.csv_file.close()
        if self.jsonl_file is not None:
            self.jsonl_file.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--enemy", required=True)
    parser.add_argument("--measurements", required=True)
    parser.add_argument("--lever-enu", default="0,0,0")
    parser.add_argument("--lever-body", default=None,
                        help="antenna->camera lever in body frame (FLU), converted with --rig-yaw-deg")
    parser.add_argument("--rig-yaw-deg", type=float, default=0.0,
                        help="compass heading of the body X axis at setup, degrees from north")
    parser.add_argument("--anchor", help="force static own position lat,lon,alt (bench/plumbing tests)")
    parser.add_argument("--clock-offset", type=float, default=None,
                        help="force measurement wall->UTC offset in seconds (wall-minus-UTC)")
    parser.add_argument("--out-csv", default=None, help="append compared samples here (Excel-friendly)")
    parser.add_argument("--out-jsonl", default=None)
    parser.add_argument("--min-fix", type=int, default=6)
    parser.add_argument("--max-gap-s", type=float, default=0.3)
    parser.add_argument("--layers", default="5,10,15,20,30")
    parser.add_argument("--window-s", type=float, default=10.0, help="rolling stats window")
    parser.add_argument("--interval-s", type=float, default=2.0, help="console status interval")
    parser.add_argument("--stale-s", type=float, default=10.0, help="warn when no new input records for this many seconds")
    parser.add_argument("--anchor-min-fixes", type=int, default=10, help="own fixes needed to fix the session ENU origin")
    parser.add_argument("--once", action="store_true", help="process current content and exit with summary")
    args = parser.parse_args()

    for path in (args.out_csv, args.out_jsonl):
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)

    enemy_tail = JsonlTailer(args.enemy)
    meas_tail = JsonlTailer(args.measurements)
    monitor = LiveMonitor(args)
    try:
        while True:
            for record in enemy_tail.read_new():
                monitor.feed_enemy(record)
            for record in meas_tail.read_new():
                monitor.feed_measurements(record)
            for sample in monitor.drain_pending():
                monitor.export_sample(sample)
            now = time.monotonic()
            if now - monitor.last_print >= args.interval_s:
                print(monitor.status_line(), flush=True)
                monitor.last_print = now
            if args.once:
                break
            time.sleep(0.25)
    except KeyboardInterrupt:
        pass
    print(monitor.summary(), flush=True)
    monitor.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
