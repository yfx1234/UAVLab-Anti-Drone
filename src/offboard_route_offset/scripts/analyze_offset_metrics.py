#!/usr/bin/env python3
# Copyright 2026 danyu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Summarize legacy position and velocity-path trajectory CSV recordings."""

import csv
import math
import statistics
import sys


def finite(value):
    """Return whether a numeric value is finite."""
    return math.isfinite(value)


def read_rows(path):
    """Read a metrics CSV and convert numeric columns to floats."""
    rows = []
    with open(path) as f:
        for raw in csv.DictReader(f):
            row = {'event': raw.get('event', '')}
            for key, value in raw.items():
                if key == 'event':
                    continue
                try:
                    row[key] = float(value)
                except (TypeError, ValueError):
                    row[key] = float('nan')
            rows.append(row)
    return rows


def summarize(rows, cruise_speed):
    """Calculate comparable command, actual-speed, lead, and path metrics."""
    velocity_mode = any(
        finite(r.get('control_backend', float('nan')))
        and r.get('control_backend') == 1.0
        for r in rows
    )
    command_key = 'total_speed_command' if velocity_mode else 'sp_speed'
    actual_key = 'actual_horizontal_speed_debug' if velocity_mode else 'horiz_speed'
    active = [
        r
        for r in rows
        if finite(r.get(command_key, float('nan')))
        and finite(r.get(actual_key, float('nan')))
        and r[command_key] > 0.05
    ]
    if not active:
        return {'samples': 0}

    t0 = active[0]['t']
    start = [r for r in active if t0 <= r['t'] <= t0 + 8.0]

    def metrics(data):
        max_speed = max(
            (r[actual_key] for r in data if finite(r[actual_key])),
            default=float('nan'),
        )
        max_sp = max(
            (r[command_key] for r in data if finite(r[command_key])),
            default=float('nan'),
        )
        max_lead = max(
            (r['lead_distance'] for r in data if finite(r['lead_distance'])),
            default=float('nan'),
        )
        overshoot = max_speed - cruise_speed
        overshoot_pct = (
            100.0 * overshoot / cruise_speed if cruise_speed > 0.0 else float('nan')
        )
        commands = [r[command_key] for r in data if finite(r[command_key])]
        actual = [r[actual_key] for r in data if finite(r[actual_key])]
        errors = [
            r[actual_key] - r[command_key]
            for r in data
            if finite(r[actual_key]) and finite(r[command_key])
        ]
        cross_track = [
            abs(r['cross_track_error'])
            for r in data
            if finite(r.get('cross_track_error', float('nan')))
        ]

        def mean(values):
            return statistics.fmean(values) if values else float('nan')

        def stddev(values):
            return statistics.pstdev(values) if len(values) > 1 else float('nan')

        return {
            'samples': len(data),
            'max_horiz_speed': max_speed,
            'max_sp_speed': max_sp,
            'overshoot': overshoot,
            'overshoot_pct': overshoot_pct,
            'max_lead': max_lead,
            'command_mean': mean(commands),
            'command_std': stddev(commands),
            'actual_mean': mean(actual),
            'actual_std': stddev(actual),
            'error_std': stddev(errors),
            'cross_track_abs_mean': mean(cross_track),
            'cross_track_abs_max': max(cross_track, default=float('nan')),
        }

    top = sorted(active, key=lambda r: r[actual_key], reverse=True)[:5]
    return {
        'active': metrics(active),
        'start8s': metrics(start),
        'top': top,
        'command_key': command_key,
        'actual_key': actual_key,
        'velocity_mode': velocity_mode,
    }


def print_metrics(label, summary):
    """Print one human-readable metrics summary."""
    print(f'\n{label}')
    if summary.get('samples') == 0:
        print('no active trajectory samples')
        return
    for name in ('active', 'start8s'):
        m = summary[name]
        print(
            f'{name}: samples={m["samples"]} '
            f'max_horiz_speed={m["max_horiz_speed"]:.3f} '
            f'max_sp_speed={m["max_sp_speed"]:.3f} '
            f'overshoot={m["overshoot"]:.3f} ({m["overshoot_pct"]:.1f}%) '
            f'max_lead={m["max_lead"]:.3f}'
        )
        print(
            f'  command={m["command_mean"]:.3f}±{m["command_std"]:.3f} '
            f'actual={m["actual_mean"]:.3f}±{m["actual_std"]:.3f} '
            f'error_std={m["error_std"]:.3f} '
            f'|cross_track|={m["cross_track_abs_mean"]:.3f} mean, '
            f'{m["cross_track_abs_max"]:.3f} max'
        )
    print('top speed samples:')
    command_key = summary['command_key']
    actual_key = summary['actual_key']
    for r in summary['top']:
        print(
            f'  t={r["t"]:.2f} actual={r[actual_key]:.3f} '
            f'command={r[command_key]:.3f} '
            f'lead={r.get("lead_distance", float("nan")):.3f} '
            f'xy=({r["x"]:.2f},{r["y"]:.2f})'
        )


def main():
    """Parse command-line arguments and summarize each requested CSV."""
    if len(sys.argv) < 3:
        print(
            'usage: analyze_offset_metrics.py CRUISE_SPEED CSV [CSV...]',
            file=sys.stderr,
        )
        return 2
    cruise_speed = float(sys.argv[1])
    for path in sys.argv[2:]:
        print_metrics(path, summarize(read_rows(path), cruise_speed))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
