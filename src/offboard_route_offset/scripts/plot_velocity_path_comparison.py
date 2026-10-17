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

"""Generate the velocity-path before/after Gazebo Sim acceptance plots."""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def finite_rows(frame, columns):
    """Return rows for which every requested numeric column is finite."""
    mask = np.ones(len(frame), dtype=bool)
    for column in columns:
        mask &= np.isfinite(pd.to_numeric(frame[column], errors='coerce'))
    return frame.loc[mask].copy()


def active_time(frame, backend=False):
    """Select active trajectory samples and add time relative to first sample."""
    if backend and 'control_backend' in frame:
        active = frame[pd.to_numeric(frame.control_backend, errors='coerce').eq(1)]
        if 'debug_timestamp' in active:
            active = active[np.isfinite(pd.to_numeric(
                active.debug_timestamp, errors='coerce'))]
            active = active.drop_duplicates('debug_timestamp')
    else:
        speed = pd.to_numeric(frame.sp_speed, errors='coerce')
        active = frame[np.isfinite(speed) & speed.gt(0.05)]
    active = active.copy()
    if not active.empty:
        active['active_t'] = active.t - active.t.iloc[0]
    return active


def thin(frame, maximum=4000):
    """Limit plot density without changing the source CSV."""
    step = max(1, len(frame) // maximum)
    return frame.iloc[::step]


def line(ax, frame, x, y, label, **kwargs):
    """Plot only finite samples."""
    data = thin(finite_rows(frame, [x, y]))
    if not data.empty:
        ax.plot(data[x], data[y], label=label, **kwargs)


def configure(ax, title, xlabel='time from trajectory start [s]', ylabel=None):
    """Apply common, compact plot formatting."""
    ax.set_title(title, fontsize=9)
    ax.set_xlabel(xlabel, fontsize=8)
    if ylabel:
        ax.set_ylabel(ylabel, fontsize=8)
    ax.grid(True, alpha=0.25)
    ax.tick_params(labelsize=7)


def unwrap_angles(frame, columns):
    """Unwrap finite yaw columns without changing the recorded CSV."""
    result = frame.copy()
    for column in columns:
        numeric = pd.to_numeric(result[column], errors='coerce')
        valid = np.isfinite(numeric)
        result[column] = np.nan
        result.loc[valid, column] = np.unwrap(numeric.loc[valid].to_numpy())
    return result


def main():
    """Load the acceptance CSV files and write the comparison figure."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--before', required=True)
    parser.add_argument('--after', required=True)
    parser.add_argument('--curve', required=True)
    parser.add_argument('--fixed-yaw', required=True)
    parser.add_argument('--tangent-yaw', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    before = active_time(pd.read_csv(args.before), backend=False)
    after = active_time(pd.read_csv(args.after), backend=True)
    curve = active_time(pd.read_csv(args.curve), backend=True)
    fixed = active_time(pd.read_csv(args.fixed_yaw), backend=True)
    tangent = active_time(pd.read_csv(args.tangent_yaw), backend=True)

    figure, axes = plt.subplots(4, 3, figsize=(16, 14), constrained_layout=True)
    axes = axes.ravel()

    line(axes[0], before, 'active_t', 'sp_speed', 'position+FF target', alpha=0.8)
    line(axes[0], before, 'active_t', 'horiz_speed', 'position+FF actual', alpha=0.8)
    line(axes[0], after, 'active_t', 'total_speed_command', 'velocity target')
    line(axes[0], after, 'active_t', 'actual_horizontal_speed_debug', 'velocity actual')
    configure(axes[0], '1. horizontal target and actual speed', ylabel='speed [m/s]')
    axes[0].legend(fontsize=7)

    before_error = before.copy()
    before_error['speed_error'] = before_error.horiz_speed - before_error.sp_speed
    after_error = after.copy()
    after_error['speed_error'] = (
        after_error.actual_horizontal_speed_debug - after_error.total_speed_command)
    line(axes[1], before_error, 'active_t', 'speed_error', 'position+FF')
    line(axes[1], after_error, 'active_t', 'speed_error', 'velocity path')
    configure(axes[1], '2. horizontal speed error', ylabel='actual - command [m/s]')
    axes[1].legend(fontsize=7)

    line(axes[2], after, 'active_t', 'tangent_speed_command', 'tangent command')
    line(axes[2], after, 'active_t', 'normal_speed_command', 'normal command')
    line(axes[2], after, 'active_t', 'actual_tangent_speed', 'actual tangent', alpha=0.75)
    line(axes[2], after, 'active_t', 'actual_normal_speed', 'actual normal', alpha=0.75)
    configure(axes[2], '3. Frenet velocity components', ylabel='speed [m/s]')
    axes[2].legend(fontsize=7)

    line(axes[3], before, 'active_t', 'planned_acceleration', 'position planner')
    line(axes[3], after, 'active_t', 'velocity_planner_acceleration', 'velocity planner')
    configure(axes[3], '4. planned tangential acceleration', ylabel='accel [m/s²]')
    axes[3].legend(fontsize=7)

    line(axes[4], curve, 'x', 'y', 'actual XY')
    line(axes[4], curve, 'projected_path_x', 'projected_path_y', 'geometric path')
    configure(axes[4], '5. actual XY and projected curve',
              xlabel='north [m]', ylabel='east [m]')
    axes[4].axis('equal')
    axes[4].legend(fontsize=7)

    line(axes[5], curve, 'active_t', 'cross_track_error', 'sine path')
    line(axes[5], after, 'active_t', 'cross_track_error', 'straight path')
    configure(axes[5], '6. cross-track error', ylabel='error [m]')
    axes[5].legend(fontsize=7)

    line(axes[6], curve, 'active_t', 's_vehicle_raw', 'raw projection')
    line(axes[6], curve, 'active_t', 's_vehicle_filtered', 'filtered projection')
    configure(axes[6], '7. raw and filtered vehicle progress', ylabel='arc length [m]')
    axes[6].legend(fontsize=7)

    line(axes[7], before, 'active_t', 'progress_lead', 'lead')
    axes[7].axhline(0.5, color='tab:red', linestyle='--', label='configured hard limit')
    configure(axes[7], '8. legacy reference lead gate', ylabel='lead [m]')
    axes[7].legend(fontsize=7)

    terminal = curve[pd.to_numeric(curve.controller_state, errors='coerce').eq(5)].copy()
    if not terminal.empty:
        terminal['terminal_t'] = terminal.t - terminal.t.iloc[0]
    line(axes[8], terminal, 'terminal_t', 'total_speed_command', 'command')
    line(axes[8], terminal, 'terminal_t', 'actual_horizontal_speed_debug', 'actual')
    line(axes[8], terminal, 'terminal_t', 'terminal_speed_limit', 'terminal limit')
    configure(axes[8], '9. terminal capture detail', xlabel='time from capture [s]',
              ylabel='speed [m/s]')
    axes[8].legend(fontsize=7)

    line(axes[9], curve, 'active_t', 'curvature', 'signed curvature')
    limit = curve.copy()
    limit['curve_limit_scaled'] = limit.curve_speed_limit
    line(axes[9], limit, 'active_t', 'curve_limit_scaled', 'curve speed limit')
    configure(axes[9], '10. curvature and curvature speed limit', ylabel='1/m or m/s')
    axes[9].legend(fontsize=7)

    tangent_unwrapped = unwrap_angles(
        tangent, ['yaw_setpoint_debug', 'actual_heading_debug'])
    line(axes[10], tangent_unwrapped, 'active_t', 'yaw_setpoint_debug', 'yaw setpoint')
    line(axes[10], tangent_unwrapped, 'active_t', 'actual_heading_debug', 'actual heading')
    configure(axes[10], '11. reciprocal tangent yaw and heading', ylabel='yaw [rad]')
    axes[10].legend(fontsize=7)

    line(axes[11], fixed, 'active_t', 'actual_horizontal_speed_debug', 'fixed yaw')
    line(axes[11], tangent, 'active_t', 'actual_horizontal_speed_debug', 'tangent yaw')
    configure(axes[11], '12. fixed vs tangent yaw speed response', ylabel='speed [m/s]')
    axes[11].legend(fontsize=7)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    figure.suptitle('offboard_route_offset Gazebo Sim comparison', fontsize=14)
    figure.savefig(output, dpi=160)
    print(output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
