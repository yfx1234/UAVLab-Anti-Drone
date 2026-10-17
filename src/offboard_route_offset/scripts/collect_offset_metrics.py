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

"""Record PX4 state, setpoints, and route debug data to a CSV file."""

import csv
import math
import signal
import sys
import time

from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
import rclpy
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray


def finite(value):
    """Return whether a value can be converted to a finite float."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def norm(values):
    """Calculate a Euclidean norm, returning NaN for invalid input."""
    values = [float(v) for v in values]
    if not all(finite(v) for v in values):
        return float('nan')
    return math.sqrt(sum(v * v for v in values))


def main():
    """Collect topic samples until the duration expires or a signal arrives."""
    if len(sys.argv) < 2:
        print(
            'usage: collect_offset_metrics.py OUT.csv [duration_s] '
            '[--wait-active [max_wait_s]] [--active-index N]',
            file=sys.stderr,
        )
        return 2

    out_path = sys.argv[1]
    duration_s = float(sys.argv[2]) if len(sys.argv) > 2 else 60.0
    wait_active = '--wait-active' in sys.argv[3:]
    active_index = 1
    if '--active-index' in sys.argv[3:]:
        idx = sys.argv.index('--active-index')
        if idx + 1 >= len(sys.argv):
            print('--active-index requires a positive integer', file=sys.stderr)
            return 2
        active_index = int(sys.argv[idx + 1])
        if active_index < 1:
            print('--active-index must be >= 1', file=sys.stderr)
            return 2
    max_wait_s = 240.0
    if wait_active:
        idx = sys.argv.index('--wait-active')
        if idx + 1 < len(sys.argv):
            try:
                max_wait_s = float(sys.argv[idx + 1])
            except ValueError:
                max_wait_s = 240.0

    rclpy.init()
    node = rclpy.create_node('collect_offset_metrics')
    qos = QoSProfile(
        depth=50,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )

    start = time.monotonic()
    latest_pos = None
    latest_sp = None
    latest_status = None
    latest_debug = None
    latest_debug_time = None
    rows = []
    stop_requested = False

    def request_stop(_signum, _frame):
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    def record(event):
        lp = latest_pos
        sp = latest_sp
        st = latest_status
        debug_fresh = (
            latest_debug is not None and latest_debug_time is not None and
            time.monotonic() - latest_debug_time <= 0.2)
        debug = latest_debug.data if debug_fresh and len(latest_debug.data) >= 11 else []
        velocity_debug = debug if len(debug) >= 36 else []
        row = {
            't': time.monotonic() - start,
            'event': event,
            'nav_state': getattr(st, 'nav_state', ''),
            'arming_state': getattr(st, 'arming_state', ''),
            'x': getattr(lp, 'x', float('nan')) if lp else float('nan'),
            'y': getattr(lp, 'y', float('nan')) if lp else float('nan'),
            'z': getattr(lp, 'z', float('nan')) if lp else float('nan'),
            'vx': getattr(lp, 'vx', float('nan')) if lp else float('nan'),
            'vy': getattr(lp, 'vy', float('nan')) if lp else float('nan'),
            'vz': getattr(lp, 'vz', float('nan')) if lp else float('nan'),
            'ax': getattr(lp, 'ax', float('nan')) if lp else float('nan'),
            'ay': getattr(lp, 'ay', float('nan')) if lp else float('nan'),
            'az': getattr(lp, 'az', float('nan')) if lp else float('nan'),
            'heading': getattr(lp, 'heading', float('nan')) if lp else float('nan'),
            'sp_x': float(sp.position[0])
            if sp and finite(sp.position[0])
            else float('nan'),
            'sp_y': float(sp.position[1])
            if sp and finite(sp.position[1])
            else float('nan'),
            'sp_z': float(sp.position[2])
            if sp and finite(sp.position[2])
            else float('nan'),
            'sp_vx': float(sp.velocity[0])
            if sp and finite(sp.velocity[0])
            else float('nan'),
            'sp_vy': float(sp.velocity[1])
            if sp and finite(sp.velocity[1])
            else float('nan'),
            'sp_vz': float(sp.velocity[2])
            if sp and finite(sp.velocity[2])
            else float('nan'),
            'sp_ax': float(sp.acceleration[0])
            if sp and finite(sp.acceleration[0])
            else float('nan'),
            'sp_ay': float(sp.acceleration[1])
            if sp and finite(sp.acceleration[1])
            else float('nan'),
            'sp_az': float(sp.acceleration[2])
            if sp and finite(sp.acceleration[2])
            else float('nan'),
            'sp_yaw': float(sp.yaw) if sp and finite(sp.yaw) else float('nan'),
            'planner_phase': debug[0] if debug else float('nan'),
            's_ref': debug[1] if debug else float('nan'),
            's_vehicle': debug[2] if debug else float('nan'),
            'progress_lead': debug[3] if debug else float('nan'),
            'actual_path_speed': debug[4] if debug else float('nan'),
            'planned_speed': debug[5] if debug else float('nan'),
            'remaining_distance': debug[6] if debug else float('nan'),
            'braking_distance': debug[7] if debug else float('nan'),
            'planned_acceleration': debug[8] if debug else float('nan'),
            'planner_target_speed': debug[9] if debug else float('nan'),
            'dynamic_lookahead': debug[10] if debug else float('nan'),
            'motion_state': debug[11] if len(debug) >= 12 else float('nan'),
            'debug_timestamp': velocity_debug[0] if velocity_debug else float('nan'),
            'trajectory_code': velocity_debug[1] if velocity_debug else float('nan'),
            'controller_state': velocity_debug[2] if velocity_debug else float('nan'),
            'control_backend': velocity_debug[3] if velocity_debug else 0.0,
            'yaw_mode': velocity_debug[4] if velocity_debug else float('nan'),
            's_vehicle_raw': velocity_debug[5] if velocity_debug else float('nan'),
            's_vehicle_filtered': velocity_debug[6] if velocity_debug else float('nan'),
            's_plan': velocity_debug[7] if velocity_debug else float('nan'),
            'path_remaining_distance': velocity_debug[8] if velocity_debug else float('nan'),
            'projected_path_x': velocity_debug[9] if velocity_debug else float('nan'),
            'projected_path_y': velocity_debug[10] if velocity_debug else float('nan'),
            'cross_track_error': velocity_debug[11] if velocity_debug else float('nan'),
            'along_track_error': velocity_debug[12] if velocity_debug else float('nan'),
            'velocity_planner_target_speed': (
                velocity_debug[13] if velocity_debug else float('nan')),
            'velocity_planner_speed': velocity_debug[14] if velocity_debug else float('nan'),
            'velocity_planner_acceleration': (
                velocity_debug[15] if velocity_debug else float('nan')),
            'tangent_speed_command': velocity_debug[16] if velocity_debug else float('nan'),
            'normal_speed_command': velocity_debug[17] if velocity_debug else float('nan'),
            'total_speed_command': velocity_debug[18] if velocity_debug else float('nan'),
            'vertical_speed_command': velocity_debug[19] if velocity_debug else float('nan'),
            'actual_horizontal_speed_debug': (
                velocity_debug[20] if velocity_debug else float('nan')),
            'actual_tangent_speed': velocity_debug[21] if velocity_debug else float('nan'),
            'actual_normal_speed': velocity_debug[22] if velocity_debug else float('nan'),
            'actual_vertical_speed': velocity_debug[23] if velocity_debug else float('nan'),
            'curvature': velocity_debug[24] if velocity_debug else float('nan'),
            'curve_speed_limit': velocity_debug[25] if velocity_debug else float('nan'),
            'terminal_speed_limit': velocity_debug[26] if velocity_debug else float('nan'),
            'cross_track_speed_scale': velocity_debug[27] if velocity_debug else float('nan'),
            'yaw_setpoint_debug': velocity_debug[28] if velocity_debug else float('nan'),
            'yaw_rate_setpoint': velocity_debug[29] if velocity_debug else float('nan'),
            'actual_heading_debug': velocity_debug[30] if velocity_debug else float('nan'),
            'endpoint_position_ok': velocity_debug[31] if velocity_debug else float('nan'),
            'endpoint_speed_ok': velocity_debug[32] if velocity_debug else float('nan'),
            'endpoint_altitude_ok': velocity_debug[33] if velocity_debug else float('nan'),
            'endpoint_vertical_speed_ok': velocity_debug[34] if velocity_debug else float('nan'),
            'endpoint_settle_elapsed': velocity_debug[35] if velocity_debug else float('nan'),
        }
        row['speed'] = norm([row['vx'], row['vy'], row['vz']])
        row['horiz_speed'] = norm([row['vx'], row['vy']])
        row['sp_speed'] = norm([row['sp_vx'], row['sp_vy'], row['sp_vz']])
        row['acceleration'] = norm([row['ax'], row['ay'], row['az']])
        row['sp_acceleration'] = norm([row['sp_ax'], row['sp_ay'], row['sp_az']])
        if all(finite(row[k]) for k in ('x', 'y', 'z', 'sp_x', 'sp_y', 'sp_z')):
            row['lead_distance'] = norm(
                [row['sp_x'] - row['x'], row['sp_y'] - row['y'], row['sp_z'] - row['z']]
            )
            row['lead_xy'] = norm([row['sp_x'] - row['x'], row['sp_y'] - row['y']])
        else:
            row['lead_distance'] = float('nan')
            row['lead_xy'] = float('nan')
        rows.append(row)

    def pos_cb(msg):
        nonlocal latest_pos
        latest_pos = msg
        record('pos')

    def sp_cb(msg):
        nonlocal latest_sp
        latest_sp = msg
        record('sp')

    def status_cb(msg):
        nonlocal latest_status
        latest_status = msg

    def debug_cb(msg):
        nonlocal latest_debug, latest_debug_time
        latest_debug = msg
        latest_debug_time = time.monotonic()

    node.create_subscription(
        VehicleLocalPosition, '/fmu/out/vehicle_local_position', pos_cb, qos
    )
    node.create_subscription(
        TrajectorySetpoint, '/fmu/in/trajectory_setpoint', sp_cb, qos
    )
    node.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', status_cb, qos)
    node.create_subscription(
        Float64MultiArray,
        '/offboard_route_offset/trajectory_debug',
        debug_cb,
        20,
    )

    end = time.monotonic() + duration_s
    if wait_active:
        end = None
        wait_deadline = time.monotonic() + max_wait_s
        was_active = False
        active_transitions = 0
    while True:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.monotonic()
        if stop_requested:
            break
        if wait_active and end is None:
            if latest_debug is not None:
                debug_values = latest_debug.data
                is_velocity_path = (
                    len(debug_values) >= 36 and debug_values[3] == 1.0)
                position_speed = norm(latest_sp.velocity) if latest_sp else float('nan')
                is_position_path = (
                    11 <= len(debug_values) < 36 and
                    finite(position_speed) and position_speed > 0.05)
                is_active = (
                    (is_velocity_path and debug_values[2] in (4.0, 5.0)) or
                    is_position_path)
                if is_active and not was_active:
                    active_transitions += 1
                    print(
                        f'active setpoint transition {active_transitions}/{active_index}'
                    )
                was_active = is_active
                if is_active and active_transitions >= active_index:
                    end = now + duration_s
                    print(f'active trajectory detected; collecting {duration_s:.1f}s')
            if end is None and now >= wait_deadline:
                print('timed out waiting for active setpoint', file=sys.stderr)
                end = now
        if end is not None and now >= end:
            break

    fields = [
        't',
        'event',
        'nav_state',
        'arming_state',
        'x',
        'y',
        'z',
        'vx',
        'vy',
        'vz',
        'ax',
        'ay',
        'az',
        'heading',
        'speed',
        'horiz_speed',
        'sp_x',
        'sp_y',
        'sp_z',
        'sp_vx',
        'sp_vy',
        'sp_vz',
        'sp_ax',
        'sp_ay',
        'sp_az',
        'sp_speed',
        'acceleration',
        'sp_acceleration',
        'sp_yaw',
        'lead_distance',
        'lead_xy',
        'planner_phase',
        's_ref',
        's_vehicle',
        'progress_lead',
        'actual_path_speed',
        'planned_speed',
        'remaining_distance',
        'braking_distance',
        'planned_acceleration',
        'planner_target_speed',
        'dynamic_lookahead',
        'motion_state',
        'debug_timestamp',
        'trajectory_code',
        'controller_state',
        'control_backend',
        'yaw_mode',
        's_vehicle_raw',
        's_vehicle_filtered',
        's_plan',
        'path_remaining_distance',
        'projected_path_x',
        'projected_path_y',
        'cross_track_error',
        'along_track_error',
        'velocity_planner_target_speed',
        'velocity_planner_speed',
        'velocity_planner_acceleration',
        'tangent_speed_command',
        'normal_speed_command',
        'total_speed_command',
        'vertical_speed_command',
        'actual_horizontal_speed_debug',
        'actual_tangent_speed',
        'actual_normal_speed',
        'actual_vertical_speed',
        'curvature',
        'curve_speed_limit',
        'terminal_speed_limit',
        'cross_track_speed_scale',
        'yaw_setpoint_debug',
        'yaw_rate_setpoint',
        'actual_heading_debug',
        'endpoint_position_ok',
        'endpoint_speed_ok',
        'endpoint_altitude_ok',
        'endpoint_vertical_speed_ok',
        'endpoint_settle_elapsed',
    ]
    with open(out_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    print(f'wrote {len(rows)} rows to {out_path}')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
