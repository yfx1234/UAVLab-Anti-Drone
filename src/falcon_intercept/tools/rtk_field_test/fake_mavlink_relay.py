#!/usr/bin/env python3
"""Synthetic MAVLink UDP relay for bench-testing rtk_truth_logger.py.

Pretends to be a target drone flying a circle around a fixed home point,
sending SYSTEM_TIME + GPS_RAW_INT like a real PX4/MAVProxy relay would.
Used only for tooling verification — never during the real field test.

Example:
    python3 fake_mavlink_relay.py --target 127.0.0.1:14560 --radius-m 20
"""

from __future__ import annotations

import argparse
import math
import sys
import time

# Roughly the field-test site; any plausible coordinate works for testing.
HOME_LAT = 30.5278000
HOME_LON = 114.3567000
HOME_ALT = 25.0

# Site-calibrated against rtk_truth_core.geodetic_to_enu so the synthetic
# truth matches the comparator's exact ENU conversion.
M_PER_DEG_LAT = 110_862.0
M_PER_DEG_LON = 95_972.0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--target", default="127.0.0.1:14560", help="udpout host:port")
    parser.add_argument("--radius-m", type=float, default=20.0)
    parser.add_argument("--speed-mps", type=float, default=5.0)
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--sysid", type=int, default=42)
    parser.add_argument(
        "--gcs-heartbeat",
        action="store_true",
        help="also emit sysid=255 GCS heartbeats (pollutes the stream like a "
        "MAVProxy/GCS endpoint would; the logger must not lock onto 255)",
    )
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        print("pymavlink is required", file=sys.stderr)
        return 2

    connection = mavutil.mavlink_connection(
        f"udpout:{args.target}", source_system=args.sysid, dialect="common"
    )
    gcs_connection = None
    if args.gcs_heartbeat:
        gcs_connection = mavutil.mavlink_connection(
            f"udpout:{args.target}", source_system=255, dialect="common"
        )
    boot = time.monotonic()
    wall0 = time.time()
    period = 1.0 / args.rate_hz
    m_per_deg_lon = M_PER_DEG_LON
    print(
        f"[fake_relay] circle r={args.radius_m} m v={args.speed_mps} m/s -> udp {args.target}",
        flush=True,
    )
    sequence = 0
    while True:
        now_mono = time.monotonic()
        elapsed = now_mono - boot
        boot_us = int(elapsed * 1e6)
        utc = wall0 + elapsed
        if gcs_connection is not None and sequence % int(args.rate_hz) == 0:
            # GCS endpoint heartbeat (MAV_TYPE_GCS=6, AUTOPILOT_INVALID=8).
            gcs_connection.mav.heartbeat_send(6, 8, 0, 0, 0, 3)
        theta = (args.speed_mps * elapsed) / args.radius_m
        east = args.radius_m * math.cos(theta)
        north = args.radius_m * math.sin(theta)
        lat = HOME_LAT + north / M_PER_DEG_LAT
        lon = HOME_LON + east / m_per_deg_lon
        # SYSTEM_TIME at every pass is fine for a bench test (real: 5 Hz).
        connection.mav.system_time_send(int(utc * 1e6), boot_us // 1000)
        connection.mav.gps_raw_int_send(
            boot_us,
            6,  # fix_type: RTK fixed
            int(lat * 1e7),
            int(lon * 1e7),
            int(HOME_ALT * 1000),
            30,   # eph [cm] -> 0.30 m reported accuracy
            50,   # epv [cm]
            int(args.speed_mps * 100),
            0,
            18,   # satellites
        )
        sequence += 1
        if sequence % int(args.rate_hz * 10) == 0:
            print(f"[fake_relay] sent={sequence} theta={math.degrees(theta) % 360:.0f}deg", flush=True)
        time.sleep(period)


if __name__ == "__main__":
    sys.exit(main())
