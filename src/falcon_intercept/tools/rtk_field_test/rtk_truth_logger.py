#!/usr/bin/env python3
"""Log the target drone's RTK truth from a relayed MAVLink UDP stream.

Laptop A (QGC machine) relays the target drone telemetry over the field
LAN, e.g. with MAVProxy:

    mavproxy.py --master=COM7 --baudrate 57600 \
        --out=udp:127.0.0.1:14550 --out=udp:<jetson-ip>:14560

This logger listens on UDP, requests the messages we need at useful rates
and writes one JSON object per line:

    {"kind": "gps_raw_int",  "wall": <local unix s>, "boot_us": ...,
     "lat": deg, "lon": deg, "alt": m AMSL, "fix": 6, "eph": m, "epv": m,
     "sats": n}
    {"kind": "system_time",  "wall": <local unix s>, "boot_ms": ...,
     "utc": <gps-disciplined unix s>}

The (wall, utc) pairs from system_time let the offline comparator map the
GPS-time axis onto the Jetson wall-clock axis used by the stereo
measurements, so the two laptops never need perfect NTP.

Requires pymavlink (Jetson: already installed, v2.4.49).
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

GPS_RAW_INT_ID = 24
GLOBAL_POSITION_INT_ID = 33
SYSTEM_TIME_ID = 2

AUTOPILOT_MESSAGE_TYPES = {"GPS_RAW_INT", "GLOBAL_POSITION_INT", "SYSTEM_TIME"}
GCS_SYSID = 255


def should_lock_sysid(message_type: str, src_system: int, requested: int) -> bool:
    """Decide whether this message identifies the autopilot to lock onto.

    Never auto-lock a GCS sysid (255): MAVProxy/radio links may emit GCS
    heartbeats before the autopilot's traffic arrives, and locking 255
    would starve the logger forever. Only autopilot-originated message
    types (GPS/SYSTEM_TIME) may trigger an automatic lock. With an
    explicit --source-system, only that sysid may lock.
    """
    if requested > 0:
        return src_system == requested and message_type in AUTOPILOT_MESSAGE_TYPES
    if src_system == GCS_SYSID:
        return False
    return message_type in AUTOPILOT_MESSAGE_TYPES


def request_streams(mav, system_id: int, component_id: int) -> None:
    """Ask the autopilot for the message rates this test needs."""
    # SET_MESSAGE_INTERVAL (MAVLink common #511).
    for message_id, interval_us in (
        (GPS_RAW_INT_ID, 100_000),      # 10 Hz truth positions
        (GLOBAL_POSITION_INT_ID, 100_000),
        (SYSTEM_TIME_ID, 200_000),      # 5 Hz clock mapping
    ):
        mav.command_long_send(
            system_id,
            component_id,
            511,
            0,
            message_id,
            interval_us,
            0,
            0,
            0,
            0,
            0,
        )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--listen", default="0.0.0.0:14560", help="UDP host:port to listen on")
    parser.add_argument("--output", required=True, help="JSONL output path")
    parser.add_argument("--source-system", type=int, default=0, help="expected sysid (0 = accept first seen)")
    parser.add_argument("--no-request", action="store_true", help="do not send SET_MESSAGE_INTERVAL requests")
    args = parser.parse_args()

    try:
        from pymavlink import mavutil
    except ImportError:
        print("pymavlink is required (pip install pymavlink)", file=sys.stderr)
        return 2

    host, _, port_text = args.listen.partition(":")
    connection = mavutil.mavlink_connection(
        f"udp:{host}:{int(port_text)}", source_system=255, dialect="common"
    )
    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"[rtk_truth_logger] listening udp {args.listen}, writing {output_path}", flush=True)

    accepted_sysid = args.source_system if args.source_system > 0 else None
    autopilot: tuple[int, int] | None = None
    last_request = 0.0
    counts = {"gps_raw_int": 0, "global_position_int": 0, "system_time": 0}
    last_report = time.monotonic()

    with output_path.open("a", encoding="utf-8") as sink:
        while True:
            message = connection.recv_match(blocking=True, timeout=1.0)
            now_wall = time.time()
            now_mono = time.monotonic()
            if message is None:
                continue
            mtype = message.get_type()
            src_system = message.get_srcSystem()
            if accepted_sysid is None:
                if not should_lock_sysid(mtype, src_system, args.source_system):
                    continue
                accepted_sysid = src_system
                print(f"[rtk_truth_logger] locked sysid={accepted_sysid}", flush=True)
            elif src_system != accepted_sysid:
                continue
            if autopilot is None:
                autopilot = (src_system, message.get_srcComponent())

            record = None
            if mtype == "GPS_RAW_INT":
                record = {
                    "kind": "gps_raw_int",
                    "wall": now_wall,
                    "boot_us": int(message.time_usec),
                    "lat": message.lat / 1e7,
                    "lon": message.lon / 1e7,
                    "alt": message.alt / 1000.0,
                    "fix": int(message.fix_type),
                    "eph": (message.eph / 100.0) if message.eph != 65535 else None,
                    "epv": (message.epv / 100.0) if message.epv != 65535 else None,
                    "sats": int(message.satellites_visible),
                }
            elif mtype == "GLOBAL_POSITION_INT":
                record = {
                    "kind": "global_position_int",
                    "wall": now_wall,
                    "boot_ms": int(message.time_boot_ms),
                    "lat": message.lat / 1e7,
                    "lon": message.lon / 1e7,
                    "alt": message.alt / 1000.0,
                    "relative_alt": message.relative_alt / 1000.0,
                }
            elif mtype == "SYSTEM_TIME":
                record = {
                    "kind": "system_time",
                    "wall": now_wall,
                    "boot_ms": int(message.time_boot_ms),
                    "utc": int(message.time_unix_usec) / 1e6,
                }
            elif mtype == "HEARTBEAT" and not args.no_request:
                pass
            if record is not None:
                counts[record["kind"]] = counts.get(record["kind"], 0) + 1
                sink.write(json.dumps(record) + "\n")
                sink.flush()

            if not args.no_request and autopilot and now_mono - last_request > 10.0:
                request_streams(connection.mav, autopilot[0], autopilot[1])
                last_request = now_mono
            if now_mono - last_report > 15.0:
                print(f"[rtk_truth_logger] counts={counts}", flush=True)
                last_report = now_mono


if __name__ == "__main__":
    sys.exit(main())
