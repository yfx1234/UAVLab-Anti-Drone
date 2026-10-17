#!/usr/bin/env bash
# Board-side end-to-end smoke for the RTK field-test toolchain.
# fake MAVLink relay -> rtk_truth_logger -> enemy_rtk.jsonl
# fake ROS publishers -> field_measurement_logger -> measurements.jsonl
# compare_rtk_stereo_truth.py -> expect errors at the injected noise floor.
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
TOOLS_DIR="${WORKSPACE_DIR}/src/falcon_intercept/tools/rtk_field_test"
OUT=/tmp/rtk_smoke
rm -rf "${OUT}" && mkdir -p "${OUT}"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1

PIDS=()
cleanup() { kill -INT "${PIDS[@]}" 2>/dev/null || true; sleep 2; kill -KILL "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT

python3 "${TOOLS_DIR}/rtk_truth_logger.py" --listen 0.0.0.0:14560 \
  --output "${OUT}/enemy.jsonl" --no-request >"${OUT}/logger.log" 2>&1 &
PIDS+=($!)
sleep 2
python3 "${TOOLS_DIR}/fake_mavlink_relay.py" --target 127.0.0.1:14560 \
  --radius-m 20 --speed-mps 5 >"${OUT}/relay.log" 2>&1 &
PIDS+=($!)
python3 "${TOOLS_DIR}/field_measurement_logger.py" \
  --output "${OUT}/measurements.jsonl" \
  --own-rtk-topic /test/gps --async-topic /test/async --cam-pos-topic /test/cam \
  >"${OUT}/meas_logger.log" 2>&1 &
PIDS+=($!)
sleep 3

# Fake interceptor-side publishers: own RTK (static anchor) + async
# measurement whose norm tracks the relay's 20 m circle radius.
python3 - <<'PYEOF' &
import math, random, time
import rclpy
from rclpy.node import Node
from px4_msgs.msg import SensorGps
from falcon_intercept_msgs.msg import AsyncStereoMeasurement

rclpy.init()
node = Node("rtk_smoke_publisher")
gps_pub = node.create_publisher(SensorGps, "/test/gps", 10)
meas_pub = node.create_publisher(AsyncStereoMeasurement, "/test/async", 10)
rng = random.Random(3)
start = time.time()
while time.time() - start < 10.0:
    now = time.time()
    sec, nanosec = int(now), int((now % 1) * 1e9)

    gps = SensorGps()
    gps.latitude_deg = 30.5278
    gps.longitude_deg = 114.3567
    gps.altitude_msl_m = 25.0
    gps.altitude_ellipsoid_m = 25.0
    gps.fix_type = 6
    gps.eph = 0.02
    gps.epv = 0.03
    gps.satellites_used = 20
    gps.time_utc_usec = int(now * 1e6)
    gps_pub.publish(gps)

    meas = AsyncStereoMeasurement()
    meas.header.stamp.sec = sec
    meas.header.stamp.nanosec = nanosec
    meas.header.frame_id = "smoke"
    meas.valid = True
    meas.reason = "accepted"
    radius = 20.0 + rng.uniform(-0.03, 0.03)
    meas.position_camera.x = 0.0
    meas.position_camera.y = 0.0
    meas.position_camera.z = radius
    meas.position_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
    meas.timing_delta = 0.012
    meas.timing_sigma = 0.0015
    meas_pub.publish(meas)
    time.sleep(0.1)
node.destroy_node()
rclpy.shutdown()
PYEOF
PIDS+=($!)
sleep 12

echo "== enemy lines: $(wc -l < "${OUT}/enemy.jsonl")"
echo "== measurement lines: $(wc -l < "${OUT}/measurements.jsonl")"
python3 "${TOOLS_DIR}/compare_rtk_stereo_truth.py" \
  --enemy "${OUT}/enemy.jsonl" --measurements "${OUT}/measurements.jsonl" \
  --output "${OUT}/comparison.json"
echo "SMOKE_DONE"
