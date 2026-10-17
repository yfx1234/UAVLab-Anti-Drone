#!/usr/bin/env bash
# Field truth-test orchestrator (runs on the interceptor Jetson).
#
# FINAL both-fly architecture (2026-07-30 frozen):
#   - laptop A runs QGC + 2x MAVProxy + the ENEMY rtk_truth_logger locally
#     (no network dependency on the Jetson at all);
#   - this Jetson (ground bench OR airborne) records ONLY its own side:
#     own RTK (onboard DDS), async stereo measurements, rosbag evidence;
#   - after landing, the two file sets are merged on one machine and
#     compare_rtk_stereo_truth.py aligns them on the GPS UTC axis (clock
#     skew between laptop A and Jetson is irrelevant by design).
#
# Modes:
#   --airborne        flight form: own measurement logger + rosbag only
#                     (+ optional chain). No enemy logger, no monitor.
#   (default)         ground-bench form: also runs the enemy truth logger
#                     (MAVProxy relays to this Jetson :14560), optional
#                     --own-relay interceptor MAVLink logger and the
#                     --lever-enu live monitor.
#
# Operator must start the DeepStream three-camera pipeline separately
# (tools/deepstream_three_camera_smoke_test.sh or the preview entry).
#
# Usage:
#   bash record_field_truth_test.sh <run_name> [--with-chain] [--airborne] \
#        [--listen 0.0.0.0:14560] [--lever-enu de,dn,du] [--own-relay 0.0.0.0:14561] [--with-roi]
#
# --with-roi also launches the search->stereo ROI projection node with the
# 2026-08-05 field parameters (margin_px=96 until the search pose is refit
# with more correspondences; max_source_age_s=0.5 while the USB search
# camera timestamp offset is under investigation; depth_min_m=1.0).
set -eo pipefail

RUN_NAME="${1:?run name required, e.g. field_0730_am}"
shift || true
WITH_CHAIN=0
AIRBORNE=0
WITH_ROI=0
LISTEN="0.0.0.0:14560"
OWN_RELAY=""
LEVER_ENU=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --with-chain) WITH_CHAIN=1; shift ;;
    --airborne) AIRBORNE=1; shift ;;
    --with-roi) WITH_ROI=1; shift ;;
    --listen) LISTEN="$2"; shift 2 ;;
    --lever-enu) LEVER_ENU="$2"; shift 2 ;;
    --own-relay) OWN_RELAY="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
TOOLS_DIR="${WORKSPACE_DIR}/src/falcon_intercept/tools/rtk_field_test"
OUT_DIR="${WORKSPACE_DIR}/bags/rtk_field_test/${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "${OUT_DIR}"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

PIDS=()
cleanup() {
  echo "[field_test] stopping..."
  kill -INT "${PIDS[@]}" 2>/dev/null || true
  sleep 3
  kill -TERM "${PIDS[@]}" 2>/dev/null || true
  sleep 2
  kill -KILL "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "[field_test] output dir: ${OUT_DIR}"
if [[ "${AIRBORNE}" == "1" ]]; then
  echo "[field_test] AIRBORNE mode: own-side recording only (no enemy logger/monitor)"
fi

if [[ "${WITH_CHAIN}" == "1" ]]; then
  ros2 launch falcon_intercept full_v0.launch.py adapter_mode:=shadow \
    >"${OUT_DIR}/shadow_chain.log" 2>&1 &
  PIDS+=($!)
  sleep 5
fi

if [[ "${WITH_ROI}" == "1" ]]; then
  ros2 launch falcon_intercept roi_projection_v0.launch.py \
    depth_min_m:=1.0 margin_px:=96 max_source_age_s:=0.5 \
    >"${OUT_DIR}/roi_projection.log" 2>&1 &
  PIDS+=($!)
  echo "[field_test] ROI projection node started (margin=96 max_source_age=0.5)"
fi

if [[ "${AIRBORNE}" == "0" ]]; then
  python3 "${TOOLS_DIR}/rtk_truth_logger.py" \
    --listen "${LISTEN}" \
    --output "${OUT_DIR}/enemy_rtk.jsonl" \
    >"${OUT_DIR}/rtk_truth_logger.log" 2>&1 &
  PIDS+=($!)
fi

python3 "${TOOLS_DIR}/field_measurement_logger.py" \
  --output "${OUT_DIR}/measurements.jsonl" \
  >"${OUT_DIR}/field_measurement_logger.log" 2>&1 &
PIDS+=($!)

if [[ "${AIRBORNE}" == "0" && -n "${OWN_RELAY}" ]]; then
  python3 "${TOOLS_DIR}/rtk_truth_logger.py" \
    --listen "${OWN_RELAY}" \
    --output "${OUT_DIR}/own_rtk_mavlink.jsonl" \
    --no-request \
    >"${OUT_DIR}/own_rtk_mavlink_logger.log" 2>&1 &
  PIDS+=($!)
  echo "[field_test] own relay logger on ${OWN_RELAY} -> own_rtk_mavlink.jsonl"
fi

ros2 bag record -o "${OUT_DIR}/evidence_bag" \
  /stereo/front/detections_json \
  /stereo/back/detections_json \
  /search/detections_json \
  /perception/async_stereo_measurement \
  /perception/target/camera_position \
  /target/track \
  /fmu/out/vehicle_gps_position \
  >"${OUT_DIR}/rosbag_record.log" 2>&1 &
PIDS+=($!)

if [[ "${AIRBORNE}" == "0" && -n "${LEVER_ENU}" ]]; then
  python3 "${TOOLS_DIR}/live_range_monitor.py" \
    --enemy "${OUT_DIR}/enemy_rtk.jsonl" \
    --measurements "${OUT_DIR}/measurements.jsonl" \
    --lever-enu "${LEVER_ENU}" \
    --out-csv "${OUT_DIR}/live_comparison.csv" \
    --out-jsonl "${OUT_DIR}/live_comparison.jsonl" \
    >"${OUT_DIR}/live_monitor.log" 2>&1 &
  PIDS+=($!)
  echo "[field_test] live monitor: tail -f ${OUT_DIR}/live_monitor.log"
  echo "[field_test] live export:  ${OUT_DIR}/live_comparison.csv"
fi

echo "[field_test] running (pids: ${PIDS[*]}). Ctrl-C to stop."
if [[ "${AIRBORNE}" == "0" ]]; then
  echo "[field_test] afterwards compare with:"
  echo "  python3 ${TOOLS_DIR}/compare_rtk_stereo_truth.py \\"
  echo "    --enemy ${OUT_DIR}/enemy_rtk.jsonl \\"
  echo "    --measurements ${OUT_DIR}/measurements.jsonl \\"
  echo "    --lever-enu <de,dn,du> --output ${OUT_DIR}/comparison.json"
else
  echo "[field_test] after landing: copy this dir to the machine holding enemy_rtk.jsonl and compare there."
fi
wait -n "${PIDS[@]}" || true
echo "[field_test] a child exited; shutting down the rest."
