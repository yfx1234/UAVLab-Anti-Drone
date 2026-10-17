#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RUN_SECONDS="${RUN_SECONDS:-60}"
BACKEND="${BACKEND:-ultralytics}"
MODEL_PATH="${MODEL_PATH:-}"
DEVICE="${DEVICE:-cuda:0}"
HALF="${HALF:-true}"
IMGSZ="${IMGSZ:-640}"
FRAME_STRIDE="${FRAME_STRIDE:-1}"
ENABLE_SEARCH="${ENABLE_SEARCH:-true}"
ENABLE_STEREO_LEFT="${ENABLE_STEREO_LEFT:-true}"
ENABLE_STEREO_RIGHT="${ENABLE_STEREO_RIGHT:-true}"
LOG_FILE="${LOG_FILE:-/tmp/falcon_yolo_backend_benchmark.log}"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON=1

LAUNCH_ARGS=(
  "detector_backend:=${BACKEND}"
  "device:=${DEVICE}"
  "half:=${HALF}"
  "imgsz:=${IMGSZ}"
  "frame_stride:=${FRAME_STRIDE}"
  "enable_search:=${ENABLE_SEARCH}"
  "enable_stereo_front:=${ENABLE_STEREO_LEFT}"
  "enable_stereo_back:=${ENABLE_STEREO_RIGHT}"
)
if [[ -n "${MODEL_PATH}" ]]; then
  LAUNCH_ARGS+=("model_path:=${MODEL_PATH}")
fi

rm -f "${LOG_FILE}"
setsid ros2 launch falcon_intercept three_yolo_v0.launch.py \
  "${LAUNCH_ARGS[@]}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "BACKEND=${BACKEND} DEVICE=${DEVICE} HALF=${HALF} IMGSZ=${IMGSZ}"
echo "ENABLE_SEARCH=${ENABLE_SEARCH} ENABLE_STEREO_LEFT=${ENABLE_STEREO_LEFT} ENABLE_STEREO_RIGHT=${ENABLE_STEREO_RIGHT}"
echo "RUN_SECONDS=${RUN_SECONDS} LOG_FILE=${LOG_FILE}"

sleep "${RUN_SECONDS}"

echo "=== LAST DETECTOR STATS ==="
for node in search_yolo_detector_node stereo_front_yolo_detector_node stereo_back_yolo_detector_node; do
  grep "${node}.*detector stats" "${LOG_FILE}" | tail -n 1 || true
done

echo "=== LAST CAMERA STATS ==="
grep -E "camera stats" "${LOG_FILE}" | tail -n 6 || true

echo "=== ERRORS ==="
grep -E "ERROR|inference failed|camera startup failed" "${LOG_FILE}" | tail -n 30 || true

echo "BENCHMARK_COMPLETE"
