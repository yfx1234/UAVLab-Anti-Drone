#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SOURCE_WORKSPACE="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
if [[ -d "${SOURCE_WORKSPACE}/src" ]]; then
  WORKSPACE_DIR="${WORKSPACE_DIR:-${SOURCE_WORKSPACE}}"
else
  PACKAGE_PREFIX="$(cd "${SCRIPT_DIR}/../.." && pwd)"
  WORKSPACE_DIR="${WORKSPACE_DIR:-$(cd "${PACKAGE_PREFIX}/../.." && pwd)}"
fi
RUN_SECONDS="${RUN_SECONDS:-30}"
LOG_FILE="${LOG_FILE:-/tmp/falcon_deepstream_three_camera.log}"
MODEL_ENGINE_PATH="${MODEL_ENGINE_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine}"
MODEL_ONNX_PATH="${MODEL_ONNX_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16.onnx}"
CLASS_NAMES="${CLASS_NAMES:-drone}"
NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-1}"
BATCH_SIZE="${BATCH_SIZE:-1}"
MIN_STATS_LINES="${MIN_STATS_LINES:-3}"
SOURCE_MODE="${SOURCE_MODE:-all}"
MUX_WIDTH="${MUX_WIDTH:-1280}"
MUX_HEIGHT="${MUX_HEIGHT:-720}"
WORKSPACE_SIZE_MB="${WORKSPACE_SIZE_MB:-0}"
NETWORK_MODE="${NETWORK_MODE:-2}"
MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON=1

rm -f "${LOG_FILE}"
setsid ros2 run falcon_intercept_deepstream deepstream_three_camera_node \
  --ros-args \
  -p "model_engine_path:=${MODEL_ENGINE_PATH}" \
  -p "model_onnx_path:=${MODEL_ONNX_PATH}" \
  -p "class_names:=${CLASS_NAMES}" \
  -p "num_detected_classes:=${NUM_DETECTED_CLASSES}" \
  -p "batch_size:=${BATCH_SIZE}" \
  -p "source_mode:=${SOURCE_MODE}" \
  -p "mux_width:=${MUX_WIDTH}" \
  -p "mux_height:=${MUX_HEIGHT}" \
  -p "workspace_size_mb:=${WORKSPACE_SIZE_MB}" \
  -p "network_mode:=${NETWORK_MODE}" \
  -p "model_output_format:=${MODEL_OUTPUT_FORMAT}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -INT -- "-${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
  sleep 2
  kill -KILL -- "-${LAUNCH_PID}" 2>/dev/null || true
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "RUN_SECONDS=${RUN_SECONDS} BATCH_SIZE=${BATCH_SIZE} SOURCE_MODE=${SOURCE_MODE} "\
"MUX=${MUX_WIDTH}x${MUX_HEIGHT} WORKSPACE_SIZE_MB=${WORKSPACE_SIZE_MB} LOG_FILE=${LOG_FILE}"
for ((elapsed = 0; elapsed < RUN_SECONDS; elapsed++)); do
  sleep 1
  stats_count="$(grep -c "deepstream stats" "${LOG_FILE}" 2>/dev/null || true)"
  if (( stats_count >= MIN_STATS_LINES )); then
    echo "Collected ${stats_count} statistics lines after ${elapsed}s"
    break
  fi
  if ! kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    echo "DeepStream process exited after ${elapsed}s"
    break
  fi
done

echo "=== LAST DEEPSTREAM STATS ==="
grep "deepstream stats" "${LOG_FILE}" | tail -n 5 || true

echo "=== PIPELINE ERRORS ==="
grep -E "DeepStream error|startup failed|ERROR|Failed to" "${LOG_FILE}" | tail -n 30 || true

if ! grep -q "deepstream stats" "${LOG_FILE}"; then
  echo "DEEPSTREAM_SMOKE_FAIL no statistics received"
  exit 1
fi
if grep -qE "DeepStream error|startup failed" "${LOG_FILE}"; then
  echo "DEEPSTREAM_SMOKE_FAIL pipeline error"
  exit 1
fi

echo "DEEPSTREAM_SMOKE_PASS"
