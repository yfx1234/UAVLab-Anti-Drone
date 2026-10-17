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
MODEL_ENGINE_PATH="${MODEL_ENGINE_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/yolo26n_deepstream_b1.engine}"
MODEL_ONNX_PATH="${MODEL_ONNX_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/yolo26n.onnx}"
LOG_FILE="${LOG_FILE:-/tmp/falcon_deepstream_topic_check.log}"
TOPIC_TIMEOUT="${TOPIC_TIMEOUT:-20}"
REQUIRE_POSITIVE="${REQUIRE_POSITIVE:-}"

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
  -p batch_size:=1 >"${LOG_FILE}" 2>&1 &
NODE_PID=$!

cleanup() {
  kill -INT -- "-${NODE_PID}" 2>/dev/null || true
  sleep 2
  kill -TERM -- "-${NODE_PID}" 2>/dev/null || true
  sleep 2
  kill -KILL -- "-${NODE_PID}" 2>/dev/null || true
  kill "${NODE_PID}" 2>/dev/null || true
  wait "${NODE_PID}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

for _ in $(seq 1 20); do
  if grep -q "DeepStream native pipeline started" "${LOG_FILE}" 2>/dev/null; then
    break
  fi
  if ! kill -0 "${NODE_PID}" 2>/dev/null; then
    echo "DeepStream node exited before topic validation"
    tail -n 60 "${LOG_FILE}" || true
    exit 1
  fi
  sleep 1
done

PROBE_ARGUMENTS=(--timeout "${TOPIC_TIMEOUT}")
if [[ -n "${REQUIRE_POSITIVE}" ]]; then
  PROBE_ARGUMENTS+=(--require-positive "${REQUIRE_POSITIVE}")
fi
python3 src/falcon_intercept_deepstream/tools/deepstream_detection_topic_check.py \
  "${PROBE_ARGUMENTS[@]}"

echo "=== LAST DEEPSTREAM STATS ==="
grep "deepstream stats" "${LOG_FILE}" | tail -n 3 || true
echo "DEEPSTREAM_ROS_TOPIC_SMOKE_PASS"
