#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG_FILE="/tmp/falcon_search_camera_smoke.log"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

DEVICE_PATH="${1:-/dev/video0}"
PUBLISHER_BACKEND="${PUBLISHER_BACKEND:-image_tools}"
DEVICE_ID="${DEVICE_ID:-0}"
RESOLVED_DEVICE="$(readlink -f "${DEVICE_PATH}" 2>/dev/null || true)"
if [[ "${RESOLVED_DEVICE}" =~ /dev/video([0-9]+)$ ]]; then
  DEVICE_ID="${BASH_REMATCH[1]}"
fi

rm -f "${LOG_FILE}"
setsid ros2 launch falcon_intercept search_camera_v0.launch.py \
  device_path:="${DEVICE_PATH}" \
  publisher_backend:="${PUBLISHER_BACKEND}" \
  device_id:="${DEVICE_ID}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || true
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 4

echo "NODES"
timeout 6 ros2 node list || echo "WARN: ros2 node list failed"

echo "TOPICS"
timeout 6 ros2 topic list --no-daemon || echo "WARN: ros2 topic list failed; continuing with direct image topic check"

echo "IMAGE_SAMPLE"
timeout 6 ros2 topic echo --once /search/image_raw --field header --qos-profile sensor_data || echo "WARN: header echo timed out"

echo "IMAGE_TOPIC_CHECK"
python3 src/falcon_intercept/tools/check_image_topic.py --topic /search/image_raw --duration 5 --timeout 10 || echo "WARN: image topic check failed"

kill -- "-${LAUNCH_PID}" 2>/dev/null || true
kill "${LAUNCH_PID}" 2>/dev/null || true
wait "${LAUNCH_PID}" 2>/dev/null || true
sleep 2

if [[ "${PUBLISHER_BACKEND}" == "python" ]]; then
  echo "IMAGE_LOOPBACK_CHECK"
  python3 src/falcon_intercept/tools/v4l2_image_loopback_check.py --device "${DEVICE_PATH}" --topic /search/image_raw --duration 5 --timeout 10 || echo "WARN: image loopback check failed"
else
  echo "IMAGE_LOOPBACK_CHECK=SKIPPED backend=${PUBLISHER_BACKEND}"
fi

echo "LOG_TAIL"
tail -n 40 "${LOG_FILE}"
