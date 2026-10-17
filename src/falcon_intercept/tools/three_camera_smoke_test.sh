#!/usr/bin/env bash
set -o pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG_FILE="/tmp/falcon_three_camera_smoke.log"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

SEARCH_DEVICE_PATH="${SEARCH_DEVICE_PATH:-/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._HJ_4K_Camera_SN0001-video-index0}"
SEARCH_RESOLVED="$(readlink -f "${SEARCH_DEVICE_PATH}" 2>/dev/null || true)"
if [[ ! "${SEARCH_RESOLVED}" =~ /dev/video[0-9]+$ ]]; then
  echo "FAIL: stable search camera path does not resolve: ${SEARCH_DEVICE_PATH}"
  exit 1
fi

SEARCH_WIDTH="${SEARCH_WIDTH:-2560}"
SEARCH_HEIGHT="${SEARCH_HEIGHT:-1440}"
SEARCH_FPS="${SEARCH_FPS:-25.0}"
STEREO_WIDTH="${STEREO_WIDTH:-3840}"
STEREO_HEIGHT="${STEREO_HEIGHT:-2160}"
STEREO_FPS="${STEREO_FPS:-30.0}"

rm -f "${LOG_FILE}"
setsid ros2 launch falcon_intercept three_cameras_v0.launch.py \
  search_device_path:="${SEARCH_DEVICE_PATH}" \
  search_width:="${SEARCH_WIDTH}" \
  search_height:="${SEARCH_HEIGHT}" \
  search_fps:="${SEARCH_FPS}" \
  stereo_width:="${STEREO_WIDTH}" \
  stereo_height:="${STEREO_HEIGHT}" \
  stereo_fps:="${STEREO_FPS}" >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || true
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

sleep 8
echo "NODES"
timeout 8 ros2 node list || true
echo "TOPICS"
timeout 8 ros2 topic list --no-daemon || true

FAILURES=0
check_topic() {
  local topic="$1"
  echo "IMAGE_TOPIC_CHECK topic=${topic}"
  python3 src/falcon_intercept/tools/check_image_topic.py \
    --topic "${topic}" --duration 5 --timeout 12 || FAILURES=$((FAILURES + 1))
}

check_topic /search/image_raw
check_topic /stereo/front/image_raw
check_topic /stereo/back/image_raw

echo "LOG_TAIL"
tail -n 80 "${LOG_FILE}"
echo "FAILURES=${FAILURES}"
exit "${FAILURES}"
