#!/usr/bin/env bash
set -o pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG_FILE="/tmp/falcon_three_yolo_camera_smoke.log"
WARMUP_SECONDS="${WARMUP_SECONDS:-15}"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

rm -f "${LOG_FILE}"
setsid ros2 launch falcon_intercept three_yolo_v0.launch.py \
  detector_backend:=empty \
  search_width:=640 \
  search_height:=480 \
  search_fps:=25.0 \
  search_source_width:=640 \
  search_source_height:=480 \
  search_source_fps:=25.0 \
  stereo_width:=640 \
  stereo_height:=480 \
  stereo_fps:=30.0 \
  stereo_source_width:=3840 \
  stereo_source_height:=2160 \
  stereo_source_fps:=30.0 >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || true
  kill "${LAUNCH_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
}
trap cleanup EXIT

echo "WARMUP_SECONDS=${WARMUP_SECONDS}"
sleep "${WARMUP_SECONDS}"

echo "NODES"
timeout 8 ros2 node list || true
echo "TOPICS"
timeout 8 ros2 topic list --no-daemon || true

FAILURES=0
check_topic() {
  local topic="$1"
  echo "IMAGE_TOPIC_CHECK topic=${topic}"
  python3 src/falcon_intercept/tools/check_image_topic.py \
    --topic "${topic}" --duration 5 --timeout 15 || FAILURES=$((FAILURES + 1))
}

check_topic /search/image_raw
check_topic /stereo/front/image_raw
check_topic /stereo/back/image_raw

echo "LOG_TAIL"
tail -n 120 "${LOG_FILE}"
echo "FAILURES=${FAILURES}"
exit "${FAILURES}"
