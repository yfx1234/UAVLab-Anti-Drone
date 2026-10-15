#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
LOG_FILE="/tmp/falcon_search_yolo_contract_smoke.log"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

DEVICE_PATH="${1:-/dev/video0}"

echo "CONTRACT_CHECK"
python3 src/falcon_intercept/tools/search_detector_contract_check.py --device "${DEVICE_PATH}"

echo "LAUNCH_CHECK"
rm -f "${LOG_FILE}"
set +e
timeout 10 ros2 launch falcon_intercept search_yolo_v0.launch.py \
  input_mode:=direct_camera \
  device_path:="${DEVICE_PATH}" \
  detector_backend:=empty >"${LOG_FILE}" 2>&1
STATUS=$?
set -e

if [[ "${STATUS}" != "0" && "${STATUS}" != "124" ]]; then
  echo "ERROR: launch exited with status ${STATUS}"
  tail -n 80 "${LOG_FILE}"
  exit "${STATUS}"
fi

if ! grep -q "detector stats publishes=" "${LOG_FILE}"; then
  echo "ERROR: detector stats were not observed"
  tail -n 80 "${LOG_FILE}"
  exit 1
fi

echo "LOG_TAIL"
tail -n 40 "${LOG_FILE}"
