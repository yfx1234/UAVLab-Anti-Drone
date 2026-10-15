#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
BAG_NAME="${1:-decision_success_fake_$(date +%Y%m%d)}"
BAG_DIR="${WORKSPACE_DIR}/bags/${BAG_NAME}"
LOG_FILE="/tmp/falcon_decision_bag.log"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash

if [[ -e "${BAG_DIR}" ]]; then
  echo "Bag directory already exists: ${BAG_DIR}" >&2
  exit 2
fi

mkdir -p "$(dirname "${BAG_DIR}")"
rm -f "${LOG_FILE}"

setsid ros2 launch falcon_intercept_control real_flight_decision_smoke.launch.py >"${LOG_FILE}" 2>&1 &
LAUNCH_PID=$!
DRIVER_PID=""
BAG_PID=""

cleanup() {
  kill -- "-${LAUNCH_PID}" 2>/dev/null || true
  if [[ -n "${DRIVER_PID}" ]]; then kill "${DRIVER_PID}" 2>/dev/null || true; fi
  if [[ -n "${BAG_PID}" ]]; then kill -INT "${BAG_PID}" 2>/dev/null || true; fi
  wait "${LAUNCH_PID}" 2>/dev/null || true
  if [[ -n "${DRIVER_PID}" ]]; then wait "${DRIVER_PID}" 2>/dev/null || true; fi
  if [[ -n "${BAG_PID}" ]]; then wait "${BAG_PID}" 2>/dev/null || true; fi
}
trap cleanup EXIT

ros2 bag record \
  -o "${BAG_DIR}" \
  /target/track \
  /control/px4_status \
  /decision/search_command \
  /decision/mission_status \
  /decision/guidance_proposal \
  /decision/command_proposal \
  /decision/safety_status \
  /decision/capture_window \
  /intercept/guidance_command \
  /intercept/fire_event >>"${LOG_FILE}" 2>&1 &
BAG_PID=$!

# Allow DDS discovery before the finite-duration driver begins publishing.
sleep 3
python3 src/falcon_intercept_control/test/real_flight_decision_smoke.py >>"${LOG_FILE}" 2>&1 &
DRIVER_PID=$!
wait "${DRIVER_PID}"
DRIVER_PID=""
sleep 1
kill -INT "${BAG_PID}" 2>/dev/null || true
wait "${BAG_PID}" || true
BAG_PID=""

echo "Recorded bag: ${BAG_DIR}"
echo "Launch log tail:"
tail -n 40 "${LOG_FILE}"
