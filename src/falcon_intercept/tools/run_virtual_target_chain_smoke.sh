#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${1:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
LOG_PATH="/tmp/virtual_target_chain_smoke_launch.log"
LAUNCH_PID=""

cleanup() {
  if [[ -n "${LAUNCH_PID}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -TERM -- "-${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "${WORKSPACE}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

setsid ros2 launch falcon_intercept real_virtual_target_v0.launch.py \
  adapter_mode:=shadow \
  target_scenario:=stationary \
  attack_enable_delay_s:=0.0 \
  required_shots:=1 >"${LOG_PATH}" 2>&1 &
LAUNCH_PID=$!
sleep 2
ros2 param get /safety_node capture_minimum_range_m
ros2 param get /safety_node capture_maximum_range_m

set +e
python3 src/falcon_intercept/tools/virtual_target_chain_smoke.py
SMOKE_STATUS=$?
set -e
cleanup
LAUNCH_PID=""
tail -n 100 "${LOG_PATH}"
exit "${SMOKE_STATUS}"
