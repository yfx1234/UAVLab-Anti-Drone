#!/usr/bin/env bash
set -eo pipefail

PROFILE="${1:-}"
WORKSPACE="${2:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
if [[ ! "${PROFILE}" =~ ^(search|tracking|strike|full)$ ]]; then
  echo "usage: $0 {search|tracking|strike|full} [workspace]" >&2
  exit 2
fi

LOG_PATH="/tmp/virtual_target_${PROFILE}_test_launch.log"
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

case "${PROFILE}" in
  search)
    DURATION=6
    LAUNCH_ARGS=(
      target_scenario:=stationary
      detection_mode:=search_only
      attack_enable_delay_s:=3600.0
      required_shots:=1
    )
    ;;
  tracking)
    DURATION=8
    LAUNCH_ARGS=(
      target_scenario:=stationary
      detection_mode:=stereo_only
      attack_enable_delay_s:=3600.0
      required_shots:=1
    )
    ;;
  strike)
    DURATION=9
    LAUNCH_ARGS=(
      target_scenario:=stationary
      detection_mode:=stereo_only
      attack_enable_delay_s:=0.0
      required_shots:=1
      reattack_cooldown_s:=0.5
    )
    ;;
  full)
    DURATION=14
    LAUNCH_ARGS=(
      target_scenario:=straight
      target_speed_mps:=0.5
      target_motion_delay_s:=0.0
      target_motion_ramp_s:=1.0
      detection_mode:=all
      detection_start_delay_s:=1.0
      attack_enable_delay_s:=0.5
      required_shots:=2
      reattack_cooldown_s:=1.0
    )
    ;;
esac

setsid ros2 launch falcon_intercept real_virtual_target_v0.launch.py \
  adapter_mode:=shadow \
  allow_live_control:=false \
  "${LAUNCH_ARGS[@]}" >"${LOG_PATH}" 2>&1 &
LAUNCH_PID=$!
sleep 2

set +e
python3 src/falcon_intercept/tools/virtual_target_chain_smoke.py \
  --profile "${PROFILE}" --duration "${DURATION}"
STATUS=$?
set -e
cleanup
LAUNCH_PID=""
tail -n 80 "${LOG_PATH}"
exit "${STATUS}"
