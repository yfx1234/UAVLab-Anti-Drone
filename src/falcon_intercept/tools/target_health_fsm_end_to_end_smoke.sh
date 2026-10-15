#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FALCON_WORKSPACE:-$HOME/Jetson_ROS2_V0/ros2_ws}"
KF_PID=""
FSM_PID=""
cleanup() {
  [[ -z "$KF_PID" ]] || kill -- "-$KF_PID" 2>/dev/null || true
  [[ -z "$FSM_PID" ]] || kill -- "-$FSM_PID" 2>/dev/null || true
  [[ -z "$KF_PID" ]] || wait "$KF_PID" 2>/dev/null || true
  [[ -z "$FSM_PID" ]] || wait "$FSM_PID" 2>/dev/null || true
}
trap cleanup EXIT

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

setsid timeout 15s ros2 run falcon_intercept target_kf_node >/tmp/falcon_health_fsm_kf.log 2>&1 &
KF_PID=$!
setsid timeout 15s ros2 run falcon_intercept intercept_fsm_node --ros-args \
  -p start_enabled:=true -p require_target_health:=true \
  -p max_search_rounds:=0 >/tmp/falcon_health_fsm_node.log 2>&1 &
FSM_PID=$!
sleep 1
python3 src/falcon_intercept/tools/target_kf_runtime_smoke.py
sleep 1
grep -q 'FSM_STATE TRACK' /tmp/falcon_health_fsm_node.log
grep -q 'FSM_STATE APPROACH' /tmp/falcon_health_fsm_node.log
if [[ "$(grep -c 'FSM_STATE SEARCH' /tmp/falcon_health_fsm_node.log)" -lt 2 ]]; then
  echo "TARGET_HEALTH_FSM_SEARCH_FALLBACK_FAIL"
  exit 1
fi
echo "TARGET_HEALTH_FSM_SMOKE_PASS acquire=PASS invalidation=PASS search_fallback=PASS"
echo "TARGET_HEALTH_FSM_LOG_TAIL"
tail -n 40 /tmp/falcon_health_fsm_kf.log
tail -n 40 /tmp/falcon_health_fsm_node.log
