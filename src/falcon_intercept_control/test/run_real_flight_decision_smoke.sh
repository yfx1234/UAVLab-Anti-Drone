#!/usr/bin/env bash
set -eo pipefail

cd "$(dirname "$0")/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

launch_pid=""
cleanup() {
  if [[ -n "$launch_pid" ]]; then
    kill "$launch_pid" 2>/dev/null || true
    wait "$launch_pid" 2>/dev/null || true
  fi
}
trap cleanup EXIT

run_case() {
  local name="$1"
  shift
  local log="/tmp/falcon_${name}.log"
  timeout 12s ros2 launch falcon_intercept_control real_flight_decision_smoke.launch.py \
    >"$log" 2>&1 &
  launch_pid=$!
  sleep 2
  "$@"
  cleanup
  launch_pid=""
}

run_case happy python3 src/falcon_intercept_control/test/real_flight_decision_smoke.py
run_case timeout python3 src/falcon_intercept_control/test/real_flight_fault_smoke.py track_timeout
run_case takeover python3 src/falcon_intercept_control/test/real_flight_fault_smoke.py takeover_reset
run_case search_timeout python3 src/falcon_intercept_control/test/real_flight_fault_smoke.py search_timeout
