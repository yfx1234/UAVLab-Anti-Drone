#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FALCON_WORKSPACE:-$HOME/Jetson_ROS2_V0/ros2_ws}"
LOG_PATH="${KF_SMOKE_LOG:-/tmp/falcon_target_kf.log}"
KF_PID=""

cleanup() {
  if [[ -n "$KF_PID" ]]; then
    kill -- "-$KF_PID" 2>/dev/null || true
    wait "$KF_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

setsid timeout 20s ros2 run falcon_intercept target_kf_node >"$LOG_PATH" 2>&1 &
KF_PID=$!
for _ in $(seq 1 30); do
  if ros2 topic list 2>/dev/null | grep -q '^/target/state_health$'; then
    break
  fi
  sleep 0.2
done
python3 src/falcon_intercept/tools/target_kf_runtime_smoke.py
echo "TARGET_KF_LOG_TAIL"
tail -n 80 "$LOG_PATH"
