#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:-/home/uavlab/Jetson_ROS2_V0/test_results/px4_shadow_axis/axis-$(date +%Y%m%d-%H%M%S)}"
mkdir -p "$OUTPUT_DIR"

set +u
source /opt/ros/humble/setup.bash
source /home/uavlab/px4_bridge_ws/install/setup.bash
source /home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

ADAPTER_PID=""
cleanup() {
  if [[ -n "$ADAPTER_PID" ]] && kill -0 -- "-$ADAPTER_PID" 2>/dev/null; then
    kill -INT -- "-$ADAPTER_PID" 2>/dev/null || true
    wait "$ADAPTER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

setsid ros2 run falcon_intercept px4_adapter_node --ros-args \
  -p mode:=shadow \
  -p allow_live_control:=false \
  -p calibration_ready:=false \
  >"$OUTPUT_DIR/adapter.log" 2>&1 &
ADAPTER_PID=$!

for _ in $(seq 1 30); do
  grep -q 'PX4 velocity adapter ready mode=shadow' "$OUTPUT_DIR/adapter.log" \
    2>/dev/null && break
  sleep 0.2
done
if ! kill -0 "$ADAPTER_PID" 2>/dev/null; then
  echo "shadow adapter exited during startup" >&2
  exit 1
fi

/usr/bin/python3 \
  /home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/px4_shadow_axis_probe.py \
  --json-output "$OUTPUT_DIR/axis_probe.json"

for topic in \
  /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command
do
  count="$(ros2 topic info "$topic" | awk '/Publisher count:/ {print $3}')"
  echo "$topic publisher_count=${count:-unknown}" | tee -a "$OUTPUT_DIR/input_ownership.txt"
  [[ "$count" == "0" ]]
done

cleanup
trap - EXIT INT TERM
echo "PX4_SHADOW_AXIS_HARDWARE_PROBE=PASS output=$OUTPUT_DIR"
