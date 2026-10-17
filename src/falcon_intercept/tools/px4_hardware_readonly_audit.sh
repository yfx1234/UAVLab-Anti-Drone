#!/usr/bin/env bash
set -euo pipefail

RESULT_ROOT="${1:-/home/uavlab/Jetson_ROS2_V0/test_results/px4_readonly_audit}"
RUN_ID="audit-$(date +%Y%m%d-%H%M%S)"
OUTPUT_DIR="${RESULT_ROOT}/${RUN_ID}"
mkdir -p "$OUTPUT_DIR/topics"

set +u
source /opt/ros/humble/setup.bash
source /home/uavlab/px4_bridge_ws/install/setup.bash
source /home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash
set -u
export ROS_LOCALHOST_ONLY=1

capture_topic() {
  local topic=$1
  local label=${topic#/fmu/out/}
  if ros2 topic list | grep -qx "$topic"; then
    timeout 8s ros2 topic echo "$topic" --once \
      --qos-reliability best_effort \
      --qos-durability transient_local \
      --qos-history keep_last \
      --qos-depth 1 \
      >"$OUTPUT_DIR/topics/${label}.txt" 2>&1 || true
  else
    printf 'MISSING: %s\n' "$topic" >"$OUTPUT_DIR/topics/${label}.txt"
  fi
}

{
  echo "run_id=$RUN_ID"
  echo "collected=$(date --iso-8601=seconds)"
  echo "hostname=$(hostname)"
  echo "agent_enabled=$(systemctl is-enabled micro-xrce-agent.service 2>/dev/null || true)"
  echo "agent_active=$(systemctl is-active micro-xrce-agent.service 2>/dev/null || true)"
  systemctl show micro-xrce-agent.service \
    -p NRestarts -p MainPID -p ActiveEnterTimestamp --no-pager 2>/dev/null || true
  echo
  echo "[serial]"
  ls -l /dev/ttyTHS1 /dev/serial/by-id 2>/dev/null || true
  echo
  echo "[fmu topics]"
  ros2 topic list | grep '^/fmu/' | sort || true
} >"$OUTPUT_DIR/system_report.txt"

for topic in \
  /fmu/out/vehicle_status \
  /fmu/out/vehicle_control_mode \
  /fmu/out/vehicle_local_position \
  /fmu/out/vehicle_gps_position \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_attitude \
  /fmu/out/estimator_status_flags \
  /fmu/out/failsafe_flags \
  /fmu/out/manual_control_setpoint \
  /fmu/out/timesync_status
do
  capture_topic "$topic"
done

{
  for topic in \
    /fmu/in/offboard_control_mode \
    /fmu/in/trajectory_setpoint \
    /fmu/in/vehicle_command
  do
    echo "[$topic]"
    ros2 topic info "$topic" 2>&1 || true
  done
} >"$OUTPUT_DIR/input_topic_ownership.txt"

echo "PX4_READONLY_AUDIT_COMPLETE output=$OUTPUT_DIR"
