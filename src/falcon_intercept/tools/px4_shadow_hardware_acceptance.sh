#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BRIDGE_SETUP="${BRIDGE_SETUP:-/home/uavlab/px4_bridge_ws/install/setup.bash}"
PROJECT_SETUP="${PROJECT_SETUP:-/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash}"
RESULT_ROOT="${RESULT_ROOT:-/home/uavlab/Jetson_ROS2_V0/test_results/px4_shadow_hardware}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
START_AGENT_IF_NEEDED="${START_AGENT_IF_NEEDED:-false}"
AGENT_DEVICE="${AGENT_DEVICE:-/dev/ttyTHS1}"
AGENT_BAUD="${AGENT_BAUD:-921600}"

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${BRIDGE_SETUP}"
# shellcheck disable=SC1090
source "${PROJECT_SETUP}"
export ROS_DOMAIN_ID
export ROS_LOCALHOST_ONLY=1
set -u

run_id="shadow-$(date +%Y%m%d-%H%M%S)"
result_dir="${RESULT_ROOT}/${run_id}"
mkdir -p "${result_dir}"
adapter_log="${result_dir}/adapter.log"
report="${result_dir}/report.txt"

adapter_pid=""
agent_pid=""
cleanup() {
  if [[ -n "${adapter_pid}" ]] && kill -0 -- "-${adapter_pid}" 2>/dev/null; then
    kill -INT -- "-${adapter_pid}" 2>/dev/null || true
    for _ in $(seq 1 20); do
      kill -0 -- "-${adapter_pid}" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 -- "-${adapter_pid}" 2>/dev/null; then
      kill -KILL -- "-${adapter_pid}" 2>/dev/null || true
    fi
    wait "${adapter_pid}" 2>/dev/null || true
  fi
  if [[ -n "${agent_pid}" ]] && kill -0 "${agent_pid}" 2>/dev/null; then
    kill -TERM "${agent_pid}" 2>/dev/null || true
    wait "${agent_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if ! ros2 topic list | grep -qx '/fmu/out/vehicle_status'; then
  if [[ "${START_AGENT_IF_NEEDED}" != "true" ]]; then
    echo "FAIL: /fmu/out/vehicle_status is unavailable; start the Agent and PX4 first" | tee "${report}"
    exit 1
  fi
  if fuser "${AGENT_DEVICE}" >/dev/null 2>&1; then
    echo "FAIL: ${AGENT_DEVICE} is occupied but PX4 topics are unavailable" | tee "${report}"
    exit 1
  fi
  MicroXRCEAgent serial --dev "${AGENT_DEVICE}" -b "${AGENT_BAUD}" \
    >"${result_dir}/agent.log" 2>&1 &
  agent_pid=$!
  for _ in $(seq 1 50); do
    if ros2 topic list 2>/dev/null | grep -qx '/fmu/out/vehicle_status'; then
      break
    fi
    sleep 0.2
  done
  if ! ros2 topic list | grep -qx '/fmu/out/vehicle_status'; then
    echo "FAIL: temporary Agent started but PX4 topics did not appear" | tee "${report}"
    exit 1
  fi
fi

setsid ros2 run falcon_intercept px4_adapter_node --ros-args \
  -p mode:=shadow \
  -p allow_live_control:=false \
  -p calibration_ready:=false \
  >"${adapter_log}" 2>&1 &
adapter_pid=$!

for _ in $(seq 1 30); do
  if ros2 node list 2>/dev/null | grep -qx '/px4_velocity_adapter'; then
    break
  fi
  sleep 0.2
done

if ! ros2 node list | grep -qx '/px4_velocity_adapter'; then
  echo "FAIL: px4_velocity_adapter did not start" | tee "${report}"
  exit 1
fi

{
  echo "RUN_ID=${run_id}"
  ros2 param get /px4_velocity_adapter mode
  ros2 param get /px4_velocity_adapter allow_live_control
  ros2 param get /px4_velocity_adapter calibration_ready
  ros2 node info /px4_velocity_adapter
} >"${report}"

for topic in \
  /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command
do
  count="$(ros2 topic info "${topic}" | awk '/Publisher count:/ {print $3}')"
  echo "${topic} publisher_count=${count:-unknown}" | tee -a "${report}"
  if [[ "${count}" != "0" ]]; then
    echo "FAIL: shadow acceptance found a publisher on ${topic}" | tee -a "${report}"
    exit 1
  fi
done

timeout 10 ros2 topic echo /control/px4_status --once \
  >"${result_dir}/px4_status.txt"
timeout 10 ros2 topic echo /intercept/px4_shadow --once \
  >"${result_dir}/shadow_output.txt"

if ! grep -q 'mode=shadow' "${adapter_log}"; then
  echo "FAIL: adapter did not report shadow mode" | tee -a "${report}"
  exit 1
fi

echo "PX4_SHADOW_HARDWARE_ACCEPTANCE=PASS" | tee -a "${report}"
echo "RESULT_DIR=${result_dir}" | tee -a "${report}"
