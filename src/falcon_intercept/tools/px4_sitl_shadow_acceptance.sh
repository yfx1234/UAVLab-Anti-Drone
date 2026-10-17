#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BRIDGE_SETUP="${BRIDGE_SETUP:-/home/uavlab/px4_bridge_ws/install/setup.bash}"
PROJECT_SETUP="${PROJECT_SETUP:-/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash}"
PX4_BUILD="${PX4_BUILD:-/home/uavlab/px4_sitl_staging/PX4-Autopilot/build/px4_sitl_default}"
RESULT_ROOT="${RESULT_ROOT:-/home/uavlab/Jetson_ROS2_V0/test_results/px4_sitl_shadow}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
XRCE_PORT="${XRCE_PORT:-8890}"

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${BRIDGE_SETUP}"
# shellcheck disable=SC1090
source "${PROJECT_SETUP}"
export ROS_DOMAIN_ID
set -u

run_id="sitl-shadow-$(date +%Y%m%d-%H%M%S)"
result_dir="${RESULT_ROOT}/${run_id}"
mkdir -p "${result_dir}/rootfs"
report="${result_dir}/report.txt"

agent_pid=""
px4_pid=""
adapter_pid=""
cleanup() {
  if [[ -x "${PX4_BUILD}/bin/px4-shutdown" ]]; then
    "${PX4_BUILD}/bin/px4-shutdown" --instance 0 \
      >>"${result_dir}/shutdown.log" 2>&1 || true
  fi
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
  for pid in "${px4_pid}" "${agent_pid}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
      kill -INT "${pid}" 2>/dev/null || true
      for _ in $(seq 1 20); do
        kill -0 "${pid}" 2>/dev/null || break
        sleep 0.1
      done
      if kill -0 "${pid}" 2>/dev/null; then
        kill -KILL "${pid}" 2>/dev/null || true
      fi
      wait "${pid}" 2>/dev/null || true
    fi
  done
  pkill -TERM -f "^MicroXRCEAgent udp4 -p ${XRCE_PORT}$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ ! -x "${PX4_BUILD}/bin/px4" ]]; then
  echo "FAIL: missing ${PX4_BUILD}/bin/px4" | tee "${report}"
  exit 1
fi

"${PX4_BUILD}/bin/px4-shutdown" --instance 0 \
  >"${result_dir}/preclean.log" 2>&1 || true
pkill -TERM -f "^MicroXRCEAgent udp4 -p ${XRCE_PORT}$" 2>/dev/null || true
sleep 1

MicroXRCEAgent udp4 -p "${XRCE_PORT}" >"${result_dir}/agent.log" 2>&1 &
agent_pid=$!

(
  cd "${result_dir}/rootfs"
  # PX4's built-in SIH quadrotor simulator requires no Gazebo installation.
  export PX4_SYS_AUTOSTART=10040
  unset PX4_SIM_MODEL
  export PX4_UXRCE_DDS_PORT="${XRCE_PORT}"
  export ROS_DOMAIN_ID
  exec "${PX4_BUILD}/bin/px4" -i 0 -d "${PX4_BUILD}/etc"
) >"${result_dir}/px4.log" 2>&1 &
px4_pid=$!

for _ in $(seq 1 100); do
  if ros2 topic list 2>/dev/null | grep -qx '/fmu/out/vehicle_status'; then
    break
  fi
  sleep 0.2
done
if ! ros2 topic list | grep -qx '/fmu/out/vehicle_status'; then
  echo "FAIL: SITL did not expose /fmu/out/vehicle_status" | tee "${report}"
  exit 1
fi

setsid ros2 run falcon_intercept px4_adapter_node --ros-args \
  -p mode:=shadow \
  -p allow_live_control:=false \
  -p calibration_ready:=false \
  >"${result_dir}/adapter.log" 2>&1 &
adapter_pid=$!

for _ in $(seq 1 50); do
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
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "XRCE_PORT=${XRCE_PORT}"
  "${PX4_BUILD}/bin/px4" --version 2>/dev/null || true
  ros2 param get /px4_velocity_adapter mode
  ros2 param get /px4_velocity_adapter allow_live_control
  ros2 param get /px4_velocity_adapter calibration_ready
} >"${report}"

for topic in \
  /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command
do
  count="$(ros2 topic info "${topic}" | awk '/Publisher count:/ {print $3}')"
  echo "${topic} publisher_count=${count:-unknown}" | tee -a "${report}"
  if [[ "${count}" != "0" ]]; then
    echo "FAIL: shadow mode published to ${topic}" | tee -a "${report}"
    exit 1
  fi
done

timeout 15 ros2 topic echo /fmu/out/vehicle_status px4_msgs/msg/VehicleStatus \
  --once --qos-reliability best_effort --qos-durability transient_local \
  --qos-history keep_last --qos-depth 1 >"${result_dir}/vehicle_status.txt"
timeout 15 ros2 topic echo /control/px4_status --once \
  >"${result_dir}/px4_status.txt"
timeout 15 ros2 topic echo /intercept/px4_shadow --once \
  >"${result_dir}/shadow_output.txt"

grep -q 'mode=shadow' "${result_dir}/adapter.log"
grep -Eq 'session established|create_client.*client_key' "${result_dir}/agent.log"

echo "PX4_SITL_SHADOW_ACCEPTANCE=PASS" | tee -a "${report}"
echo "RESULT_DIR=${result_dir}" | tee -a "${report}"
