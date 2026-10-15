#!/usr/bin/env bash
set -eo pipefail

ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BRIDGE_SETUP="${BRIDGE_SETUP:-/home/uavlab/px4_bridge_ws/install/setup.bash}"
PROJECT_SETUP="${PROJECT_SETUP:-/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash}"
PX4_BUILD="${PX4_BUILD:-/home/uavlab/px4_sitl_staging/PX4-Autopilot/build/px4_sitl_default}"
PROBE="${PROBE:-/home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/sitl_live_probe.py}"
RESULT_ROOT="${RESULT_ROOT:-/home/uavlab/Jetson_ROS2_V0/test_results/px4_sitl_live}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-44}"
XRCE_PORT="${XRCE_PORT:-8891}"

# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${BRIDGE_SETUP}"
# shellcheck disable=SC1090
source "${PROJECT_SETUP}"
export ROS_DOMAIN_ID
set -u

run_id="sitl-live-$(date +%Y%m%d-%H%M%S)"
result_dir="${RESULT_ROOT}/${run_id}"
rootfs="${result_dir}/rootfs"
mkdir -p "${rootfs}"
report="${result_dir}/report.txt"

agent_pid=""
px4_pid=""
adapter_pid=""
command_pid=""
stop_pid() {
  local pid="$1"
  if [[ -z "${pid}" ]] || ! kill -0 "${pid}" 2>/dev/null; then
    return
  fi
  kill -INT "${pid}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 "${pid}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "${pid}" 2>/dev/null; then
    kill -KILL "${pid}" 2>/dev/null || true
  fi
  wait "${pid}" 2>/dev/null || true
}
stop_process_group() {
  local leader="$1"
  if [[ -z "${leader}" ]] || ! kill -0 -- "-${leader}" 2>/dev/null; then
    return
  fi
  kill -INT -- "-${leader}" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-${leader}" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 -- "-${leader}" 2>/dev/null; then
    kill -KILL -- "-${leader}" 2>/dev/null || true
  fi
  wait "${leader}" 2>/dev/null || true
}
cleanup() {
  stop_pid "${command_pid}"
  if [[ -x "${PX4_BUILD}/bin/px4-commander" ]]; then
    (cd "${rootfs}" && "${PX4_BUILD}/bin/px4-commander" --instance 0 disarm) \
      >>"${result_dir}/commander.log" 2>&1 || true
  fi
  if [[ -x "${PX4_BUILD}/bin/px4-shutdown" ]]; then
    "${PX4_BUILD}/bin/px4-shutdown" --instance 0 \
      >>"${result_dir}/shutdown.log" 2>&1 || true
  fi
  stop_process_group "${adapter_pid}"
  stop_pid "${px4_pid}"
  stop_pid "${agent_pid}"
  pkill -TERM -f "^MicroXRCEAgent udp4 -p ${XRCE_PORT}$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

px4_command() {
  (cd "${rootfs}" && "${PX4_BUILD}/bin/px4-commander" --instance 0 "$@") \
    >>"${result_dir}/commander.log" 2>&1
}

"${PX4_BUILD}/bin/px4-shutdown" --instance 0 \
  >"${result_dir}/preclean.log" 2>&1 || true
pkill -TERM -f "^MicroXRCEAgent udp4 -p ${XRCE_PORT}$" 2>/dev/null || true
sleep 1

MicroXRCEAgent udp4 -p "${XRCE_PORT}" >"${result_dir}/agent.log" 2>&1 &
agent_pid=$!
(
  cd "${rootfs}"
  export PX4_SYS_AUTOSTART=10040
  export PX4_UXRCE_DDS_PORT="${XRCE_PORT}"
  export ROS_DOMAIN_ID
  unset PX4_SIM_MODEL
  exec "${PX4_BUILD}/bin/px4" -i 0 -d "${PX4_BUILD}/etc"
) >"${result_dir}/px4.log" 2>&1 &
px4_pid=$!

for _ in $(seq 1 150); do
  if ros2 topic list 2>/dev/null | grep -qx '/fmu/out/vehicle_status'; then
    break
  fi
  sleep 0.2
done
if ! ros2 topic list | grep -qx '/fmu/out/vehicle_status'; then
  echo "FAIL: SITL topics unavailable" | tee "${report}"
  exit 1
fi

setsid ros2 run falcon_intercept px4_adapter_node --ros-args \
  -p mode:=live \
  -p allow_live_control:=true \
  -p calibration_ready:=true \
  >"${result_dir}/adapter.log" 2>&1 &
adapter_pid=$!
for _ in $(seq 1 50); do
  ros2 node list 2>/dev/null | grep -qx '/px4_velocity_adapter' && break
  sleep 0.2
done
ros2 node list | grep -qx '/px4_velocity_adapter'

{
  echo "RUN_ID=${run_id}"
  echo "SIMULATOR=PX4_SIH_QUADX"
  echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
  echo "XRCE_PORT=${XRCE_PORT}"
  ros2 param get /px4_velocity_adapter mode
  ros2 param get /px4_velocity_adapter allow_live_control
  ros2 param get /px4_velocity_adapter calibration_ready
} >"${report}"

# These are explicit simulated-operator actions. The production adapter never
# issues arm, takeoff, or mode-switch commands.
sleep 10
armed=false
for _ in $(seq 1 10); do
  px4_command arm || true
  if python3 "${PROBE}" wait armed --timeout 2 \
      >>"${result_dir}/arm_wait.jsonl"; then
    armed=true
    break
  fi
done
if [[ "${armed}" != "true" ]]; then
  echo "FAIL: simulated operator could not arm SIH" | tee -a "${report}"
  exit 1
fi
px4_command takeoff
python3 "${PROBE}" wait prestreaming --timeout 30 \
  | tee "${result_dir}/prestreaming.json"

python3 "${PROBE}" command --duration 12 --vx 0.10 \
  >"${result_dir}/guidance_probe.log" 2>&1 &
command_pid=$!
sleep 1
px4_command mode offboard
python3 "${PROBE}" wait ready --timeout 20 \
  | tee "${result_dir}/ready.json"

timeout 10 ros2 topic echo /fmu/in/trajectory_setpoint px4_msgs/msg/TrajectorySetpoint \
  --once --qos-reliability best_effort --qos-durability transient_local \
  --qos-history keep_last --qos-depth 1 >"${result_dir}/trajectory_setpoint.txt"
grep -Eq '(^|[[:space:]-])0\.0?9[0-9]*|(^|[[:space:]-])0\.1([0-9]*)?$' \
  "${result_dir}/trajectory_setpoint.txt"
sleep 2
python3 "${PROBE}" wait ready --timeout 5 \
  | tee "${result_dir}/ready_after_motion.json"

stop_pid "${command_pid}"
command_pid=""
python3 "${PROBE}" command --duration 1.5 --land-request \
  >"${result_dir}/land_probe.log" 2>&1
python3 "${PROBE}" wait landed --timeout 35 \
  | tee "${result_dir}/landed.json"
px4_command disarm || true

grep -q 'PX4 native LAND requested once' "${result_dir}/adapter.log"
grep -q 'PX4 LAND command ack result=' "${result_dir}/adapter.log"
echo "PX4_SITL_LIVE_ACCEPTANCE=PASS" | tee -a "${report}"
echo "RESULT_DIR=${result_dir}" | tee -a "${report}"
