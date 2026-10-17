#!/usr/bin/env bash
# Board-side test for the consolidated laptop-A architecture:
#   - enemy logger must lock the AUTOPILOT sysid even when sysid=255 GCS
#     heartbeats pollute the stream (MAVProxy endpoint scenario);
#   - a second logger instance (interceptor relay path) must lock its own
#     aircraft sysid independently.
# Uses fake_mavlink_relay only; no radios or aircraft needed.
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
TOOLS_DIR="${WORKSPACE_DIR}/src/falcon_intercept/tools/rtk_field_test"
OUT=/tmp/rtk_dual_link
rm -rf "${OUT}" && mkdir -p "${OUT}"

PIDS=()
cleanup() { kill -INT "${PIDS[@]}" 2>/dev/null || true; sleep 1; kill -KILL "${PIDS[@]}" 2>/dev/null || true; }
trap cleanup EXIT

# enemy logger (auto lock) + polluted enemy relay
python3 "${TOOLS_DIR}/rtk_truth_logger.py" --listen 0.0.0.0:14560 \
  --output "${OUT}/enemy.jsonl" --no-request >"${OUT}/enemy_logger.log" 2>&1 &
PIDS+=($!)
# interceptor relay logger (auto lock, no requests, mimics --own-relay)
python3 "${TOOLS_DIR}/rtk_truth_logger.py" --listen 0.0.0.0:14561 \
  --output "${OUT}/own_relay.jsonl" --no-request >"${OUT}/own_logger.log" 2>&1 &
PIDS+=($!)
sleep 2
python3 "${TOOLS_DIR}/fake_mavlink_relay.py" --target 127.0.0.1:14560 \
  --sysid 42 --gcs-heartbeat >"${OUT}/relay_enemy.log" 2>&1 &
PIDS+=($!)
python3 "${TOOLS_DIR}/fake_mavlink_relay.py" --target 127.0.0.1:14561 \
  --sysid 7 --radius-m 15 --speed-mps 3 >"${OUT}/relay_own.log" 2>&1 &
PIDS+=($!)
sleep 8

FAIL=0
check_lock() {
  local log="$1" expect="$2" name="$3"
  if grep -q "locked sysid=${expect}" "${log}"; then
    echo "PASS: ${name} locked sysid=${expect}"
  else
    echo "FAIL: ${name} did not lock sysid=${expect}"; FAIL=1
  fi
}
check_lock "${OUT}/enemy_logger.log" 42 enemy
check_lock "${OUT}/own_logger.log" 7 own-relay

ENEMY_GPS=$(grep -c '"kind": "gps_raw_int"' "${OUT}/enemy.jsonl" || true)
OWN_GPS=$(grep -c '"kind": "gps_raw_int"' "${OUT}/own_relay.jsonl" || true)
echo "enemy gps_raw_int records: ${ENEMY_GPS}"
echo "own relay gps_raw_int records: ${OWN_GPS}"
[[ "${ENEMY_GPS}" -gt 30 ]] || { echo "FAIL: enemy gps records too few"; FAIL=1; }
[[ "${OWN_GPS}" -gt 30 ]] || { echo "FAIL: own relay gps records too few"; FAIL=1; }

if grep -q "locked sysid=255" "${OUT}/enemy_logger.log" "${OUT}/own_logger.log"; then
  echo "FAIL: a logger locked GCS sysid=255"; FAIL=1
fi

if [[ "${FAIL}" == "0" ]]; then
  echo "DUAL_LINK_TEST_PASS"
else
  echo "DUAL_LINK_TEST_FAIL"
fi
exit "${FAIL}"
