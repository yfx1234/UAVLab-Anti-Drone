#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
RUN_SECONDS="${RUN_SECONDS:-1800}"
RESULT_ROOT="${RESULT_ROOT:-/home/uavlab/Jetson_ROS2_V0/test_results/control_shadow_stability}"
MODEL_ENGINE_PATH="${MODEL_ENGINE_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine}"
MODEL_ONNX_PATH="${MODEL_ONNX_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16.onnx}"
NETWORK_MODE="${NETWORK_MODE:-2}"
MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
INFERENCE_INTERVAL="${INFERENCE_INTERVAL:-0}"
CSI_FPS="${CSI_FPS:-25}"
MIN_MEAN_FPS="${MIN_MEAN_FPS:-24}"
OC3_LIMIT="${OC3_LIMIT:-30}"
RUN_ID="control-shadow-$(date +%Y%m%d-%H%M%S)"
RESULT_DIR="${RESULT_ROOT}/${RUN_ID}"
DEEPSTREAM_LOG="${RESULT_DIR}/deepstream.log"
CONTROL_LOG="${RESULT_DIR}/control_shadow.log"
TEGRA_LOG="${RESULT_DIR}/tegrastats.log"
META_FILE="${RESULT_DIR}/meta.txt"
OC_ROOT="/sys/devices/platform/soctherm-oc-event/hwmon/hwmon4"

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON=1

mkdir -p "${RESULT_DIR}"
if pgrep -af 'deepstream_three_camera_node|intercept_decision_node|px4_adapter_node' >"${RESULT_DIR}/preflight_processes.txt"; then
  echo "STABILITY_FAIL conflicting process already running" | tee -a "${META_FILE}"
  cat "${RESULT_DIR}/preflight_processes.txt"
  exit 2
fi

power_mode="$(nvpmodel -q 2>/dev/null | tr '\n' ' ' || true)"
oc1_before="$(cat "${OC_ROOT}/oc1_event_cnt")"
oc2_before="$(cat "${OC_ROOT}/oc2_event_cnt")"
oc3_before="$(cat "${OC_ROOT}/oc3_event_cnt")"
{
  echo "run_id=${RUN_ID}"
  echo "start_time=$(date --iso-8601=seconds)"
  echo "run_seconds=${RUN_SECONDS}"
  echo "inference_interval=${INFERENCE_INTERVAL}"
  echo "csi_fps=${CSI_FPS}"
  echo "min_mean_fps=${MIN_MEAN_FPS}"
  echo "oc3_limit=${OC3_LIMIT}"
  echo "power_mode=${power_mode}"
  echo "model_engine=${MODEL_ENGINE_PATH}"
  echo "network_mode=${NETWORK_MODE}"
  echo "model_output_format=${MODEL_OUTPUT_FORMAT}"
  sha256sum "${MODEL_ENGINE_PATH}"
  echo "oc1_before=${oc1_before}"
  echo "oc2_before=${oc2_before}"
  echo "oc3_before=${oc3_before}"
} >"${META_FILE}"

CONTROL_PID=""
DEEPSTREAM_PID=""
TEGRA_PID=""
FINAL_METRICS_RECORDED=false
record_final_metrics() {
  if [[ "${FINAL_METRICS_RECORDED}" == true ]]; then
    return
  fi
  local oc1_after oc2_after oc3_after
  oc1_after="$(cat "${OC_ROOT}/oc1_event_cnt")"
  oc2_after="$(cat "${OC_ROOT}/oc2_event_cnt")"
  oc3_after="$(cat "${OC_ROOT}/oc3_event_cnt")"
  {
    echo "end_time=$(date --iso-8601=seconds)"
    echo "oc1_after=${oc1_after}"
    echo "oc2_after=${oc2_after}"
    echo "oc3_after=${oc3_after}"
    echo "oc1_delta=$((oc1_after - oc1_before))"
    echo "oc2_delta=$((oc2_after - oc2_before))"
    echo "oc3_delta=$((oc3_after - oc3_before))"
  } >>"${META_FILE}"
  FINAL_METRICS_RECORDED=true
}
cleanup() {
  set +e
  record_final_metrics
  for pid in "${DEEPSTREAM_PID}" "${CONTROL_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill -INT -- "-${pid}" 2>/dev/null || true
    fi
  done
  sleep 2
  for pid in "${DEEPSTREAM_PID}" "${CONTROL_PID}"; do
    if [[ -n "${pid}" ]]; then
      kill -TERM -- "-${pid}" 2>/dev/null || true
      wait "${pid}" 2>/dev/null || true
    fi
  done
  if [[ -n "${TEGRA_PID}" ]]; then
    kill "${TEGRA_PID}" 2>/dev/null || true
    wait "${TEGRA_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

tegrastats --interval 1000 --logfile "${TEGRA_LOG}" &
TEGRA_PID=$!

setsid ros2 launch falcon_intercept full_v0.launch.py adapter_mode:=shadow \
  >"${CONTROL_LOG}" 2>&1 &
CONTROL_PID=$!

setsid ros2 run falcon_intercept_deepstream deepstream_three_camera_node --ros-args \
  -p "model_engine_path:=${MODEL_ENGINE_PATH}" \
  -p "model_onnx_path:=${MODEL_ONNX_PATH}" \
  -p "class_names:=drone" \
  -p "num_detected_classes:=1" \
  -p "batch_size:=1" \
  -p "source_mode:=all" \
  -p "mux_width:=1280" \
  -p "mux_height:=720" \
  -p "workspace_size_mb:=0" \
  -p "network_mode:=${NETWORK_MODE}" \
  -p "model_output_format:=${MODEL_OUTPUT_FORMAT}" \
  -p "inference_interval:=${INFERENCE_INTERVAL}" \
  -p "csi_fps:=${CSI_FPS}" >"${DEEPSTREAM_LOG}" 2>&1 &
DEEPSTREAM_PID=$!

startup_ok=false
startup_good_windows=0
startup_last_line=""
for ((second = 0; second < 90; second++)); do
  stats_line="$(grep 'deepstream stats' "${DEEPSTREAM_LOG}" 2>/dev/null | tail -n 1 || true)"
  if [[ -n "${stats_line}" && "${stats_line}" != "${startup_last_line}" ]]; then
    startup_last_line="${stats_line}"
    read -r left_fps right_fps search_fps < <(
      sed -E 's/.*left_fps=([0-9.]+).*right_fps=([0-9.]+).*search_fps=([0-9.]+).*/\1 \2 \3/' <<<"${stats_line}"
    )
    if awk -v l="${left_fps}" -v r="${right_fps}" -v s="${search_fps}" \
      'BEGIN { exit !(l >= 20.0 && r >= 20.0 && s >= 20.0) }'; then
      startup_good_windows=$((startup_good_windows + 1))
      if (( startup_good_windows >= 3 )); then
        startup_ok=true
        break
      fi
    else
      startup_good_windows=0
    fi
  fi
  if ! kill -0 "${DEEPSTREAM_PID}" 2>/dev/null || ! kill -0 "${CONTROL_PID}" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ "${startup_ok}" != true ]]; then
  echo "STABILITY_FAIL startup_three_source_fps last='${startup_last_line}'" | tee -a "${META_FILE}"
  tail -n 80 "${DEEPSTREAM_LOG}" || true
  tail -n 80 "${CONTROL_LOG}" || true
  exit 1
fi

echo "STABILITY_STARTED result_dir=${RESULT_DIR} run_seconds=${RUN_SECONDS}"
runtime_last_line=""
zero_fps_windows=0
for ((elapsed = 0; elapsed < RUN_SECONDS; elapsed++)); do
  if ! kill -0 "${DEEPSTREAM_PID}" 2>/dev/null; then
    echo "STABILITY_FAIL deepstream exited elapsed=${elapsed}" | tee -a "${META_FILE}"
    exit 1
  fi
  if ! kill -0 "${CONTROL_PID}" 2>/dev/null; then
    echo "STABILITY_FAIL control exited elapsed=${elapsed}" | tee -a "${META_FILE}"
    exit 1
  fi
  oc3_now="$(cat "${OC_ROOT}/oc3_event_cnt")"
  if (( oc3_now - oc3_before > OC3_LIMIT )); then
    echo "STABILITY_FAIL oc3_delta=$((oc3_now - oc3_before)) limit=${OC3_LIMIT} elapsed=${elapsed}" | tee -a "${META_FILE}"
    exit 1
  fi
  stats_line="$(grep 'deepstream stats' "${DEEPSTREAM_LOG}" 2>/dev/null | tail -n 1 || true)"
  if [[ -n "${stats_line}" && "${stats_line}" != "${runtime_last_line}" ]]; then
    runtime_last_line="${stats_line}"
    read -r left_fps right_fps search_fps < <(
      sed -E 's/.*left_fps=([0-9.]+).*right_fps=([0-9.]+).*search_fps=([0-9.]+).*/\1 \2 \3/' <<<"${stats_line}"
    )
    if awk -v l="${left_fps}" -v r="${right_fps}" -v s="${search_fps}" \
      'BEGIN { exit !(l < 1.0 || r < 1.0 || s < 1.0) }'; then
      zero_fps_windows=$((zero_fps_windows + 1))
      if (( zero_fps_windows >= 3 )); then
        echo "STABILITY_FAIL source_stalled elapsed=${elapsed} stats='${stats_line}'" | tee -a "${META_FILE}"
        exit 1
      fi
    else
      zero_fps_windows=0
    fi
  fi
  if (( elapsed > 0 && elapsed % 60 == 0 )); then
    stats_count="$(grep -c 'deepstream stats' "${DEEPSTREAM_LOG}" || true)"
    echo "STABILITY_PROGRESS elapsed=${elapsed} stats=${stats_count} oc3=$(cat "${OC_ROOT}/oc3_event_cnt")"
  fi
  sleep 1
done

record_final_metrics
oc1_after="$(cat "${OC_ROOT}/oc1_event_cnt")"
oc2_after="$(cat "${OC_ROOT}/oc2_event_cnt")"
oc3_after="$(cat "${OC_ROOT}/oc3_event_cnt")"

python3 src/falcon_intercept_deepstream/tools/summarize_deepstream_fps.py \
  --log "${DEEPSTREAM_LOG}" --source front --skip 3 --min-samples 10 --min-mean "${MIN_MEAN_FPS}" | tee -a "${META_FILE}"
python3 src/falcon_intercept_deepstream/tools/summarize_deepstream_fps.py \
  --log "${DEEPSTREAM_LOG}" --source back --skip 3 --min-samples 10 --min-mean "${MIN_MEAN_FPS}" | tee -a "${META_FILE}"
python3 src/falcon_intercept_deepstream/tools/summarize_deepstream_fps.py \
  --log "${DEEPSTREAM_LOG}" --source search --skip 3 --min-samples 10 --min-mean "${MIN_MEAN_FPS}" | tee -a "${META_FILE}"

errors="$(cat "${DEEPSTREAM_LOG}" "${CONTROL_LOG}" | \
  grep -Ec 'DeepStream error|startup failed|ERROR|Failed to|Traceback' || true)"
echo "error_matches=${errors}" >>"${META_FILE}"
if (( errors > 0 )); then
  echo "STABILITY_FAIL runtime errors=${errors}" | tee -a "${META_FILE}"
  exit 1
fi
if (( oc3_after - oc3_before > OC3_LIMIT )); then
  echo "STABILITY_FAIL oc3_delta=$((oc3_after - oc3_before)) limit=${OC3_LIMIT}" | tee -a "${META_FILE}"
  exit 1
fi

echo "STABILITY_PASS result_dir=${RESULT_DIR}" | tee -a "${META_FILE}"
