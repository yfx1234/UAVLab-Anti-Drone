#!/usr/bin/env bash
set -Eeuo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/home/uavlab/Jetson_ROS2_V0/flight_yolo_records}"
MODEL_PROFILE="${MODEL_PROFILE:-fp16_0728_rect}"
DURATION_SEC="${DURATION_SEC:-0}"
CONFIDENCE="${CONFIDENCE:-0.50}"
RECORD_BITRATE="${RECORD_BITRATE:-4000000}"
RECORD_WIDTH="${RECORD_WIDTH:-960}"
RECORD_HEIGHT="${RECORD_HEIGHT:-540}"
RECORD_SEGMENT_SECONDS="${RECORD_SEGMENT_SECONDS:-5}"
SEARCH_SUN_FILTER="${SEARCH_SUN_FILTER:-true}"
SEARCH_EXPOSURE_TIME="${SEARCH_EXPOSURE_TIME:-5}"
SEARCH_GAIN="${SEARCH_GAIN:-0}"
SEARCH_BRIGHTNESS="${SEARCH_BRIGHTNESS:--8}"
SEARCH_BACKLIGHT_COMPENSATION="${SEARCH_BACKLIGHT_COMPENSATION:-0}"

case "${MODEL_PROFILE}" in
  fp16_0728_rect|latest)
    MODEL_ENGINE_PATH="/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine"
    MODEL_ONNX_PATH="/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16.onnx"
    NETWORK_MODE=2
    MODEL_OUTPUT_FORMAT=end_to_end
    ;;
  int8)
    MODEL_ENGINE_PATH="/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280_int8_opt3.engine"
    MODEL_ONNX_PATH="/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280_int8.onnx"
    NETWORK_MODE=1
    MODEL_OUTPUT_FORMAT=raw
    ;;
  fp16)
    MODEL_ENGINE_PATH="/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280_opt3.engine"
    MODEL_ONNX_PATH="/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280.onnx"
    NETWORK_MODE=2
    MODEL_OUTPUT_FORMAT=end_to_end
    ;;
  fp16_rect)
    MODEL_ENGINE_PATH="/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280x736_fp16_opt3.engine"
    MODEL_ONNX_PATH="/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280x736_fp16.onnx"
    NETWORK_MODE=2
    MODEL_OUTPUT_FORMAT=end_to_end
    ;;
  *)
    echo "MODEL_PROFILE must be fp16_0728_rect/latest, int8, fp16 or fp16_rect" >&2
    exit 2
    ;;
esac

for path in "${MODEL_ENGINE_PATH}" "${MODEL_ONNX_PATH}"; do
  if [[ ! -f "${path}" ]]; then
    echo "Missing model asset: ${path}" >&2
    exit 1
  fi
done
if ! [[ "${DURATION_SEC}" =~ ^[0-9]+$ ]]; then
  echo "DURATION_SEC must be a non-negative integer" >&2
  exit 2
fi
if ! [[ "${SEARCH_EXPOSURE_TIME}" =~ ^[0-9]+$ ]]; then
  echo "SEARCH_EXPOSURE_TIME must be a non-negative integer" >&2
  exit 2
fi
if ! [[ "${SEARCH_GAIN}" =~ ^-?[0-9]+$ ]]; then
  echo "SEARCH_GAIN must be an integer" >&2
  exit 2
fi
if ! [[ "${SEARCH_BRIGHTNESS}" =~ ^-?[0-9]+$ ]]; then
  echo "SEARCH_BRIGHTNESS must be an integer" >&2
  exit 2
fi
if ! [[ "${SEARCH_BACKLIGHT_COMPENSATION}" =~ ^-?[0-9]+$ ]]; then
  echo "SEARCH_BACKLIGHT_COMPENSATION must be an integer" >&2
  exit 2
fi
if pgrep -af 'deepstream_three_camera_node' >/dev/null; then
  echo "A DeepStream three-camera process is already running; stop it before recording." >&2
  exit 2
fi

RUN_ID="flight-yolo-${MODEL_PROFILE}-$(date +%Y%m%d-%H%M%S)"
OUTPUT_DIR="${OUTPUT_ROOT}/${RUN_ID}"
mkdir -p "${OUTPUT_DIR}/ros_logs"

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON=1
export ROS_LOG_DIR="${OUTPUT_DIR}/ros_logs"

{
  echo "run_id=${RUN_ID}"
  echo "start_time=$(date --iso-8601=seconds)"
  echo "model_profile=${MODEL_PROFILE}"
  echo "model_engine=${MODEL_ENGINE_PATH}"
  echo "model_onnx=${MODEL_ONNX_PATH}"
  echo "network_mode=${NETWORK_MODE}"
  echo "model_output_format=${MODEL_OUTPUT_FORMAT}"
  echo "confidence=${CONFIDENCE}"
  echo "record_codec=h264"
  echo "record_bitrate=${RECORD_BITRATE}"
  echo "record_width=${RECORD_WIDTH}"
  echo "record_height=${RECORD_HEIGHT}"
  echo "record_segment_seconds=${RECORD_SEGMENT_SECONDS}"
  echo "duration_sec=${DURATION_SEC}"
  echo "search_sun_filter=${SEARCH_SUN_FILTER}"
  echo "search_exposure_time=${SEARCH_EXPOSURE_TIME}"
  echo "search_gain=${SEARCH_GAIN}"
  echo "search_brightness=${SEARCH_BRIGHTNESS}"
  echo "search_backlight_compensation=${SEARCH_BACKLIGHT_COMPENSATION}"
  sha256sum "${MODEL_ENGINE_PATH}" "${MODEL_ONNX_PATH}"
} >"${OUTPUT_DIR}/manifest.txt"

STATS_TOOL="${WORKSPACE_DIR}/install/falcon_intercept_deepstream"
STATS_TOOL+="/lib/falcon_intercept_deepstream/video_detection_stats.py"
"${STATS_TOOL}" --output-dir "${OUTPUT_DIR}" >"${OUTPUT_DIR}/detection_stats.log" 2>&1 &
STATS_PID=$!

stop_process() {
  local pid="$1"
  if [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 10); do
      if ! kill -0 "${pid}" 2>/dev/null; then
        break
      fi
      sleep 1
    done
    if kill -0 "${pid}" 2>/dev/null; then
      kill -TERM "${pid}" 2>/dev/null || true
    fi
    wait "${pid}" 2>/dev/null || true
  fi
}

cleanup_all() {
  stop_process "${STATS_PID}"
}
handle_interrupt() {
  :
}
trap cleanup_all EXIT
trap handle_interrupt INT TERM

echo "Recording directory: ${OUTPUT_DIR}"
echo "Model profile: ${MODEL_PROFILE}"
echo "Files: front_yolo_*.mkv, back_yolo_*.mkv, search_yolo_*.mkv"
if [[ "${DURATION_SEC}" -eq 0 ]]; then
  echo "Press Ctrl+C once to stop and finalize all three videos."
else
  echo "Automatic stop after ${DURATION_SEC} seconds."
fi

launch_command=(
  ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py
  "model_engine_path:=${MODEL_ENGINE_PATH}"
  "model_onnx_path:=${MODEL_ONNX_PATH}"
  "network_mode:=${NETWORK_MODE}"
  "model_output_format:=${MODEL_OUTPUT_FORMAT}"
  "confidence:=${CONFIDENCE}"
  "class_names:=drone"
  "num_detected_classes:=1"
  "source_mode:=all"
  "csi_fps:=20"
  "mux_width:=1280"
  "mux_height:=720"
  "inference_interval:=0"
  "stereo_roi_enabled:=false"
  "display:=false"
  "display_sync:=false"
  "record:=true"
  "record_output_dir:=${OUTPUT_DIR}"
  "record_bitrate:=${RECORD_BITRATE}"
  "record_width:=${RECORD_WIDTH}"
  "record_height:=${RECORD_HEIGHT}"
  "record_segment_seconds:=${RECORD_SEGMENT_SECONDS}"
  "search_sun_filter:=${SEARCH_SUN_FILTER}"
  "search_exposure_time:=${SEARCH_EXPOSURE_TIME}"
  "search_gain:=${SEARCH_GAIN}"
  "search_brightness:=${SEARCH_BRIGHTNESS}"
  "search_backlight_compensation:=${SEARCH_BACKLIGHT_COMPENSATION}"
)

set +e
if [[ "${DURATION_SEC}" -gt 0 ]]; then
  timeout --foreground --signal=INT --kill-after=15 \
    "${DURATION_SEC}s" "${launch_command[@]}" >"${OUTPUT_DIR}/deepstream.log" 2>&1
  LAUNCH_STATUS=$?
  if [[ "${LAUNCH_STATUS}" -eq 124 ]]; then
    LAUNCH_STATUS=0
  fi
else
  "${launch_command[@]}" >"${OUTPUT_DIR}/deepstream.log" 2>&1
  LAUNCH_STATUS=$?
  if [[ "${LAUNCH_STATUS}" -eq 130 ]]; then
    LAUNCH_STATUS=0
  fi
fi
set -e

stop_process "${STATS_PID}"
STATS_PID=""
trap - EXIT INT TERM

STATUS="${LAUNCH_STATUS}"
for name in left right search; do
  videos=("${OUTPUT_DIR}/${name}_yolo_"*.mkv)
  if [[ ! -e "${videos[0]}" ]]; then
    echo "Missing output segments for ${name}" >&2
    STATUS=1
    continue
  fi
  : >"${OUTPUT_DIR}/${name}_video_probe.txt"
  for video in "${videos[@]}"; do
    if [[ ! -s "${video}" ]] ||
      ! gst-discoverer-1.0 "${video}" \
      >>"${OUTPUT_DIR}/${name}_video_probe.txt" 2>&1; then
      echo "Video validation failed: ${video}" >&2
      STATUS=1
    fi
  done
done
if grep -Eqi \
  'process has died|DeepStream error|cudaError|Internal data stream error|not-negotiated' \
  "${OUTPUT_DIR}/deepstream.log"; then
  echo "DeepStream runtime errors found in ${OUTPUT_DIR}/deepstream.log" >&2
  STATUS=1
fi

{
  echo "end_time=$(date --iso-8601=seconds)"
  echo "status=${STATUS}"
} >>"${OUTPUT_DIR}/manifest.txt"

if [[ "${STATUS}" -eq 0 ]]; then
  echo "THREE_YOLO_RECORD_PASS output=${OUTPUT_DIR}"
else
  echo "THREE_YOLO_RECORD_FAIL output=${OUTPUT_DIR}" >&2
fi
exit "${STATUS}"
