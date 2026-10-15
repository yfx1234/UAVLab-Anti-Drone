#!/usr/bin/env bash
set -euo pipefail

PROFILE="${1:-latest}"
OUTPUT_ROOT="${2:-/home/uavlab/Jetson_ROS2_V0/test_results/precision_ab}"
VIDEO_DIR="${VIDEO_DIR:-/home/uavlab/Downloads/record-preview-20260715-180001/review-from-180s}"
# Legacy csi0/csi1 recordings predate the physical occlusion audit.  On this
# rig csi1 is the physical front camera and csi0 is the physical back camera.
LEFT_FILE="${LEFT_FILE:-${VIDEO_DIR}/left_sensor1_3840x2160_15fps_mjpeg.mkv}"
RIGHT_FILE="${RIGHT_FILE:-${VIDEO_DIR}/right_sensor0_3840x2160_15fps_mjpeg.mkv}"
[[ -f "$LEFT_FILE" ]] || LEFT_FILE="${VIDEO_DIR}/csi1_3840x2160_15fps_mjpeg.mkv"
[[ -f "$RIGHT_FILE" ]] || RIGHT_FILE="${VIDEO_DIR}/csi0_3840x2160_15fps_mjpeg.mkv"
SEARCH_FILE="${SEARCH_FILE:-${VIDEO_DIR}/usb_2560x1440_25fps_native_mjpeg.mkv}"
FP16_ENGINE="${FP16_ENGINE:-/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280_opt3.engine}"
FP16_RECT_ENGINE="${FP16_RECT_ENGINE:-/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280x736_fp16_opt3.engine}"
INT8_ENGINE="${INT8_ENGINE:-/home/uavlab/Jetson_ROS2_V0/weights/uav_best_1280_int8_opt3.engine}"
LATEST_ENGINE="${LATEST_ENGINE:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine}"
case "$PROFILE" in
  latest)
    ENGINE="$LATEST_ENGINE"
    NETWORK_MODE=2
    OUTPUT_FORMAT=end_to_end
    ;;
  fp16)
    ENGINE="$FP16_ENGINE"
    NETWORK_MODE=2
    OUTPUT_FORMAT=end_to_end
    ;;
  fp16_rect)
    ENGINE="$FP16_RECT_ENGINE"
    NETWORK_MODE=2
    OUTPUT_FORMAT=end_to_end
    ;;
  int8)
    ENGINE="$INT8_ENGINE"
    NETWORK_MODE=1
    OUTPUT_FORMAT=raw
    ;;
  *)
    echo "profile must be latest, fp16, fp16_rect or int8" >&2
    exit 2
    ;;
esac

RUN_ID="${PROFILE}-$(date +%Y%m%d-%H%M%S)"
OUTPUT_DIR="${OUTPUT_ROOT}/${RUN_ID}"
mkdir -p "$OUTPUT_DIR"
set +u
source /opt/ros/humble/setup.bash
source /home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash
set -u

STATS_TOOL="/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/falcon_intercept_deepstream/lib/falcon_intercept_deepstream/video_detection_stats.py"
"$STATS_TOOL" --output-dir "$OUTPUT_DIR" >"$OUTPUT_DIR/stats.log" 2>&1 &
STATS_PID=$!
cleanup() {
  if kill -0 "$STATS_PID" 2>/dev/null; then
    kill -INT "$STATS_PID" 2>/dev/null || true
    wait "$STATS_PID" || true
  fi
}
trap cleanup EXIT INT TERM

printf 'profile=%s\nengine=%s\nnetwork_mode=%s\noutput_format=%s\n' \
  "$PROFILE" "$ENGINE" "$NETWORK_MODE" "$OUTPUT_FORMAT" >"$OUTPUT_DIR/profile.txt"
echo "FILE_EVAL_OUTPUT=$OUTPUT_DIR"
set +e
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  input_mode:=files \
  front_file_path:="$LEFT_FILE" \
  back_file_path:="$RIGHT_FILE" \
  search_file_path:="$SEARCH_FILE" \
  model_engine_path:="$ENGINE" \
  source_mode:=all \
  network_mode:="$NETWORK_MODE" \
  model_output_format:="$OUTPUT_FORMAT" \
  stereo_roi_enabled:=false \
  stereo_color_correction:=false \
  display:=false \
  2>&1 | tee "$OUTPUT_DIR/deepstream.log"
PIPELINE_STATUS=${PIPESTATUS[0]}
set -e
cleanup
trap - EXIT INT TERM
if grep -Eq "process has died|startup failed|DeepStream error" "$OUTPUT_DIR/deepstream.log"; then
  PIPELINE_STATUS=1
fi
echo "pipeline_status=$PIPELINE_STATUS" >"$OUTPUT_DIR/status.txt"
echo "FILE_EVAL_COMPLETE profile=$PROFILE status=$PIPELINE_STATUS output=$OUTPUT_DIR"
exit "$PIPELINE_STATUS"
