#!/usr/bin/env bash
set -euo pipefail

SESSION_DIR="${1:?usage: evaluate_calibration_session.sh SESSION_DIR OUTPUT_DIR [true|false]}"
OUTPUT_DIR="${2:?usage: evaluate_calibration_session.sh SESSION_DIR OUTPUT_DIR [true|false]}"
COLOR_CORRECTION="${3:-true}"

# Historical sessions were named by Argus enumeration.  The physical audit
# established csi1=front and csi0=back.
LEFT_FILE="${SESSION_DIR}/left_sensor1_3840x2160_15fps_uncorrected_mjpeg.mkv"
RIGHT_FILE="${SESSION_DIR}/right_sensor0_3840x2160_15fps_uncorrected_mjpeg.mkv"
[[ -f "$LEFT_FILE" ]] || LEFT_FILE="${SESSION_DIR}/csi1_3840x2160_15fps_uncorrected_mjpeg.mkv"
[[ -f "$RIGHT_FILE" ]] || RIGHT_FILE="${SESSION_DIR}/csi0_3840x2160_15fps_uncorrected_mjpeg.mkv"
SEARCH_FILE="${SESSION_DIR}/usb_2560x1440_25fps_native_mjpeg.mkv"
ENGINE="/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine"

for path in "$LEFT_FILE" "$RIGHT_FILE" "$SEARCH_FILE" "$ENGINE"; do
  if [[ ! -f "$path" ]]; then
    echo "missing required file: $path" >&2
    exit 2
  fi
done
if [[ "$COLOR_CORRECTION" != "true" && "$COLOR_CORRECTION" != "false" ]]; then
  echo "color correction must be true or false" >&2
  exit 2
fi

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

printf 'session_dir=%s\ncolor_correction=%s\nengine=%s\n' \
  "$SESSION_DIR" "$COLOR_CORRECTION" "$ENGINE" >"$OUTPUT_DIR/profile.txt"
echo "CALIBRATION_SESSION_EVAL_OUTPUT=$OUTPUT_DIR"
set +e
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  input_mode:=files \
  front_file_path:="$LEFT_FILE" \
  back_file_path:="$RIGHT_FILE" \
  search_file_path:="$SEARCH_FILE" \
  model_engine_path:="$ENGINE" \
  source_mode:=all \
  network_mode:=2 \
  model_output_format:=end_to_end \
  stereo_roi_enabled:=false \
  stereo_color_correction:="$COLOR_CORRECTION" \
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
echo "CALIBRATION_SESSION_EVAL_COMPLETE status=$PIPELINE_STATUS output=$OUTPUT_DIR"
exit "$PIPELINE_STATUS"
