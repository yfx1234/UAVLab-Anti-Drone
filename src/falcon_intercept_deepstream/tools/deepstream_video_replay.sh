#!/usr/bin/env bash
set -euo pipefail

VIDEO_DIR="${1:-/home/uavlab/Downloads/record-preview-20260715-180001}"
RESULT_ROOT="${2:-/home/uavlab/Jetson_ROS2_V0/test_results/video_replay}"
CLIP_DIR="${VIDEO_DIR}/review-from-180s"
RUN_ID="video-replay-$(date +%Y%m%d-%H%M%S)"
OUTPUT_DIR="${RESULT_ROOT}/${RUN_ID}"
PREPARE_TOOL="/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/falcon_intercept_deepstream/lib/falcon_intercept_deepstream/prepare_video_review_clips.py"
if [[ ! -f "${CLIP_DIR}/READY" ]]; then
  "$PREPARE_TOOL" --input-dir "$VIDEO_DIR" --output-dir "$CLIP_DIR" --start-sec 180.0
fi
LEFT="${CLIP_DIR}/left_sensor1_3840x2160_15fps_mjpeg.mkv"
RIGHT="${CLIP_DIR}/right_sensor0_3840x2160_15fps_mjpeg.mkv"
SEARCH="${CLIP_DIR}/usb_2560x1440_25fps_native_mjpeg.mkv"

for path in "$LEFT" "$RIGHT" "$SEARCH"; do
  if [[ ! -f "$path" ]]; then
    echo "missing input video: $path" >&2
    exit 2
  fi
done

mkdir -p "$OUTPUT_DIR"
set +u
source /opt/ros/humble/setup.bash
source /home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash
set -u
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/home/uavlab/.Xauthority}"

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

echo "VIDEO_REPLAY_OUTPUT=$OUTPUT_DIR"
echo "Default-model synchronized review replay: all sources 15 FPS (search uniformly sampled from 25 FPS)"
set +e
ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  input_mode:=files \
  front_file_path:="$LEFT" \
  back_file_path:="$RIGHT" \
  search_file_path:="$SEARCH" \
  source_mode:=all \
  network_mode:=2 \
  model_output_format:=end_to_end \
  stereo_roi_enabled:=false \
  stereo_color_correction:=false \
  display:=true \
  display_sync:=false \
  2>&1 | tee "$OUTPUT_DIR/deepstream.log"
PIPELINE_STATUS=${PIPESTATUS[0]}
set -e
cleanup
trap - EXIT INT TERM
if grep -Eq "process has died|startup failed|DeepStream error" "$OUTPUT_DIR/deepstream.log"; then
  PIPELINE_STATUS=1
fi
echo "pipeline_status=$PIPELINE_STATUS" >"$OUTPUT_DIR/status.txt"
echo "VIDEO_REPLAY_COMPLETE status=$PIPELINE_STATUS output=$OUTPUT_DIR"
exit "$PIPELINE_STATUS"
