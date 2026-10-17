#!/usr/bin/env bash
set -euo pipefail

OUTPUT_DIR="${1:?usage: live_perception_acceptance.sh OUTPUT_DIR [DURATION_SEC]}"
DURATION_SEC="${2:-35}"
ENGINE="${ENGINE:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine}"
NETWORK_MODE="${NETWORK_MODE:-2}"
MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
CALIBRATION="/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/falcon_intercept/share/falcon_intercept/config/camera/stereo_calib.yaml"
STATS_TOOL="/home/uavlab/Jetson_ROS2_V0/ros2_ws/install/falcon_intercept_deepstream/lib/falcon_intercept_deepstream/video_detection_stats.py"
CONTINUITY_TOOL="/home/uavlab/Jetson_ROS2_V0/ros2_ws/src/falcon_intercept/tools/perception_continuity_check.py"

mkdir -p "$OUTPUT_DIR/detection_stats"
set +u
source /opt/ros/humble/setup.bash
source /home/uavlab/Jetson_ROS2_V0/ros2_ws/install/setup.bash
set -u

if pgrep -f 'deepstream_three_camera_node' >/dev/null; then
  echo "another DeepStream camera node is already running" >&2
  exit 2
fi

PIPELINE_PID=""
SOLVER_PID=""
STATS_PID=""
cleanup() {
  for pid in "$STATS_PID" "$SOLVER_PID" "$PIPELINE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  for pid in "$STATS_PID" "$SOLVER_PID" "$PIPELINE_PID"; do
    if [[ -n "$pid" ]]; then
      wait "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup EXIT INT TERM

timeout -s INT 50s ros2 launch \
  falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  model_engine_path:="$ENGINE" \
  source_mode:=all \
  network_mode:="$NETWORK_MODE" \
  model_output_format:="$MODEL_OUTPUT_FORMAT" \
  confidence:=0.50 \
  stereo_roi_enabled:=false \
  stereo_color_correction:=true \
  display:=false \
  >"$OUTPUT_DIR/deepstream.log" 2>&1 &
PIPELINE_PID=$!

timeout -s INT 50s ros2 run falcon_intercept stereo_target_solver_node \
  --ros-args -p calibration_path:="$CALIBRATION" \
  >"$OUTPUT_DIR/solver.log" 2>&1 &
SOLVER_PID=$!

timeout -s INT 47s "$STATS_TOOL" --output-dir "$OUTPUT_DIR/detection_stats" \
  >"$OUTPUT_DIR/detection_stats/stats.log" 2>&1 &
STATS_PID=$!

sleep 6
if ! kill -0 "$PIPELINE_PID" 2>/dev/null; then
  echo "DeepStream exited during startup" >&2
  exit 1
fi
if ! kill -0 "$SOLVER_PID" 2>/dev/null; then
  echo "stereo solver exited during startup" >&2
  exit 1
fi

/usr/bin/python3 "$CONTINUITY_TOOL" \
  --duration-sec "$DURATION_SEC" \
  --json-output "$OUTPUT_DIR/perception_continuity.json" \
  | tee "$OUTPUT_DIR/perception_continuity.log"

cleanup
trap - EXIT INT TERM
echo "LIVE_PERCEPTION_ACCEPTANCE_COMPLETE output=$OUTPUT_DIR"
