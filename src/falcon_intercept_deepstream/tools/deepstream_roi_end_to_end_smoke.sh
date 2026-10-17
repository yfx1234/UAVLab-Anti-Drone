#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${FALCON_WORKSPACE:-$HOME/Jetson_ROS2_V0/ros2_ws}"
LOG_PATH="${ROI_SMOKE_LOG:-/tmp/falcon_roi_runtime.log}"
MODEL_ENGINE_PATH="${MODEL_ENGINE_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16_opt3.engine}"
MODEL_ONNX_PATH="${MODEL_ONNX_PATH:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/0728/uav_0728_1280x736_fp16.onnx}"
CLASS_NAMES="${CLASS_NAMES:-drone}"
NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-1}"
NETWORK_MODE="${NETWORK_MODE:-2}"
MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
PIPE_PID=""

cleanup() {
  if [[ -n "$PIPE_PID" ]]; then
    kill -- "-$PIPE_PID" 2>/dev/null || true
    wait "$PIPE_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

cd "$WORKSPACE"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export LD_LIBRARY_PATH="/opt/nvidia/deepstream/deepstream-7.1/lib:${LD_LIBRARY_PATH:-}"
export GST_PLUGIN_PATH="$WORKSPACE/install/falcon_intercept_deepstream/lib/gstreamer-1.0:${GST_PLUGIN_PATH:-}"

setsid timeout 35s ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  display:=false batch_size:=1 mux_width:=1280 mux_height:=720 \
  "model_engine_path:=${MODEL_ENGINE_PATH}" "model_onnx_path:=${MODEL_ONNX_PATH}" \
  "class_names:=${CLASS_NAMES}" "num_detected_classes:=${NUM_DETECTED_CLASSES}" \
  "network_mode:=${NETWORK_MODE}" "model_output_format:=${MODEL_OUTPUT_FORMAT}" \
  >"$LOG_PATH" 2>&1 &
PIPE_PID=$!

ready=0
for _ in $(seq 1 40); do
  if grep -q 'DeepStream native pipeline started' "$LOG_PATH" 2>/dev/null; then
    ready=1
    break
  fi
  sleep 0.5
done
if [[ "$ready" -ne 1 ]]; then
  echo "PIPELINE_NOT_READY"
  tail -n 100 "$LOG_PATH"
  exit 1
fi

python3 src/falcon_intercept_deepstream/tools/deepstream_roi_runtime_smoke_test.py
echo "ROI_PIPELINE_LOG_TAIL"
tail -n 100 "$LOG_PATH"
