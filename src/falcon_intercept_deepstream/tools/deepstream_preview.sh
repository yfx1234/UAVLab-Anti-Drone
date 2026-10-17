#!/usr/bin/env bash
set -eo pipefail

PROFILE="${1:-uav0728x736fp16}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
WEIGHTS_DIR="${WEIGHTS_DIR:-/home/uavlab/Jetson_ROS2_V0/weights}"
CONFIDENCE="${CONFIDENCE:-0.50}"
CLASSES="${CLASSES:-}"
DISPLAY_WIDTH="${DISPLAY_WIDTH:-1280}"
DISPLAY_HEIGHT="${DISPLAY_HEIGHT:-720}"
STEREO_COLOR_CORRECTION="${STEREO_COLOR_CORRECTION:-true}"
STEREO_EXPOSURE_COMPENSATION="${STEREO_EXPOSURE_COMPENSATION:-0.0}"
STEREO_MAGENTA_SUPPRESSION="${STEREO_MAGENTA_SUPPRESSION:-1.0}"
STEREO_COLOR_GAMMA="${STEREO_COLOR_GAMMA:-0.55}"
STEREO_COLOR_MIX="${STEREO_COLOR_MIX:-0.85}"
CLASS_NAMES="${CLASS_NAMES:-}"
NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-}"

case "${PROFILE}" in
  uav1280)
    MODEL_ENGINE_PATH="${WEIGHTS_DIR}/uav_best_1280_int8_opt3.engine"
    MODEL_ONNX_PATH="${WEIGHTS_DIR}/uav_best_1280_int8.onnx"
    CLASS_NAMES="${CLASS_NAMES:-drone}"
    NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-1}"
    NETWORK_MODE="${NETWORK_MODE:-1}"
    MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-raw}"
    ;;
  uav1280fp16)
    MODEL_ENGINE_PATH="${WEIGHTS_DIR}/uav_best_1280_opt3.engine"
    MODEL_ONNX_PATH="${WEIGHTS_DIR}/uav_best_1280.onnx"
    CLASS_NAMES="${CLASS_NAMES:-drone}"
    NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-1}"
    NETWORK_MODE="${NETWORK_MODE:-2}"
    MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
    ;;
  uav1280x736fp16)
    MODEL_ENGINE_PATH="${WEIGHTS_DIR}/uav_best_1280x736_fp16_opt3.engine"
    MODEL_ONNX_PATH="${WEIGHTS_DIR}/uav_best_1280x736_fp16.onnx"
    CLASS_NAMES="${CLASS_NAMES:-drone}"
    NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-1}"
    NETWORK_MODE="${NETWORK_MODE:-2}"
    MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
    ;;
  uav0728x736fp16)
    MODEL_ENGINE_PATH="${WEIGHTS_DIR}/candidates/0728/uav_0728_1280x736_fp16_opt3.engine"
    MODEL_ONNX_PATH="${WEIGHTS_DIR}/candidates/0728/uav_0728_1280x736_fp16.onnx"
    CLASS_NAMES="${CLASS_NAMES:-drone}"
    NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-1}"
    NETWORK_MODE="${NETWORK_MODE:-2}"
    MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
    ;;
  640)
    MODEL_ENGINE_PATH="${WEIGHTS_DIR}/yolo26n_deepstream_b1.engine"
    MODEL_ONNX_PATH="${WEIGHTS_DIR}/yolo26n.onnx"
    NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-80}"
    NETWORK_MODE="${NETWORK_MODE:-2}"
    MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
    ;;
  1280)
    MODEL_ENGINE_PATH="${WEIGHTS_DIR}/yolo26n_1280_deepstream_b1.engine"
    MODEL_ONNX_PATH="${WEIGHTS_DIR}/yolo26n_1280.onnx"
    NUM_DETECTED_CLASSES="${NUM_DETECTED_CLASSES:-80}"
    NETWORK_MODE="${NETWORK_MODE:-2}"
    MODEL_OUTPUT_FORMAT="${MODEL_OUTPUT_FORMAT:-end_to_end}"
    ;;
  *)
    echo "Usage: $0 {uav1280|uav1280fp16|uav1280x736fp16|uav0728x736fp16|640|1280}" >&2
    exit 2
    ;;
esac

for path in "${MODEL_ENGINE_PATH}" "${MODEL_ONNX_PATH}"; do
  if [[ ! -f "${path}" ]]; then
    echo "Missing model file: ${path}" >&2
    exit 1
  fi
done

if [[ -z "${DISPLAY:-}" ]]; then
  export DISPLAY=:0
fi
if [[ -z "${XAUTHORITY:-}" ]]; then
  candidate="/run/user/$(id -u)/gdm/Xauthority"
  if [[ -r "${candidate}" ]]; then
    export XAUTHORITY="${candidate}"
  fi
fi

source /opt/ros/humble/setup.bash
cd "${WORKSPACE_DIR}"
source install/setup.bash
set -u

export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS2CLI_DISABLE_DAEMON=1

echo "Starting three-camera YOLO preview"
echo "  profile:     ${PROFILE} network_mode=${NETWORK_MODE} output=${MODEL_OUTPUT_FORMAT}"
echo "  engine:      ${MODEL_ENGINE_PATH}"
echo "  display:     ${DISPLAY}"
echo "  XAUTHORITY:  ${XAUTHORITY:-<session default>}"
echo "  layout:      top-left=left, top-right=right, bottom-left=search"
echo "  confidence:  ${CONFIDENCE}"
echo "  stereo WB:   ${STEREO_COLOR_CORRECTION}"
echo "  stereo EV:   ${STEREO_EXPOSURE_COMPENSATION}"
echo "  color tone:  magenta=${STEREO_MAGENTA_SUPPRESSION} gamma=${STEREO_COLOR_GAMMA} mix=${STEREO_COLOR_MIX}"
echo "Close with Ctrl+C in this terminal."

launch_arguments=(
  "model_engine_path:=${MODEL_ENGINE_PATH}"
  "model_onnx_path:=${MODEL_ONNX_PATH}"
  "batch_size:=1"
  "source_mode:=all"
  "mux_width:=1280"
  "mux_height:=720"
  "confidence:=${CONFIDENCE}"
  "class_names:=${CLASS_NAMES}"
  "num_detected_classes:=${NUM_DETECTED_CLASSES}"
  "network_mode:=${NETWORK_MODE}"
  "model_output_format:=${MODEL_OUTPUT_FORMAT}"
  "display:=true"
  "display_width:=${DISPLAY_WIDTH}"
  "display_height:=${DISPLAY_HEIGHT}"
  "display_sync:=false"
  "stereo_color_correction:=${STEREO_COLOR_CORRECTION}"
  "stereo_exposure_compensation:=${STEREO_EXPOSURE_COMPENSATION}"
  "stereo_magenta_suppression:=${STEREO_MAGENTA_SUPPRESSION}"
  "stereo_color_gamma:=${STEREO_COLOR_GAMMA}"
  "stereo_color_mix:=${STEREO_COLOR_MIX}"
)
if [[ -n "${CLASSES}" ]]; then
  launch_arguments+=("classes:=${CLASSES}")
fi

exec ros2 launch falcon_intercept_deepstream deepstream_three_camera_v0.launch.py \
  "${launch_arguments[@]}"
