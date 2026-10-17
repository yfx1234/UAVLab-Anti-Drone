#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/uavlab/Jetson_ROS2_V0"
CANDIDATES_DIR="${ROOT}/weights/candidates"
PACKAGE_DIR="${ROOT}/ros2_ws/src/falcon_intercept_deepstream"
EXPORTER="${PACKAGE_DIR}/tools/export_yolo26_onnx.py"
TRTEXEC="/usr/src/tensorrt/bin/trtexec"
LAUNCH_PACKAGE="falcon_intercept_deepstream"
LAUNCH_FILE="deepstream_three_camera_v0.launch.py"

PRODUCTION_PT="${CANDIDATES_DIR}/0728/uav_0728_1280x736_fp16.pt"
PLATFORM_EXP_PT="${CANDIDATES_DIR}/platform_exp_20260909/exp.pt"

usage() {
  cat <<'EOF'
Usage:
  select_yolo_model.sh list
  select_yolo_model.sh status <production|platform-exp|PT_PATH>
  select_yolo_model.sh prepare <production|platform-exp|PT_PATH>
  select_yolo_model.sh command <production|platform-exp|PT_PATH> [ROS_LAUNCH_ARG ...]
  select_yolo_model.sh run <production|platform-exp|PT_PATH> [ROS_LAUNCH_ARG ...]

Examples:
  ./select_yolo_model.sh list
  ./select_yolo_model.sh prepare platform-exp
  ./select_yolo_model.sh command platform-exp source_mode:=search display:=true
  ./select_yolo_model.sh run production source_mode:=all display:=true

Each PT uses separate sibling files:
  MODEL.pt -> MODEL.onnx -> MODEL_opt3.engine
Existing files are reused and are never overwritten by this script.
EOF
}

resolve_pt() {
  local selector="$1"
  case "${selector}" in
    production|prod|0728)
      printf '%s\n' "${PRODUCTION_PT}"
      ;;
    platform-exp|exp|new)
      printf '%s\n' "${PLATFORM_EXP_PT}"
      ;;
    *)
      readlink -f -- "${selector}"
      ;;
  esac
}

set_model_paths() {
  PT_PATH="$(resolve_pt "$1")"
  [[ -f "${PT_PATH}" ]] || {
    echo "PT file not found: ${PT_PATH}" >&2
    exit 2
  }
  [[ "${PT_PATH}" == *.pt ]] || {
    echo "Model path must end in .pt: ${PT_PATH}" >&2
    exit 2
  }
  ONNX_PATH="${PT_PATH%.pt}.onnx"
  ENGINE_PATH="${PT_PATH%.pt}_opt3.engine"
  BUILD_LOG="${PT_PATH%.pt}_trtexec_fp16_opt3.log"
}

print_status() {
  printf 'pt=%s [%s]\n' "${PT_PATH}" "$(test -f "${PT_PATH}" && echo ready || echo missing)"
  printf 'onnx=%s [%s]\n' "${ONNX_PATH}" "$(test -f "${ONNX_PATH}" && echo ready || echo missing)"
  printf 'engine=%s [%s]\n' "${ENGINE_PATH}" "$(test -f "${ENGINE_PATH}" && echo ready || echo missing)"
}

prepare_model() {
  if [[ ! -f "${ONNX_PATH}" ]]; then
    echo "Exporting static end-to-end ONNX: ${ONNX_PATH}"
    python3 "${EXPORTER}" \
      --source "${PT_PATH}" \
      --output "${ONNX_PATH}" \
      --imgsz 736 1280 \
      --device 0
  else
    echo "Reusing ONNX: ${ONNX_PATH}"
  fi

  if [[ ! -f "${ENGINE_PATH}" ]]; then
    echo "Building Jetson FP16 TensorRT engine: ${ENGINE_PATH}"
    "${TRTEXEC}" \
      --onnx="${ONNX_PATH}" \
      --saveEngine="${ENGINE_PATH}" \
      --fp16 \
      --builderOptimizationLevel=3 \
      --profilingVerbosity=detailed \
      2>&1 | tee "${BUILD_LOG}"
  else
    echo "Reusing engine: ${ENGINE_PATH}"
  fi

  print_status
}

print_launch_command() {
  local -a command=(
    ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}"
    "model_engine_path:=${ENGINE_PATH}"
    "model_onnx_path:=${ONNX_PATH}"
    "network_mode:=2"
    "model_output_format:=end_to_end"
    "$@"
  )
  printf '%q ' "${command[@]}"
  printf '\n'
}

list_models() {
  printf '%-14s %-7s %-7s %s\n' ALIAS ONNX ENGINE PT_PATH
  while IFS= read -r -d '' pt; do
    local alias="-"
    [[ "${pt}" == "${PRODUCTION_PT}" ]] && alias="production"
    [[ "${pt}" == "${PLATFORM_EXP_PT}" ]] && alias="platform-exp"
    local onnx="${pt%.pt}.onnx"
    local engine="${pt%.pt}_opt3.engine"
    printf '%-14s %-7s %-7s %s\n' \
      "${alias}" \
      "$(test -f "${onnx}" && echo ready || echo missing)" \
      "$(test -f "${engine}" && echo ready || echo missing)" \
      "${pt}"
  done < <(find "${CANDIDATES_DIR}" -type f -name '*.pt' -print0 | sort -z)
}

ACTION="${1:-list}"
case "${ACTION}" in
  list)
    list_models
    ;;
  status|prepare|command|run)
    [[ $# -ge 2 ]] || {
      usage >&2
      exit 2
    }
    SELECTOR="$2"
    shift 2
    set_model_paths "${SELECTOR}"
    case "${ACTION}" in
      status)
        print_status
        ;;
      prepare)
        prepare_model
        ;;
      command)
        print_status
        print_launch_command "$@"
        ;;
      run)
        prepare_model
        set +u
        source /opt/ros/humble/setup.bash
        source "${ROOT}/ros2_ws/install/setup.bash"
        set -u
        echo "Selected PT: ${PT_PATH}"
        echo "Selected engine: ${ENGINE_PATH}"
        exec ros2 launch "${LAUNCH_PACKAGE}" "${LAUNCH_FILE}" \
          "model_engine_path:=${ENGINE_PATH}" \
          "model_onnx_path:=${ONNX_PATH}" \
          "network_mode:=2" \
          "model_output_format:=end_to_end" \
          "$@"
        ;;
    esac
    ;;
  -h|--help|help)
    usage
    ;;
  *)
    echo "Unknown action: ${ACTION}" >&2
    usage >&2
    exit 2
    ;;
esac
