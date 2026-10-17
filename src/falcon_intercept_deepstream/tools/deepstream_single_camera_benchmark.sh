#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
SOURCE_MODE="${SOURCE_MODE:-}"
case "${SOURCE_MODE}" in
  search)
    MUX_WIDTH="${MUX_WIDTH:-2560}"
    MUX_HEIGHT="${MUX_HEIGHT:-1440}"
    ;;
  front|back)
    MUX_WIDTH="${MUX_WIDTH:-3840}"
    MUX_HEIGHT="${MUX_HEIGHT:-2160}"
    ;;
  *)
    echo "SOURCE_MODE must be search, front, or back" >&2
    exit 2
    ;;
esac

export SOURCE_MODE MUX_WIDTH MUX_HEIGHT
export RUN_SECONDS="${RUN_SECONDS:-2400}"
export MIN_STATS_LINES="${MIN_STATS_LINES:-18}"
export WORKSPACE_SIZE_MB="${WORKSPACE_SIZE_MB:-2048}"
export LOG_FILE="${LOG_FILE:-/tmp/falcon_deepstream_${SOURCE_MODE}_native.log}"

bash "${SCRIPT_DIR}/deepstream_three_camera_smoke_test.sh"
python3 "${SCRIPT_DIR}/summarize_deepstream_fps.py" \
  --log "${LOG_FILE}" \
  --source "${SOURCE_MODE}" \
  --skip "${SKIP_STATS_LINES:-3}" \
  --min-samples "${MIN_STABLE_SAMPLES:-10}"

echo "DEEPSTREAM_SINGLE_CAMERA_BENCHMARK_PASS source=${SOURCE_MODE} "\
"mux=${MUX_WIDTH}x${MUX_HEIGHT}"
