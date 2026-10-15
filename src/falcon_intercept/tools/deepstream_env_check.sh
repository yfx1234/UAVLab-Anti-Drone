#!/usr/bin/env bash
set -eo pipefail

DEEPSTREAM_ROOT="${DEEPSTREAM_ROOT:-/opt/nvidia/deepstream/deepstream-7.1}"
export LD_LIBRARY_PATH="${DEEPSTREAM_ROOT}/lib:${LD_LIBRARY_PATH:-}"
export GST_PLUGIN_PATH="${DEEPSTREAM_ROOT}/lib/gst-plugins:${GST_PLUGIN_PATH:-}"

failures=0

check_command() {
  local label="$1"
  shift
  if "$@" >/dev/null 2>&1; then
    echo "[PASS] ${label}"
  else
    echo "[FAIL] ${label}"
    failures=$((failures + 1))
  fi
}

echo "DeepStream root: ${DEEPSTREAM_ROOT}"

if dpkg-query -W -f='${Status} ${Version}\n' deepstream-7.1 2>/dev/null \
  | grep -q '^install ok installed '; then
  echo "[PASS] deepstream-7.1 package installed"
  dpkg-query -W -f='       version=${Version}\n' deepstream-7.1
else
  echo "[FAIL] deepstream-7.1 package installed"
  failures=$((failures + 1))
fi

check_command "deepstream-app executable" test -x "${DEEPSTREAM_ROOT}/bin/deepstream-app"
check_command "deepstream-app runtime" "${DEEPSTREAM_ROOT}/bin/deepstream-app" --version-all
check_command "nvstreammux GStreamer plugin" gst-inspect-1.0 nvstreammux
check_command "nvinfer GStreamer plugin" gst-inspect-1.0 nvinfer
check_command "nvvideoconvert GStreamer plugin" gst-inspect-1.0 nvvideoconvert
check_command "nvdspreprocess GStreamer plugin" gst-inspect-1.0 nvdspreprocess
check_command "nvtracker GStreamer plugin" gst-inspect-1.0 nvtracker
check_command "Jetson Argus source remains available" gst-inspect-1.0 nvarguscamerasrc
check_command "Jetson hardware JPEG encoder remains available" gst-inspect-1.0 nvjpegenc

if [[ -x "${DEEPSTREAM_ROOT}/bin/deepstream-app" ]]; then
  ldd_missing="$(ldd "${DEEPSTREAM_ROOT}/bin/deepstream-app" 2>/dev/null \
    | grep 'not found' || true)"
  if [[ -n "${ldd_missing}" ]]; then
    echo "[FAIL] deepstream-app shared libraries"
    echo "${ldd_missing}"
    failures=$((failures + 1))
  else
    echo "[PASS] deepstream-app shared libraries"
  fi
fi

if (( failures > 0 )); then
  echo "DEEPSTREAM_CHECK_FAIL failures=${failures}"
  exit 1
fi

echo "DEEPSTREAM_CHECK_PASS"
