#!/usr/bin/env bash
set -euo pipefail

OUT_DIR="${1:-$HOME/color_ab/wb_sweep_$(date +%Y%m%d-%H%M%S)}"
FRAMES="${FRAMES:-60}"
SOURCE_WIDTH="${SOURCE_WIDTH:-3840}"
SOURCE_HEIGHT="${SOURCE_HEIGHT:-2160}"
PREVIEW_WIDTH="${PREVIEW_WIDTH:-1280}"
PREVIEW_HEIGHT="${PREVIEW_HEIGHT:-720}"

declare -a MODES=(
  "1:auto"
  "2:incandescent"
  "3:fluorescent"
  "4:warm_fluorescent"
  "5:daylight"
  "6:cloudy_daylight"
  "7:twilight"
  "8:shade"
)

command -v gst-launch-1.0 >/dev/null
gst-inspect-1.0 nvarguscamerasrc >/dev/null
gst-inspect-1.0 nvvidconv >/dev/null
gst-inspect-1.0 nvjpegenc >/dev/null

mkdir -p "$OUT_DIR"

{
  printf 'created=%s\n' "$(date --iso-8601=seconds)"
  printf 'frames=%s\n' "$FRAMES"
  printf 'source=%sx%s@30\n' "$SOURCE_WIDTH" "$SOURCE_HEIGHT"
  printf 'saved=%sx%s JPEG\n' "$PREVIEW_WIDTH" "$PREVIEW_HEIGHT"
  if [[ -f /var/nvidia/nvcam/settings/camera_overrides.isp ]]; then
    printf 'camera_override=active\n'
  else
    printf 'camera_override=disabled\n'
  fi
} > "$OUT_DIR/manifest.txt"

capture_one() {
  local sensor_id="$1"
  local side="$2"
  local wb_mode="$3"
  local label="$4"

  gst-launch-1.0 -q \
    nvarguscamerasrc sensor-id="$sensor_id" wbmode="$wb_mode" num-buffers="$FRAMES" \
    ! "video/x-raw(memory:NVMM),width=$SOURCE_WIDTH,height=$SOURCE_HEIGHT,framerate=30/1" \
    ! nvvidconv \
    ! "video/x-raw(memory:NVMM),width=$PREVIEW_WIDTH,height=$PREVIEW_HEIGHT,format=I420" \
    ! nvjpegenc quality=90 \
    ! multifilesink location="$OUT_DIR/${side}_${label}_%03d.jpg" max-files=1 \
    > "$OUT_DIR/${side}_${label}.log" 2>&1
}

for entry in "${MODES[@]}"; do
  wb_mode="${entry%%:*}"
  label="${entry#*:}"
  printf '[wb-sweep] mode=%s label=%s\n' "$wb_mode" "$label"

  capture_one 0 front "$wb_mode" "$label" &
  left_pid=$!
  capture_one 1 back "$wb_mode" "$label" &
  right_pid=$!

  left_rc=0
  right_rc=0
  wait "$left_pid" || left_rc=$?
  wait "$right_pid" || right_rc=$?

  if (( left_rc != 0 || right_rc != 0 )); then
    printf '[wb-sweep] FAIL mode=%s left_rc=%s right_rc=%s\n' \
      "$wb_mode" "$left_rc" "$right_rc" >&2
    exit 1
  fi
done

expected_count="${#MODES[@]}"
left_count="$(find "$OUT_DIR" -maxdepth 1 -type f -name 'left_*.jpg' | wc -l)"
right_count="$(find "$OUT_DIR" -maxdepth 1 -type f -name 'right_*.jpg' | wc -l)"

printf '[wb-sweep] left=%s right=%s expected=%s\n' \
  "$left_count" "$right_count" "$expected_count"

if [[ "$left_count" != "$expected_count" || "$right_count" != "$expected_count" ]]; then
  printf '[wb-sweep] FAIL incomplete image set\n' >&2
  exit 1
fi

printf '[wb-sweep] PASS output=%s\n' "$OUT_DIR"
