#!/usr/bin/env bash
set -Eeuo pipefail

# Power-loss tolerant three-camera recorder for outdoor material collection.
#
# This is intentionally separate from three_camera_preview.sh. It keeps the same
# source modes and search-camera exposure controls, but writes short streamable
# Matroska segments through splitmuxsink. On a hard power loss, only the segment
# currently being written is expected to be at risk; older segments have already
# been finalized asynchronously.

MODE="help"
SAVE_DIR_CLI=""
DURATION_SECONDS=0
CSI_RECORD_FPS_CLI=""
CSI_JPEG_QUALITY="${CSI_JPEG_QUALITY:-85}"
VERIFY_SECONDS="${VERIFY_SECONDS:-8}"
SEGMENT_SECONDS="${SEGMENT_SECONDS:-5}"

FRONT_SENSOR_ID="${FRONT_SENSOR_ID:-1}"
BACK_SENSOR_ID="${BACK_SENSOR_ID:-0}"
CSI_SENSOR_MODE="${CSI_SENSOR_MODE:-0}"

CSI_WIDTH=3840
CSI_HEIGHT=2160
CSI_SOURCE_FPS=25

USB_WIDTH=2560
USB_HEIGHT=1440
USB_FPS=25

USB_DEVICE="${USB_DEVICE:-/dev/v4l/by-id/usb-Sonix_Technology_Co.__Ltd._HJ_4K_Camera_SN0001-video-index0}"
EXPECTED_UVC_SRCVERSION="${EXPECTED_UVC_SRCVERSION:-40AA64593CC5D0CC8766449}"
SEARCH_SUN_FILTER="${SEARCH_SUN_FILTER:-true}"
SEARCH_EXPOSURE_TIME="${SEARCH_EXPOSURE_TIME:-5}"
SEARCH_GAIN="${SEARCH_GAIN:-0}"
SEARCH_BRIGHTNESS="${SEARCH_BRIGHTNESS:--8}"
SEARCH_BACKLIGHT_COMPENSATION="${SEARCH_BACKLIGHT_COMPENSATION:-0}"

DISPLAY_SINK="${DISPLAY_SINK:-nveglglessink}"
SAVE_ROOT="${SAVE_ROOT:-$HOME/search_camera_test/recordings_power_safe}"

STEREO_COLOR_CORRECTION="${STEREO_COLOR_CORRECTION:-true}"
STEREO_COLOR_MATRIX="${STEREO_COLOR_MATRIX:-0.92,0,0,0,2.08,0,0,0,0.91}"
STEREO_COLOR_CONTRAST_PIVOT="${STEREO_COLOR_CONTRAST_PIVOT:-116.0}"
STEREO_COLOR_CONTRAST="${STEREO_COLOR_CONTRAST:-1.10}"
STEREO_MAGENTA_SUPPRESSION="${STEREO_MAGENTA_SUPPRESSION:-1.0}"
STEREO_COLOR_GAMMA="${STEREO_COLOR_GAMMA:-0.55}"
STEREO_COLOR_MIX="${STEREO_COLOR_MIX:-0.85}"
STEREO_EXPOSURE_COMPENSATION="${STEREO_EXPOSURE_COMPENSATION:-0.0}"
FALCON_GST_PLUGIN_DIR="${FALCON_GST_PLUGIN_DIR:-$HOME/Jetson_ROS2_V0/ros2_ws/install/falcon_intercept_deepstream/lib/gstreamer-1.0}"
export GST_PLUGIN_PATH="${FALCON_GST_PLUGIN_DIR}${GST_PLUGIN_PATH:+:${GST_PLUGIN_PATH}}"

MOSAIC_WIDTH="${MOSAIC_WIDTH:-1920}"
MOSAIC_HEIGHT="${MOSAIC_HEIGHT:-1080}"
MOSAIC_FORMAT="${MOSAIC_FORMAT:-RGBA}"
MIN_FREE_GIB="${MIN_FREE_GIB:-10}"
MIN_VALID_WALL_EPOCH="${MIN_VALID_WALL_EPOCH:-1704067200}"

CSI_RECORD_FPS=""
SESSION_DIR=""
STAMP=""
CLOCK_STATE=""
CLOCK_EPOCH=""
CLOCK_ISO=""
CLOCK_BOOT_ID=""
CLOCK_UPTIME_SECONDS=""

log() { printf '[power-safe-recorder] %s\n' "$*"; }
warn() { printf '[power-safe-recorder] WARNING: %s\n' "$*" >&2; }
die() { printf '[power-safe-recorder] ERROR: %s\n' "$*" >&2; exit 1; }
is_positive_integer() { [[ "$1" =~ ^[1-9][0-9]*$ ]]; }
safe_wall_iso()
{
    local epoch
    epoch="$(date +%s)"
    if [[ "$epoch" =~ ^[0-9]+$ ]] && (( epoch >= MIN_VALID_WALL_EPOCH )); then
        date --iso-8601=seconds
    else
        printf 'unavailable\n'
    fi
}

usage()
{
    cat <<EOF
Usage:
  $(basename "$0") check [--save-dir DIR]
  $(basename "$0") verify [--save-dir DIR] [--duration SEC]
  $(basename "$0") record [--save-dir DIR] [--duration SEC]
  $(basename "$0") record-preview [--save-dir DIR] [--duration SEC]

Power-loss tolerant output:
  <session>/left_sensor1_3840x2160_<fps>fps_color_corrected_mjpeg_%05d.mkv
  <session>/right_sensor0_3840x2160_<fps>fps_color_corrected_mjpeg_%05d.mkv
  <session>/usb_2560x1440_25fps_native_mjpeg_%05d.mkv

Options:
  --save-dir DIR
      Parent output directory. Default: ${SAVE_ROOT}

  --duration SEC
      Stop automatically after SEC seconds. Without this option, record until
      Ctrl+C. A sudden power cut still cannot finalize the last open segment.

  --segment-sec SEC
      Segment length in seconds. Default: ${SEGMENT_SECONDS}. Outdoor material
      collection should keep this at 5 unless storage overhead matters more than
      power-loss recovery.

  --csi-fps FPS
      CSI recording FPS. Default: 15.

  --quality N
      CSI JPEG quality, 1-100. Default: ${CSI_JPEG_QUALITY}.

  --search-auto
      Leave the USB search camera on automatic exposure. Useful indoors only.

  --search-exposure N
      Manual USB search-camera exposure. Outdoor sky starting point: 5.
      If overexposed, try 2 or 1. If too dark, try 10 then 20.

  --search-gain N
  --search-brightness N
  --search-backlight N
      Override USB search-camera UVC controls.

Recommended outdoor workflow:
  $(basename "$0") check
  $(basename "$0") verify --duration 8
  $(basename "$0") record-preview --search-exposure 5
  $(basename "$0") record --duration 1800 --search-exposure 5

Use Ctrl+C for a clean stop. In hard power loss, completed 5-second segments
should remain playable; inspect or discard only the last segment per stream.
EOF
}

parse_arguments()
{
    while (( $# > 0 )); do
        case "$1" in
            check|verify|record|record-preview|help)
                [[ "$MODE" == "help" ]] || die "Multiple modes supplied"
                MODE="$1"; shift ;;
            -h|--help)
                MODE="help"; shift ;;
            --save-dir)
                [[ $# -ge 2 ]] || die "--save-dir requires a directory"
                SAVE_DIR_CLI="$2"; shift 2 ;;
            --save-dir=*)
                SAVE_DIR_CLI="${1#*=}"; shift ;;
            --duration)
                [[ $# -ge 2 ]] || die "--duration requires seconds"
                DURATION_SECONDS="$2"; shift 2 ;;
            --duration=*)
                DURATION_SECONDS="${1#*=}"; shift ;;
            --segment-sec)
                [[ $# -ge 2 ]] || die "--segment-sec requires seconds"
                SEGMENT_SECONDS="$2"; shift 2 ;;
            --segment-sec=*)
                SEGMENT_SECONDS="${1#*=}"; shift ;;
            --csi-fps)
                [[ $# -ge 2 ]] || die "--csi-fps requires an integer"
                CSI_RECORD_FPS_CLI="$2"; shift 2 ;;
            --csi-fps=*)
                CSI_RECORD_FPS_CLI="${1#*=}"; shift ;;
            --quality)
                [[ $# -ge 2 ]] || die "--quality requires an integer"
                CSI_JPEG_QUALITY="$2"; shift 2 ;;
            --quality=*)
                CSI_JPEG_QUALITY="${1#*=}"; shift ;;
            --color-correction)
                STEREO_COLOR_CORRECTION=true; shift ;;
            --no-color-correction)
                STEREO_COLOR_CORRECTION=false; shift ;;
            --search-auto)
                SEARCH_SUN_FILTER=false; shift ;;
            --search-manual|--search-sun-filter)
                SEARCH_SUN_FILTER=true; shift ;;
            --search-exposure)
                [[ $# -ge 2 ]] || die "--search-exposure requires an integer"
                SEARCH_EXPOSURE_TIME="$2"; SEARCH_SUN_FILTER=true; shift 2 ;;
            --search-exposure=*)
                SEARCH_EXPOSURE_TIME="${1#*=}"; SEARCH_SUN_FILTER=true; shift ;;
            --search-gain)
                [[ $# -ge 2 ]] || die "--search-gain requires an integer"
                SEARCH_GAIN="$2"; SEARCH_SUN_FILTER=true; shift 2 ;;
            --search-gain=*)
                SEARCH_GAIN="${1#*=}"; SEARCH_SUN_FILTER=true; shift ;;
            --search-brightness)
                [[ $# -ge 2 ]] || die "--search-brightness requires an integer"
                SEARCH_BRIGHTNESS="$2"; SEARCH_SUN_FILTER=true; shift 2 ;;
            --search-brightness=*)
                SEARCH_BRIGHTNESS="${1#*=}"; SEARCH_SUN_FILTER=true; shift ;;
            --search-backlight)
                [[ $# -ge 2 ]] || die "--search-backlight requires an integer"
                SEARCH_BACKLIGHT_COMPENSATION="$2"; SEARCH_SUN_FILTER=true; shift 2 ;;
            --search-backlight=*)
                SEARCH_BACKLIGHT_COMPENSATION="${1#*=}"; SEARCH_SUN_FILTER=true; shift ;;
            *)
                die "Unknown argument: $1" ;;
        esac
    done

    [[ "$DURATION_SECONDS" == "0" ]] || is_positive_integer "$DURATION_SECONDS" \
        || die "--duration must be a positive integer"
    is_positive_integer "$SEGMENT_SECONDS" || die "--segment-sec must be a positive integer"
    (( SEGMENT_SECONDS <= 60 )) || die "--segment-sec must be <= 60"
    [[ -z "$CSI_RECORD_FPS_CLI" ]] || is_positive_integer "$CSI_RECORD_FPS_CLI" \
        || die "--csi-fps must be a positive integer"
    [[ -z "$CSI_RECORD_FPS_CLI" ]] || (( CSI_RECORD_FPS_CLI <= CSI_SOURCE_FPS )) \
        || die "--csi-fps cannot exceed ${CSI_SOURCE_FPS}"
    [[ "$CSI_JPEG_QUALITY" =~ ^[0-9]+$ ]] || die "--quality must be an integer"
    (( CSI_JPEG_QUALITY >= 1 && CSI_JPEG_QUALITY <= 100 )) \
        || die "--quality must be between 1 and 100"
    [[ "$FRONT_SENSOR_ID" =~ ^[0-9]+$ && "$BACK_SENSOR_ID" =~ ^[0-9]+$ ]] \
        || die "FRONT_SENSOR_ID and BACK_SENSOR_ID must be non-negative integers"
    [[ "$FRONT_SENSOR_ID" != "$BACK_SENSOR_ID" ]] \
        || die "FRONT_SENSOR_ID and BACK_SENSOR_ID must be different"
    [[ "$SEARCH_EXPOSURE_TIME" =~ ^[0-9]+$ ]] \
        || die "--search-exposure must be a non-negative integer"
    [[ "$SEARCH_GAIN" =~ ^-?[0-9]+$ ]] || die "--search-gain must be an integer"
    [[ "$SEARCH_BRIGHTNESS" =~ ^-?[0-9]+$ ]] || die "--search-brightness must be an integer"
    [[ "$SEARCH_BACKLIGHT_COMPENSATION" =~ ^-?[0-9]+$ ]] \
        || die "--search-backlight must be an integer"

    case "${STEREO_COLOR_CORRECTION,,}" in
        1|true|yes|on) STEREO_COLOR_CORRECTION=true ;;
        0|false|no|off) STEREO_COLOR_CORRECTION=false ;;
        *) die "STEREO_COLOR_CORRECTION must be true or false" ;;
    esac
    case "${SEARCH_SUN_FILTER,,}" in
        1|true|yes|on) SEARCH_SUN_FILTER=true ;;
        0|false|no|off) SEARCH_SUN_FILTER=false ;;
        *) die "SEARCH_SUN_FILTER must be true or false" ;;
    esac
}

has_plugin() { gst-inspect-1.0 "$1" >/dev/null 2>&1; }
require_plugin() { has_plugin "$1" || die "Missing GStreamer plugin: $1"; }

check_commands()
{
    local command_name
    for command_name in gst-launch-1.0 gst-inspect-1.0 v4l2-ctl awk grep df sha256sum timeout; do
        command -v "$command_name" >/dev/null 2>&1 || die "Missing command: $command_name"
    done
}

check_required_plugins()
{
    require_plugin nvarguscamerasrc
    require_plugin nvvidconv
    require_plugin v4l2src
    require_plugin jpegparse
    require_plugin matroskamux
    require_plugin splitmuxsink
    require_plugin nvjpegenc
    require_plugin videorate
    require_plugin queue
    require_plugin tee
    require_plugin falconcolor
    if [[ "$MODE" == "record-preview" ]]; then
        require_plugin nvcompositor
        require_plugin nvv4l2decoder
        require_plugin "$DISPLAY_SINK"
    fi
}

configure_display_environment()
{
    if [[ "$MODE" != "record-preview" ]]; then
        return
    fi

    if [[ -z "${DISPLAY:-}" && -S /tmp/.X11-unix/X0 ]]; then
        export DISPLAY=:0
        log "DISPLAY was empty; using Jetson desktop DISPLAY=${DISPLAY}"
    fi

    if [[ -z "${XAUTHORITY:-}" ]]; then
        local candidate="/run/user/$(id -u)/gdm/Xauthority"
        if [[ -r "$candidate" ]]; then
            export XAUTHORITY="$candidate"
            log "XAUTHORITY was empty; using ${XAUTHORITY}"
        elif [[ -r "$HOME/.Xauthority" ]]; then
            export XAUTHORITY="$HOME/.Xauthority"
            log "XAUTHORITY was empty; using ${XAUTHORITY}"
        fi
    fi

    [[ -n "${DISPLAY:-}" ]] \
        || die "record-preview needs a Jetson desktop DISPLAY. Use record for SSH/headless recording."
}

check_uvc_v2()
{
    local loaded=""
    if [[ -r /sys/module/uvcvideo/srcversion ]]; then
        loaded="$(cat /sys/module/uvcvideo/srcversion)"
    fi
    [[ "$loaded" == "$EXPECTED_UVC_SRCVERSION" ]] \
        || die "HJ4K V2 uvcvideo is not loaded. Expected $EXPECTED_UVC_SRCVERSION, loaded ${loaded:-not loaded}"
}

configure_search_camera()
{
    if [[ "$SEARCH_SUN_FILTER" == "true" ]]; then
        log "Search electronic ND: manual exposure=${SEARCH_EXPOSURE_TIME}, gain=${SEARCH_GAIN}, brightness=${SEARCH_BRIGHTNESS}, backlight=${SEARCH_BACKLIGHT_COMPENSATION}"
        v4l2-ctl -d "$USB_DEVICE" --set-ctrl=auto_exposure=1
        v4l2-ctl -d "$USB_DEVICE" --set-ctrl=exposure_dynamic_framerate=0
        v4l2-ctl -d "$USB_DEVICE" --set-ctrl="exposure_time_absolute=${SEARCH_EXPOSURE_TIME},gain=${SEARCH_GAIN},brightness=${SEARCH_BRIGHTNESS},backlight_compensation=${SEARCH_BACKLIGHT_COMPENSATION}"
    else
        log "Search electronic ND: disabled (camera auto exposure)"
    fi
}

restore_search_camera()
{
    if [[ "$SEARCH_SUN_FILTER" == "true" ]]; then
        v4l2-ctl -d "$USB_DEVICE" --set-ctrl=auto_exposure=3 >/dev/null 2>&1 || true
        v4l2-ctl -d "$USB_DEVICE" --set-ctrl=exposure_dynamic_framerate=1 >/dev/null 2>&1 || true
        log "Search camera exposure restored to auto"
    fi
}

find_imx477_nodes()
{
    local dev node_name
    IMX477_NODES=()
    for dev in /dev/video*; do
        node_name="$(cat "/sys/class/video4linux/${dev##*/}/name" 2>/dev/null || true)"
        if [[ "$node_name" == vi-output*,\ imx477* ]]; then
            IMX477_NODES+=("$dev")
        fi
    done
    (( ${#IMX477_NODES[@]} == 2 )) \
        || die "Expected exactly 2 IMX477 image nodes; found ${#IMX477_NODES[@]}"
}

check_csi_mode()
{
    local dev="$1" listing
    listing="$(v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null)"
    grep -q "'RG10'" <<< "$listing" || die "$dev does not advertise RG10 Bayer"
    awk '
        /Size: Discrete 3840x2160/ { in_mode=1; next }
        /Size: Discrete/ && in_mode { in_mode=0 }
        in_mode && /30.000 fps/ { found=1 }
        END { exit(found ? 0 : 1) }
    ' <<< "$listing" || die "$dev does not advertise 3840x2160@30"
    log "$dev: RG10 3840x2160@30 advertised"
}

check_usb_mode()
{
    [[ -e "$USB_DEVICE" ]] || die "USB camera device does not exist: $USB_DEVICE"
    local listing
    listing="$(v4l2-ctl -d "$USB_DEVICE" --list-formats-ext 2>/dev/null)"
    awk '
        /Size: Discrete 2560x1440/ { in_mode=1; next }
        /Size: Discrete/ && in_mode { in_mode=0 }
        in_mode && /25.000 fps/ { found=1 }
        END { exit(found ? 0 : 1) }
    ' <<< "$listing" || die "USB camera does not advertise 2560x1440 MJPEG@25"
    log "USB: MJPEG 2560x1440@25 advertised"
}

prepare_parent_directory()
{
    local parent_dir
    if [[ -n "$SAVE_DIR_CLI" ]]; then
        parent_dir="$SAVE_DIR_CLI"
    else
        parent_dir="$SAVE_ROOT"
    fi
    mkdir -p "$parent_dir"
    SAVE_ROOT_RESOLVED="$(readlink -f "$parent_dir")"
    [[ -w "$SAVE_ROOT_RESOLVED" ]] || die "Output parent directory is not writable: $SAVE_ROOT_RESOLVED"
}

check_storage()
{
    local available_kib available_gib
    available_kib="$(df -Pk "$SAVE_ROOT_RESOLVED" | awk 'NR==2 {print $4}')"
    [[ "$available_kib" =~ ^[0-9]+$ ]] || die "Could not determine free disk space"
    available_gib=$(( available_kib / 1024 / 1024 ))
    log "Free storage: ${available_gib} GiB at $SAVE_ROOT_RESOLVED"
    (( available_gib >= MIN_FREE_GIB )) || die "At least ${MIN_FREE_GIB} GiB free space is required"
}

run_checks()
{
    check_commands
    configure_display_environment
    check_required_plugins
    check_uvc_v2
    find_imx477_nodes
    check_csi_mode "${IMX477_NODES[0]}"
    check_csi_mode "${IMX477_NODES[1]}"
    check_usb_mode
    prepare_parent_directory
    check_storage
    CSI_RECORD_FPS="${CSI_RECORD_FPS_CLI:-15}"
}

print_configuration()
{
    cat <<EOF
Power-safe recording configuration:
  Logical left:     ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_SOURCE_FPS}, sensor-id=${FRONT_SENSOR_ID}
  Logical right:    ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_SOURCE_FPS}, sensor-id=${BACK_SENSOR_ID}
  CSI recording:    ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_RECORD_FPS}, MJPEG, segment=${SEGMENT_SECONDS}s
  CSI JPEG quality: ${CSI_JPEG_QUALITY}
  Stereo color:     ${STEREO_COLOR_CORRECTION}
  Search exposure:  manual=${SEARCH_SUN_FILTER} exposure=${SEARCH_EXPOSURE_TIME}, gain=${SEARCH_GAIN}, brightness=${SEARCH_BRIGHTNESS}, backlight=${SEARCH_BACKLIGHT_COMPENSATION}
  USB source/save:  ${USB_WIDTH}x${USB_HEIGHT} MJPEG@${USB_FPS}, native remux, segment=${SEGMENT_SECONDS}s
  Output parent:    ${SAVE_ROOT_RESOLVED}
  Duration:         $(if (( DURATION_SECONDS > 0 )); then printf '%s seconds' "$DURATION_SECONDS"; else printf 'until Ctrl+C'; fi)
EOF
}

create_session_directory()
{
    local prefix="${1:-record-power-safe}" color_tag="uncorrected"
    CLOCK_EPOCH="$(date +%s)"
    CLOCK_BOOT_ID="$(cat /proc/sys/kernel/random/boot_id 2>/dev/null || printf unknown)"
    CLOCK_UPTIME_SECONDS="$(cut -d. -f1 /proc/uptime 2>/dev/null || printf 0)"
    if [[ "$CLOCK_EPOCH" =~ ^[0-9]+$ ]] && (( CLOCK_EPOCH >= MIN_VALID_WALL_EPOCH )); then
        CLOCK_STATE="valid"
        CLOCK_ISO="$(date --iso-8601=seconds)"
        STAMP="$(date '+%Y%m%d-%H%M%S')"
    else
        CLOCK_STATE="unsynchronized"
        CLOCK_ISO="unavailable"
        STAMP="unsynced-${CLOCK_BOOT_ID:0:8}-uptime${CLOCK_UPTIME_SECONDS}s"
        warn "System wall clock is invalid (${CLOCK_EPOCH}); using unique boot/uptime session stamp"
        warn "Run /usr/local/sbin/falcon-clock-guard status before collecting formal evidence"
    fi
    SESSION_DIR="$SAVE_ROOT_RESOLVED/${prefix}-${STAMP}"
    mkdir -p "$SESSION_DIR"
    SESSION_DIR="$(readlink -f "$SESSION_DIR")"
    [[ "$STEREO_COLOR_CORRECTION" == "true" ]] && color_tag="color_corrected"
    LEFT_PATTERN="$SESSION_DIR/front_sensor${FRONT_SENSOR_ID}_${CSI_WIDTH}x${CSI_HEIGHT}_${CSI_RECORD_FPS}fps_${color_tag}_mjpeg_%05d.mkv"
    RIGHT_PATTERN="$SESSION_DIR/back_sensor${BACK_SENSOR_ID}_${CSI_WIDTH}x${CSI_HEIGHT}_${CSI_RECORD_FPS}fps_${color_tag}_mjpeg_%05d.mkv"
    USB_PATTERN="$SESSION_DIR/usb_${USB_WIDTH}x${USB_HEIGHT}_${USB_FPS}fps_native_mjpeg_%05d.mkv"
    cat > "$SESSION_DIR/session.info" <<EOF
created_at=$CLOCK_ISO
clock_state=$CLOCK_STATE
clock_epoch=$CLOCK_EPOCH
clock_boot_id=$CLOCK_BOOT_ID
clock_uptime_seconds=$CLOCK_UPTIME_SECONDS
hostname=$(hostname)
kernel=$(uname -r)
uvc_srcversion=$(cat /sys/module/uvcvideo/srcversion 2>/dev/null || true)

power_loss_tolerant=true
segment_seconds=$SEGMENT_SECONDS
container=matroska
splitmuxsink_async_finalize=true
matroska_streamable=true
expected_power_loss_behavior=completed_segments_playable_last_open_segment_at_risk

front_sensor_id=$FRONT_SENSOR_ID
back_sensor_id=$BACK_SENSOR_ID
left_physical_position=rig_left
right_physical_position=rig_right
csi_sensor_mode=$CSI_SENSOR_MODE
csi_source_width=$CSI_WIDTH
csi_source_height=$CSI_HEIGHT
csi_source_fps=$CSI_SOURCE_FPS
csi_record_width=$CSI_WIDTH
csi_record_height=$CSI_HEIGHT
csi_record_fps=$CSI_RECORD_FPS
csi_encoder=nvjpegenc
csi_jpeg_quality=$CSI_JPEG_QUALITY
csi_color_correction=$STEREO_COLOR_CORRECTION
csi_color_matrix=$STEREO_COLOR_MATRIX
csi_color_contrast_pivot=$STEREO_COLOR_CONTRAST_PIVOT
csi_color_contrast=$STEREO_COLOR_CONTRAST
csi_magenta_suppression=$STEREO_MAGENTA_SUPPRESSION
csi_color_gamma=$STEREO_COLOR_GAMMA
csi_color_mix=$STEREO_COLOR_MIX
csi_exposure_compensation=$STEREO_EXPOSURE_COMPENSATION
search_sun_filter=$SEARCH_SUN_FILTER
search_exposure_time=$SEARCH_EXPOSURE_TIME
search_gain=$SEARCH_GAIN
search_brightness=$SEARCH_BRIGHTNESS
search_backlight_compensation=$SEARCH_BACKLIGHT_COMPENSATION
usb_device=$USB_DEVICE
usb_width=$USB_WIDTH
usb_height=$USB_HEIGHT
usb_fps=$USB_FPS
usb_codec=native_mjpeg_no_reencode
preview=$([[ "$MODE" == "record-preview" ]] && printf enabled || printf disabled)
EOF
    log "Session directory: $SESSION_DIR"
}

append_csi_record_branch()
{
    local sensor_id="$1" label="$2" pattern="$3"
    GST_COMMAND+=(
        nvarguscamerasrc
            sensor-id="$sensor_id"
            sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1
            awblock=false
            exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        !
        videorate drop-only=true
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label="${label}-record"
        !
        nvjpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=16 max-size-bytes=0 max-size-time=0
        !
        splitmuxsink location="$pattern"
            max-size-time="$(( SEGMENT_SECONDS * 1000000000 ))"
            max-size-bytes=0
            async-finalize=true
            muxer-factory=matroskamux
            muxer-properties="properties,streamable=true"
    )
}

build_record_only_command()
{
    GST_COMMAND=(gst-launch-1.0 -e)
    append_csi_record_branch "$FRONT_SENSOR_ID" left "$LEFT_PATTERN"
    append_csi_record_branch "$BACK_SENSOR_ID" right "$RIGHT_PATTERN"
    GST_COMMAND+=(
        v4l2src device="$USB_DEVICE" io-mode=2
        !
        "image/jpeg,width=${USB_WIDTH},height=${USB_HEIGHT},framerate=${USB_FPS}/1"
        !
        jpegparse
        !
        queue max-size-buffers=32 max-size-bytes=0 max-size-time=0
        !
        splitmuxsink location="$USB_PATTERN"
            max-size-time="$(( SEGMENT_SECONDS * 1000000000 ))"
            max-size-bytes=0
            async-finalize=true
            muxer-factory=matroskamux
            muxer-properties="properties,streamable=true"
    )
}

build_record_preview_command()
{
    GST_COMMAND=(
        gst-launch-1.0 -e
        nvcompositor name=comp
            sink_0::xpos=0 sink_0::ypos=0 sink_0::width=960 sink_0::height=540
            sink_1::xpos=960 sink_1::ypos=0 sink_1::width=960 sink_1::height=540
            sink_2::xpos=480 sink_2::ypos=540 sink_2::width=960 sink_2::height=540
        !
        "video/x-raw(memory:NVMM),width=${MOSAIC_WIDTH},height=${MOSAIC_HEIGHT},format=${MOSAIC_FORMAT}"
        !
        "$DISPLAY_SINK" sync=false

        nvarguscamerasrc sensor-id="$FRONT_SENSOR_ID" sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1 awblock=false exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label=left-shared
        !
        tee name=left_stream
        left_stream. ! queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        ! videorate drop-only=true
        ! "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        ! nvjpegenc quality="$CSI_JPEG_QUALITY" ! jpegparse
        ! splitmuxsink location="$LEFT_PATTERN" max-size-time="$(( SEGMENT_SECONDS * 1000000000 ))"
            max-size-bytes=0 async-finalize=true muxer-factory=matroskamux
            muxer-properties="properties,streamable=true"
        left_stream. ! queue max-size-buffers=2 leaky=downstream
        ! nvvidconv interpolation-method=2
        ! "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        ! comp.sink_0

        nvarguscamerasrc sensor-id="$BACK_SENSOR_ID" sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1 awblock=false exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label=right-shared
        !
        tee name=right_stream
        right_stream. ! queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        ! videorate drop-only=true
        ! "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        ! nvjpegenc quality="$CSI_JPEG_QUALITY" ! jpegparse
        ! splitmuxsink location="$RIGHT_PATTERN" max-size-time="$(( SEGMENT_SECONDS * 1000000000 ))"
            max-size-bytes=0 async-finalize=true muxer-factory=matroskamux
            muxer-properties="properties,streamable=true"
        right_stream. ! queue max-size-buffers=2 leaky=downstream
        ! nvvidconv interpolation-method=2
        ! "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        ! comp.sink_1

        v4l2src device="$USB_DEVICE" io-mode=2
        !
        "image/jpeg,width=${USB_WIDTH},height=${USB_HEIGHT},framerate=${USB_FPS}/1"
        !
        jpegparse
        !
        tee name=usb
        usb. ! queue max-size-buffers=32 max-size-bytes=0 max-size-time=0
        ! splitmuxsink location="$USB_PATTERN" max-size-time="$(( SEGMENT_SECONDS * 1000000000 ))"
            max-size-bytes=0 async-finalize=true muxer-factory=matroskamux
            muxer-properties="properties,streamable=true"
        usb. ! queue max-size-buffers=2 leaky=downstream
        ! nvv4l2decoder mjpeg=1
        ! nvvidconv interpolation-method=2
        ! "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        ! comp.sink_2
    )
}

run_gstreamer()
{
    local rc=0
    log "Starting power-safe segmented recording"
    log "Left pattern (sensor ${FRONT_SENSOR_ID}):  $LEFT_PATTERN"
    log "Right pattern (sensor ${BACK_SENSOR_ID}): $RIGHT_PATTERN"
    log "USB pattern:  $USB_PATTERN"
    if (( DURATION_SECONDS > 0 )); then
        log "Automatic stop after ${DURATION_SECONDS} seconds"
        set +e
        timeout --foreground --signal=INT --kill-after=15s \
            "${DURATION_SECONDS}s" "${GST_COMMAND[@]}"
        rc=$?
        set -e
        [[ "$rc" -eq 124 ]] && rc=0
    else
        log "Press Ctrl+C to stop cleanly. Completed segments are already finalized during recording."
        set +e
        "${GST_COMMAND[@]}"
        rc=$?
        set -e
        [[ "$rc" -eq 130 ]] && rc=0
    fi
    sync
    (( rc == 0 )) || warn "GStreamer exited with status $rc"
    return "$rc"
}

validate_segments_for_prefix()
{
    local glob_pattern="$1" expected_width="$2" expected_height="$3" label="$4"
    local failed=0 video
    shopt -s nullglob
    local videos=( $glob_pattern )
    shopt -u nullglob
    if (( ${#videos[@]} == 0 )); then
        warn "Missing output segments for ${label}"
        return 1
    fi
    : > "$SESSION_DIR/${label}.discoverer.txt"
    for video in "${videos[@]}"; do
        [[ -s "$video" ]] || { warn "Empty segment: $video"; failed=1; continue; }
        if command -v gst-discoverer-1.0 >/dev/null 2>&1; then
            if ! gst-discoverer-1.0 "$video" >> "$SESSION_DIR/${label}.discoverer.txt" 2>&1; then
                warn "Could not parse segment: $video"
                failed=1
                continue
            fi
            grep -Eq "Width:[[:space:]]+${expected_width}" "$SESSION_DIR/${label}.discoverer.txt" \
                || failed=1
            grep -Eq "Height:[[:space:]]+${expected_height}" "$SESSION_DIR/${label}.discoverer.txt" \
                || failed=1
        fi
    done
    log "Validated ${label}: ${#videos[@]} segment(s)"
    return "$failed"
}

validate_outputs()
{
    local failed=0
    validate_segments_for_prefix "$SESSION_DIR/front_sensor${FRONT_SENSOR_ID}_${CSI_WIDTH}x${CSI_HEIGHT}_${CSI_RECORD_FPS}fps_"'*.mkv' "$CSI_WIDTH" "$CSI_HEIGHT" left || failed=1
    validate_segments_for_prefix "$SESSION_DIR/back_sensor${BACK_SENSOR_ID}_${CSI_WIDTH}x${CSI_HEIGHT}_${CSI_RECORD_FPS}fps_"'*.mkv' "$CSI_WIDTH" "$CSI_HEIGHT" right || failed=1
    validate_segments_for_prefix "$SESSION_DIR/usb_${USB_WIDTH}x${USB_HEIGHT}_${USB_FPS}fps_native_mjpeg_"'*.mkv' "$USB_WIDTH" "$USB_HEIGHT" usb || failed=1
    if (( failed == 0 )); then
        ( cd "$SESSION_DIR" && sha256sum *.mkv > SHA256SUMS )
        log "All segmented video files passed validation"
    fi
    return "$failed"
}

run_check_mode()
{
    run_checks
    print_configuration
    log "CHECK PASS"
}

run_record_mode()
{
    local pipeline_rc=0 validation_rc=0 prefix="record-power-safe"
    run_checks
    if [[ "$MODE" == "verify" ]]; then
        DURATION_SECONDS="${DURATION_SECONDS:-0}"
        (( DURATION_SECONDS > 0 )) || DURATION_SECONDS="$VERIFY_SECONDS"
        prefix="verify-power-safe"
        log "Verification will record ${DURATION_SECONDS} seconds"
    elif [[ "$MODE" == "record-preview" ]]; then
        prefix="record-preview-power-safe"
    fi
    print_configuration
    create_session_directory "$prefix"
    configure_search_camera
    if [[ "$MODE" == "record-preview" ]]; then
        build_record_preview_command
    else
        build_record_only_command
    fi
    run_gstreamer || pipeline_rc=$?
    restore_search_camera
    validate_outputs || validation_rc=$?
    cat >> "$SESSION_DIR/session.info" <<EOF

pipeline_exit_status=$pipeline_rc
validation_exit_status=$validation_rc
completed_at=$(safe_wall_iso)
completed_epoch=$(date +%s)
EOF
    log "Session: $SESSION_DIR"
    if (( pipeline_rc == 0 && validation_rc == 0 )); then
        [[ "$MODE" == "verify" ]] && log "VERIFY PASS" || log "RECORDING COMPLETE"
        return 0
    fi
    die "Recording did not pass validation; inspect $SESSION_DIR"
}

main()
{
    parse_arguments "$@"
    case "$MODE" in
        help) usage ;;
        check) run_check_mode ;;
        verify|record|record-preview) run_record_mode ;;
        *) die "Unsupported mode: $MODE" ;;
    esac
}

main "$@"
