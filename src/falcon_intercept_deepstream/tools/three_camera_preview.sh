#!/usr/bin/env bash
set -Eeuo pipefail

# ==============================================================================
# Final three-camera recorder for Jetson
#
# SOURCE CAPTURE STANDARD
#   LEFT  IMX477: Argus sensor 1, 3840x2160 @ 30 fps
#   RIGHT IMX477: Argus sensor 0, 3840x2160 @ 30 fps
#   USB HJ 4K:   2560x1440 MJPEG @ 25 fps
#
# RECORDING STANDARD
#   LEFT/RIGHT CSI:
#     Preserve 3840x2160 resolution.
#     Apply the shared stereo correction before encoding.
#     Reduce recording frame rate when required.
#     Prefer Jetson nvjpegenc; fall back to software jpegenc.
#     Store Motion JPEG in Matroska (.mkv).
#
#   USB:
#     Preserve the camera's native 2560x1440 MJPEG @ 25 fps.
#     Do not decode or re-encode for recording.
#     Store native Motion JPEG in Matroska (.mkv).
#
# MODES
#   check            Validate devices, modes, plugins, V2 UVC module and storage.
#   verify           Record an 8-second three-camera sample and validate files.
#   record           Record all three streams without preview (recommended).
#   record-preview   Record all three streams and show a low-resolution mosaic.
#
# EXAMPLES
#   ./three_camera_final_record.sh check
#   ./three_camera_final_record.sh verify
#   ./three_camera_final_record.sh record --save-dir ~/camera_records
#   ./three_camera_final_record.sh record --duration 60
#   ./three_camera_final_record.sh record-preview --csi-fps 10
#
# IMPORTANT
#   The same correction is applied once before tee, so preview and recording
#   see identical pixels and do not race on the same NVMM surface.
#   Use Ctrl+C to stop. gst-launch-1.0 -e will send EOS and finalize the files.
# ==============================================================================

MODE="help"
SAVE_DIR_CLI=""
DURATION_SECONDS=0
CSI_RECORD_FPS_CLI=""
CSI_JPEG_QUALITY="${CSI_JPEG_QUALITY:-85}"
VERIFY_SECONDS="${VERIFY_SECONDS:-8}"

# Physical lens-occlusion verification (2026-08-01): sensor 1 is rig-left
# and sensor 0 is rig-right.  Use logical names so connector enumeration is
# never confused with stereo geometry again.
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
SAVE_ROOT="${SAVE_ROOT:-$HOME/search_camera_test/recordings}"

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
# nvcompositor on this JetPack build exposes RGBA on its source pad.
MOSAIC_FORMAT="${MOSAIC_FORMAT:-RGBA}"

MIN_FREE_GIB="${MIN_FREE_GIB:-10}"
MIN_VALID_WALL_EPOCH="${MIN_VALID_WALL_EPOCH:-1704067200}"

CSI_ENCODER=""
CSI_ENCODER_DESCRIPTION=""
CSI_RECORD_FPS=""
SESSION_DIR=""
STAMP=""
CLOCK_STATE=""
CLOCK_EPOCH=""
CLOCK_ISO=""
CLOCK_BOOT_ID=""
CLOCK_UPTIME_SECONDS=""

log()
{
    printf '[final-recorder] %s\n' "$*"
}

warn()
{
    printf '[final-recorder] WARNING: %s\n' "$*" >&2
}

die()
{
    printf '[final-recorder] ERROR: %s\n' "$*" >&2
    exit 1
}

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

is_positive_integer()
{
    [[ "$1" =~ ^[1-9][0-9]*$ ]]
}

usage()
{
    cat <<EOF
Usage:
  $(basename "$0") check [--save-dir DIR]
  $(basename "$0") verify [--save-dir DIR] [--csi-fps FPS] [--quality 1-100]
  $(basename "$0") record [--save-dir DIR] [--duration SEC] [--csi-fps FPS]
  $(basename "$0") record-preview [--save-dir DIR] [--duration SEC] [--csi-fps FPS]

Final camera source modes:
  LEFT:  ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_SOURCE_FPS}, sensor-id=${FRONT_SENSOR_ID}, sensor-mode=${CSI_SENSOR_MODE}
  RIGHT: ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_SOURCE_FPS}, sensor-id=${BACK_SENSOR_ID}, sensor-mode=${CSI_SENSOR_MODE}
  USB:  ${USB_WIDTH}x${USB_HEIGHT} MJPEG@${USB_FPS}

Options:
  --save-dir DIR
      Parent output directory. Each run creates a timestamped session below it.

  --duration SEC
      Stop automatically after SEC seconds. Without this option, record until
      Ctrl+C.

  --csi-fps FPS
      CSI recording frame rate. The CSI sensors still capture 4K30.
      nvjpegenc default: 15 fps
      software jpegenc default: 5 fps

  --quality N
      CSI JPEG quality, 1-100. Default: ${CSI_JPEG_QUALITY}.

  --color-correction
      Enable the shared stereo correction (default).

  --no-color-correction
      Preserve the uncorrected Argus colors.

  --search-auto
      Leave the USB search camera on automatic exposure. Useful indoors.

  --search-exposure N
      Use manual exposure_time_absolute=N for the USB search camera.
      Default: ${SEARCH_EXPOSURE_TIME}. Outdoor sky starting point: 5.
      If overexposed, try 2 or 1. If too dark, try 10 then 20.

  --search-gain N
      Manual USB search camera gain. Default: ${SEARCH_GAIN}.

  --search-brightness N
      USB search camera brightness control. Default: ${SEARCH_BRIGHTNESS}.

  --search-backlight N
      USB search camera backlight_compensation control.
      Default: ${SEARCH_BACKLIGHT_COMPENSATION}.

Output:
  <session>/left_sensor1_3840x2160_<fps>fps_<color>_mjpeg.mkv
  <session>/right_sensor0_3840x2160_<fps>fps_<color>_mjpeg.mkv
  <session>/usb_2560x1440_25fps_native_mjpeg.mkv
  <session>/session.info
  <session>/*.discoverer.txt
  <session>/SHA256SUMS

Recommended:
  $(basename "$0") check
  $(basename "$0") verify
  $(basename "$0") record-preview --search-exposure 5
  $(basename "$0") record --search-exposure 5 --save-dir "$HOME/search_camera_test/recordings"

Use Ctrl+C to stop. Do not close the preview window to stop recording.
EOF
}

parse_arguments()
{
    while (( $# > 0 )); do
        case "$1" in
            check|verify|record|record-preview|help)
                if [[ "$MODE" != "help" ]]; then
                    die "Multiple modes supplied: $MODE and $1"
                fi
                MODE="$1"
                shift
                ;;
            -h|--help)
                MODE="help"
                shift
                ;;
            --save-dir)
                [[ $# -ge 2 ]] || die "--save-dir requires a directory"
                SAVE_DIR_CLI="$2"
                shift 2
                ;;
            --save-dir=*)
                SAVE_DIR_CLI="${1#*=}"
                shift
                ;;
            --duration)
                [[ $# -ge 2 ]] || die "--duration requires seconds"
                DURATION_SECONDS="$2"
                shift 2
                ;;
            --duration=*)
                DURATION_SECONDS="${1#*=}"
                shift
                ;;
            --csi-fps)
                [[ $# -ge 2 ]] || die "--csi-fps requires an integer"
                CSI_RECORD_FPS_CLI="$2"
                shift 2
                ;;
            --csi-fps=*)
                CSI_RECORD_FPS_CLI="${1#*=}"
                shift
                ;;
            --quality)
                [[ $# -ge 2 ]] || die "--quality requires an integer"
                CSI_JPEG_QUALITY="$2"
                shift 2
                ;;
            --quality=*)
                CSI_JPEG_QUALITY="${1#*=}"
                shift
                ;;
            --color-correction)
                STEREO_COLOR_CORRECTION=true
                shift
                ;;
            --no-color-correction)
                STEREO_COLOR_CORRECTION=false
                shift
                ;;
            --search-auto)
                SEARCH_SUN_FILTER=false
                shift
                ;;
            --search-manual|--search-sun-filter)
                SEARCH_SUN_FILTER=true
                shift
                ;;
            --search-exposure)
                [[ $# -ge 2 ]] || die "--search-exposure requires an integer"
                SEARCH_EXPOSURE_TIME="$2"
                SEARCH_SUN_FILTER=true
                shift 2
                ;;
            --search-exposure=*)
                SEARCH_EXPOSURE_TIME="${1#*=}"
                SEARCH_SUN_FILTER=true
                shift
                ;;
            --search-gain)
                [[ $# -ge 2 ]] || die "--search-gain requires an integer"
                SEARCH_GAIN="$2"
                SEARCH_SUN_FILTER=true
                shift 2
                ;;
            --search-gain=*)
                SEARCH_GAIN="${1#*=}"
                SEARCH_SUN_FILTER=true
                shift
                ;;
            --search-brightness)
                [[ $# -ge 2 ]] || die "--search-brightness requires an integer"
                SEARCH_BRIGHTNESS="$2"
                SEARCH_SUN_FILTER=true
                shift 2
                ;;
            --search-brightness=*)
                SEARCH_BRIGHTNESS="${1#*=}"
                SEARCH_SUN_FILTER=true
                shift
                ;;
            --search-backlight)
                [[ $# -ge 2 ]] || die "--search-backlight requires an integer"
                SEARCH_BACKLIGHT_COMPENSATION="$2"
                SEARCH_SUN_FILTER=true
                shift 2
                ;;
            --search-backlight=*)
                SEARCH_BACKLIGHT_COMPENSATION="${1#*=}"
                SEARCH_SUN_FILTER=true
                shift
                ;;
            *)
                die "Unknown argument: $1"
                ;;
        esac
    done

    if [[ "$DURATION_SECONDS" != "0" ]]; then
        is_positive_integer "$DURATION_SECONDS" \
            || die "--duration must be a positive integer"
    fi

    if [[ -n "$CSI_RECORD_FPS_CLI" ]]; then
        is_positive_integer "$CSI_RECORD_FPS_CLI" \
            || die "--csi-fps must be a positive integer"

        (( CSI_RECORD_FPS_CLI <= CSI_SOURCE_FPS )) \
            || die "--csi-fps cannot exceed ${CSI_SOURCE_FPS}"
    fi

    [[ "$CSI_JPEG_QUALITY" =~ ^[0-9]+$ ]] \
        || die "--quality must be an integer"

    (( CSI_JPEG_QUALITY >= 1 && CSI_JPEG_QUALITY <= 100 )) \
        || die "--quality must be between 1 and 100"

    [[ "$FRONT_SENSOR_ID" =~ ^[0-9]+$ && "$BACK_SENSOR_ID" =~ ^[0-9]+$ ]] \
        || die "FRONT_SENSOR_ID and BACK_SENSOR_ID must be non-negative integers"
    [[ "$FRONT_SENSOR_ID" != "$BACK_SENSOR_ID" ]] \
        || die "FRONT_SENSOR_ID and BACK_SENSOR_ID must be different"

    [[ "$SEARCH_EXPOSURE_TIME" =~ ^[0-9]+$ ]] \
        || die "--search-exposure must be a non-negative integer"
    [[ "$SEARCH_GAIN" =~ ^-?[0-9]+$ ]] \
        || die "--search-gain must be an integer"
    [[ "$SEARCH_BRIGHTNESS" =~ ^-?[0-9]+$ ]] \
        || die "--search-brightness must be an integer"
    [[ "$SEARCH_BACKLIGHT_COMPENSATION" =~ ^-?[0-9]+$ ]] \
        || die "--search-backlight must be an integer"

    case "${STEREO_COLOR_CORRECTION,,}" in
        1|true|yes|on)
            STEREO_COLOR_CORRECTION=true
            ;;
        0|false|no|off)
            STEREO_COLOR_CORRECTION=false
            ;;
        *)
            die "STEREO_COLOR_CORRECTION must be true or false"
            ;;
    esac
}

has_plugin()
{
    gst-inspect-1.0 "$1" >/dev/null 2>&1
}

require_plugin()
{
    has_plugin "$1" || die "Missing GStreamer plugin: $1"
}

check_commands()
{
    local command_name

    for command_name in \
        gst-launch-1.0 \
        gst-inspect-1.0 \
        v4l2-ctl \
        awk \
        grep \
        df \
        sha256sum \
        timeout
    do
        command -v "$command_name" >/dev/null 2>&1 \
            || die "Missing command: $command_name"
    done
}

select_csi_encoder()
{
    if has_plugin nvjpegenc; then
        CSI_ENCODER="nvjpegenc"
        CSI_ENCODER_DESCRIPTION="Jetson JPEG hardware accelerator"

        if [[ -n "$CSI_RECORD_FPS_CLI" ]]; then
            CSI_RECORD_FPS="$CSI_RECORD_FPS_CLI"
        else
            CSI_RECORD_FPS=15
        fi
    elif has_plugin jpegenc; then
        CSI_ENCODER="jpegenc"
        CSI_ENCODER_DESCRIPTION="CPU libjpeg fallback"

        if [[ -n "$CSI_RECORD_FPS_CLI" ]]; then
            CSI_RECORD_FPS="$CSI_RECORD_FPS_CLI"
        else
            CSI_RECORD_FPS=5
        fi

        warn "nvjpegenc is unavailable; using CPU jpegenc"
        warn "Keep CSI recording at 5 fps or lower unless verify succeeds"
    else
        die "Neither nvjpegenc nor jpegenc is available"
    fi
}

check_required_plugins()
{
    require_plugin nvarguscamerasrc
    require_plugin nvvidconv
    require_plugin v4l2src
    require_plugin jpegparse
    require_plugin matroskamux
    require_plugin filesink
    require_plugin videorate
    require_plugin queue
    require_plugin tee
    require_plugin fakesink
    require_plugin falconcolor

    if [[ "$MODE" == "record-preview" ]]; then
        require_plugin nvcompositor
        require_plugin nvv4l2decoder
        require_plugin "$DISPLAY_SINK"
    fi
}

check_uvc_v2()
{
    local loaded=""

    if [[ -r /sys/module/uvcvideo/srcversion ]]; then
        loaded="$(cat /sys/module/uvcvideo/srcversion)"
    fi

    [[ "$loaded" == "$EXPECTED_UVC_SRCVERSION" ]] || {
        cat >&2 <<EOF
[final-recorder] ERROR: HJ4K V2 uvcvideo is not loaded.
[final-recorder] Expected: $EXPECTED_UVC_SRCVERSION
[final-recorder] Loaded:   ${loaded:-not loaded}
[final-recorder] Refusing to record because the original module can generate
[final-recorder] fragmented or torn MJPEG frames.
EOF
        exit 1
    }
}

configure_search_camera()
{
    case "${SEARCH_SUN_FILTER,,}" in
        true|1|yes|on)
            SEARCH_SUN_FILTER=true
            log "Search electronic ND: manual exposure=${SEARCH_EXPOSURE_TIME}, gain=${SEARCH_GAIN}, brightness=${SEARCH_BRIGHTNESS}, backlight=${SEARCH_BACKLIGHT_COMPENSATION}"
            v4l2-ctl -d "$USB_DEVICE" --set-ctrl=auto_exposure=1
            v4l2-ctl -d "$USB_DEVICE" --set-ctrl=exposure_dynamic_framerate=0
            v4l2-ctl -d "$USB_DEVICE" --set-ctrl="exposure_time_absolute=${SEARCH_EXPOSURE_TIME},gain=${SEARCH_GAIN},brightness=${SEARCH_BRIGHTNESS},backlight_compensation=${SEARCH_BACKLIGHT_COMPENSATION}"
            ;;
        false|0|no|off)
            SEARCH_SUN_FILTER=false
            log "Search electronic ND: disabled (camera auto exposure)"
            ;;
        *)
            die "SEARCH_SUN_FILTER must be true or false"
            ;;
    esac
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
    local dev
    local node_name

    IMX477_NODES=()

    for dev in /dev/video*; do
        node_name="$(
            cat "/sys/class/video4linux/${dev##*/}/name" 2>/dev/null || true
        )"

        if [[ "$node_name" == vi-output*,\ imx477* ]]; then
            IMX477_NODES+=("$dev")
        fi
    done

    if (( ${#IMX477_NODES[@]} != 2 )); then
        die "Expected exactly 2 IMX477 image nodes; found ${#IMX477_NODES[@]}"
    fi
}

check_csi_mode()
{
    local dev="$1"
    local listing

    listing="$(v4l2-ctl -d "$dev" --list-formats-ext 2>/dev/null)"

    grep -q "'RG10'" <<< "$listing" \
        || die "$dev does not advertise RG10 Bayer"

    awk '
        /Size: Discrete 3840x2160/ { in_mode=1; next }
        /Size: Discrete/ && in_mode { in_mode=0 }
        in_mode && /30.000 fps/ { found=1 }
        END { exit(found ? 0 : 1) }
    ' <<< "$listing" \
        || die "$dev does not advertise 3840x2160@30"

    log "$dev: RG10 3840x2160@30 advertised"
}

check_usb_mode()
{
    local listing

    [[ -e "$USB_DEVICE" ]] \
        || die "USB camera device does not exist: $USB_DEVICE"

    listing="$(v4l2-ctl -d "$USB_DEVICE" --list-formats-ext 2>/dev/null)"

    awk '
        /Size: Discrete 2560x1440/ { in_mode=1; next }
        /Size: Discrete/ && in_mode { in_mode=0 }
        in_mode && /25.000 fps/ { found=1 }
        END { exit(found ? 0 : 1) }
    ' <<< "$listing" \
        || die "USB camera does not advertise 2560x1440 MJPEG@25"

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
    parent_dir="$(readlink -f "$parent_dir")"

    [[ -w "$parent_dir" ]] \
        || die "Output parent directory is not writable: $parent_dir"

    SAVE_ROOT_RESOLVED="$parent_dir"
}

check_storage()
{
    local available_kib
    local available_gib

    available_kib="$(
        df -Pk "$SAVE_ROOT_RESOLVED" \
        | awk 'NR==2 {print $4}'
    )"

    [[ "$available_kib" =~ ^[0-9]+$ ]] \
        || die "Could not determine free disk space"

    available_gib=$(( available_kib / 1024 / 1024 ))

    log "Free storage: ${available_gib} GiB at $SAVE_ROOT_RESOLVED"

    if (( available_gib < MIN_FREE_GIB )); then
        die "At least ${MIN_FREE_GIB} GiB free space is required"
    fi

    if (( available_gib < 30 )); then
        warn "Less than 30 GiB is free; 4K MJPEG files can grow quickly"
    fi
}

print_configuration()
{
    cat <<EOF
Final recording configuration:
  Logical left:      ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_SOURCE_FPS}, sensor-id=${FRONT_SENSOR_ID}
  Logical right:     ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_SOURCE_FPS}, sensor-id=${BACK_SENSOR_ID}
  CSI recording:    ${CSI_WIDTH}x${CSI_HEIGHT}@${CSI_RECORD_FPS}, MJPEG
  CSI encoder:      ${CSI_ENCODER} (${CSI_ENCODER_DESCRIPTION})
  CSI JPEG quality: ${CSI_JPEG_QUALITY}
  Stereo color:     ${STEREO_COLOR_CORRECTION}
  RGB gain matrix:  ${STEREO_COLOR_MATRIX}
  Contrast:         pivot=${STEREO_COLOR_CONTRAST_PIVOT}, gain=${STEREO_COLOR_CONTRAST}
  Magenta repair:   ${STEREO_MAGENTA_SUPPRESSION}
  Tone gamma:       ${STEREO_COLOR_GAMMA}
  CSI exposure EV:  ${STEREO_EXPOSURE_COMPENSATION}
  Scale filter:     VIC 5-Tap
  Search exposure:   manual=${SEARCH_SUN_FILTER} exposure=${SEARCH_EXPOSURE_TIME}, gain=${SEARCH_GAIN}, brightness=${SEARCH_BRIGHTNESS}, backlight=${SEARCH_BACKLIGHT_COMPENSATION}
  Final blend:      ${STEREO_COLOR_MIX} corrected color + gray remainder
  USB source/save:  ${USB_WIDTH}x${USB_HEIGHT} MJPEG@${USB_FPS}, native remux
  USB device:       ${USB_DEVICE}
  Container:        Matroska (.mkv)
  Output parent:    ${SAVE_ROOT_RESOLVED}
  Duration:         $(
      if (( DURATION_SECONDS > 0 )); then
          printf '%s seconds' "$DURATION_SECONDS"
      else
          printf 'until Ctrl+C'
      fi
  )
EOF
}

run_checks()
{
    check_commands
    select_csi_encoder
    check_required_plugins
    check_uvc_v2
    find_imx477_nodes
    check_csi_mode "${IMX477_NODES[0]}"
    check_csi_mode "${IMX477_NODES[1]}"
    check_usb_mode
    prepare_parent_directory
    check_storage
}

create_session_directory()
{
    local prefix="${1:-record}"

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

    local color_tag="uncorrected"
    if [[ "$STEREO_COLOR_CORRECTION" == "true" ]]; then
        color_tag="color_corrected"
    fi

    LEFT_FILE="$SESSION_DIR/front_sensor${FRONT_SENSOR_ID}_${CSI_WIDTH}x${CSI_HEIGHT}_${CSI_RECORD_FPS}fps_${color_tag}_mjpeg.mkv"
    RIGHT_FILE="$SESSION_DIR/back_sensor${BACK_SENSOR_ID}_${CSI_WIDTH}x${CSI_HEIGHT}_${CSI_RECORD_FPS}fps_${color_tag}_mjpeg.mkv"
    USB_FILE="$SESSION_DIR/usb_${USB_WIDTH}x${USB_HEIGHT}_${USB_FPS}fps_native_mjpeg.mkv"

    cat > "$SESSION_DIR/session.info" <<EOF
created_at=$CLOCK_ISO
clock_state=$CLOCK_STATE
clock_epoch=$CLOCK_EPOCH
clock_boot_id=$CLOCK_BOOT_ID
clock_uptime_seconds=$CLOCK_UPTIME_SECONDS
hostname=$(hostname)
kernel=$(uname -r)
uvc_srcversion=$(cat /sys/module/uvcvideo/srcversion 2>/dev/null || true)

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
csi_encoder=$CSI_ENCODER
csi_encoder_description=$CSI_ENCODER_DESCRIPTION
csi_jpeg_quality=$CSI_JPEG_QUALITY
csi_color_correction=$STEREO_COLOR_CORRECTION
csi_color_matrix=$STEREO_COLOR_MATRIX
csi_color_contrast_pivot=$STEREO_COLOR_CONTRAST_PIVOT
csi_color_contrast=$STEREO_COLOR_CONTRAST
csi_magenta_suppression=$STEREO_MAGENTA_SUPPRESSION
csi_color_gamma=$STEREO_COLOR_GAMMA
csi_color_mix=$STEREO_COLOR_MIX
csi_exposure_compensation=$STEREO_EXPOSURE_COMPENSATION
csi_scale_interpolation=vic_5tap
search_sun_filter=$SEARCH_SUN_FILTER
search_exposure_time=$SEARCH_EXPOSURE_TIME
search_gain=$SEARCH_GAIN
search_brightness=$SEARCH_BRIGHTNESS
search_backlight_compensation=$SEARCH_BACKLIGHT_COMPENSATION
falcon_gst_plugin_dir=$FALCON_GST_PLUGIN_DIR

usb_device=$USB_DEVICE
usb_width=$USB_WIDTH
usb_height=$USB_HEIGHT
usb_fps=$USB_FPS
usb_codec=native_mjpeg_no_reencode

container=matroska
preview=$(
    if [[ "$MODE" == "record-preview" ]]; then
        printf 'enabled'
    else
        printf 'disabled'
    fi
)
EOF

    log "Session directory: $SESSION_DIR"
}

build_record_only_nvjpeg_command()
{
    GST_COMMAND=(
        gst-launch-1.0 -e

        nvarguscamerasrc
            sensor-id="$FRONT_SENSOR_ID"
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
            color-mix="$STEREO_COLOR_MIX" label=left-record
        !
        nvjpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=16 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$LEFT_FILE" sync=false async=false

        nvarguscamerasrc
            sensor-id="$BACK_SENSOR_ID"
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
            color-mix="$STEREO_COLOR_MIX" label=right-record
        !
        nvjpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=16 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$RIGHT_FILE" sync=false async=false

        v4l2src device="$USB_DEVICE" io-mode=2
        !
        "image/jpeg,width=${USB_WIDTH},height=${USB_HEIGHT},framerate=${USB_FPS}/1"
        !
        jpegparse
        !
        queue max-size-buffers=32 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$USB_FILE" sync=false async=false
    )
}

build_record_only_software_command()
{
    GST_COMMAND=(
        gst-launch-1.0 -e

        nvarguscamerasrc
            sensor-id="$FRONT_SENSOR_ID"
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
            color-mix="$STEREO_COLOR_MIX" label=left-record
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw,format=I420,width=${CSI_WIDTH},height=${CSI_HEIGHT},framerate=${CSI_RECORD_FPS}/1"
        !
        jpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=8 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$LEFT_FILE" sync=false async=false

        nvarguscamerasrc
            sensor-id="$BACK_SENSOR_ID"
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
            color-mix="$STEREO_COLOR_MIX" label=right-record
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw,format=I420,width=${CSI_WIDTH},height=${CSI_HEIGHT},framerate=${CSI_RECORD_FPS}/1"
        !
        jpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=8 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$RIGHT_FILE" sync=false async=false

        v4l2src device="$USB_DEVICE" io-mode=2
        !
        "image/jpeg,width=${USB_WIDTH},height=${USB_HEIGHT},framerate=${USB_FPS}/1"
        !
        jpegparse
        !
        queue max-size-buffers=32 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$USB_FILE" sync=false async=false
    )
}

build_record_preview_nvjpeg_command()
{
    GST_COMMAND=(
        gst-launch-1.0 -e

        nvcompositor name=comp
            sink_0::xpos=0
            sink_0::ypos=0
            sink_0::width=960
            sink_0::height=540
            sink_1::xpos=960
            sink_1::ypos=0
            sink_1::width=960
            sink_1::height=540
            sink_2::xpos=480
            sink_2::ypos=540
            sink_2::width=960
            sink_2::height=540
        !
        "video/x-raw(memory:NVMM),width=${MOSAIC_WIDTH},height=${MOSAIC_HEIGHT},format=${MOSAIC_FORMAT}"
        !
        "$DISPLAY_SINK" sync=false

        nvarguscamerasrc
            sensor-id="$FRONT_SENSOR_ID"
            sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1
            awblock=false
            exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label=left-shared
        !
        tee name=left_stream

        left_stream.
        !
        queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        !
        videorate drop-only=true
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        !
        nvjpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=16 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$LEFT_FILE" sync=false async=false

        left_stream.
        !
        queue max-size-buffers=2 leaky=downstream
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        !
        comp.sink_0

        nvarguscamerasrc
            sensor-id="$BACK_SENSOR_ID"
            sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1
            awblock=false
            exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label=right-shared
        !
        tee name=right_stream

        right_stream.
        !
        queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        !
        videorate drop-only=true
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        !
        nvjpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        queue max-size-buffers=16 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$RIGHT_FILE" sync=false async=false

        right_stream.
        !
        queue max-size-buffers=2 leaky=downstream
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        !
        comp.sink_1

        v4l2src device="$USB_DEVICE" io-mode=2
        !
        "image/jpeg,width=${USB_WIDTH},height=${USB_HEIGHT},framerate=${USB_FPS}/1"
        !
        jpegparse
        !
        tee name=usb

        usb.
        !
        queue max-size-buffers=32 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$USB_FILE" sync=false async=false

        usb.
        !
        queue max-size-buffers=2 leaky=downstream
        !
        nvv4l2decoder mjpeg=1
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        !
        comp.sink_2
    )
}

build_record_preview_software_command()
{
    GST_COMMAND=(
        gst-launch-1.0 -e

        nvcompositor name=comp
            sink_0::xpos=0
            sink_0::ypos=0
            sink_0::width=960
            sink_0::height=540
            sink_1::xpos=960
            sink_1::ypos=0
            sink_1::width=960
            sink_1::height=540
            sink_2::xpos=480
            sink_2::ypos=540
            sink_2::width=960
            sink_2::height=540
        !
        "video/x-raw(memory:NVMM),width=${MOSAIC_WIDTH},height=${MOSAIC_HEIGHT},format=${MOSAIC_FORMAT}"
        !
        "$DISPLAY_SINK" sync=false

        nvarguscamerasrc
            sensor-id="$FRONT_SENSOR_ID"
            sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1
            awblock=false
            exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label=left-shared
        !
        tee name=left_stream

        left_stream.
        !
        queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        !
        videorate drop-only=true
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw,format=I420,width=${CSI_WIDTH},height=${CSI_HEIGHT},framerate=${CSI_RECORD_FPS}/1"
        !
        jpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        matroskamux
        !
        filesink location="$LEFT_FILE" sync=false async=false

        left_stream.
        !
        queue max-size-buffers=2 leaky=downstream
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        !
        comp.sink_0

        nvarguscamerasrc
            sensor-id="$BACK_SENSOR_ID"
            sensor-mode="$CSI_SENSOR_MODE"
            wbmode=1
            awblock=false
            exposurecompensation="$STEREO_EXPOSURE_COMPENSATION"
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_SOURCE_FPS}/1"
        !
        falconcolor enabled="$STEREO_COLOR_CORRECTION" matrix="$STEREO_COLOR_MATRIX"
            contrast-pivot="$STEREO_COLOR_CONTRAST_PIVOT" contrast="$STEREO_COLOR_CONTRAST"
            magenta-suppression="$STEREO_MAGENTA_SUPPRESSION" gamma="$STEREO_COLOR_GAMMA"
            color-mix="$STEREO_COLOR_MIX" label=right-shared
        !
        tee name=right_stream

        right_stream.
        !
        queue max-size-buffers=4 max-size-bytes=0 max-size-time=0
        !
        videorate drop-only=true
        !
        "video/x-raw(memory:NVMM),width=${CSI_WIDTH},height=${CSI_HEIGHT},format=NV12,framerate=${CSI_RECORD_FPS}/1"
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw,format=I420,width=${CSI_WIDTH},height=${CSI_HEIGHT},framerate=${CSI_RECORD_FPS}/1"
        !
        jpegenc quality="$CSI_JPEG_QUALITY"
        !
        jpegparse
        !
        matroskamux
        !
        filesink location="$RIGHT_FILE" sync=false async=false

        right_stream.
        !
        queue max-size-buffers=2 leaky=downstream
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        !
        comp.sink_1

        v4l2src device="$USB_DEVICE" io-mode=2
        !
        "image/jpeg,width=${USB_WIDTH},height=${USB_HEIGHT},framerate=${USB_FPS}/1"
        !
        jpegparse
        !
        tee name=usb

        usb.
        !
        queue max-size-buffers=32 max-size-bytes=0 max-size-time=0
        !
        matroskamux
        !
        filesink location="$USB_FILE" sync=false async=false

        usb.
        !
        queue max-size-buffers=2 leaky=downstream
        !
        nvv4l2decoder mjpeg=1
        !
        nvvidconv interpolation-method=2
        !
        "video/x-raw(memory:NVMM),width=960,height=540,format=NV12"
        !
        comp.sink_2
    )
}

build_pipeline()
{
    if [[ "$MODE" == "record-preview" ]]; then
        if [[ "$CSI_ENCODER" == "nvjpegenc" ]]; then
            build_record_preview_nvjpeg_command
        else
            build_record_preview_software_command
        fi
    else
        if [[ "$CSI_ENCODER" == "nvjpegenc" ]]; then
            build_record_only_nvjpeg_command
        else
            build_record_only_software_command
        fi
    fi
}

run_gstreamer()
{
    local rc=0

    log "Starting three-camera recording"
    log "Left output (sensor ${FRONT_SENSOR_ID}):  $LEFT_FILE"
    log "Right output (sensor ${BACK_SENSOR_ID}): $RIGHT_FILE"
    log "USB output:  $USB_FILE"

    if (( DURATION_SECONDS > 0 )); then
        log "Automatic stop after ${DURATION_SECONDS} seconds"

        set +e
        timeout \
            --foreground \
            --signal=INT \
            --kill-after=15s \
            "${DURATION_SECONDS}s" \
            "${GST_COMMAND[@]}"
        rc=$?
        set -e

        if (( rc == 124 )); then
            rc=0
        fi
    else
        log "Press Ctrl+C in this terminal to stop and finalize the files"

        set +e
        "${GST_COMMAND[@]}"
        rc=$?
        set -e

        if (( rc == 130 )); then
            rc=0
        fi
    fi

    sync

    if (( rc != 0 )); then
        warn "GStreamer exited with status $rc"
    fi

    return "$rc"
}

validate_one_file()
{
    local file="$1"
    local expected_width="$2"
    local expected_height="$3"
    local minimum_bytes="$4"
    local report="${file}.discoverer.txt"
    local size

    [[ -f "$file" ]] || {
        warn "Missing output file: $file"
        return 1
    }

    size="$(stat -c '%s' "$file")"

    if (( size < minimum_bytes )); then
        warn "Output is unexpectedly small: $file ($size bytes)"
        return 1
    fi

    if command -v gst-discoverer-1.0 >/dev/null 2>&1; then
        set +e
        gst-discoverer-1.0 "$file" > "$report" 2>&1
        local discover_rc=$?
        set -e

        if (( discover_rc != 0 )); then
            warn "gst-discoverer could not parse: $file"
            return 1
        fi

        grep -Eq "Width:[[:space:]]+${expected_width}" "$report" \
            || {
                warn "Expected width ${expected_width} not found in $file"
                return 1
            }

        grep -Eq "Height:[[:space:]]+${expected_height}" "$report" \
            || {
                warn "Expected height ${expected_height} not found in $file"
                return 1
            }
    else
        warn "gst-discoverer-1.0 is unavailable; only file size was checked"
    fi

    log "Validated: $(basename "$file") ($(du -h "$file" | awk '{print $1}'))"
    return 0
}

validate_outputs()
{
    local failed=0

    validate_one_file \
        "$LEFT_FILE" \
        "$CSI_WIDTH" \
        "$CSI_HEIGHT" \
        1048576 \
        || failed=1

    validate_one_file \
        "$RIGHT_FILE" \
        "$CSI_WIDTH" \
        "$CSI_HEIGHT" \
        1048576 \
        || failed=1

    validate_one_file \
        "$USB_FILE" \
        "$USB_WIDTH" \
        "$USB_HEIGHT" \
        1048576 \
        || failed=1

    if (( failed != 0 )); then
        warn "One or more outputs failed validation"
        return 1
    fi

    (
        cd "$SESSION_DIR"
        sha256sum \
            "$(basename "$LEFT_FILE")" \
            "$(basename "$RIGHT_FILE")" \
            "$(basename "$USB_FILE")" \
            > SHA256SUMS
    )

    log "All three video files passed validation"
    return 0
}

run_check_mode()
{
    run_checks
    print_configuration

    echo
    if [[ "$CSI_ENCODER" == "nvjpegenc" ]]; then
        log "CHECK PASS"
        log "Recommended CSI recording rate: ${CSI_RECORD_FPS} fps at 4K"
    else
        log "CHECK PASS WITH SOFTWARE FALLBACK"
        log "Run verify before long recording"
    fi
}

run_record_mode()
{
    local pipeline_rc=0
    local validation_rc=0
    local prefix="record"

    run_checks

    if [[ "$MODE" == "verify" ]]; then
        DURATION_SECONDS="$VERIFY_SECONDS"
        prefix="verify"
        log "Verification will record ${VERIFY_SECONDS} seconds"
    elif [[ "$MODE" == "record-preview" ]]; then
        prefix="record-preview"
    fi

    print_configuration
    create_session_directory "$prefix"
    configure_search_camera
    build_pipeline

    run_gstreamer || pipeline_rc=$?
    restore_search_camera
    validate_outputs || validation_rc=$?

    cat >> "$SESSION_DIR/session.info" <<EOF

pipeline_exit_status=$pipeline_rc
validation_exit_status=$validation_rc
completed_at=$(safe_wall_iso)
completed_epoch=$(date +%s)
EOF

    echo
    log "Session: $SESSION_DIR"

    if (( pipeline_rc == 0 && validation_rc == 0 )); then
        if [[ "$MODE" == "verify" ]]; then
            log "VERIFY PASS"
        else
            log "RECORDING COMPLETE"
        fi
        return 0
    fi

    die "Recording did not pass final validation; inspect $SESSION_DIR"
}

main()
{
    parse_arguments "$@"

    case "$MODE" in
        help)
            usage
            ;;
        check)
            run_check_mode
            ;;
        verify|record|record-preview)
            run_record_mode
            ;;
        *)
            die "Unsupported mode: $MODE"
            ;;
    esac
}

main "$@"
