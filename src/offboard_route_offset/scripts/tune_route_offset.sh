#!/usr/bin/env bash
set -euo pipefail

TRAJECTORY_TYPE="line_reciprocating"
TRAJECTORY_CONTROL_BACKEND="velocity_path"
YAW_MODE="fixed"
TRAJECTORY_SETPOINT_RATE="50.0"
ALTITUDE="5.0"
EAST_OFFSET="3.0"
YAW="0.0"
LINE_DISTANCE="10.0"
LINE_SPEED="2.0"
REPEAT="3"
ACCEL_TIME="0.5"
DECEL_TIME="0.5"
ENDPOINT_HOLD="0.0"
LINE_TURN_YAW_RATE="30.0"
PUBLISH_RATE="50.0"
LINE_DIRECTION_YAW="0.0"
APPROACH_HORIZONTAL_SPEED="1.0"
APPROACH_VERTICAL_SPEED="0.5"
LINE_LENGTH="3.0"
CIRCLE_RADIUS="1.5"
CIRCLE_TURNS="1.0"
CIRCLE_DIRECTION="cw"
CONTINUOUS_TRAJECTORY="false"
SQUARE_SIDE="3.0"
SINE_LENGTH="4.0"
SINE_AMPLITUDE="1.0"
SINE_CYCLES="1.0"
Z_WIDTH="4.0"
Z_HEIGHT="2.0"
Z_REPEATS="1"
CURVE_SAMPLES_PER_METER="8.0"
MAX_ACCELERATION="1.5"
MAX_DECELERATION="1.8"
MAX_JERK="4.0"
LOOKAHEAD_TIME="0.25"
MIN_LOOKAHEAD="0.10"
MAX_LOOKAHEAD="0.35"
MAX_REFERENCE_LEAD="0.50"
RESPONSE_DELAY="0.40"
STOPPING_MARGIN="0.15"
ENDPOINT_POSITION_TOLERANCE="0.12"
ENDPOINT_SPEED_TOLERANCE="0.20"
ENDPOINT_SETTLE_TIME="0.50"
ENDPOINT_ALTITUDE_TOLERANCE="0.25"
ENDPOINT_VERTICAL_SPEED_TOLERANCE="0.30"
MAX_LATERAL_ACCELERATION="1.5"
VERTEX_YAW_RATE="30.0"
VERTEX_POSITION_TOLERANCE="0.20"
VERTEX_SPEED_TOLERANCE="0.20"
VERTEX_YAW_TOLERANCE="5.0"
VERTEX_YAW_SETTLE_TIME="0.30"
CROSS_TRACK_GAIN="0.5"
MAX_CROSS_TRACK_SPEED="0.30"
CROSS_TRACK_SPEED_RATIO="0.40"
CROSS_TRACK_SLOWDOWN_START="0.60"
CROSS_TRACK_STOP_ERROR="1.50"
PROJECTION_FILTER_TIME_CONSTANT="0.15"
PROJECTION_SEARCH_WINDOW="2.0"
TERMINAL_CAPTURE_ENABLED="true"
TERMINAL_CAPTURE_DISTANCE="0.50"
TERMINAL_GAIN="0.5"
TERMINAL_MAX_SPEED="0.15"
ALTITUDE_VELOCITY_GAIN="0.8"
MAX_VERTICAL_SPEED="0.4"
MAX_VERTICAL_ACCELERATION="0.5"
MAX_VERTICAL_JERK="1.0"
MAX_ALTITUDE_ERROR="2.0"
VELOCITY_ACCELERATION_FF="true"
SQUARE_CORNER_MODE="rounded"
SQUARE_CORNER_RADIUS="1.0"
LINE_TURNAROUND_MODE="stop_reverse"
LINE_TURN_RADIUS="2.0"
YAW_RATE_FF="true"
MAX_YAW_RATE="45.0"
MAX_YAW_ACCELERATION="90.0"

usage() {
  cat <<'EOF'
Usage:
  ros2 run offboard_route_offset tune_route_offset.sh
  ros2 run offboard_route_offset tune_route_offset.sh [options]

No options:
  Start an interactive Chinese tuning menu, similar to the original offboard_route script.

Options:
  --trajectory-type TYPE         line_reciprocating, line, circle, sine, square, z_shape
  --control-backend TYPE         velocity_path (default) or position_feedforward
  --yaw-mode MODE                fixed (default) or tangent
  --setpoint-rate HZ             Trajectory setpoint/heartbeat rate, > 2 Hz
  --altitude METERS              Start altitude above takeoff point, > minimum (default: 5.0)
  --east-offset METERS           Start point east offset from takeoff point (default: 3.0)
  --yaw DEGREES                  Fixed-start yaw and horizontal trajectory direction
  --line-direction-yaw DEGREES   Deprecated alias for --yaw
  --line-speed MPS               Cruise speed for all trajectory modes, > 0
  --accel-time SECONDS           Smooth acceleration time, > 0
  --decel-time SECONDS           Smooth deceleration time, > 0
  --endpoint-hold SECONDS        Hold time at each endpoint/waypoint, >= 0
  --turn-yaw-rate DPS            Stationary endpoint turn yaw rate, deg/s, > 0
  --publish-rate HZ              Setpoint publication rate, > 2 Hz
  --approach-horizontal-speed MPS
  --approach-vertical-speed MPS

Closed-loop progress options:
  --max-acceleration MPS2
  --max-deceleration MPS2
  --max-jerk MPS3
  --lookahead-time SECONDS
  --min-lookahead METERS
  --max-lookahead METERS
  --max-reference-lead METERS
  --response-delay SECONDS
  --stopping-margin METERS
  --endpoint-position-tolerance METERS
  --endpoint-speed-tolerance MPS
  --endpoint-settle-time SECONDS
  --endpoint-altitude-tolerance METERS
  --endpoint-vertical-speed-tolerance MPS
  --max-lateral-acceleration MPS2
  --vertex-yaw-rate DPS
  --vertex-position-tolerance METERS
  --vertex-speed-tolerance MPS
  --vertex-yaw-tolerance DEGREES
  --vertex-yaw-settle-time SECONDS
  --cross-track-gain PER_SECOND
  --max-cross-track-speed MPS
  --cross-track-speed-ratio RATIO
  --cross-track-slowdown-start METERS
  --cross-track-stop-error METERS
  --projection-filter-time-constant SECONDS
  --projection-search-window METERS
  --terminal-capture | --no-terminal-capture
  --terminal-capture-distance METERS
  --terminal-gain PER_SECOND
  --terminal-max-speed MPS
  --altitude-velocity-gain PER_SECOND
  --max-vertical-speed MPS
  --max-vertical-acceleration MPS2
  --max-vertical-jerk MPS3
  --max-altitude-error METERS
  --velocity-acceleration-ff | --no-velocity-acceleration-ff
  --square-corner-mode stop_turn|rounded
  --square-corner-radius METERS
  --line-turnaround-mode stop_reverse|racetrack
  --line-turn-radius METERS
  --yaw-rate-ff | --no-yaw-rate-ff
  --max-yaw-rate DPS
  --max-yaw-acceleration DPS2

line_reciprocating options:
  --line-distance METERS         Reciprocating line distance, > 0
  --repeat COUNT                 Full start-end-start reciprocations; 0 means infinite

Original trajectory mode options:
  --line-length METERS
  --circle-radius METERS
  --circle-turns COUNT
  --circle-direction cw|ccw
  --continuous                   Repeat circle/square until Offboard exits
  --no-continuous
  --square-side METERS
  --sine-length METERS
  --sine-amplitude METERS
  --sine-cycles COUNT
  --z-width METERS
  --z-height METERS
  --z-repeats COUNT
  --curve-samples-per-meter COUNT

Example:
  ros2 run offboard_route_offset tune_route_offset.sh \
    --trajectory-type line_reciprocating \
    --altitude 5.0 \
    --east-offset 3.0 \
    --yaw 90 \
    --line-distance 10.0 \
    --line-speed 2.0 \
    --repeat 3 \
    --accel-time 0.5 \
    --decel-time 0.5
EOF
}

die() {
  printf '[ERROR] %s\n' "$*" >&2
  exit 1
}

info() {
  printf '\n[INFO] %s\n' "$*"
}

is_number() {
  printf '%s\n' "$1" | grep -Eq '^-?([0-9]+([.][0-9]*)?|[.][0-9]+)$'
}

is_nonnegative_integer() {
  printf '%s\n' "$1" | grep -Eq '^[0-9]+$'
}

is_positive_integer() {
  printf '%s\n' "$1" | grep -Eq '^[1-9][0-9]*$'
}

num_gt() {
  awk -v value="$1" -v limit="$2" 'BEGIN {exit !(value > limit)}'
}

num_ge() {
  awk -v value="$1" -v limit="$2" 'BEGIN {exit !(value >= limit)}'
}

normalize_yaw() {
  awk -v value="$1" 'BEGIN {
    y = value % 360
    if (y < 0) y += 360
    printf "%.6f", y
  }'
}

format_float() {
  awk -v value="$1" 'BEGIN {printf "%.6f", value}'
}

require_number() {
  local name="$1"
  local value="$2"
  is_number "$value" || die "${name} must be a number"
}

require_choice() {
  local name="$1"
  local value="$2"
  shift 2
  local choice
  for choice in "$@"; do
    if [ "$value" = "$choice" ]; then
      return 0
    fi
  done
  die "${name} must be one of: $*"
}

need_value() {
  local option="$1"
  local value="${2:-}"
  [ -n "$value" ] || die "${option} requires a value"
  printf '%s\n' "$value"
}

prompt_value() {
  local prompt="$1"
  local current="$2"
  local value
  read -r -p "${prompt} [${current}]: " value
  if [ -z "$value" ]; then
    value="$current"
  fi
  printf '%s\n' "$value"
}

prompt_number() {
  local prompt="$1"
  local current="$2"
  local value
  while true; do
    value="$(prompt_value "$prompt" "$current")"
    if is_number "$value"; then
      printf '%s\n' "$value"
      return
    fi
    printf '请输入数字，例如 3、0.5、-1.2。\n' >&2
  done
}

prompt_number_gt() {
  local prompt="$1"
  local current="$2"
  local limit="$3"
  local value
  while true; do
    value="$(prompt_number "$prompt" "$current")"
    if num_gt "$value" "$limit"; then
      printf '%s\n' "$value"
      return
    fi
    printf '请输入大于 %s 的数字。\n' "$limit" >&2
  done
}

prompt_number_ge() {
  local prompt="$1"
  local current="$2"
  local limit="$3"
  local value
  while true; do
    value="$(prompt_number "$prompt" "$current")"
    if num_ge "$value" "$limit"; then
      printf '%s\n' "$value"
      return
    fi
    printf '请输入不小于 %s 的数字。\n' "$limit" >&2
  done
}

prompt_nonnegative_integer() {
  local prompt="$1"
  local current="$2"
  local value
  while true; do
    value="$(prompt_value "$prompt" "$current")"
    if is_nonnegative_integer "$value"; then
      printf '%s\n' "$value"
      return
    fi
    printf '请输入非负整数，例如 0、1、2。\n' >&2
  done
}

prompt_positive_integer() {
  local prompt="$1"
  local current="$2"
  local value
  while true; do
    value="$(prompt_value "$prompt" "$current")"
    if is_positive_integer "$value"; then
      printf '%s\n' "$value"
      return
    fi
    printf '请输入正整数，例如 1、2、3。\n' >&2
  done
}

prompt_choice() {
  local prompt="$1"
  local current="$2"
  shift 2
  local choices="$*"
  local value
  local choice
  while true; do
    value="$(prompt_value "$prompt (${choices})" "$current")"
    for choice in "$@"; do
      if [ "$value" = "$choice" ]; then
        printf '%s\n' "$value"
        return
      fi
    done
    printf '可选值: %s\n' "$choices" >&2
  done
}

prompt_yaw_direction() {
  local current="$1"
  local choice
  choice="$(prompt_choice \
    "固定起点机头/轨迹起始方向 mission_start_yaw_deg" \
    "custom" \
    north east south west custom)"
  case "$choice" in
    north) printf '0\n' ;;
    east) printf '90\n' ;;
    south) printf '180\n' ;;
    west) printf '270\n' ;;
    custom) prompt_number "自定义 yaw 角度，0北 90东 180南 270西" "$current" ;;
  esac
}

prompt_yes_no() {
  local prompt="$1"
  local default="$2"
  local value
  case "$default" in
    true|True|TRUE|y|Y|yes|YES) default="y" ;;
    false|False|FALSE|n|N|no|NO) default="n" ;;
  esac
  while true; do
    read -r -p "${prompt} [${default}]: " value
    if [ -z "$value" ]; then
      value="$default"
    fi
    case "$value" in
      y|Y|yes|YES|true|True|TRUE) return 0 ;;
      n|N|no|NO|false|False|FALSE) return 1 ;;
      *) printf '请输入 y 或 n。\n' >&2 ;;
    esac
  done
}

interactive_menu() {
  info "offboard_route_offset 交互式调参"
  printf '说明: 该脚本只启动新增 offboard_route_offset 节点，不修改原 offboard_route。\n'
  printf '同一时间只能运行一套向 PX4 /fmu/in 发布 setpoint 的 Offboard 节点。\n\n'

  TRAJECTORY_TYPE="$(prompt_choice "轨迹类型" "$TRAJECTORY_TYPE" line_reciprocating line circle sine square z_shape)"
  TRAJECTORY_CONTROL_BACKEND="$(prompt_choice \
    "正式轨迹控制后端 trajectory_control_backend" \
    "$TRAJECTORY_CONTROL_BACKEND" velocity_path position_feedforward)"
  YAW_MODE="$(prompt_choice "航向模式 yaw_mode" "$YAW_MODE" fixed tangent)"
  ALTITUDE="$(prompt_number_gt "固定起点高度 mission_start_altitude_m，起飞点上方 m" "$ALTITUDE" 0.5)"
  EAST_OFFSET="$(prompt_number "固定起点向东偏移 mission_start_east_offset_m m" "$EAST_OFFSET")"
  YAW="$(prompt_yaw_direction "$YAW")"
  LINE_SPEED="$(prompt_number_gt "轨迹巡航速度 line_speed_mps m/s" "$LINE_SPEED" 0)"
  ACCEL_TIME="$(prompt_number_gt "平滑加速时间 accel_time_s s" "$ACCEL_TIME" 0)"
  DECEL_TIME="$(prompt_number_gt "平滑减速时间 decel_time_s s" "$DECEL_TIME" 0)"
  ENDPOINT_HOLD="$(prompt_number_ge "端点/航点停留 line_endpoint_hold_time_s s" "$ENDPOINT_HOLD" 0)"
  LINE_TURN_YAW_RATE="$(prompt_number_gt "端点原地转向速度 line_turn_yaw_rate_dps deg/s" "$LINE_TURN_YAW_RATE" 0)"

  case "$TRAJECTORY_TYPE" in
    line_reciprocating)
      LINE_DISTANCE="$(prompt_number_gt "往返直线距离 line_distance_m m" "$LINE_DISTANCE" 0)"
      REPEAT="$(prompt_nonnegative_integer "完整往返次数 line_reciprocation_count，0表示无限" "$REPEAT")"
      ;;
    line)
      LINE_LENGTH="$(prompt_number_gt "原直线长度 line_length m" "$LINE_LENGTH" 0)"
      ;;
    circle)
      CIRCLE_RADIUS="$(prompt_number_gt "圆半径 circle_radius m" "$CIRCLE_RADIUS" 0)"
      CIRCLE_DIRECTION="$(prompt_choice "圆方向 circle_direction" "$CIRCLE_DIRECTION" cw ccw)"
      if prompt_yes_no "持续转圈直到切出 Offboard continuous_trajectory" "$CONTINUOUS_TRAJECTORY"; then
        CONTINUOUS_TRAJECTORY="true"
      else
        CONTINUOUS_TRAJECTORY="false"
        CIRCLE_TURNS="$(prompt_number_gt "圆圈数 circle_turns" "$CIRCLE_TURNS" 0)"
      fi
      ;;
    sine)
      SINE_LENGTH="$(prompt_number_gt "正弦前进长度 sine_length m" "$SINE_LENGTH" 0)"
      SINE_AMPLITUDE="$(prompt_number_ge "正弦左右摆幅 sine_amplitude m" "$SINE_AMPLITUDE" 0)"
      SINE_CYCLES="$(prompt_number_gt "正弦周期数 sine_cycles" "$SINE_CYCLES" 0)"
      ;;
    square)
      SQUARE_SIDE="$(prompt_number_gt "正方形边长 square_side m" "$SQUARE_SIDE" 0)"
      if prompt_yes_no "持续绕正方形直到切出 Offboard continuous_trajectory" "$CONTINUOUS_TRAJECTORY"; then
        CONTINUOUS_TRAJECTORY="true"
      else
        CONTINUOUS_TRAJECTORY="false"
      fi
      ;;
    z_shape)
      Z_WIDTH="$(prompt_number_gt "Z 形总宽度 z_width m" "$Z_WIDTH" 0)"
      Z_HEIGHT="$(prompt_number_gt "Z 形总高度 z_height m" "$Z_HEIGHT" 0)"
      Z_REPEATS="$(prompt_positive_integer "Z 字形重复次数 z_repeats" "$Z_REPEATS")"
      ;;
  esac

  if prompt_yes_no "是否修改接近固定起点速度" "n"; then
    APPROACH_HORIZONTAL_SPEED="$(prompt_number_gt "水平接近速度 approach_horizontal_speed_mps m/s" "$APPROACH_HORIZONTAL_SPEED" 0)"
    APPROACH_VERTICAL_SPEED="$(prompt_number_gt "垂直接近速度 approach_vertical_speed_mps m/s" "$APPROACH_VERTICAL_SPEED" 0)"
  fi
  if prompt_yes_no "是否修改发布频率" "n"; then
    TRAJECTORY_SETPOINT_RATE="$(prompt_number_gt \
      "轨迹 setpoint 频率 trajectory_setpoint_rate_hz Hz" \
      "$TRAJECTORY_SETPOINT_RATE" 2.1)"
    PUBLISH_RATE="$TRAJECTORY_SETPOINT_RATE"
  fi
  if prompt_yes_no "是否修改闭环进度/制动高级参数" "n"; then
    MAX_ACCELERATION="$(prompt_number_gt "最大加速度 max_acceleration_mps2" "$MAX_ACCELERATION" 0)"
    MAX_DECELERATION="$(prompt_number_gt "最大减速度 max_deceleration_mps2" "$MAX_DECELERATION" 0)"
    MAX_JERK="$(prompt_number_gt "最大 jerk max_jerk_mps3" "$MAX_JERK" 0)"
    LOOKAHEAD_TIME="$(prompt_number_ge "预瞄时间 lookahead_time_s" "$LOOKAHEAD_TIME" 0)"
    MIN_LOOKAHEAD="$(prompt_number_ge "最小预瞄距离 min_lookahead_distance_m" "$MIN_LOOKAHEAD" 0)"
    MAX_LOOKAHEAD="$(prompt_number_gt "最大预瞄距离 max_lookahead_distance_m" "$MAX_LOOKAHEAD" 0)"
    MAX_REFERENCE_LEAD="$(prompt_number_gt "最大参考领先 max_reference_lead_distance_m" "$MAX_REFERENCE_LEAD" 0)"
    RESPONSE_DELAY="$(prompt_number_ge "制动响应延迟 response_delay_s" "$RESPONSE_DELAY" 0)"
    STOPPING_MARGIN="$(prompt_number_ge "停车安全余量 stopping_safety_margin_m" "$STOPPING_MARGIN" 0)"
    ENDPOINT_POSITION_TOLERANCE="$(prompt_number_gt "端点水平位置容差 endpoint_position_tolerance_m" "$ENDPOINT_POSITION_TOLERANCE" 0)"
    ENDPOINT_SPEED_TOLERANCE="$(prompt_number_ge "端点水平速度容差 endpoint_speed_tolerance_mps" "$ENDPOINT_SPEED_TOLERANCE" 0)"
    ENDPOINT_ALTITUDE_TOLERANCE="$(prompt_number_gt "端点高度容差 endpoint_altitude_tolerance_m" "$ENDPOINT_ALTITUDE_TOLERANCE" 0)"
    ENDPOINT_VERTICAL_SPEED_TOLERANCE="$(prompt_number_ge "端点垂向速度容差 endpoint_vertical_speed_tolerance_mps" "$ENDPOINT_VERTICAL_SPEED_TOLERANCE" 0)"
    MAX_LATERAL_ACCELERATION="$(prompt_number_gt "最大安全横向加速度 max_safe_lateral_acceleration_mps2" "$MAX_LATERAL_ACCELERATION" 0)"
  fi
  if [ "$TRAJECTORY_CONTROL_BACKEND" = "velocity_path" ] &&
    prompt_yes_no "是否修改速度路径跟踪高级参数" "n"; then
    CROSS_TRACK_GAIN="$(prompt_number_ge "横向误差增益 cross_track_gain 1/s" "$CROSS_TRACK_GAIN" 0)"
    MAX_CROSS_TRACK_SPEED="$(prompt_number_gt "最大横向修正速度 m/s" "$MAX_CROSS_TRACK_SPEED" 0)"
    CROSS_TRACK_SPEED_RATIO="$(prompt_number_gt "横向修正速度比例 (0,1]" "$CROSS_TRACK_SPEED_RATIO" 0)"
    CROSS_TRACK_SLOWDOWN_START="$(prompt_number_ge "横向误差降速起点 m" "$CROSS_TRACK_SLOWDOWN_START" 0)"
    CROSS_TRACK_STOP_ERROR="$(prompt_number_gt "严重横向误差 m" "$CROSS_TRACK_STOP_ERROR" "$CROSS_TRACK_SLOWDOWN_START")"
    PROJECTION_FILTER_TIME_CONSTANT="$(prompt_number_ge "投影低通时间常数 s" "$PROJECTION_FILTER_TIME_CONSTANT" 0)"
    PROJECTION_SEARCH_WINDOW="$(prompt_number_gt "投影搜索窗口 m" "$PROJECTION_SEARCH_WINDOW" 0)"
    TERMINAL_CAPTURE_DISTANCE="$(prompt_number_gt "终端捕获距离 m" "$TERMINAL_CAPTURE_DISTANCE" 0)"
    TERMINAL_GAIN="$(prompt_number_gt "终端速度增益 1/s" "$TERMINAL_GAIN" 0)"
    TERMINAL_MAX_SPEED="$(prompt_number_gt "终端最大速度 m/s" "$TERMINAL_MAX_SPEED" 0)"
    ALTITUDE_VELOCITY_GAIN="$(prompt_number_gt "高度速度增益 1/s" "$ALTITUDE_VELOCITY_GAIN" 0)"
    MAX_VERTICAL_SPEED="$(prompt_number_gt "最大垂直速度 m/s" "$MAX_VERTICAL_SPEED" 0)"
    MAX_VERTICAL_ACCELERATION="$(prompt_number_gt "最大垂直加速度 m/s2" "$MAX_VERTICAL_ACCELERATION" 0)"
    MAX_VERTICAL_JERK="$(prompt_number_gt "最大垂直 jerk m/s3" "$MAX_VERTICAL_JERK" 0)"
    if prompt_yes_no "发布速度路径加速度前馈" "$VELOCITY_ACCELERATION_FF"; then
      VELOCITY_ACCELERATION_FF="true"
    else
      VELOCITY_ACCELERATION_FF="false"
    fi
  fi
  if [ "$TRAJECTORY_TYPE" = "line_reciprocating" ]; then
    LINE_TURNAROUND_MODE="$(prompt_choice "往返掉头模式" "$LINE_TURNAROUND_MODE" stop_reverse racetrack)"
    if [ "$LINE_TURNAROUND_MODE" = "racetrack" ]; then
      LINE_TURN_RADIUS="$(prompt_number_gt "跑道形掉头半径 m" "$LINE_TURN_RADIUS" 0)"
    fi
  fi
  if [ "$TRAJECTORY_TYPE" = "square" ]; then
    SQUARE_CORNER_MODE="$(prompt_choice "正方形转角模式" "$SQUARE_CORNER_MODE" rounded stop_turn)"
    if [ "$SQUARE_CORNER_MODE" = "rounded" ]; then
      SQUARE_CORNER_RADIUS="$(prompt_number_gt "正方形圆角半径 m" "$SQUARE_CORNER_RADIUS" 0)"
    fi
  fi
  if [ "$TRAJECTORY_TYPE" = "square" ] && prompt_yes_no "是否修改正方形顶点停止/转向参数" "n"; then
    VERTEX_YAW_RATE="$(prompt_number_gt "顶点转向速度 vertex_yaw_rate_dps" "$VERTEX_YAW_RATE" 0)"
    VERTEX_POSITION_TOLERANCE="$(prompt_number_gt "顶点位置容差 vertex_position_tolerance_m" "$VERTEX_POSITION_TOLERANCE" 0)"
    VERTEX_SPEED_TOLERANCE="$(prompt_number_ge "顶点速度容差 vertex_speed_tolerance_mps" "$VERTEX_SPEED_TOLERANCE" 0)"
    VERTEX_YAW_TOLERANCE="$(prompt_number_gt "顶点 yaw 容差 vertex_yaw_tolerance_deg" "$VERTEX_YAW_TOLERANCE" 0)"
    VERTEX_YAW_SETTLE_TIME="$(prompt_number_ge "顶点 yaw 稳定时间 vertex_yaw_settle_time_s" "$VERTEX_YAW_SETTLE_TIME" 0)"
  fi
}

parse_args() {
  while [ "$#" -gt 0 ]; do
    case "$1" in
      --trajectory-type) TRAJECTORY_TYPE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --control-backend) TRAJECTORY_CONTROL_BACKEND="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --yaw-mode) YAW_MODE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --setpoint-rate) TRAJECTORY_SETPOINT_RATE="$(need_value "$1" "${2:-}")"; PUBLISH_RATE="$TRAJECTORY_SETPOINT_RATE"; shift 2 ;;
      --altitude) ALTITUDE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --east-offset) EAST_OFFSET="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --yaw) YAW="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --line-distance) LINE_DISTANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --line-speed) LINE_SPEED="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --repeat) REPEAT="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --accel-time) ACCEL_TIME="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --decel-time) DECEL_TIME="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --endpoint-hold) ENDPOINT_HOLD="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --turn-yaw-rate) LINE_TURN_YAW_RATE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --publish-rate) PUBLISH_RATE="$(need_value "$1" "${2:-}")"; TRAJECTORY_SETPOINT_RATE="$PUBLISH_RATE"; shift 2 ;;
      --line-direction-yaw) YAW="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --approach-horizontal-speed) APPROACH_HORIZONTAL_SPEED="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --approach-vertical-speed) APPROACH_VERTICAL_SPEED="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --line-length) LINE_LENGTH="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --circle-radius) CIRCLE_RADIUS="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --circle-turns) CIRCLE_TURNS="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --circle-direction) CIRCLE_DIRECTION="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --continuous) CONTINUOUS_TRAJECTORY="true"; shift ;;
      --no-continuous) CONTINUOUS_TRAJECTORY="false"; shift ;;
      --square-side) SQUARE_SIDE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --sine-length) SINE_LENGTH="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --sine-amplitude) SINE_AMPLITUDE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --sine-cycles) SINE_CYCLES="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --z-width) Z_WIDTH="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --z-height) Z_HEIGHT="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --z-repeats) Z_REPEATS="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --curve-samples-per-meter) CURVE_SAMPLES_PER_METER="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-acceleration) MAX_ACCELERATION="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-deceleration) MAX_DECELERATION="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-jerk) MAX_JERK="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --lookahead-time) LOOKAHEAD_TIME="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --min-lookahead) MIN_LOOKAHEAD="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-lookahead) MAX_LOOKAHEAD="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-reference-lead) MAX_REFERENCE_LEAD="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --response-delay) RESPONSE_DELAY="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --stopping-margin) STOPPING_MARGIN="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --endpoint-position-tolerance) ENDPOINT_POSITION_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --endpoint-speed-tolerance) ENDPOINT_SPEED_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --endpoint-settle-time) ENDPOINT_SETTLE_TIME="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --endpoint-altitude-tolerance) ENDPOINT_ALTITUDE_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --endpoint-vertical-speed-tolerance) ENDPOINT_VERTICAL_SPEED_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-lateral-acceleration) MAX_LATERAL_ACCELERATION="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --vertex-yaw-rate) VERTEX_YAW_RATE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --vertex-position-tolerance) VERTEX_POSITION_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --vertex-speed-tolerance) VERTEX_SPEED_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --vertex-yaw-tolerance) VERTEX_YAW_TOLERANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --vertex-yaw-settle-time) VERTEX_YAW_SETTLE_TIME="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --cross-track-gain) CROSS_TRACK_GAIN="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-cross-track-speed) MAX_CROSS_TRACK_SPEED="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --cross-track-speed-ratio) CROSS_TRACK_SPEED_RATIO="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --cross-track-slowdown-start) CROSS_TRACK_SLOWDOWN_START="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --cross-track-stop-error) CROSS_TRACK_STOP_ERROR="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --projection-filter-time-constant) PROJECTION_FILTER_TIME_CONSTANT="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --projection-search-window) PROJECTION_SEARCH_WINDOW="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --terminal-capture) TERMINAL_CAPTURE_ENABLED="true"; shift ;;
      --no-terminal-capture) TERMINAL_CAPTURE_ENABLED="false"; shift ;;
      --terminal-capture-distance) TERMINAL_CAPTURE_DISTANCE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --terminal-gain) TERMINAL_GAIN="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --terminal-max-speed) TERMINAL_MAX_SPEED="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --altitude-velocity-gain) ALTITUDE_VELOCITY_GAIN="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-vertical-speed) MAX_VERTICAL_SPEED="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-vertical-acceleration) MAX_VERTICAL_ACCELERATION="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-vertical-jerk) MAX_VERTICAL_JERK="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --velocity-acceleration-ff) VELOCITY_ACCELERATION_FF="true"; shift ;;
      --no-velocity-acceleration-ff) VELOCITY_ACCELERATION_FF="false"; shift ;;
      --square-corner-mode) SQUARE_CORNER_MODE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --square-corner-radius) SQUARE_CORNER_RADIUS="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --line-turnaround-mode) LINE_TURNAROUND_MODE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --line-turn-radius) LINE_TURN_RADIUS="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --yaw-rate-ff) YAW_RATE_FF="true"; shift ;;
      --no-yaw-rate-ff) YAW_RATE_FF="false"; shift ;;
      --max-yaw-rate) MAX_YAW_RATE="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --max-yaw-acceleration) MAX_YAW_ACCELERATION="$(need_value "$1" "${2:-}")"; shift 2 ;;
      --help|-h) usage; exit 0 ;;
      *) die "unknown option: $1" ;;
    esac
  done
}

validate_and_format() {
  command -v ros2 >/dev/null 2>&1 || die "ros2 not found. Source ROS 2 and install/setup.bash first."
  require_choice "trajectory-type" "$TRAJECTORY_TYPE" line_reciprocating line circle sine square z_shape
  require_choice "control-backend" "$TRAJECTORY_CONTROL_BACKEND" velocity_path position_feedforward
  require_choice "yaw-mode" "$YAW_MODE" fixed tangent
  require_choice "circle-direction" "$CIRCLE_DIRECTION" cw ccw
  require_choice "continuous_trajectory" "$CONTINUOUS_TRAJECTORY" true false
  require_choice "terminal_capture_enabled" "$TERMINAL_CAPTURE_ENABLED" true false
  require_choice "velocity_acceleration_ff" "$VELOCITY_ACCELERATION_FF" true false
  require_choice "square_corner_mode" "$SQUARE_CORNER_MODE" stop_turn rounded
  require_choice "line_turnaround_mode" "$LINE_TURNAROUND_MODE" stop_reverse racetrack
  require_choice "yaw_rate_ff" "$YAW_RATE_FF" true false

  for item in \
    "altitude:${ALTITUDE}" \
    "east-offset:${EAST_OFFSET}" \
    "yaw:${YAW}" \
    "line-distance:${LINE_DISTANCE}" \
    "line-speed:${LINE_SPEED}" \
    "accel-time:${ACCEL_TIME}" \
    "decel-time:${DECEL_TIME}" \
    "endpoint-hold:${ENDPOINT_HOLD}" \
    "turn-yaw-rate:${LINE_TURN_YAW_RATE}" \
    "publish-rate:${PUBLISH_RATE}" \
    "approach-horizontal-speed:${APPROACH_HORIZONTAL_SPEED}" \
    "approach-vertical-speed:${APPROACH_VERTICAL_SPEED}" \
    "line-length:${LINE_LENGTH}" \
    "circle-radius:${CIRCLE_RADIUS}" \
    "circle-turns:${CIRCLE_TURNS}" \
    "square-side:${SQUARE_SIDE}" \
    "sine-length:${SINE_LENGTH}" \
    "sine-amplitude:${SINE_AMPLITUDE}" \
    "sine-cycles:${SINE_CYCLES}" \
    "z-width:${Z_WIDTH}" \
    "z-height:${Z_HEIGHT}" \
    "curve-samples-per-meter:${CURVE_SAMPLES_PER_METER}" \
    "max-acceleration:${MAX_ACCELERATION}" \
    "max-deceleration:${MAX_DECELERATION}" \
    "max-jerk:${MAX_JERK}" \
    "lookahead-time:${LOOKAHEAD_TIME}" \
    "min-lookahead:${MIN_LOOKAHEAD}" \
    "max-lookahead:${MAX_LOOKAHEAD}" \
    "max-reference-lead:${MAX_REFERENCE_LEAD}" \
    "response-delay:${RESPONSE_DELAY}" \
    "stopping-margin:${STOPPING_MARGIN}" \
    "endpoint-position-tolerance:${ENDPOINT_POSITION_TOLERANCE}" \
    "endpoint-speed-tolerance:${ENDPOINT_SPEED_TOLERANCE}" \
    "endpoint-settle-time:${ENDPOINT_SETTLE_TIME}" \
    "endpoint-altitude-tolerance:${ENDPOINT_ALTITUDE_TOLERANCE}" \
    "endpoint-vertical-speed-tolerance:${ENDPOINT_VERTICAL_SPEED_TOLERANCE}" \
    "max-lateral-acceleration:${MAX_LATERAL_ACCELERATION}" \
    "vertex-yaw-rate:${VERTEX_YAW_RATE}" \
    "vertex-position-tolerance:${VERTEX_POSITION_TOLERANCE}" \
    "vertex-speed-tolerance:${VERTEX_SPEED_TOLERANCE}" \
    "vertex-yaw-tolerance:${VERTEX_YAW_TOLERANCE}" \
    "vertex-yaw-settle-time:${VERTEX_YAW_SETTLE_TIME}" \
    "setpoint-rate:${TRAJECTORY_SETPOINT_RATE}" \
    "cross-track-gain:${CROSS_TRACK_GAIN}" \
    "max-cross-track-speed:${MAX_CROSS_TRACK_SPEED}" \
    "cross-track-speed-ratio:${CROSS_TRACK_SPEED_RATIO}" \
    "cross-track-slowdown-start:${CROSS_TRACK_SLOWDOWN_START}" \
    "cross-track-stop-error:${CROSS_TRACK_STOP_ERROR}" \
    "projection-filter-time-constant:${PROJECTION_FILTER_TIME_CONSTANT}" \
    "projection-search-window:${PROJECTION_SEARCH_WINDOW}" \
    "terminal-capture-distance:${TERMINAL_CAPTURE_DISTANCE}" \
    "terminal-gain:${TERMINAL_GAIN}" \
    "terminal-max-speed:${TERMINAL_MAX_SPEED}" \
    "altitude-velocity-gain:${ALTITUDE_VELOCITY_GAIN}" \
    "max-vertical-speed:${MAX_VERTICAL_SPEED}" \
    "max-vertical-acceleration:${MAX_VERTICAL_ACCELERATION}" \
    "max-vertical-jerk:${MAX_VERTICAL_JERK}" \
    "max-altitude-error:${MAX_ALTITUDE_ERROR}" \
    "square-corner-radius:${SQUARE_CORNER_RADIUS}" \
    "line-turn-radius:${LINE_TURN_RADIUS}" \
    "max-yaw-rate:${MAX_YAW_RATE}" \
    "max-yaw-acceleration:${MAX_YAW_ACCELERATION}"; do
    require_number "${item%%:*}" "${item#*:}"
  done

  is_nonnegative_integer "$REPEAT" || die "repeat must be a non-negative integer"
  is_positive_integer "$Z_REPEATS" || die "z-repeats must be a positive integer"
  num_gt "$ALTITUDE" "0.5" || die "altitude must be greater than 0.5 m"
  num_gt "$LINE_DISTANCE" "0" || die "line-distance must be greater than 0"
  num_gt "$LINE_SPEED" "0" || die "line-speed must be greater than 0"
  num_gt "$ACCEL_TIME" "0" || die "accel-time must be greater than 0"
  num_gt "$DECEL_TIME" "0" || die "decel-time must be greater than 0"
  num_ge "$ENDPOINT_HOLD" "0" || die "endpoint-hold must be >= 0"
  num_gt "$LINE_TURN_YAW_RATE" "0" || die "turn-yaw-rate must be greater than 0"
  num_gt "$PUBLISH_RATE" "2.1" || die "publish-rate must be greater than 2.1 Hz"
  num_gt "$APPROACH_HORIZONTAL_SPEED" "0" || die "approach-horizontal-speed must be greater than 0"
  num_gt "$APPROACH_VERTICAL_SPEED" "0" || die "approach-vertical-speed must be greater than 0"
  num_gt "$LINE_LENGTH" "0" || die "line-length must be greater than 0"
  num_gt "$CIRCLE_RADIUS" "0" || die "circle-radius must be greater than 0"
  num_gt "$CIRCLE_TURNS" "0" || die "circle-turns must be greater than 0"
  num_gt "$SQUARE_SIDE" "0" || die "square-side must be greater than 0"
  num_gt "$SINE_LENGTH" "0" || die "sine-length must be greater than 0"
  num_ge "$SINE_AMPLITUDE" "0" || die "sine-amplitude must be >= 0"
  num_gt "$SINE_CYCLES" "0" || die "sine-cycles must be greater than 0"
  num_gt "$Z_WIDTH" "0" || die "z-width must be greater than 0"
  num_gt "$Z_HEIGHT" "0" || die "z-height must be greater than 0"
  num_gt "$CURVE_SAMPLES_PER_METER" "0" || die "curve-samples-per-meter must be greater than 0"
  num_gt "$MAX_ACCELERATION" "0" || die "max-acceleration must be greater than 0"
  num_gt "$MAX_DECELERATION" "0" || die "max-deceleration must be greater than 0"
  num_gt "$MAX_JERK" "0" || die "max-jerk must be greater than 0"
  num_ge "$LOOKAHEAD_TIME" "0" || die "lookahead-time must be >= 0"
  num_ge "$MIN_LOOKAHEAD" "0" || die "min-lookahead must be >= 0"
  num_gt "$MAX_LOOKAHEAD" "0" || die "max-lookahead must be greater than 0"
  num_gt "$MAX_REFERENCE_LEAD" "0" || die "max-reference-lead must be greater than 0"
  num_ge "$RESPONSE_DELAY" "0" || die "response-delay must be >= 0"
  num_ge "$STOPPING_MARGIN" "0" || die "stopping-margin must be >= 0"
  num_gt "$ENDPOINT_POSITION_TOLERANCE" "0" || die "endpoint-position-tolerance must be greater than 0"
  num_ge "$ENDPOINT_SPEED_TOLERANCE" "0" || die "endpoint-speed-tolerance must be >= 0"
  num_ge "$ENDPOINT_SETTLE_TIME" "0" || die "endpoint-settle-time must be >= 0"
  num_gt "$ENDPOINT_ALTITUDE_TOLERANCE" "0" || die "endpoint-altitude-tolerance must be greater than 0"
  num_ge "$ENDPOINT_VERTICAL_SPEED_TOLERANCE" "0" || die "endpoint-vertical-speed-tolerance must be >= 0"
  num_gt "$MAX_LATERAL_ACCELERATION" "0" || die "max-lateral-acceleration must be greater than 0"
  num_gt "$VERTEX_YAW_RATE" "0" || die "vertex-yaw-rate must be greater than 0"
  num_gt "$VERTEX_POSITION_TOLERANCE" "0" || die "vertex-position-tolerance must be greater than 0"
  num_ge "$VERTEX_SPEED_TOLERANCE" "0" || die "vertex-speed-tolerance must be >= 0"
  num_gt "$VERTEX_YAW_TOLERANCE" "0" || die "vertex-yaw-tolerance must be greater than 0"
  num_ge "$VERTEX_YAW_SETTLE_TIME" "0" || die "vertex-yaw-settle-time must be >= 0"
  num_gt "$TRAJECTORY_SETPOINT_RATE" "2.1" || die "setpoint-rate must be greater than 2.1 Hz"
  num_ge "200.0" "$TRAJECTORY_SETPOINT_RATE" || die "setpoint-rate must be <= 200 Hz"
  num_ge "$CROSS_TRACK_GAIN" "0" || die "cross-track-gain must be >= 0"
  num_ge "10.0" "$CROSS_TRACK_GAIN" || die "cross-track-gain must be <= 10 1/s"
  num_gt "$MAX_CROSS_TRACK_SPEED" "0" || die "max-cross-track-speed must be > 0"
  num_ge "10.0" "$MAX_CROSS_TRACK_SPEED" || die "max-cross-track-speed must be <= 10 m/s"
  num_gt "$CROSS_TRACK_SPEED_RATIO" "0" || die "cross-track-speed-ratio must be > 0"
  num_ge "1.0" "$CROSS_TRACK_SPEED_RATIO" || die "cross-track-speed-ratio must be <= 1"
  num_ge "$CROSS_TRACK_SLOWDOWN_START" "0" || die "cross-track-slowdown-start must be >= 0"
  num_gt "$CROSS_TRACK_STOP_ERROR" "$CROSS_TRACK_SLOWDOWN_START" || die "cross-track-stop-error must exceed slowdown-start"
  num_ge "$PROJECTION_FILTER_TIME_CONSTANT" "0" || die "projection-filter-time-constant must be >= 0"
  num_ge "10.0" "$PROJECTION_FILTER_TIME_CONSTANT" || die "projection-filter-time-constant must be <= 10 s"
  num_gt "$PROJECTION_SEARCH_WINDOW" "0" || die "projection-search-window must be > 0"
  num_ge "100.0" "$PROJECTION_SEARCH_WINDOW" || die "projection-search-window must be <= 100 m"
  num_gt "$TERMINAL_CAPTURE_DISTANCE" "0" || die "terminal-capture-distance must be > 0"
  num_ge "100.0" "$TERMINAL_CAPTURE_DISTANCE" || die "terminal-capture-distance must be <= 100 m"
  num_gt "$TERMINAL_GAIN" "0" || die "terminal-gain must be > 0"
  num_ge "10.0" "$TERMINAL_GAIN" || die "terminal-gain must be <= 10 1/s"
  num_gt "$TERMINAL_MAX_SPEED" "0" || die "terminal-max-speed must be > 0"
  num_ge "10.0" "$TERMINAL_MAX_SPEED" || die "terminal-max-speed must be <= 10 m/s"
  num_gt "$ALTITUDE_VELOCITY_GAIN" "0" || die "altitude-velocity-gain must be > 0"
  num_ge "10.0" "$ALTITUDE_VELOCITY_GAIN" || die "altitude-velocity-gain must be <= 10 1/s"
  num_gt "$MAX_VERTICAL_SPEED" "0" || die "max-vertical-speed must be > 0"
  num_ge "10.0" "$MAX_VERTICAL_SPEED" || die "max-vertical-speed must be <= 10 m/s"
  num_gt "$MAX_VERTICAL_ACCELERATION" "0" || die "max-vertical-acceleration must be > 0"
  num_ge "20.0" "$MAX_VERTICAL_ACCELERATION" || die "max-vertical-acceleration must be <= 20 m/s^2"
  num_gt "$MAX_VERTICAL_JERK" "0" || die "max-vertical-jerk must be > 0"
  num_ge "100.0" "$MAX_VERTICAL_JERK" || die "max-vertical-jerk must be <= 100 m/s^3"
  num_gt "$MAX_ALTITUDE_ERROR" "0" || die "max-altitude-error must be > 0"
  num_gt "$SQUARE_CORNER_RADIUS" "0" || die "square-corner-radius must be > 0"
  num_gt "$LINE_TURN_RADIUS" "0" || die "line-turn-radius must be > 0"
  num_gt "$MAX_YAW_RATE" "0" || die "max-yaw-rate must be > 0"
  num_ge "360.0" "$MAX_YAW_RATE" || die "max-yaw-rate must be <= 360 deg/s"
  num_gt "$MAX_YAW_ACCELERATION" "0" || die "max-yaw-acceleration must be > 0"
  num_ge "2000.0" "$MAX_YAW_ACCELERATION" || die "max-yaw-acceleration must be <= 2000 deg/s^2"
  awk -v min="$MIN_LOOKAHEAD" -v max="$MAX_LOOKAHEAD" -v lead="$MAX_REFERENCE_LEAD" \
    'BEGIN {exit !(min <= max && max < lead)}' || \
    die "lookahead distances must satisfy min-lookahead <= max-lookahead < max-reference-lead"

  YAW="$(normalize_yaw "$YAW")"
  LINE_DIRECTION_YAW="$YAW"
  ALTITUDE="$(format_float "$ALTITUDE")"
  EAST_OFFSET="$(format_float "$EAST_OFFSET")"
  LINE_DISTANCE="$(format_float "$LINE_DISTANCE")"
  LINE_SPEED="$(format_float "$LINE_SPEED")"
  ACCEL_TIME="$(format_float "$ACCEL_TIME")"
  DECEL_TIME="$(format_float "$DECEL_TIME")"
  ENDPOINT_HOLD="$(format_float "$ENDPOINT_HOLD")"
  LINE_TURN_YAW_RATE="$(format_float "$LINE_TURN_YAW_RATE")"
  PUBLISH_RATE="$(format_float "$PUBLISH_RATE")"
  APPROACH_HORIZONTAL_SPEED="$(format_float "$APPROACH_HORIZONTAL_SPEED")"
  APPROACH_VERTICAL_SPEED="$(format_float "$APPROACH_VERTICAL_SPEED")"
  LINE_LENGTH="$(format_float "$LINE_LENGTH")"
  CIRCLE_RADIUS="$(format_float "$CIRCLE_RADIUS")"
  CIRCLE_TURNS="$(format_float "$CIRCLE_TURNS")"
  SQUARE_SIDE="$(format_float "$SQUARE_SIDE")"
  SINE_LENGTH="$(format_float "$SINE_LENGTH")"
  SINE_AMPLITUDE="$(format_float "$SINE_AMPLITUDE")"
  SINE_CYCLES="$(format_float "$SINE_CYCLES")"
  Z_WIDTH="$(format_float "$Z_WIDTH")"
  Z_HEIGHT="$(format_float "$Z_HEIGHT")"
  CURVE_SAMPLES_PER_METER="$(format_float "$CURVE_SAMPLES_PER_METER")"
  MAX_ACCELERATION="$(format_float "$MAX_ACCELERATION")"
  MAX_DECELERATION="$(format_float "$MAX_DECELERATION")"
  MAX_JERK="$(format_float "$MAX_JERK")"
  LOOKAHEAD_TIME="$(format_float "$LOOKAHEAD_TIME")"
  MIN_LOOKAHEAD="$(format_float "$MIN_LOOKAHEAD")"
  MAX_LOOKAHEAD="$(format_float "$MAX_LOOKAHEAD")"
  MAX_REFERENCE_LEAD="$(format_float "$MAX_REFERENCE_LEAD")"
  RESPONSE_DELAY="$(format_float "$RESPONSE_DELAY")"
  STOPPING_MARGIN="$(format_float "$STOPPING_MARGIN")"
  ENDPOINT_POSITION_TOLERANCE="$(format_float "$ENDPOINT_POSITION_TOLERANCE")"
  ENDPOINT_SPEED_TOLERANCE="$(format_float "$ENDPOINT_SPEED_TOLERANCE")"
  ENDPOINT_SETTLE_TIME="$(format_float "$ENDPOINT_SETTLE_TIME")"
  ENDPOINT_ALTITUDE_TOLERANCE="$(format_float "$ENDPOINT_ALTITUDE_TOLERANCE")"
  ENDPOINT_VERTICAL_SPEED_TOLERANCE="$(format_float "$ENDPOINT_VERTICAL_SPEED_TOLERANCE")"
  MAX_LATERAL_ACCELERATION="$(format_float "$MAX_LATERAL_ACCELERATION")"
  VERTEX_YAW_RATE="$(format_float "$VERTEX_YAW_RATE")"
  VERTEX_POSITION_TOLERANCE="$(format_float "$VERTEX_POSITION_TOLERANCE")"
  VERTEX_SPEED_TOLERANCE="$(format_float "$VERTEX_SPEED_TOLERANCE")"
  VERTEX_YAW_TOLERANCE="$(format_float "$VERTEX_YAW_TOLERANCE")"
  VERTEX_YAW_SETTLE_TIME="$(format_float "$VERTEX_YAW_SETTLE_TIME")"
  TRAJECTORY_SETPOINT_RATE="$(format_float "$TRAJECTORY_SETPOINT_RATE")"
  PUBLISH_RATE="$TRAJECTORY_SETPOINT_RATE"
  CROSS_TRACK_GAIN="$(format_float "$CROSS_TRACK_GAIN")"
  MAX_CROSS_TRACK_SPEED="$(format_float "$MAX_CROSS_TRACK_SPEED")"
  CROSS_TRACK_SPEED_RATIO="$(format_float "$CROSS_TRACK_SPEED_RATIO")"
  CROSS_TRACK_SLOWDOWN_START="$(format_float "$CROSS_TRACK_SLOWDOWN_START")"
  CROSS_TRACK_STOP_ERROR="$(format_float "$CROSS_TRACK_STOP_ERROR")"
  PROJECTION_FILTER_TIME_CONSTANT="$(format_float "$PROJECTION_FILTER_TIME_CONSTANT")"
  PROJECTION_SEARCH_WINDOW="$(format_float "$PROJECTION_SEARCH_WINDOW")"
  TERMINAL_CAPTURE_DISTANCE="$(format_float "$TERMINAL_CAPTURE_DISTANCE")"
  TERMINAL_GAIN="$(format_float "$TERMINAL_GAIN")"
  TERMINAL_MAX_SPEED="$(format_float "$TERMINAL_MAX_SPEED")"
  ALTITUDE_VELOCITY_GAIN="$(format_float "$ALTITUDE_VELOCITY_GAIN")"
  MAX_VERTICAL_SPEED="$(format_float "$MAX_VERTICAL_SPEED")"
  MAX_VERTICAL_ACCELERATION="$(format_float "$MAX_VERTICAL_ACCELERATION")"
  MAX_VERTICAL_JERK="$(format_float "$MAX_VERTICAL_JERK")"
  MAX_ALTITUDE_ERROR="$(format_float "$MAX_ALTITUDE_ERROR")"
  SQUARE_CORNER_RADIUS="$(format_float "$SQUARE_CORNER_RADIUS")"
  LINE_TURN_RADIUS="$(format_float "$LINE_TURN_RADIUS")"
  MAX_YAW_RATE="$(format_float "$MAX_YAW_RATE")"
  MAX_YAW_ACCELERATION="$(format_float "$MAX_YAW_ACCELERATION")"
}

print_selected_summary() {
  printf '\n准备启动 offboard_route_offset，参数如下:\n'
  printf '  %-32s %s\n' trajectory_type "$TRAJECTORY_TYPE"
  printf '  %-32s %s\n' trajectory_control_backend "$TRAJECTORY_CONTROL_BACKEND"
  printf '  %-32s %s\n' yaw_mode "$YAW_MODE"
  printf '  %-32s %s\n' mission_start_altitude_m "$ALTITUDE"
  printf '  %-32s %s\n' mission_start_east_offset_m "$EAST_OFFSET"
  printf '  %-32s %s\n' mission_start_yaw_deg "$YAW"
  printf '  %-32s %s\n' line_speed_mps "$LINE_SPEED"
  printf '  %-32s %s\n' accel_time_s "$ACCEL_TIME"
  printf '  %-32s %s\n' decel_time_s "$DECEL_TIME"
  printf '  %-32s %s\n' line_endpoint_hold_time_s "$ENDPOINT_HOLD"
  printf '  %-32s %s\n' line_turn_yaw_rate_dps "$LINE_TURN_YAW_RATE"
  printf '  %-32s %s\n' max_acceleration_mps2 "$MAX_ACCELERATION"
  printf '  %-32s %s\n' max_deceleration_mps2 "$MAX_DECELERATION"
  printf '  %-32s %s\n' max_jerk_mps3 "$MAX_JERK"
  printf '  %-32s %s\n' max_reference_lead_distance_m "$MAX_REFERENCE_LEAD"
  case "$TRAJECTORY_TYPE" in
    line_reciprocating)
      printf '  %-32s %s\n' line_distance_m "$LINE_DISTANCE"
      printf '  %-32s %s\n' line_reciprocation_count "$REPEAT"
      ;;
    line)
      printf '  %-32s %s\n' line_length "$LINE_LENGTH"
      ;;
    circle)
      printf '  %-32s %s\n' circle_radius "$CIRCLE_RADIUS"
      printf '  %-32s %s\n' circle_direction "$CIRCLE_DIRECTION"
      printf '  %-32s %s\n' continuous_trajectory "$CONTINUOUS_TRAJECTORY"
      printf '  %-32s %s\n' circle_turns "$CIRCLE_TURNS"
      ;;
    sine)
      printf '  %-32s %s\n' sine_length "$SINE_LENGTH"
      printf '  %-32s %s\n' sine_amplitude "$SINE_AMPLITUDE"
      printf '  %-32s %s\n' sine_cycles "$SINE_CYCLES"
      ;;
    square)
      printf '  %-32s %s\n' square_side "$SQUARE_SIDE"
      printf '  %-32s %s\n' continuous_trajectory "$CONTINUOUS_TRAJECTORY"
      ;;
    z_shape)
      printf '  %-32s %s\n' z_width "$Z_WIDTH"
      printf '  %-32s %s\n' z_height "$Z_HEIGHT"
      printf '  %-32s %s\n' z_repeats "$Z_REPEATS"
      ;;
  esac
  printf '  %-32s %s\n' approach_horizontal_speed_mps "$APPROACH_HORIZONTAL_SPEED"
  printf '  %-32s %s\n' approach_vertical_speed_mps "$APPROACH_VERTICAL_SPEED"
  printf '  %-32s %s\n' trajectory_setpoint_rate_hz "$TRAJECTORY_SETPOINT_RATE"
  printf '  %-32s %s\n' cross_track_gain "$CROSS_TRACK_GAIN"
  printf '  %-32s %s\n' terminal_capture_enabled "$TERMINAL_CAPTURE_ENABLED"
  printf '  %-32s %s\n' square_corner_mode "$SQUARE_CORNER_MODE"
  if [[ "$TRAJECTORY_TYPE" == "square" && "$SQUARE_CORNER_MODE" == "stop_turn" && \
    "$TRAJECTORY_CONTROL_BACKEND" == "velocity_path" ]]; then
    printf '  warning: square stop_turn uses the retained position controller for exact vertices\n'
  fi
}

write_config() {
  local config_file="$1"
  cat >"$config_file" <<EOF
offboard_route_offset:
  ros__parameters:
    trajectory_type: "${TRAJECTORY_TYPE}"
    trajectory_control_backend: "${TRAJECTORY_CONTROL_BACKEND}"
    trajectory_setpoint_rate_hz: ${TRAJECTORY_SETPOINT_RATE}
    mission_start_altitude_m: ${ALTITUDE}
    mission_start_east_offset_m: ${EAST_OFFSET}
    mission_start_yaw_deg: ${YAW}
    approach_horizontal_speed_mps: ${APPROACH_HORIZONTAL_SPEED}
    approach_vertical_speed_mps: ${APPROACH_VERTICAL_SPEED}
    approach_arrival_tolerance_m: 0.20
    approach_yaw_tolerance_deg: 5.0
    approach_settle_time_s: 0.5
    approach_yaw_rate_dps: 45.0
    line_distance_m: ${LINE_DISTANCE}
    line_speed_mps: ${LINE_SPEED}
    line_reciprocation_count: ${REPEAT}
    line_endpoint_hold_time_s: ${ENDPOINT_HOLD}
    line_endpoint_arrival_tolerance_m: 0.25
    line_endpoint_velocity_tolerance_mps: 0.25
    line_turn_yaw_rate_dps: ${LINE_TURN_YAW_RATE}
    line_turn_yaw_tolerance_deg: 5.0
    line_turn_settle_time_s: 0.2
    line_direction_yaw_deg: ${YAW}
    line_length: ${LINE_LENGTH}
    circle_radius: ${CIRCLE_RADIUS}
    circle_turns: ${CIRCLE_TURNS}
    circle_direction: "${CIRCLE_DIRECTION}"
    continuous_trajectory: ${CONTINUOUS_TRAJECTORY}
    square_side: ${SQUARE_SIDE}
    sine_length: ${SINE_LENGTH}
    sine_amplitude: ${SINE_AMPLITUDE}
    sine_cycles: ${SINE_CYCLES}
    z_width: ${Z_WIDTH}
    z_height: ${Z_HEIGHT}
    z_repeats: ${Z_REPEATS}
    curve_samples_per_meter: ${CURVE_SAMPLES_PER_METER}
    accel_time_s: ${ACCEL_TIME}
    decel_time_s: ${DECEL_TIME}
    max_acceleration_mps2: ${MAX_ACCELERATION}
    max_deceleration_mps2: ${MAX_DECELERATION}
    max_jerk_mps3: ${MAX_JERK}
    lookahead_time_s: ${LOOKAHEAD_TIME}
    min_lookahead_distance_m: ${MIN_LOOKAHEAD}
    max_lookahead_distance_m: ${MAX_LOOKAHEAD}
    max_reference_lead_distance_m: ${MAX_REFERENCE_LEAD}
    response_delay_s: ${RESPONSE_DELAY}
    stopping_safety_margin_m: ${STOPPING_MARGIN}
    endpoint_position_tolerance_m: ${ENDPOINT_POSITION_TOLERANCE}
    endpoint_speed_tolerance_mps: ${ENDPOINT_SPEED_TOLERANCE}
    endpoint_settle_time_s: ${ENDPOINT_SETTLE_TIME}
    endpoint_altitude_tolerance_m: ${ENDPOINT_ALTITUDE_TOLERANCE}
    endpoint_vertical_speed_tolerance_mps: ${ENDPOINT_VERTICAL_SPEED_TOLERANCE}
    max_safe_lateral_acceleration_mps2: ${MAX_LATERAL_ACCELERATION}
    cross_track_gain: ${CROSS_TRACK_GAIN}
    max_cross_track_correction_speed_mps: ${MAX_CROSS_TRACK_SPEED}
    cross_track_speed_ratio: ${CROSS_TRACK_SPEED_RATIO}
    cross_track_slowdown_start_m: ${CROSS_TRACK_SLOWDOWN_START}
    cross_track_stop_error_m: ${CROSS_TRACK_STOP_ERROR}
    projection_filter_time_constant_s: ${PROJECTION_FILTER_TIME_CONSTANT}
    projection_search_window_m: ${PROJECTION_SEARCH_WINDOW}
    terminal_capture_enabled: ${TERMINAL_CAPTURE_ENABLED}
    terminal_capture_distance_m: ${TERMINAL_CAPTURE_DISTANCE}
    terminal_gain: ${TERMINAL_GAIN}
    terminal_max_speed_mps: ${TERMINAL_MAX_SPEED}
    altitude_velocity_gain: ${ALTITUDE_VELOCITY_GAIN}
    max_vertical_speed_mps: ${MAX_VERTICAL_SPEED}
    max_vertical_acceleration_mps2: ${MAX_VERTICAL_ACCELERATION}
    max_vertical_jerk_mps3: ${MAX_VERTICAL_JERK}
    max_altitude_error_m: ${MAX_ALTITUDE_ERROR}
    velocity_path_acceleration_feedforward_enabled: ${VELOCITY_ACCELERATION_FF}
    square_corner_mode: "${SQUARE_CORNER_MODE}"
    square_corner_radius_m: ${SQUARE_CORNER_RADIUS}
    line_turnaround_mode: "${LINE_TURNAROUND_MODE}"
    line_turn_radius_m: ${LINE_TURN_RADIUS}
    yaw_mode: "${YAW_MODE}"
    yaw_rate_feedforward_enabled: ${YAW_RATE_FF}
    max_yaw_rate_dps: ${MAX_YAW_RATE}
    max_yaw_acceleration_dps2: ${MAX_YAW_ACCELERATION}
    vertex_yaw_rate_dps: ${VERTEX_YAW_RATE}
    vertex_position_tolerance_m: ${VERTEX_POSITION_TOLERANCE}
    vertex_speed_tolerance_mps: ${VERTEX_SPEED_TOLERANCE}
    vertex_yaw_tolerance_deg: ${VERTEX_YAW_TOLERANCE}
    vertex_yaw_settle_time_s: ${VERTEX_YAW_SETTLE_TIME}
    publish_rate_hz: ${PUBLISH_RATE}
    local_position_timeout_s: 1.0
    prestream_after_exit_delay_s: 1.0
    minimum_start_altitude_m: 0.5
    takeoff_reference_candidate_timeout_s: 3.0
EOF
}

INTERACTIVE=false
if [ "$#" -eq 0 ] && [ -t 0 ]; then
  INTERACTIVE=true
fi

if [ "$INTERACTIVE" = "true" ]; then
  interactive_menu
else
  parse_args "$@"
fi

validate_and_format
print_selected_summary

if ros2 node list 2>/dev/null | grep -qx '/offboard_route_offset'; then
  die "node /offboard_route_offset is already running; stop it before starting another controller"
fi

if [ "$INTERACTIVE" = "true" ]; then
  if ! prompt_yes_no "确认启动新增节点" "y"; then
    info "已取消启动。"
    exit 0
  fi
fi

CONFIG_FILE="$(mktemp /tmp/offboard_route_offset.XXXXXX.yaml)"
trap 'rm -f "$CONFIG_FILE"' EXIT
write_config "$CONFIG_FILE"

info "Launching offboard_route_offset with temporary config: ${CONFIG_FILE}"
ros2 launch offboard_route_offset offboard_route_offset.launch.py config_file:="$CONFIG_FILE"
