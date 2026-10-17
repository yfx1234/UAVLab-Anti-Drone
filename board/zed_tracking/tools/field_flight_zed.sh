#!/usr/bin/env bash
# field_flight_zed.sh:ZED X Mini 新链路一键飞行管理(check/start/status/stop)。
# 替代旧 field_flight.sh 的相机相关部分;安全阀门(upstream_gate +
# px4_upstream_adapter)继续沿用,实控确认位流程不变。
set -u

WS=/home/uavlab/Jetson_ROS2_V0/ros2_ws
RUNROOT=${FALCON_RUN_ROOT:-/tmp/falcon_zed_flight}
ENGINE=${FALCON_ENGINE:-/home/uavlab/Jetson_ROS2_V0/weights/candidates/platform_exp_20260909/exp_portrait_1280x736_opt3.engine}
ADAPTER_MODE=${FALCON_ADAPTER_MODE:-shadow}   # shadow=只记录;live=实控
CMD=${1:-help}

env_setup() {
  set +u   # ROS setup.bash 未绑定变量与 set -u 冲突
  source /opt/ros/humble/setup.bash
  source "$WS/install/setup.bash"
  set -u
  export ROS_LOCALHOST_ONLY=1
}

do_check() {
  local fail=0
  echo "== [1/3] 残留进程"
  local res
  res=$(pgrep -af 'zed_perception_node|zed_track_filter_node|track_hold_node|shot_event_bridge|virtual_projectile_referee|px4_upstream_adapter|upstream_gate_node' | grep -v pgrep || true)
  if [ -n "$res" ]; then echo "$res"; fail=1; else echo "  无残留"; fi
  echo "== [2/3] 引擎文件"
  if [ -s "$ENGINE" ]; then echo "  OK: $ENGINE"; else echo "  缺失: $ENGINE"; fail=1; fi
  echo "== [3/3] 飞控链路 (/fmu/out/vehicle_attitude)"
  env_setup
  local hz
  hz=$(timeout 6 ros2 topic hz /fmu/out/vehicle_attitude 2>/dev/null | grep -oE 'average rate: [0-9.]+' | head -1)
  if [ -n "$hz" ]; then
    echo "  $hz"
  else
    echo "  无输出 —— 飞控链路不通"
    [ "${FALCON_CHECK_SKIP_FC:-0}" = "1" ] || fail=1
    [ "${FALCON_CHECK_SKIP_FC:-0}" = "1" ] && echo "  (台架模式:跳过硬性判定)"
  fi
  [ "$fail" -eq 0 ] && { echo "== 检查通过"; return 0; }
  echo "== 检查存在失败项"; return 1
}

start_component() {  # name command...
  local NAME=$1; shift
  setsid nohup "$@" > "$RUNDIR/$NAME.log" 2>&1 &
  echo $! > "$RUNDIR/$NAME.pid"
}

do_start() {
  echo "== 飞前检查"
  do_check || { echo "启动中止"; exit 1; }
  local ts
  ts=$(date +%Y%m%d_%H%M%S)
  RUNDIR="$RUNROOT/$ts"
  mkdir -p "$RUNDIR" "$RUNROOT"
  echo "$RUNDIR" > "$RUNROOT/latest"
  echo "$ADAPTER_MODE" > "$RUNROOT/mode"

  echo "== [1/5] 安全阀门 + 裁判"
  start_component gate ros2 run falcon_intercept upstream_gate_node
  start_component adapter ros2 run falcon_intercept px4_upstream_adapter_node --ros-args \
    -p mode:="$ADAPTER_MODE" \
    -p allow_live_control:="$([ "$ADAPTER_MODE" = live ] && echo true || echo false)" \
    -p calibration_ready:=true
  start_component referee ros2 launch virtual_interception_referee referee.launch.py \
    pursuer_local_topic:=/fmu/out/vehicle_local_position \
    pursuer_attitude_topic:=/fmu/out/vehicle_attitude
  start_component bridge python3 "$WS/src/zed_tracking/tools/shot_event_bridge.py" --ros-args \
    -p truth_source:=track
  sleep 3

  echo "== [2/5] 感知(ZED + 检测 + 深度 + LOSSLESS SVO)"
  start_component perception ros2 run zed_tracking zed_perception_node --ros-args \
    -p engine_path:="$ENGINE" \
    -p depth_mode:=NEURAL_LIGHT \
    -p record_enabled:=true -p svo_compression:=LOSSLESS \
    -p svo_directory:="$RUNDIR/svo"
  sleep 5

  echo "== [3/5] 滤波"
  start_component filter ros2 run zed_tracking zed_track_filter_node.py

  echo "== [4/5] 跟踪保持控制(输出经安全阀门,$ADAPTER_MODE 模式)"
  start_component control ros2 run zed_tracking track_hold_node.py
  sleep 2

  echo "== [5/5] 证据记录"
  start_component evidence ros2 bag record -o "$RUNDIR/evidence" \
    /zed_perception_node/target_json /zed_tracking/track_json \
    /zed_tracking/track_hold_status /zed_tracking/shot_event \
    /evaluation/virtual_hit_result /fmu/out/vehicle_local_position

  echo "== 全链路就绪: $RUNDIR"
  echo "   监控: bash $0 status"
  echo "   停止: bash $0 stop"
}

do_stop() {
  rm -f "$RUNROOT/mode"
  local d
  d=$(cat "$RUNROOT/latest" 2>/dev/null || true)
  [ -z "$d" ] && { echo "无启动记录"; return 0; }
  RUNDIR="$d"
  pkill -INT -f 'ros2 bag record' 2>/dev/null; sleep 3
  pkill -INT -f 'ros2 bag record' 2>/dev/null; sleep 2
  pkill -f "[t]rack_hold_node" 2>/dev/null; sleep 2
  pkill -f "[z]ed_track_filter_node" 2>/dev/null
  pkill -f "[z]ed_perception_node" 2>/dev/null; sleep 4
  pkill -f "[s]hot_event_bridge" 2>/dev/null
  pkill -f "[v]irtual_projectile_referee" 2>/dev/null
  pkill -f "[u]pstream_gate_node" 2>/dev/null
  pkill -f "[p]x4_upstream_adapter" 2>/dev/null
  sleep 2
  pgrep -af 'zed_perception|track_hold|referee|adapter' | grep -v pgrep || echo "== 全部停止"
}

do_status() {
  local d
  d=$(cat "$RUNROOT/latest" 2>/dev/null || true)
  [ -z "$d" ] && { echo "无启动记录"; exit 0; }
  echo "== 运行目录: $d"
  env_setup
  timeout 5 ros2 topic echo --once /zed_perception_node/status 2>/dev/null | head -2 || echo "  感知状态不可达"
  timeout 5 ros2 topic echo --once /zed_tracking/track_hold_status 2>/dev/null | head -2 || echo "  控制状态不可达"
  echo "-- 最近射击/控制日志 --"
  tail -5 "$d/control.log" 2>/dev/null
}

case "$CMD" in
  check) do_check ;;
  start) env_setup; do_start ;;
  stop) do_stop ;;
  status) env_setup; do_status ;;
  *) sed -n '2,8p' "$0"; exit 1 ;;
esac
