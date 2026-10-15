#!/usr/bin/env bash
# 板上拦截链启动/停止脚本（实机部署，2026-10-15 首版）。
#
# 用法：
#   ./run_interceptor_stack.sh start [launch 参数...]   # 启动（含感知桥）
#   ./run_interceptor_stack.sh stop                     # 停止（SIGINT 优雅退出）
#   ./run_interceptor_stack.sh status                   # 进程 + 最近日志
#   ./run_interceptor_stack.sh log                      # 跟踪最新日志
#
# 启动内容：netgun_real.launch.py 四节点
#   zed_perception_bridge + relative_estimator + netgun_guidance_node + netgun_strike_node
# 感知（zed_perception_standalone）不归本脚本管，由感知平台（PC 8767 面板/
# supervisor）单独启停；本链只消费其 /perception/observations_json。
#
# 环境链（顺序不可变）：
#   humble -> Jetson_ROS2_V0/ros2_ws/install（px4_msgs 等底座）
#          -> install_perception（感知部署，仅运行参考，无构建耦合）
#          -> ~/interceptor_ws/install（拦截链，最后覆盖生效）
# nohup+setsid 保证图传/SSH 断连不杀进程；日志写 ~/netgun_logs/。

set -euo pipefail

INTERCEPTOR_WS="${INTERCEPTOR_WS:-$HOME/interceptor_ws}"
LOG_DIR="${NETGUN_LOG_DIR:-$HOME/netgun_logs}"
PID_FILE="$LOG_DIR/interceptor_stack.pid"
LAUNCH_PKG="netgun_intercept"
LAUNCH_FILE="netgun_real.launch.py"

mkdir -p "$LOG_DIR"

env_setup() {
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$HOME/Jetson_ROS2_V0/ros2_ws/install/setup.bash"
  # shellcheck disable=SC1091
  source "$HOME/Jetson_ROS2_V0/ros2_ws/install_perception/setup.bash"
  # shellcheck disable=SC1091
  source "$INTERCEPTOR_WS/install/setup.bash"
  export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1
}

running_pid() {
  [ -f "$PID_FILE" ] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 1
  echo "$pid"
}

latest_log() {
  ls -t "$LOG_DIR"/interceptor_stack_*.log 2>/dev/null | head -1 || true
}

cmd_start() {
  if pid="$(running_pid)"; then
    echo "拦截链已在运行 (pid $pid)，日志 $(latest_log)"
    exit 0
  fi
  if [ ! -d "$INTERCEPTOR_WS/install" ]; then
    echo "错误：$INTERCEPTOR_WS/install 不存在，先在板上构建拦截链" >&2
    exit 1
  fi
  env_setup
  local stamp log_file
  stamp="$(date +%Y%m%d_%H%M%S)"
  log_file="$LOG_DIR/interceptor_stack_${stamp}.log"
  nohup setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" "${@:2}" \
    >"$log_file" 2>&1 &
  local pid=$!
  echo "$pid" >"$PID_FILE"
  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "启动失败，日志尾部：" >&2
    tail -20 "$log_file" >&2 || true
    rm -f "$PID_FILE"
    exit 1
  fi
  echo "已启动 pid $pid，日志 $log_file"
}

cmd_stop() {
  local pid
  if ! pid="$(running_pid)"; then
    echo "拦截链未运行"
    # 兜底清理残留节点（PID 文件丢失场景）
    pkill -INT -f "netgun_real[.]launch[.]py" 2>/dev/null || true
    exit 0
  fi
  # setsid 后 pid 即进程组号；SIGINT 让 ros2 launch 优雅收节点
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 10); do
    kill -0 "$pid" 2>/dev/null || break
    sleep 1
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  rm -f "$PID_FILE"
  echo "已停止"
}

cmd_status() {
  local pid
  if pid="$(running_pid)"; then
    echo "运行中 pid $pid"
  else
    echo "未运行"
  fi
  pgrep -af "netgun_(guidance|strike)_node|relative_estimator|zed_perception_bridge" 2>/dev/null || true
  local log_file
  log_file="$(latest_log)"
  if [ -n "$log_file" ]; then
    echo "---- 最近日志（$log_file）----"
    tail -5 "$log_file" || true
  fi
}

cmd_log() {
  local log_file
  log_file="$(latest_log)"
  if [ -z "$log_file" ]; then
    echo "暂无日志" >&2
    exit 1
  fi
  tail -n 50 -f "$log_file"
}

case "${1:-start}" in
  start) cmd_start "$@" ;;
  stop) cmd_stop ;;
  status) cmd_status ;;
  log) cmd_log ;;
  *)
    echo "用法：$0 {start|stop|status|log}" >&2
    exit 2
    ;;
esac
