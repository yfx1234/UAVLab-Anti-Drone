#!/usr/bin/env bash
# 板上拦截链启动/停止脚本（实机部署，2026-10-15 首版）。
#
# 用法：
#   ./run_interceptor_stack.sh start [launch 参数...]   # 启动（含感知桥）
#   ./run_interceptor_stack.sh stop                     # 停止（SIGINT 优雅退出）
#   ./run_interceptor_stack.sh status                   # 进程 + 最近日志
#   ./run_interceptor_stack.sh log                      # 跟踪最新日志
#   ./run_interceptor_stack.sh check                    # 立即跑一次启动自检
#
# 上电顺序（N-2，2026-10-17）：面板对时 → 启动感知+YOLO → 启动拦截链
# → 飞控控制台 uxrce_dds_client status 确认 connected/timesync converged
# → 日志 attmiss 不随 obs 增长、netgun diag fcu=1。拦截链启动后若再对时，
# 必须 stop/start 重启拦截链（时钟跳变影响全部计时）。
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
#
# 安装参数（2026-10-22）：面板「安装参数」保存在 ~/netgun_config/
# mounting_geometry.json（以网枪中心为基准）。start 时经
# deploy/mounting_geometry.py launch-args 换算成相机外参/枪口偏置/枪轴偏角
# launch 参数，排在命令行参数之前（命令行显式参数仍可覆盖）；文件无效时拒绝
# 启动，不静默退回默认值。实际使用的参数以 NETGUN_GEOMETRY 行写入运行日志。

set -euo pipefail

INTERCEPTOR_WS="${INTERCEPTOR_WS:-$HOME/interceptor_ws}"
LOG_DIR="${NETGUN_LOG_DIR:-$HOME/netgun_logs}"
PID_FILE="$LOG_DIR/interceptor_stack.pid"
CHECK_FILE="$LOG_DIR/last_start_check.txt"
LAUNCH_PKG="netgun_intercept"
LAUNCH_FILE="netgun_real.launch.py"
GEOMETRY_TOOL="$INTERCEPTOR_WS/deploy/mounting_geometry.py"

mkdir -p "$LOG_DIR"

env_setup() {
  # ROS 的 setup 脚件不兼容 set -u（引用未定义变量），sourcing 期间临时关闭
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "$HOME/Jetson_ROS2_V0/ros2_ws/install/setup.bash"
  # shellcheck disable=SC1091
  source "$HOME/Jetson_ROS2_V0/ros2_ws/install_perception/setup.bash"
  # shellcheck disable=SC1091
  source "$INTERCEPTOR_WS/install/setup.bash"
  set -u
  # 域与板上感知节点一致（domain 0）；感知节点 ROS_LOCALHOST_ONLY=0，
  # 这里必须保持一致否则 DDS 互相不可见（2026-10-15 板上实测）
  export ROS_DOMAIN_ID=0
  unset ROS_LOCALHOST_ONLY
  # N-1（2026-10-17）：打印实际生效配置——DDS 发现必须四方一致
  #（飞控 PTCFG/本脚本/感知/agent），登录 shell 可能是 LOCALHOST_ONLY=1，
  # 在里面 ros2 topic hz /fmu/out 看到数据不代表拦截链也看得到
  echo "拦截链环境：ROS_DOMAIN_ID=$ROS_DOMAIN_ID ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-未设置(=0)}"
}

running_pid() {
  [ -f "$PID_FILE" ] || return 1
  local pid
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 1
  echo "$pid"
}

# I-3（2026-10-21）：兜底清理只认生产域（ROS_DOMAIN_ID 为空或 0）的
# netgun_real.launch.py——隔离域（如 42）的 fullchain/rehearsal 测试栈不归
# 本脚本管（原 pkill -f 会把正在跑的测试栈一并 SIGINT 掉）
production_launch_pids() {
  local p dom
  for p in $(pgrep -f "netgun_real[.]launch[.]py" 2>/dev/null || true); do
    # set -euo pipefail 下进程刚退出读不到 environ 不能让脚本整体退出
    dom="$(tr '\0' '\n' 2>/dev/null <"/proc/$p/environ" |
      sed -n 's/^ROS_DOMAIN_ID=//p' | head -1 || true)"
    if [ -z "$dom" ] || [ "$dom" = "0" ]; then
      echo "$p"
    fi
  done
}

latest_log() {
  ls -t "$LOG_DIR"/interceptor_stack_*.log 2>/dev/null | head -1 || true
}

# N-1/F-6（2026-10-18）：启动自检——估计器 diag 里 attmiss 随 obs 增长
# 即 DDS 发现不通（PTCFG/ROS_LOCALHOST_ONLY/域不一致）。结果追加到
# $CHECK_FILE（面板/interceptor_status.py 可读），非 OK 同时追加进运行
# 日志——原实现整个子 shell >/dev/null 2>&1，告警无人可见。
run_start_check() {
  local log_file="$1" pid="${2:-}"
  local line obs miss status msg alive
  if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
    alive="进程运行中"
  else
    alive="进程已退出"
  fi
  if ! grep -q "estimator diag" "$log_file" 2>/dev/null; then
    status="UNKNOWN"
    msg="15s 内未见 estimator diag 行（节点未起或日志未落盘；$alive）"
  else
    line="$(grep "estimator diag" "$log_file" 2>/dev/null | tail -1)"
    obs="$(echo "$line" | grep -o "obs=[0-9]*" | cut -d= -f2)"
    miss="$(echo "$line" | grep -o "attmiss=[0-9]*" | cut -d= -f2)"
    if [ -n "$obs" ] && [ -n "$miss" ] && [ "$obs" -gt 30 ] &&
      [ "$miss" -eq "$obs" ]; then
      status="WARN"
      msg="拦截链未收到飞控姿态（attmiss=obs=$miss）——检查 UXRCE_DDS_PTCFG/UXRCE_DDS_DOM_ID 与本机 ROS_LOCALHOST_ONLY 是否一致"
    else
      status="OK"
      msg="estimator diag 正常（obs=${obs:-0} attmiss=${miss:-0}；$alive）"
    fi
  fi
  printf '[%s] %s %s (log: %s)\n' \
    "$(date '+%F %T')" "$status" "$msg" "$log_file" >>"$CHECK_FILE"
  if [ "$status" != "OK" ]; then
    printf '%s：%s\n' "$status" "$msg" >>"$log_file"
  fi
  echo "$status $msg"
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
  # 安装参数先于环境与日志检查：无效时不启动、不留空日志
  local -a geometry_args=()
  local geometry_out="" geometry_line="" geometry_summary="" geometry_brief=""
  if [ -f "$GEOMETRY_TOOL" ]; then
    if ! geometry_out="$(python3 "$GEOMETRY_TOOL" launch-args 2>&1)"; then
      echo "错误：${geometry_out}；拦截链未启动（在面板「安装参数」修正后重试）" >&2
      exit 1
    fi
    while IFS= read -r geometry_line; do
      if [ -n "$geometry_line" ]; then
        geometry_args+=("$geometry_line")
      fi
    done <<<"$geometry_out"
    geometry_summary="$(python3 "$GEOMETRY_TOOL" summary 2>/dev/null || true)"
    geometry_brief="$(python3 "$GEOMETRY_TOOL" brief 2>/dev/null || true)"
  fi
  env_setup
  local stamp log_file
  stamp="$(date +%Y%m%d_%H%M%S)"
  log_file="$LOG_DIR/interceptor_stack_${stamp}.log"
  if [ -n "$geometry_summary" ]; then
    printf '%s\n' "$geometry_summary" >>"$log_file"
  fi
  # G-1（2026-10-18）：必须用追加模式——run_start_check 会 >> 同一文件
  # 写告警，截断写会让 launch 按自己的旧偏移把追加行覆盖掉
  nohup setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
    "${geometry_args[@]}" "${@:2}" >>"$log_file" 2>&1 &
  local pid=$!
  echo "$pid" >"$PID_FILE"
  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "启动失败，日志尾部：" >&2
    tail -20 "$log_file" >&2 || true
    rm -f "$PID_FILE"
    exit 1
  fi
  if [ -n "$geometry_brief" ]; then
    echo "$geometry_brief"
  fi
  echo "已启动 pid $pid，日志 $log_file"
  # 15s 后自检（run_start_check 内部落 CHECK_FILE 与运行日志）
  ( sleep 15; run_start_check "$log_file" "$pid" >/dev/null 2>&1 ) &
}

cmd_check() {
  local log_file pid
  log_file="$(latest_log)"
  if [ -z "$log_file" ]; then
    echo "暂无日志" >&2
    exit 1
  fi
  pid="$(cat "$PID_FILE" 2>/dev/null || true)"
  run_start_check "$log_file" "$pid"
}

cmd_stop() {
  local pid
  if ! pid="$(running_pid)"; then
    echo "拦截链未运行"
    # 兜底清理残留节点（PID 文件丢失场景；只清生产域，见 production_launch_pids）
    local p
    for p in $(production_launch_pids); do
      kill -INT "$p" 2>/dev/null || true
    done
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
  if [ -f "$CHECK_FILE" ]; then
    echo "---- 最近启动自检（$CHECK_FILE）----"
    tail -3 "$CHECK_FILE"
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
  check) cmd_check ;;
  *)
    echo "用法：$0 {start|stop|status|log|check}" >&2
    exit 2
    ;;
esac
