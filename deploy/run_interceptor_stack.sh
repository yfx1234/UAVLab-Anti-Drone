#!/usr/bin/env bash
# 板上拦截链启动/停止脚本（实机部署，2026-10-15 首版）。
#
# 用法：
#   ./run_interceptor_stack.sh start [launch 参数...]   # 启动（含感知桥）
#   ./run_interceptor_stack.sh stop                     # 停止（SIGINT 优雅退出）
#   ./run_interceptor_stack.sh status                   # 进程 + 最近日志
#   ./run_interceptor_stack.sh log                      # 跟踪最新日志
#   ./run_interceptor_stack.sh check                    # 立即跑一次启动自检
#   ./run_interceptor_stack.sh review [N]               # 最近 N 次复盘记录（默认 5）
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
# 安装参数（2026-10-23 修订）：面板「安装参数」保存在 ~/netgun_config/
# mounting_geometry.json（以网枪顶面中心为基准）。start 时经
# deploy/mounting_geometry.py launch-args 换算成相机外参与枪口偏置
# launch 参数，排在命令行参数之前（命令行显式参数仍可覆盖）；文件无效时拒绝
# 启动，不静默退回默认值。实际使用的参数以 NETGUN_GEOMETRY 行写入运行日志。
#
# 常驻发射通道（2026-10-23）：start 随拦截链起 deploy/netgun_fire_button.py serve
# （--parent-pid=launch，launch 退出即自停），面板「发射网枪」经本机套接字直达
# 火控，按键到发布约 0.1 s；通道未就绪时按钮自动退回直连（约 2–3 s）。
#
# 实物击发（2026-10-23）：start 默认装订 trigger_command:=python3 -I -S deploy/netgun_pwm_link.py
# fire——火控开火时经常驻击发服务（用户级 systemd netgun-pwm-link.service，见
# deploy/install_netgun_pwm_link.sh）→ MAVProxy → 飞控 FMU PWM OUT 5：已解锁才发
# DO_SET_ACTUATOR 击发值，保持后回安全值；未解锁拒绝且不发任何指令。环境变量
# NETGUN_TRIGGER_COMMAND 覆盖（置空 = 仅虚拟事件），命令行 trigger_command:=… 最后生效；
# 实际装订写入运行日志 NETGUN_TRIGGER 行，启动时打印链路状态。拦截链进程带
# NETGUN_REVIEW_DIR=本次复盘目录，击发记录同时写入其中 pwm_events.jsonl。
#
# 复盘记录（2026-10-23）：start 同时在 ~/netgun_logs/review_<时间>/ 录 ros2 bag
# （REVIEW_TOPICS 白名单：飞控收发、四节点调试/估计/授权/开火事件、发射指令、
# 感知观测、/rosout），写 run_info.txt（提交号/launch 参数）、安装参数副本，
# 15s 后存启动自检 start_check.txt 并导出四节点参数 params/*.yaml；stop 时 SIGINT 录制器收尾（metadata.yaml），
# 并拷入本次运行日志 stack.log。sqlite3 用 resilient 预设（WAL），断电也只丢
# 最后几秒；缺 metadata.yaml 时 ros2 bag reindex 修复。剩余空间
# < NETGUN_REVIEW_MIN_FREE_MB（默认 5120）时不录 bag，只留日志与参数。

set -euo pipefail

INTERCEPTOR_WS="${INTERCEPTOR_WS:-$HOME/interceptor_ws}"
LOG_DIR="${NETGUN_LOG_DIR:-$HOME/netgun_logs}"
PID_FILE="$LOG_DIR/interceptor_stack.pid"
CHECK_FILE="$LOG_DIR/last_start_check.txt"
LAUNCH_PKG="netgun_intercept"
LAUNCH_FILE="netgun_real.launch.py"
GEOMETRY_TOOL="$INTERCEPTOR_WS/deploy/mounting_geometry.py"
GEOMETRY_FILE="${NETGUN_CONFIG_DIR:-$HOME/netgun_config}/mounting_geometry.json"
FIRE_TOOL="$INTERCEPTOR_WS/deploy/netgun_fire_button.py"
PWM_TOOL="$INTERCEPTOR_WS/deploy/netgun_pwm_link.py"
FIRE_PID_FILE="$LOG_DIR/netgun_fire_server.pid"
BAG_PID_FILE="$LOG_DIR/interceptor_bag.pid"
REVIEW_FILE="$LOG_DIR/interceptor_review.current"
REVIEW_MIN_FREE_MB="${NETGUN_REVIEW_MIN_FREE_MB:-5120}"
# ros2 bag record 的话题列表与 --regex 取交集，白名单只能写成一条正则；
# /fmu/in 下没有发布者的话题（agent 侧订阅）不会被录
REVIEW_TOPICS='^/(fmu/(in|out)/.*|debug/.*|tracking/.*|netgun/.*|perception/(observations_json|depth/observation)|simulation/pursuer_takeoff_ready|rosout|parameter_events)$'
# 全名（含命名空间，见 netgun_real.launch.py：三节点在 /pursuer 下）；文件名取末段
REVIEW_NODES="zed_perception_bridge pursuer/relative_estimator pursuer/netgun_guidance_node pursuer/netgun_strike_node"

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

# PID 文件可能是断电/重启前留下的：进程号被复用时命令行对不上，视为未运行
#（否则 stop 会 SIGINT 无关进程组）
pid_from_file() {
  local file="$1" pattern="${2:-}" pid cmdline
  [ -f "$file" ] || return 1
  pid="$(cat "$file" 2>/dev/null || true)"
  [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null || return 1
  if [ -n "$pattern" ]; then
    cmdline="$(tr '\0' ' ' 2>/dev/null <"/proc/$pid/cmdline" || true)"
    case "$cmdline" in
      *"$pattern"*) ;;
      *) return 1 ;;
    esac
  fi
  echo "$pid"
}

running_pid() {
  pid_from_file "$PID_FILE" "netgun_real"
}

# SIGINT 整个进程组（setsid 后 pid 即组号）并等待组长退出，超时补 SIGTERM。
# 返回 0=按 SIGINT 正常退出，1=超时
stop_group() {
  local pid="$1" wait_s="$2" status=1
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in $(seq 1 $((wait_s * 5))); do
    if ! kill -0 "$pid" 2>/dev/null; then
      status=0
      break
    fi
    sleep 0.2
  done
  kill -TERM -- "-$pid" 2>/dev/null || true
  return "$status"
}

# 同判据（只认生产域）找残留进程：PID 文件丢失时兜底清理用
production_pids_matching() {
  local p dom
  for p in $(pgrep -f "$1" 2>/dev/null || true); do
    dom="$(tr '\0' '\n' 2>/dev/null <"/proc/$p/environ" |
      sed -n 's/^ROS_DOMAIN_ID=//p' | head -1 || true)"
    if [ -z "$dom" ] || [ "$dom" = "0" ]; then
      echo "$p"
    fi
  done
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

pwm_link_brief() {
  if [ -f "$PWM_TOOL" ]; then
    timeout 5 python3 "$PWM_TOOL" status --brief 2>/dev/null || true
  else
    echo "实物击发链路：缺少 $PWM_TOOL"
  fi
}

trigger_summary() {
  local command="$1" link
  case "$command" in
    "") echo "实物击发：未装订（仅虚拟开火事件，不驱动网枪）" ;;
    *netgun_pwm_link.py*)
      link="$(pwm_link_brief)"
      case "$command" in
        *--bench*) echo "实物击发：已装订台架模式（仅未解锁可用）；${link#实物击发链路：}" ;;
        *) echo "实物击发：已装订 netgun_pwm_link；${link#实物击发链路：}" ;;
      esac
      ;;
    *) echo "实物击发：已装订自定义命令（$command）" ;;
  esac
}

start_fire_server() {
  local launch_pid="$1" log_file="$2" pid
  stop_fire_server
  if [ ! -f "$FIRE_TOOL" ]; then
    echo "WARN：缺少 $FIRE_TOOL，发射按钮只能走直连（约 2–3 s）" >>"$log_file"
    return 0
  fi
  nohup setsid python3 -u "$FIRE_TOOL" serve --parent-pid "$launch_pid" \
    >>"$log_file" 2>&1 </dev/null &
  pid=$!
  echo "$pid" >"$FIRE_PID_FILE"
}

stop_fire_server() {
  local pid p
  if pid="$(pid_from_file "$FIRE_PID_FILE" "netgun_fire_button")"; then
    stop_group "$pid" 3 || true
  fi
  rm -f "$FIRE_PID_FILE"
  # 兜底只认本工作区的助手（隔离测试副本/其他工作区的通道不受影响）
  for p in $(production_pids_matching "$FIRE_TOOL serve"); do
    kill -INT "$p" 2>/dev/null || true
  done
}

start_review() {
  local review_dir="$1" log_file="$2" free_mb pid
  shift 2
  finish_review >/dev/null 2>&1 || true
  mkdir -p "$review_dir"
  {
    echo "started=$(date '+%F %T %z')"
    echo "stack_log=$(basename "$log_file")"
    echo "commit=$(git -C "$INTERCEPTOR_WS" rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "launch_args=$*"
  } >"$review_dir/run_info.txt"
  if [ -f "$GEOMETRY_FILE" ]; then
    cp "$GEOMETRY_FILE" "$review_dir/mounting_geometry.json" 2>/dev/null || true
  fi
  echo "$review_dir" >"$REVIEW_FILE"
  free_mb="$(df -Pm "$LOG_DIR" 2>/dev/null | awk 'NR==2 {print $4}' || true)"
  if [ -n "$free_mb" ] && [ "$free_mb" -lt "$REVIEW_MIN_FREE_MB" ]; then
    echo "WARN：磁盘剩余 ${free_mb} MB（< ${REVIEW_MIN_FREE_MB} MB），本次不录复盘 bag" |
      tee -a "$review_dir/run_info.txt" >>"$log_file"
    return 0
  fi
  nohup setsid ros2 bag record -o "$review_dir/bag" --regex "$REVIEW_TOPICS" \
    --storage-preset-profile resilient >>"$review_dir/bag_record.log" 2>&1 </dev/null &
  pid=$!
  echo "$pid" >"$BAG_PID_FILE"
  echo "NETGUN_REVIEW $review_dir" >>"$log_file"
}

# 复盘记录收尾：SIGINT 录制器（写 metadata.yaml）、拷入本次运行日志、记结束时间。
# 输出一行摘要（无复盘记录时不输出）
finish_review() {
  local review_dir pid bag_state="未录制" stack_log size p
  review_dir="$(cat "$REVIEW_FILE" 2>/dev/null || true)"
  if pid="$(pid_from_file "$BAG_PID_FILE" "bag record")"; then
    if stop_group "$pid" 20; then
      bag_state="已收尾"
    else
      bag_state="收尾超时"
    fi
  elif [ -n "$review_dir" ] && [ -d "$review_dir/bag" ]; then
    bag_state="录制器提前退出"
  fi
  rm -f "$BAG_PID_FILE"
  for p in $(production_pids_matching "ros2 bag record -o $LOG_DIR/review_"); do
    kill -INT "$p" 2>/dev/null || true
  done
  if [ -z "$review_dir" ] || [ ! -d "$review_dir" ]; then
    rm -f "$REVIEW_FILE"
    return 0
  fi
  if [ -d "$review_dir/bag" ] && [ ! -f "$review_dir/bag/metadata.yaml" ]; then
    bag_state="$bag_state，缺 metadata.yaml（ros2 bag reindex 可修复）"
  fi
  stack_log="$(sed -n 's/^stack_log=//p' "$review_dir/run_info.txt" 2>/dev/null | head -1 || true)"
  if [ -n "$stack_log" ] && [ -f "$LOG_DIR/$stack_log" ]; then
    cp "$LOG_DIR/$stack_log" "$review_dir/stack.log" 2>/dev/null || true
  fi
  {
    echo "stopped=$(date '+%F %T %z')"
    echo "bag=$bag_state"
  } >>"$review_dir/run_info.txt"
  size="$(du -sh "$review_dir" 2>/dev/null | cut -f1 || true)"
  rm -f "$REVIEW_FILE"
  echo "复盘记录 $(basename "$review_dir")（${size:-?}，bag $bag_state）"
}

# 15s 后（节点参数已声明）导出四节点参数；失败只记 errors.log，不影响拦截链
dump_params() {
  local review_dir="$1" node out errors
  [ -d "$review_dir" ] || return 0
  mkdir -p "$review_dir/params"
  errors="$review_dir/params/errors.log"
  for node in $REVIEW_NODES; do
    out="$review_dir/params/${node##*/}.yaml"
    timeout 30 ros2 param dump "/$node" --no-daemon --spin-time 5 \
      >"$out" 2>>"$errors" || true
    if [ ! -s "$out" ]; then
      rm -f "$out"
      echo "/$node：参数导出失败" >>"$errors"
    fi
  done
  [ -s "$errors" ] || rm -f "$errors"
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
  # 实物击发装订：默认常驻击发服务；NETGUN_TRIGGER_COMMAND 覆盖（空=仅虚拟），
  # 命令行 trigger_command:=… 排在最后、最终生效（日志按它记录）
  local trigger_command="" arg
  local -a trigger_args=()
  if [ "${NETGUN_TRIGGER_COMMAND+set}" = set ]; then
    trigger_command="$NETGUN_TRIGGER_COMMAND"
  elif [ -f "$PWM_TOOL" ]; then
    # -I -S：客户端只用标准库，不加载 site/PYTHON* 环境，解释器启动快约 25 ms
    trigger_command="python3 -I -S $PWM_TOOL fire"
  fi
  if [ -n "$trigger_command" ]; then
    trigger_args=("trigger_command:=$trigger_command")
  fi
  for arg in "${@:2}"; do
    case "$arg" in trigger_command:=*) trigger_command="${arg#trigger_command:=}" ;; esac
  done
  env_setup
  local stamp log_file review_dir
  stamp="$(date +%Y%m%d_%H%M%S)"
  log_file="$LOG_DIR/interceptor_stack_${stamp}.log"
  review_dir="$LOG_DIR/review_${stamp}"
  if [ -n "$geometry_summary" ]; then
    printf '%s\n' "$geometry_summary" >>"$log_file"
  fi
  printf 'NETGUN_TRIGGER %s\n' "$trigger_command" >>"$log_file"
  # G-1（2026-10-18）：必须用追加模式——run_start_check 会 >> 同一文件
  # 写告警，截断写会让 launch 按自己的旧偏移把追加行覆盖掉
  NETGUN_REVIEW_DIR="$review_dir" nohup setsid ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE" \
    "${geometry_args[@]}" "${trigger_args[@]}" "${@:2}" >>"$log_file" 2>&1 &
  local pid=$!
  echo "$pid" >"$PID_FILE"
  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "启动失败，日志尾部：" >&2
    tail -20 "$log_file" >&2 || true
    rm -f "$PID_FILE"
    exit 1
  fi
  start_review "$review_dir" "$log_file" "${geometry_args[@]}" "${trigger_args[@]}" "${@:2}"
  start_fire_server "$pid" "$log_file"
  if [ -n "$geometry_brief" ]; then
    echo "$geometry_brief"
  fi
  local trigger_line
  trigger_line="$(trigger_summary "$trigger_command")"
  printf '%s\n' "$trigger_line" >>"$log_file"
  echo "$trigger_line"
  echo "复盘录制 $review_dir；常驻发射通道随链启动"
  echo "已启动 pid $pid，日志 $log_file"
  # 15s 后自检（run_start_check 内部落 CHECK_FILE 与运行日志，结论另存复盘目录
  # start_check.txt），再导出节点参数；后台子 shell 不占用本脚本的输出管道——
  # 面板 SSH 调用 start 时不必陪等
  ( sleep 15; run_start_check "$log_file" "$pid" >"$review_dir/start_check.txt" 2>&1
    dump_params "$review_dir" >/dev/null 2>&1 ) >/dev/null 2>&1 </dev/null &
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
  local pid review_line
  stop_fire_server
  if ! pid="$(running_pid)"; then
    # 兜底清理残留节点（PID 文件丢失场景；只清生产域，见 production_launch_pids）
    local p
    for p in $(production_launch_pids); do
      kill -INT "$p" 2>/dev/null || true
    done
    review_line="$(finish_review)"
    echo "拦截链未运行${review_line:+；$review_line}"
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
  # 录制器最后停：拦截链退出过程也录进复盘
  review_line="$(finish_review)"
  echo "已停止${review_line:+；$review_line}"
}

cmd_status() {
  local pid
  if pid="$(running_pid)"; then
    echo "运行中 pid $pid"
  else
    echo "未运行"
  fi
  pgrep -af "netgun_(guidance|strike)_node|relative_estimator|zed_perception_bridge" 2>/dev/null || true
  local review_dir
  if pid="$(pid_from_file "$FIRE_PID_FILE" "netgun_fire_button")"; then
    echo "常驻发射通道：运行中 pid $pid"
  else
    echo "常驻发射通道：未运行（按钮走直连）"
  fi
  local log_file bound
  log_file="$(latest_log)"
  if running_pid >/dev/null && [ -n "$log_file" ] && grep -q '^NETGUN_TRIGGER ' "$log_file"; then
    bound="$(sed -n 's/^NETGUN_TRIGGER //p' "$log_file" | tail -1)"
    echo "实物击发装订：${bound:-未装订（仅虚拟开火事件）}"
  fi
  pwm_link_brief
  review_dir="$(cat "$REVIEW_FILE" 2>/dev/null || true)"
  if pid="$(pid_from_file "$BAG_PID_FILE" "bag record")"; then
    echo "复盘录制：录制中 pid $pid → $review_dir（$(du -sh "$review_dir" 2>/dev/null | cut -f1)）"
  else
    echo "复盘录制：未录制"
  fi
  if [ -n "$log_file" ]; then
    echo "---- 最近日志（$log_file）----"
    tail -5 "$log_file" || true
  fi
  if [ -f "$CHECK_FILE" ]; then
    echo "---- 最近启动自检（$CHECK_FILE）----"
    tail -3 "$CHECK_FILE"
  fi
}

cmd_review() {
  local count="${1:-5}" dir state shots log
  case "$count" in '' | *[!0-9]*) count=5 ;; esac
  for dir in $(ls -dt "$LOG_DIR"/review_* 2>/dev/null | head -"$count" || true); do
    [ -d "$dir" ] || continue
    if [ "$(cat "$REVIEW_FILE" 2>/dev/null || true)" = "$dir" ] && pid_from_file "$BAG_PID_FILE" "bag record" >/dev/null; then
      state="录制中"
    else
      state="$(sed -n 's/^bag=//p' "$dir/run_info.txt" 2>/dev/null | tail -1 || true)"
      state="${state:-未收尾}"
    fi
    log="$dir/stack.log"
    if [ ! -f "$log" ]; then
      log="$LOG_DIR/$(sed -n 's/^stack_log=//p' "$dir/run_info.txt" 2>/dev/null | head -1 || true)"
    fi
    shots="$(grep -c "NETGUN FIRE" "$log" 2>/dev/null || true)"
    printf '%s  %s  开始 %s  bag %s  开火 %s 次\n' "$(basename "$dir")" \
      "$(du -sh "$dir" 2>/dev/null | cut -f1)" \
      "$(sed -n 's/^started=//p' "$dir/run_info.txt" 2>/dev/null | head -1 || true)" \
      "$state" "${shots:-0}"
  done
  echo "取回到电脑（PowerShell）：scp -r uavlab@192.168.50.100:~/netgun_logs/<目录> ."
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
  review) cmd_review "${2:-5}" ;;
  *)
    echo "用法：$0 {start|stop|status|log|check|review}" >&2
    exit 2
    ;;
esac
