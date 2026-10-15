#!/usr/bin/env bash
# One-command live range monitor (foolproof field entry).
#
# Finds the ACTIVE run directory from the running field_measurement_logger
# process itself (not ls/timestamps), waits for inputs, then starts
# live_range_monitor.py with field-tested defaults.
#
# Usage:
#   bash watch_live_monitor.sh                                      # lever 0,0,0
#   bash watch_live_monitor.sh 0.15,0,-0.05                         # lever-enu shorthand (东,北,上)
#   bash watch_live_monitor.sh --lever-body 0.133,0,0.0462 --rig-yaw-deg 90
#       (final CAD lever, see extrinsics.yaml rtk_antenna; yaw = 机头朝向角, deg from north)
#
# Exit codes: 1 = recorder not running; 2 = input streams not ready;
#             3 = ambiguous/duplicate recorder; 4 = monitor already running.
set -eo pipefail

WORKSPACE_DIR="${WORKSPACE_DIR:-/home/uavlab/Jetson_ROS2_V0/ros2_ws}"
TOOLS_DIR="${WORKSPACE_DIR}/src/falcon_intercept/tools/rtk_field_test"

LEVER_ARGS=()
if [[ $# -gt 0 && "${1:0:1}" != "-" ]]; then
  LEVER_ARGS=(--lever-enu "$1")
  shift
fi

mapfile -t LOGGER_LINES < <(pgrep -af '[f]ield_measurement_logger.py' || true)
if [[ "${#LOGGER_LINES[@]}" -eq 0 ]]; then
  echo "[watch] 录制没在跑（找不到 field_measurement_logger 进程）。先启动终端 2 的录制脚本，再开我。" >&2
  exit 1
fi
if [[ "${#LOGGER_LINES[@]}" -ne 1 ]]; then
  echo "[watch] 找到 ${#LOGGER_LINES[@]} 个 measurement logger，无法安全判断当前 run。先 Ctrl-C 停掉残留录制，只保留一个：" >&2
  printf '  %s\n' "${LOGGER_LINES[@]}" >&2
  exit 3
fi

OUT_LINE="${LOGGER_LINES[0]}"
MEAS="$(sed -n 's/.*--output \([^ ]*measurements\.jsonl\).*/\1/p' <<<"${OUT_LINE}")"
if [[ -z "${MEAS}" ]]; then
  echo "[watch] 找到 logger 进程但解析不出输出路径：${OUT_LINE}" >&2
  exit 2
fi
DIR="$(dirname "${MEAS}")"
ENEMY="${DIR}/enemy_rtk.jsonl"

echo "[watch] run dir: ${DIR}"
echo "[watch] lever args: ${LEVER_ARGS[*]:-（无，默认0,0,0）} $*"

# Refuse a second monitor for the same run. Concurrent appenders can corrupt
# live exports, while a restarted monitor replays the JSONL inputs from offset 0.
mapfile -t MONITOR_LINES < <(pgrep -af '[l]ive_range_monitor.py' || true)
for line in "${MONITOR_LINES[@]}"; do
  if [[ "${line}" == *"--measurements ${MEAS}"* ]]; then
    echo "[watch] 当前 run 已有实时监控器，直接查看原终端，不要重复启动：${line}" >&2
    exit 4
  fi
done

# File existence is not enough: both loggers create their JSONL before data
# arrives. Require both enemy MAVLink and own DDS streams to grow within 30 s.
echo "[watch] 等敌机真值和我机测量流连续增长（最长 30 秒）..."
prev_enemy=-1
prev_meas=-1
enemy_growths=0
meas_growths=0
streams_ready=0
for _ in $(seq 1 30); do
  enemy_size="$(stat -c '%s' -- "${ENEMY}" 2>/dev/null || echo 0)"
  meas_size="$(stat -c '%s' -- "${MEAS}" 2>/dev/null || echo 0)"
  if [[ "${prev_enemy}" -ge 0 && "${enemy_size}" -gt "${prev_enemy}" ]]; then
    enemy_growths=$((enemy_growths + 1))
  fi
  if [[ "${prev_meas}" -ge 0 && "${meas_size}" -gt "${prev_meas}" ]]; then
    meas_growths=$((meas_growths + 1))
  fi
  if [[ "${enemy_growths}" -ge 2 && "${meas_growths}" -ge 2 ]]; then
    streams_ready=1
    break
  fi
  prev_enemy="${enemy_size}"
  prev_meas="${meas_size}"
  sleep 1
done
if [[ "${streams_ready}" -ne 1 ]]; then
  echo "[watch] 30 秒内敌我两个输入未各自增长至少两次（enemy=${prev_enemy}B/${enemy_growths}次 measurements=${prev_meas}B/${meas_growths}次）。" >&2
  echo "[watch] 查 MAVProxy --out/IP、ufw 14560/udp，以及 field_measurement_logger.log 的 own_rtk 计数。" >&2
  exit 2
fi

CSV_OUT="${DIR}/live_comparison.csv"
JSONL_OUT="${DIR}/live_comparison.jsonl"
if [[ -s "${CSV_OUT}" || -s "${JSONL_OUT}" ]]; then
  restart_tag="$(date +%Y%m%d_%H%M%S)"
  CSV_OUT="${DIR}/live_comparison_${restart_tag}.csv"
  JSONL_OUT="${DIR}/live_comparison_${restart_tag}.jsonl"
  echo "[watch] 检测到旧监控导出；本次重启写入独立文件，避免历史样本重复："
  echo "[watch]   ${CSV_OUT}"
  echo "[watch]   ${JSONL_OUT}"
fi

echo "[watch] 判读：own_fixes/enemy_fixes 在涨=数据活着；compared 在涨=出误差；出现 !!STALE!!=数据断了查录制。"
exec python3 "${TOOLS_DIR}/live_range_monitor.py" \
  --enemy "${ENEMY}" \
  --measurements "${MEAS}" \
  "${LEVER_ARGS[@]}" \
  --max-gap-s 0.6 \
  --out-csv "${CSV_OUT}" \
  --out-jsonl "${JSONL_OUT}" \
  "$@"
