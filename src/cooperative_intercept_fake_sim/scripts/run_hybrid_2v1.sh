#!/usr/bin/env bash
# -*- coding: utf-8 -*-
# 2 虚 1 实混合协同拦截：实机 14 m/s 统一拦截方案 + 两架 PX4 僚机包抄。
#
# 流程（复刻 run_predictive_shot_2mps_scenario.sh 的环境骨架）：
#   MicroXRCEAgent -> startpx4_quad.sh（目标 + 实机 + PX4 僚机×2）-> 等四机就绪
#   -> ros2 launch hybrid_2v1.launch.py（referee launch + hybrid 节点）
#   -> 等 hybrid 节点记录完成（duration 秒）-> 清理
#
# 用法：
#   TARGET_SPEED_MPS=8.0 bash scripts/run_hybrid_2v1.sh straight
#   TARGET_SPEED_MPS=8.0 bash scripts/run_hybrid_2v1.sh s_curve
# 可选环境变量：TARGET_SPEED_MPS / COMMAND_MAXIMUM_SPEED_MPS /
#   SAFETY_MAXIMUM_SPEED_MPS / TARGET_HEADING_RAD / HYBRID_DURATION_S
set -Eeuo pipefail

scenario="${1:?用法: $0 straight|circle|s_curve|figure_eight|rectangle}"
case "${scenario}" in
  straight|circle|s_curve|figure_eight|rectangle) ;;
  *)
    printf '不支持的轨迹: %s\n' "${scenario}" >&2
    exit 2
    ;;
esac

# 几何：HEAD_ON_MODE=true 为直线迎头（目标在实机前方 60 m 相向而行）；
# 否则为尾追（目标从原点出发，实机在其后 5 m）
head_on_mode="${HEAD_ON_MODE:-false}"
case "${head_on_mode}" in
  true|false) ;;
  *) printf 'HEAD_ON_MODE 仅支持 true/false\n' >&2; exit 2 ;;
esac
if [[ "${head_on_mode}" == "true" ]]; then
  target_initial_north_m="-60.0"
  target_initial_east_m="0.0"
  pursuer_initial_north_m="0.0"
  pursuer_initial_east_m="0.0"
  known_course_enabled="true"
  known_course_ned_rad="${TARGET_HEADING_RAD:-0.0}"
  # 搜索初始方位：指向目标（与成功迎头组一致：atan2(0, -60) = π）
  known_bearing_ned_rad="$(awk 'BEGIN { printf "%.12f", atan2(0 - 0, -60 - 0) }')"
  # 迎头成功组时序：目标已在入航线，攻击无需延迟、追踪保持更宽松
  attack_enable_delay_s="0.0"
  estimate_stability_hold_s="0.3"
  first_attack_tracking_hold_s="0.4"
  terminal_alignment_tracking_hold_s="0.15"
  terminal_alignment_ramp_s="0.25"
  known_search_hold_s="12.0"
else
  target_initial_north_m="0.0"
  target_initial_east_m="0.0"
  pursuer_initial_north_m="0.0"
  pursuer_initial_east_m="-5.0"
  known_course_enabled="false"
  known_course_ned_rad="0.0"
  known_bearing_ned_rad="0.0"
  attack_enable_delay_s="10.0"
  estimate_stability_hold_s="1.0"
  first_attack_tracking_hold_s="3.0"
  terminal_alignment_tracking_hold_s="2.0"
  terminal_alignment_ramp_s="1.0"
  known_search_hold_s="6.0"
fi

workspace="/home/key/UAV_ws"
timestamp="$(date +%Y%m%d_%H%M%S)"
target_speed_mps="${TARGET_SPEED_MPS:-8.0}"
command_maximum_speed_mps="${COMMAND_MAXIMUM_SPEED_MPS:-14.0}"
safety_maximum_speed_mps="${SAFETY_MAXIMUM_SPEED_MPS:-16.0}"
target_heading_rad="${TARGET_HEADING_RAD:-0.0}"
hybrid_duration_s="${HYBRID_DURATION_S:-60.0}"
output_root="${OUTPUT_ROOT:-/home/key/codex_work/hybrid_2v1_${scenario}_$(date +%Y%m%d)}"
run_dir="${output_root}/${timestamp}"
raw_dir="${run_dir}/原始数据"
mkdir -p "${raw_dir}"

agent_pid=""
sim_pid=""
launch_pid=""
px4_bin="${workspace}/../PX4-Autopilot/build/px4_sitl_default/bin"
px4_bin="/home/key/PX4-Autopilot/build/px4_sitl_default/bin"

wait_for_exit() {
  local pid="${1}" limit="${2:-10}"
  local i
  for ((i = 0; i < limit * 4; ++i)); do
    kill -0 "${pid}" 2>/dev/null || return 0
    sleep 0.25
  done
  return 1
}

stop_process_group() {
  local pid="${1}" sig="${2:-INT}"
  kill "-${sig}" "-${pid}" 2>/dev/null || true
  wait_for_exit "${pid}" 15 || kill -KILL "-${pid}" 2>/dev/null || true
}

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM
  if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
    stop_process_group "${launch_pid}" INT
  fi
  if [[ -n "${sim_pid}" ]] && kill -0 "${sim_pid}" 2>/dev/null; then
    stop_process_group "${sim_pid}" INT
  fi
  if [[ -n "${agent_pid}" ]] && kill -0 "${agent_pid}" 2>/dev/null; then
    kill -TERM "${agent_pid}" 2>/dev/null || true
    wait_for_exit "${agent_pid}" 5 || true
  fi
  printf '清理完成（exit=%s），结果目录: %s\n' "${exit_code}" "${run_dir}"
  exit "${exit_code}"
}
trap cleanup EXIT INT TERM

if pgrep -x px4 >/dev/null 2>&1 ||
  pgrep -f 'gz sim server' >/dev/null 2>&1 ||
  ss -lun 2>/dev/null | grep -qE '(^|[[:space:]])[^[:space:]]*:8888[[:space:]]'
then
  printf '检测到残留仿真或DDS进程，请先清理后再运行。\n' >&2
  exit 3
fi

# ---- 1. Micro XRCE-DDS Agent ----
/usr/local/bin/MicroXRCEAgent udp4 -p 8888 >"${raw_dir}/dds_agent.log" 2>&1 &
agent_pid=$!
for _ in {1..20}; do
  ss -lun 2>/dev/null |
    grep -qE '(^|[[:space:]])[^[:space:]]*:8888[[:space:]]' && break
  kill -0 "${agent_pid}" 2>/dev/null || exit 4
  sleep 0.25
done

# ---- 2. 四机 PX4 SITL + Gazebo（目标 x500 / 实机 22001 / 僚机 22001 ×2）----
# Gazebo 位姿为 ENU(east,north,up)；僚机在实机后两翼：
#   P2 NED(-40, 30) → ENU(30, -40)；P3 NED(-40, -30) → ENU(-30, -40)
WINGMAN_L_POSE="30,-40,0.01,0,0,1.570796" \
WINGMAN_R_POSE="-30,-40,0.01,0,0,1.570796" \
TARGET_POSE="${target_initial_east_m},${target_initial_north_m},0.01,0,0,1.570796" \
PURSUER_POSE="${pursuer_initial_east_m},${pursuer_initial_north_m},0.01,0,0,1.570796" \
TARGET_AUTOSTART=4001 TARGET_SIM_MODEL=gz_x500 \
PURSUER_AUTOSTART=22001 PURSUER_SIM_MODEL=gz_my_quad_real \
WINGMAN_AUTOSTART=22001 WINGMAN_SIM_MODEL=gz_my_quad_real \
DUAL_SIM_LOG_DIR="${raw_dir}/px4_gazebo" \
DUAL_SIM_ULOG_OUTPUT_DIR="${raw_dir}" \
HEADLESS=true setsid "${workspace}/src/cooperative_intercept_fake_sim/scripts/startpx4_quad.sh" \
  >"${raw_dir}/simulator.log" 2>&1 &
sim_pid=$!

sim_ready=false
for _ in {1..120}; do
  if grep -q 'All four PX4 instances are ready' "${raw_dir}/simulator.log" 2>/dev/null; then
    sim_ready=true
    break
  fi
  kill -0 "${sim_pid}" 2>/dev/null || exit 5
  sleep 1
done
[[ "${sim_ready}" == true ]] || { printf 'PX4 四机未就绪\n' >&2; exit 6; }
printf '四机 PX4 就绪，启动 ROS 节点\n'

# ---- 3. ROS launch（referee 方案 + hybrid 节点）----
set +u  # setup.bash 在 set -u 下会因未定义变量报错
source /opt/ros/humble/setup.bash
source "${workspace}/install/setup.bash"
set -u
setsid ros2 launch cooperative_intercept_fake_sim hybrid_2v1.launch.py \
  target_scenario:="${scenario}" \
  target_speed_mps:="${target_speed_mps}" \
  target_heading_rad:="${target_heading_rad}" \
  command_maximum_speed_mps:="${command_maximum_speed_mps}" \
  safety_maximum_speed_mps:="${safety_maximum_speed_mps}" \
  known_target_direction_enabled:="${known_course_enabled}" \
  known_target_course_enabled:="${known_course_enabled}" \
  known_target_course_ned_rad:="${known_course_ned_rad}" \
  known_target_search_velocity_mps:="${target_speed_mps}" \
  known_target_search_hold_s:="${known_search_hold_s}" \
  known_target_bearing_ned_rad:="${known_bearing_ned_rad}" \
  attack_enable_delay_s:="${attack_enable_delay_s}" \
  estimate_stability_hold_s:="${estimate_stability_hold_s}" \
  first_attack_tracking_hold_s:="${first_attack_tracking_hold_s}" \
  terminal_alignment_tracking_hold_s:="${terminal_alignment_tracking_hold_s}" \
  terminal_alignment_ramp_s:="${terminal_alignment_ramp_s}" \
  duration_s:="${hybrid_duration_s}" \
  wingman_enabled:="${WINGMAN_ENABLED:-true}" \
  >"${raw_dir}/tracking.log" 2>&1 &
launch_pid=$!

# ---- 4. 等待 hybrid 节点记录完成（或 300 s 超时）----
wait_status=124
for ((i = 0; i < 600; ++i)); do
  if grep -q 'Hybrid 2V1 记录完成，节点退出' "${raw_dir}/tracking.log" 2>/dev/null; then
    wait_status=0
    sleep 2
    break
  fi
  if grep -qE '\[ERROR\] \[(offboard_vehicle|hybrid_2v1_sim|pure_interceptor_controller)[^]]*\]: process has died|InvalidParameterType' \
      "${raw_dir}/tracking.log" 2>/dev/null; then
    wait_status=2
    printf '关键 ROS 节点失败，提前结束本轮\n' >&2
    break
  fi
  if ! kill -0 "${launch_pid}" 2>/dev/null; then
    wait_status=1
    break
  fi
  sleep 0.5
done

printf 'hybrid wait_status=%s\n' "${wait_status}" | tee -a "${run_dir}/运行结果.txt"
grep -E 'Hybrid 2V1 结果' "${raw_dir}/tracking.log" | tail -1 >>"${run_dir}/运行结果.txt" || true
printf '结果目录: %s\n' "${run_dir}"
[[ "${wait_status}" == 0 ]]
