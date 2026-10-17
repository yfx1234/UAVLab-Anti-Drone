#!/usr/bin/env bash
# 网枪链 12m 布置一键联仿（方案 §11 S3 配方，全部结论已在本会话验证）。
# 用法（WSL 内）: bash run_netgun_sim.sh [仿真时长秒，默认 180] [火控档: std|relaxed]
# 前置：~/falcon_verify_ws（含 netgun_intercept）、~/UAV_ws、~/px4sim、
#       /usr/local/bin/MicroXRCEAgent。
# 关键配方（勿"简化"）：单会话完成（WSL 空闲~60s 会回收清 /tmp）；
# 清理必须含 gz-sim 进程模式（原脚本 pkill 匹配不到 gz-sim8.server）；
# GZ_SIM_RESOURCE_PATH 显式导出；RMW 用默认 fastrtps（勿 cyclonedds）。

# 注意：不能用 set -u（ROS setup.bash 有未绑定变量）；健壮性靠显式检查。
DURATION="${1:-180}"
FIRE_PROFILE="${2:-std}"
VERIFY_WS="${VERIFY_WS:-$HOME/falcon_verify_ws}"
export PURSUER_POSE="${PURSUER_POSE:-0,-2,0.01}"   # 垂直 12m 布置（水平 2m 安全偏置；12m 水平间距超出 4mm 视场几何上不可见）

cleanup() {
  pkill -f "bin/px4" 2>/dev/null; sleep 2
  pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null
  pkill -f MicroXRCEAgent 2>/dev/null
}
trap cleanup EXIT

cleanup; sleep 1
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source "$VERIFY_WS/install/setup.bash"
unset ROS_LOCALHOST_ONLY CYCLONEDDS_URI FASTRTPS_DEFAULT_PROFILES_FILE
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"

echo "== [1/6] agent =="
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/netgun_agent.log 2>&1 &
AGENT_PID=$!
sleep 2

echo "== [2/6] 双机 SITL（目标 x500 / 追踪机 my_quad_real，PURSUER_POSE=$PURSUER_POSE） =="
nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/netgun_boot.log 2>&1 &
ready=0
for i in $(seq 1 75); do
  grep -q "Both PX4 instances are ready" /tmp/netgun_boot.log 2>/dev/null && { ready=1; break; }
  grep -qE "ERROR" /tmp/netgun_boot.log 2>/dev/null && break
  sleep 1
done
echo "boot ready=$ready after ${i}s"
sleep 8

echo "== [3/6] 健康检查 =="
MODELS=$(gz topic -l 2>/dev/null | grep -c "model/x500_0")
STATS=$(timeout 6 gz topic -e -n 1 -t /world/default/stats 2>/dev/null | grep -c "sim_time")
echo "models=$MODELS stats=$STATS"
if [ "$MODELS" -lt 1 ] || [ "$STATS" -lt 1 ]; then
  echo "WORLD UNHEALTHY —— 见 ~/px4sim/logs/dual/ 最新 gazebo.log"; exit 1
fi

echo "== [4/6] 结果记录器 =="
: > /tmp/netgun_hits.txt
ros2 topic echo /evaluation/virtual_hit_result > /tmp/netgun_hits.txt 2>&1 &
RECORDER_PID=$!

echo "== [5/6] 网枪链 ${DURATION}s（火控档=${FIRE_PROFILE}） =="
run_one() {  # $1=scenario $2=speed $3=radius（单套固定参数，仅场景/速度变化）
  echo "---- 场景 $1 @ $2 m/s (radius=$3)"
  timeout "$DURATION" ros2 launch netgun_intercept netgun_sim_12m.launch.py     target_scenario:=$1 target_speed_mps:=$2 target_radius_m:=$3     > /tmp/netgun_sim.log 2>&1
  fired=$(grep -c "NETGUN FIRE" /tmp/netgun_sim.log)
  reason=$(grep "NETGUN FIRE" /tmp/netgun_sim.log | tail -1 | grep -o "reason=[A-Z_]*")
  hits=$(grep -c "REFEREE RESULT shot=.* hit=1" /tmp/netgun_sim.log 2>/dev/null)
  echo "RESULT|$1|$2|fire=$fired|$reason|hits=$hits"
  : > /tmp/netgun_hits.txt
}
if [ "${MATRIX:-0}" = "1" ]; then
  # 矩阵：直线/圆周/S × 1..5 m/s；圆周半径=2×速度（同镜像）
  for s in 1 2 3 4 5; do run_one straight "$s.0" 20.0; done
  for s in 1 2 3 4 5; do run_one circle "$s.0" "$(python3 -c "print(max(2.0, 2*$s))")"; done
  for s in 1 2 3 4 5; do run_one s_curve "$s.0" 20.0; done
else
  run_one "${SCENARIO:-straight}" "${SPEED:-5.0}" "${RADIUS:-20.0}"
fi
echo "launch_exit=$?"

echo "== [6/6] 结果判读 =="
grep -E "NETGUN FIRE|reason=" /tmp/netgun_sim.log | tail -5
echo "--- 关键进度："
grep -E "offboard_vehicle|simple_below_tracker" /tmp/netgun_sim.log | grep -vE "waiting for valid|process started" | tail -8
echo "--- 裁判命中："
grep -E "hit|success" /tmp/netgun_hits.txt | tail -3
kill $RECORDER_PID $AGENT_PID 2>/dev/null
echo "NETGUN_SIM_DONE（完整日志 /tmp/netgun_sim.log /tmp/netgun_hits.txt）"
