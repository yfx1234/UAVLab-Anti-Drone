#!/usr/bin/env bash
# 多轮全矩阵（对齐镜像 12 轮统计口径）。
# 用法（WSL 内）: ROUNDS=3 DURATION=110 bash run_matrix_rounds.sh
# 输出: ~/netgun_logs/matrix_rounds/results.txt（RESULT|轮|场景|速度|fire|reason|hit|miss）
ROUNDS="${ROUNDS:-3}"
DURATION="${DURATION:-110}"
OUT=~/netgun_logs/matrix_rounds
mkdir -p "$OUT"
echo "=== netgun 多轮矩阵 ROUNDS=$ROUNDS $(date) ===" > "$OUT/results.txt"
run_one() {
  local r=$1 sc=$2 sp=$3 rd=$4
  pkill -f "bin/px4" 2>/dev/null; sleep 2
  pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -f MicroXRCEAgent 2>/dev/null
  sleep 1
  source /opt/ros/humble/setup.bash
  source ~/UAV_ws/install/setup.bash
  source ~/falcon_verify_ws/install/setup.bash
  export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
  export PURSUER_POSE="0,-2,0.01"
  nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
  local agent_pid=$!
  sleep 2
  nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
  local ok=0
  for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { ok=1; break; }; sleep 1; done
  if [ "$ok" != "1" ]; then
    echo "RESULT|$r|$sc|$sp|BOOT_FAIL||0|" >> "$OUT/results.txt"
    pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $agent_pid 2>/dev/null
    return
  fi
  sleep 8
  timeout "$DURATION" ros2 launch netgun_intercept netgun_sim_12m.launch.py \
    target_scenario:=$sc target_speed_mps:=$sp target_radius_m:=$rd \
    > /tmp/netgun_sim.log 2>&1
  local fired=$(grep -c "NETGUN FIRE" /tmp/netgun_sim.log)
  local reason=$(grep "NETGUN FIRE" /tmp/netgun_sim.log | tail -1 | grep -o "reason=[A-Z_]*")
  local hit=$(grep "REFEREE RESULT" /tmp/netgun_sim.log | tail -1 | grep -o "hit=[01]" | cut -d= -f2)
  local miss=$(grep "REFEREE RESULT" /tmp/netgun_sim.log | tail -1 | grep -o "miss=[0-9.]*" | cut -d= -f2)
  echo "RESULT|$r|$sc|$sp|fire=$fired|$reason|hit=${hit:-0}|miss=${miss:-}" >> "$OUT/results.txt"
  cp /tmp/netgun_sim.log "$OUT/r${r}_${sc}_${sp}.log" 2>/dev/null
  pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
}
for r in $(seq 1 "$ROUNDS"); do
  for s in 1.0 2.0 3.0 4.0 5.0; do run_one "$r" straight "$s" 20.0; done
  for s in 1.0 2.0 3.0 4.0 5.0; do run_one "$r" circle "$s" "$(python3 -c "print(max(2.0, 2*$s))")"; done
  for s in 1.0 2.0 3.0 4.0 5.0; do run_one "$r" s_curve "$s" 20.0; done
done
echo "=== 完成 $(date)；按格汇总： ===" >> "$OUT/results.txt"
awk -F'|' 'BEGIN{OFS="|"} $5!=""{split($5,a,"=");key=$3"@"$4; tot[key]++; if(a[2]>0) fire[key]++; split($7,h,"="); if(h[2]==1) hits[key]++} END{for (k in tot) print k, "fire="fire[k]+0"/"tot[k], "hit="hits[k]+0}' "$OUT/results.txt" >> "$OUT/results.txt" 2>/dev/null
echo "MATRIX_ROUNDS_DONE"
