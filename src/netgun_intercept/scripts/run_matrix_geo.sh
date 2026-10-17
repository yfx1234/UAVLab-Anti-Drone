#!/usr/bin/env bash
# 全几何多轮矩阵：追击(straight/circle/s_curve) + 迎敌(straight 逼近)。
# 用法: ROUNDS=2 DURATION=110 bash run_matrix_geo.sh
# 输出: ~/netgun_logs/matrix_geo/results.txt
ROUNDS="${ROUNDS:-2}"
DURATION="${DURATION:-110}"
OUT=~/netgun_logs/matrix_geo
mkdir -p "$OUT"
echo "=== 全几何矩阵 ROUNDS=$ROUNDS $(date) ===" > "$OUT/results.txt"
run_one() {
  local r=$1 sc=$2 sp=$3 extra=$4 traj=${5:-$sc}
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
    echo "RESULT|$r|$sc|$sp|BOOT_FAIL||0||" >> "$OUT/results.txt"; return
  fi
  sleep 8
  timeout "$DURATION" ros2 launch netgun_intercept netgun_sim_12m.launch.py \
    target_scenario:=$traj target_speed_mps:=$sp $extra \
    > /tmp/netgun_sim.log 2>&1
  local fired=$(grep -c "NETGUN FIRE" /tmp/netgun_sim.log)
  local reason=$(grep "NETGUN FIRE" /tmp/netgun_sim.log | tail -1 | grep -o "reason=[A-Z_]*")
  local hit=$(grep "REFEREE RESULT" /tmp/netgun_sim.log | tail -1 | grep -o "hit=[01]" | cut -d= -f2)
  local miss=$(grep "REFEREE RESULT" /tmp/netgun_sim.log | tail -1 | grep -o "miss=[0-9.]*" | cut -d= -f2)
  # 跟踪质量：auth=0 且 est=1 期间的轴误差均值/峰值（跟踪验证指标）
  local trk=$(grep "netgun diag" /tmp/netgun_sim.log | grep "est=1" | grep "auth=0" | grep -oE "axis_err=[0-9.]+" | cut -d= -f2 | sort -n | awk '{a[NR]=$1; s+=$1} END{if(NR>0) printf "%.2f/%.2f/%d", s/NR, a[NR], NR; else printf "na/na/0"}')
  echo "RESULT|$r|$sc|$sp|fire=$fired|$reason|hit=${hit:-0}|miss=${miss:-}|track(avg/max/n)=$trk" >> "$OUT/results.txt"
  cp /tmp/netgun_sim.log "$OUT/r${r}_${sc}_${sp}.log" 2>/dev/null
  pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
}
for r in $(seq 1 "$ROUNDS"); do
  for s in 1.0 2.0 3.0 4.0 5.0; do
    run_one "$r" pursuit_straight "$s" "target_radius_m:=20.0 attack_enable_delay_s:=16.0" straight
  done
  for s in 1.0 2.0 3.0 4.0 5.0; do
    run_one "$r" headon_straight "$s" "target_radius_m:=20.0 target_initial_north_m:=-25.0" straight
  done
  for s in 1.0 2.0 3.0 4.0 5.0; do
    run_one "$r" pursuit_circle "$s" "target_radius_m:=$(python3 -c "print(max(2.0, 2*$s))") attack_enable_delay_s:=16.0" circle
  done
  for s in 1.0 2.0 3.0 4.0 5.0; do
    run_one "$r" pursuit_s_curve "$s" "target_radius_m:=20.0 attack_enable_delay_s:=16.0" s_curve
  done
done
echo "=== 完成 $(date) ===" >> "$OUT/results.txt"
echo MATRIX_GEO_DONE
