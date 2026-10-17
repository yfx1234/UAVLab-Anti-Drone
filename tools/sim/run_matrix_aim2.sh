#!/usr/bin/env bash
# 全几何矩阵复测 v2（2026-10-14）：当前完整配置（安装定案+yaw伺服+
# bearing①②层+方位伺服+摆动抑制）× 20 格 × 2 轮 = 40 轮
# 口径对齐 2026-10-12 matrix_geo 基线（同场景/速度/结果格式）
OUT=~/netgun_logs/matrix_aim2
mkdir -p $OUT
RESULTS=$OUT/results.txt
echo "=== 全几何矩阵v2 ROUNDS=2 $(date) ===" > $RESULTS

run_cell() {
  local round=$1 name=$2 sc=$3 sp=$4 north=$5 radius=$6
  pkill -f "bin/px4" 2>/dev/null; sleep 2
  pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -f MicroXRCEAgent 2>/dev/null
  sleep 1
  source /opt/ros/humble/setup.bash
  source ~/UAV_ws/install/setup.bash
  source ~/interceptor_ws/install/setup.bash
  export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
  export PURSUER_POSE="0,-2,0.01"
  nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
  local agent_pid=$!
  sleep 2
  nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
  local r=0
  for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { r=1; break; }; sleep 1; done
  local logf=$OUT/r${round}_${name}_${sp}.log
  if [ "$r" != "1" ]; then
    echo "RESULT|$round|$name|$sp|BOOT_FAIL" >> $RESULTS
    pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $agent_pid 2>/dev/null
    return
  fi
  sleep 8
  timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
    target_scenario:=$sc target_speed_mps:=$sp target_radius_m:=$radius \
    target_initial_north_m:=$north \
    yaw_course_offset_rad:=0.0 depth_valid_max_m:=-1.0 \
    > $logf 2>&1
  local fired=$(grep -c "NETGUN FIRE" $logf)
  local reason=$(grep "NETGUN FIRE" $logf | tail -1 | grep -o "reason=[A-Z_]*" | cut -d= -f2)
  local hit=$(grep "REFEREE RESULT" $logf | tail -1 | grep -o "hit=[0-9]*" | cut -d= -f2)
  local miss=$(grep "REFEREE RESULT" $logf | tail -1 | grep -o "miss=[0-9.]*" | cut -d= -f2)
  local track=$(python3 - "$logf" <<'PYEOF'
import re, sys
v = []
try:
    for l in open(sys.argv[1], errors="ignore"):
        if "netgun diag" in l and "est=1(" in l:
            m = re.search(r"range=([0-9.]+)", l)
            if m and float(m.group(1)) > 0.1:
                v.append(float(m.group(1)))
except Exception:
    pass
if v:
    print(f"track(avg/max/n)={sum(v)/len(v):.2f}/{max(v):.2f}/{len(v)}")
else:
    print("track(avg/max/n)=na/na/0")
PYEOF
)
  echo "RESULT|$round|$name|$sp|fire=$fired|$reason|hit=$hit|miss=$miss|$track" >> $RESULTS
  kill $agent_pid 2>/dev/null
  pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
}

for ROUND in 1 2; do
  for SP in 1.0 2.0 3.0 4.0 5.0; do
    run_cell $ROUND pursuit_straight straight $SP 0.0 20.0
    run_cell $ROUND pursuit_circle circle $SP 0.0 "$(python3 -c "print(max(2.0, 2*$SP))")"
    run_cell $ROUND pursuit_s_curve s_curve $SP 0.0 20.0
    run_cell $ROUND headon_straight straight $SP -25.0 20.0
  done
done
echo "=== 完成 $(date) ===" >> $RESULTS
