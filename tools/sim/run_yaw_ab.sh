#!/usr/bin/env bash
# Yaw servo A/B: offset 0 = wide axis along course (real mount), pi/2 = narrow along course
OUT=~/netgun_logs/yaw_ab
mkdir -p $OUT
run_ab() {
  local tag=$1 offset=$2 sc=$3 sp=$4 north=$5 dm=$6
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
  if [ "$r" != "1" ]; then
    echo "RESULT|$tag|BOOT_FAIL" >> $OUT/results.txt
    pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $agent_pid 2>/dev/null
    return
  fi
  sleep 8
  timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
    target_scenario:=$sc target_speed_mps:=$sp target_radius_m:=20.0 \
    target_initial_north_m:=$north yaw_course_offset_rad:=$offset \
    depth_valid_max_m:=$dm \
    > $OUT/${tag}.log 2>&1
  local logf=$OUT/${tag}.log
  local fired=$(grep -c "NETGUN FIRE" $logf)
  local reason=$(grep "NETGUN FIRE" $logf | tail -1 | grep -o "reason=[A-Z_]*")
  local rng=$(grep "NETGUN FIRE" $logf | tail -1 | grep -o "range=[0-9.]*")
  local hits=$(grep -c "REFEREE RESULT shot=.* hit=1" $logf 2>/dev/null)
  local miss=$(grep "REFEREE RESULT" $logf | tail -1 | grep -o "miss=[0-9.]*")
  local gated=$(grep -c "logical camera gated" $logf)
  local yawline=$(grep "yaw servo engaged" $logf | tail -1)
  local obs=$(grep "estimator diag" $logf | tail -1)
  local vmax=$(grep "netgun diag" $logf | grep -o "axis_err=[0-9.]*" | sort -t= -k2 -n | tail -1)
  echo "RESULT|$tag|fire=$fired|$reason|$rng|hits=$hits|$miss|gated_att=$gated|$vmax" >> $OUT/results.txt
  echo "  YAW: $yawline" >> $OUT/results.txt
  echo "  OBS: $obs" >> $OUT/results.txt
  kill $agent_pid 2>/dev/null
  pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
}
echo "=== yaw A/B $(date) ===" > $OUT/results.txt
run_ab tw3_straight5 0.0 straight 5.0 0.0 -1.0
run_ab tw3_headon3 0.0 straight 3.0 -25.0 -1.0
echo "=== DONE $(date) ===" >> $OUT/results.txt
cat $OUT/results.txt
