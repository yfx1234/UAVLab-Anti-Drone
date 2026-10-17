#!/usr/bin/env bash
RESULT_FILE=~/netgun_matrix_results.txt
echo "=== netgun Gazebo 全矩阵 $(date) ===" > $RESULT_FILE
run_scenario() {
  local sc=$1 sp=$2 rd=$3
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
    echo "RESULT|$sc|$sp|BOOT_FAIL" >> $RESULT_FILE
    pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; kill $agent_pid 2>/dev/null
    return
  fi
  sleep 8
  : > /tmp/netgun_hits.txt
  # launch 后台起，20 秒后挂结果记录器（echo 遇无发布者会立即退出，不能先起）
  timeout 85 ros2 launch netgun_intercept netgun_sim_12m.launch.py \
    target_scenario:=$sc target_speed_mps:=$sp target_radius_m:=$rd \
    > /tmp/netgun_sim.log 2>&1 &
  local launch_pid=$!
  sleep 20
  ros2 topic echo /evaluation/virtual_hit_result > /tmp/netgun_hits.txt 2>&1 &
  local rec_pid=$!
  wait $launch_pid
  local fired=$(grep -c "NETGUN FIRE" /tmp/netgun_sim.log)
  local reason=$(grep "NETGUN FIRE" /tmp/netgun_sim.log | tail -1 | grep -o "reason=[A-Z_]*")
  local rng=$(grep "NETGUN FIRE" /tmp/netgun_sim.log | tail -1 | grep -o "range=[0-9.]*")
  local hits=$(grep -c "REFEREE RESULT shot=.* hit=1" /tmp/netgun_sim.log 2>/dev/null)
  echo "RESULT|$sc|$sp|fire=$fired|$reason|$rng|hits=$hits" >> $RESULT_FILE
  kill $rec_pid $agent_pid 2>/dev/null
  pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -f MicroXRCEAgent
}
for s in 1.0 2.0 3.0 4.0 5.0; do run_scenario straight $s 20.0; done
for s in 1.0 2.0 3.0 4.0 5.0; do run_scenario circle $s $(python3 -c "print(max(2.0, 2*$s))"); done
for s in 1.0 2.0 3.0 4.0 5.0; do run_scenario s_curve $s 20.0; done
echo "=== MATRIX COMPLETE $(date) ===" >> $RESULT_FILE
