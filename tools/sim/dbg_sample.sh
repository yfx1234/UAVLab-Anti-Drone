#!/usr/bin/env bash
# 调试话题采样（轻量）：boot + launch + t+25s 三个话题各抓一条 + 清场
source /opt/ros/humble/setup.bash
source ~/UAV_ws/install/setup.bash
source ~/interceptor_ws/install/setup.bash
pkill -f "bin/px4" 2>/dev/null; sleep 2
pkill -f "gz sim" 2>/dev/null; pkill -f "gz-sim" 2>/dev/null; pkill -x MicroXRCEAgent 2>/dev/null; sleep 1
export GZ_SIM_RESOURCE_PATH="$HOME/PX4-Autopilot/Tools/simulation/gz/models:$HOME/PX4-Autopilot/Tools/simulation/gz/worlds"
export PURSUER_POSE="0,-2,0.01"
nohup MicroXRCEAgent udp4 -p 8888 > /tmp/agent.log 2>&1 &
AGENT=$!
sleep 2
nohup ~/px4sim/startpx4_dual_x500_22001.sh > /tmp/boot.log 2>&1 &
R=0
for i in $(seq 1 75); do grep -q "Both PX4 instances are ready" /tmp/boot.log 2>/dev/null && { R=1; break; }; sleep 1; done
[ "$R" = "1" ] || { echo BOOT_FAIL; kill $AGENT; exit 1; }
sleep 8
ros2 launch netgun_intercept netgun_sim_12m.launch.py \
  target_scenario:=straight target_speed_mps:=1.0 target_radius_m:=20.0 \
  > /tmp/dbg_sim.log 2>&1 &
LAUNCH=$!
sleep 25
echo "=== guidance ==="
timeout 3 ros2 topic echo --once /debug/guidance_state 2>/dev/null | head -2
echo "=== strike ==="
timeout 3 ros2 topic echo --once /debug/strike_state 2>/dev/null | head -2
echo "=== estimator ==="
timeout 3 ros2 topic echo --once /debug/estimator_state 2>/dev/null | head -2
kill $LAUNCH 2>/dev/null; sleep 2
kill $AGENT 2>/dev/null
pkill -f "bin/px4"; sleep 2; pkill -f "gz sim"; pkill -f "gz-sim"; pkill -x MicroXRCEAgent
echo done
